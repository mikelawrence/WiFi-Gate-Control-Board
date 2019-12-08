/*
  WiFi Gate Control Board 
  
  This code is designed to replace the functionality of US Automatic gate control board
  while adding WiFi access and Home Assistant control via MQTT.
  The board is compatible with an Arduino MKR1000.
  
  Built with Arduino IDE 1.8.10
  
  The following libraries must be installed using Library Manager:
  
    WiFi101 version 0.16.0 by Arduino
      WINC1501 Model B firmware version 19.6.1
    WiFi101OTA version 1.0.2 by Arduino
    SD version 1.2.4 by Arduino
    FreeRTOS_SAMD21 version 1.0.0 by BriscoeTech
    MQTT version 2.4.5 by Joel Gaehwiler
    I2C_DMAC version 1.1.8 by Martin Lindupp
    OneWire version 2.3.5 by Paul Stoffregen and many others
    DallasTemperature version 3.8.0 by Miles Burton and others

  Clock configuration
    XOSC32k is enabled, Arduino startup.c
    DFLL48M is enabled, Arduino startup.c
    OSC8M is disable, WiFiGate_Control_Board.c, changed from Arduino startup.c, ADC clock
  
  Generic Clock configuration
    GCLK0, Src DFLL48M, 48MHz Out, Arduino startup.c, Main clock
    GCLK1, Src XOSC32K, 32.768kHz Out, Arduino startup.c, ref clock to DFLL48M, ref clock to GCLK4
    GCLK2, Src OSCULP32K, 1.024kHz Out, WDTZero.ino, Watchdog Timer (WDT) clock 
    GCLK3, Src DFLL48M, 8MHz Out, hal.ino, changed from Arduino startup.c, ADC clock
    GCLK4, Src XOSC32K, 1.024kHz Out, RTCZero.cpp, RTC clock

  Status LED Flashing Amber or Red indicates a hard fault
    1 flash   - RTOS stack overflow
    2 flashes - RTOS malloc failed
    3 flashes - RTOS fatal error
    4 flashes - I2C hardware not detected
    5 flashes - WINC1500 module not detected

  Copyright (c) 2019 Mike Lawrence
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
// Modify ../WiFi101/src/bsp/include/nm_bsp_internal.h library file
// so that CONF_PERIPH is defined. This will enabled LEDS from WiFi Module
#include "state.h"
#include "network.h"
#include "WiFi_Gate_Control_Board.h"

/******************************************************************
 * Definitions in arduino_secrets.h
 ******************************************************************/
/*
#define SECRET_SSID             "SSID of your WiFi network"
#define SECRET_PASSWORD         "Password for your WIFi network"
// IP address of MQTT server, host name may work but I've never tried
#define MQTT_SERVER             "192.168.0.230"
#define MQTT_SERVERPORT         1883
// MQTT user name and password leave as empty string if not used
#define MQTT_USERNAME           ""
#define MQTT_PASSWORD           ""
// Over-The-Air Update password if used
#define OTA_PASSWORD            "password"
*/

/******************************************************************
 * Global Variables
 ******************************************************************/
// FreeRTOS task handles
TaskHandle_t h_HALTask;
TaskHandle_t h_NETTask;
TaskHandle_t h_STATETask;

/******************************************************************
 * Standard Arduino setup function
 ******************************************************************/
void setup() {
  // Serial setup
  #ifdef ENABLE_SERIAL
  Serial.begin(115200);
  //delay(2000);
  vNopDelayMS(2000);
  #endif
  
  // Announce who we are and software version
  Println("\nWiFi Gate Control Board: " BOARD_NAME);
  Println("  Software Version: " VERSION);
  Println("Initializing HAL");

  // Initialize HAL
  if (!HAL.begin()) {
    Println("Failed to initialize hardware devices. Nothing can be done. Halt!");
    ERR.setHaltError(ERROR_HAL_HALT);                 // This function will flash LED error and never return 
  }

  // WiFi setup
  if (WiFi.status() == WL_NO_SHIELD) {                // check for the presence of the WINC1500
    Println("Failed to find WINC1500 WiFi Module. Nothing can be done. Halt!");
    ERR.setHaltError(ERROR_WIFI_HALT);                // This function will flash error LED and never return 
  }
  Println("Found WINC1500 WiFi Module");
  #ifdef ENABLE_SERIAL
  // Display Firmware Version
  Print("  Firmware Version: ");
  Println(WiFi.firmwareVersion());
  Print("  Library Version: ");
  Println(WIFI_FIRMWARE_LATEST_MODEL_B);
  #endif

  // arduino startup configures OSC8M, source to GCLK3, output 8MHz
  // not sure why, motor.ino changes GCLK3 source to DFLL48M, output 8MHz
  // disable OSC8M because it is no longer used
  SYSCTRL->OSC8M.bit.ENABLE = 1;

  // Watchdog initialize
  if (WDTZero.begin()) {
    Println("Watchdog enabled");
  } else {
    Println("Watchdog NOT enabled");
  }

  Println("");
  
  // FreeRTOS error LED connections
  vSetErrorLed(STATUS_RED_LED, HIGH);

  // create tasks
  xTaskCreate(thread_net, "NET Task", 2048, NULL, tskIDLE_PRIORITY + 1, &h_NETTask);
  xTaskCreate(thread_hal, "HAL Task", 256, NULL, tskIDLE_PRIORITY + 2, &h_HALTask);
  xTaskCreate(thread_state, "STATE Task", 256, NULL, tskIDLE_PRIORITY + 3, &h_STATETask);

  // start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();
}

/******************************************************************
 * FreeRTOS idle loop
 * Should never be called because the Network Task has a 
 * higher priority and should never yield to this task.
 ******************************************************************/
void loop() {
  static uint32_t lastUpdate = 0;

  if (lastUpdate + 1000 < xTaskGetTickCount()) {
    Print(".");
    lastUpdate = xTaskGetTickCount();
  }
  
//  yield();
//  static uint32_t lastTempPublish = 0 - (TEMP_RATE + 10);
//  static uint8_t lastPushbuttonState = false;
//  static uint8_t calSampleInProgress = false;
//      
//  // Reset the watchdog every time loop() is called
//  WDTZero.reset();
//  
////  if (calSampleInProgress) {
////    // we are waiting for a calibration sample to finish
////    if (!MOTOR.calSampleBusy()) {
////      // calibration sample is done
////      //MOTOR.motorOff();
////      Print("Motor calibration sample average = "); Println(MOTOR.calSampleGet());
////      calSampleInProgress = false;
////    }
////  } else {
//    // there is no calibration sample in progress
//    if (HAL.inputRead(IN_PUSHBUTTON)) {
//      // pushbutton was pressed, start a new calibration sample
//      calSampleInProgress = true;
//      Println("Starting Motor calibration sample average...");
////      MOTOR.motorOn(MOTOR_DIR_EXTEND);
//      delay(100);
////      digitalWrite(PIN_MOTOR_EXT_H, LOW);       // Set Half-bridges both low 
////      digitalWrite(PIN_MOTOR_EXT_L, LOW);
////      digitalWrite(PIN_MOTOR_EN, HIGH);         // Half-Bridges enabled
//      for (int i = 0; i < 600; i++) {
////        MOTOR.calSampleStart();
////        while (MOTOR.calSampleBusy()) {
//          delay(1000);
//          Print("Seconds="); Println(i+1);
////        }
////        Print("Motor calibration sample average = "); Println(MOTOR.calSampleGet());
//      }
//      MOTOR.motorOff();
////    }
//  }
//  // update last pushbutton state
//  lastPushbuttonState = HAL.inputRead(IN_PUSHBUTTON);
//  
//  // time to publish temperature?
//  if (xTaskGetTickCount() > lastTempPublish + TEMP_RATE) {
//    Serial.print("Free RAM = "); Serial.println(freeMemory());
//    lastTempPublish = xTaskGetTickCount();   // we just published
//  }
//
//
//  // We are using Scheduler library so we should yield or delay
//  delay(10);
}

/******************************************************************
 * State module thread function
 ******************************************************************/
static void thread_state(void *pvParameters) {
  Logln("State Task now running");
  
  // Initialize State Machine
  STATE.begin();

  // Loop forever
  while (true) {
    STATE.loop();
  }
}

/******************************************************************
 * HAL module thread function
 ******************************************************************/
static void thread_hal(void *pvParameters) {
  TickType_t previousWakeTick = xTaskGetTickCount();

  // Initialization of HAL is done in setup
  Logln("HAL Task now running");
  
  // Loop forever
  while (true) {
    // run HAL loop frequently
    HAL.loop();
    // we want to run every so often
    delayMsUntil(&previousWakeTick, 5);
  }
}

/******************************************************************
 * Network module thread function (lowest priority task)
 ******************************************************************/
static void thread_net(void *pvParameters) {
  Logln("Network Task now running");
  
  // Initialize network
  NET.begin();

  // Loop forever
  while (true) {
    NET.loop();
  }
}
