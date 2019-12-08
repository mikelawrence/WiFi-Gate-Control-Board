/*
  WiFi Gate Control Board Project wide defines

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

#ifndef WIFI_GATE_CONTROL_BOARD_H
#define WIFI_GATE_CONTROL_BOARD_H

#include <WiFi101.h>
#include <FreeRTOS_SAMD21.h>
#include "WiFiRTC.h"
#include "WDTZero.h"
#include "error.h"
#include "hal.h"
#include "arduino_secrets.h"

/******************************************************************
 * Build defines
 ******************************************************************/
// Enable Watchdog Timer
#define ENABLE_WATCHDOG
// Enable OTA updates
#define ENABLE_OTA_UPDATES
// Enable Serial on USB
#define ENABLE_SERIAL
// Enable Low Power Mode on WiFi
#define ENABLE_WIFI_LOW_POWER
// enable calibration mode
#define ENABLE_CALIBRATION_MODE
// Current Version
#define VERSION                   "1.0"

/******************************************************************
 * Enumerations
 ******************************************************************/
// The two types of Gate directions
enum Gate_Dir_Enum {GATE_PULL_TO_OPEN = 0, GATE_PUSH_TO_OPEN};

/******************************************************************
 * Application defines
 ******************************************************************/
// Name of this board, each board should be unique
// Used as MQTT Client ID, HASS Name, and OTA Name
#define BOARD_NAME                "New Front Gate"
// NTP server used to update local time, can be IP Address or name
#define NTP_SERVER                "pool.ntp.org"
// timezone difference from GMT in hours (Standard Time difference)
#define TZDIFF                    -6
// input debounce time in milliseconds
#define DEBOUNCE_TIME             100
// temperature measurement time in milliseconds, Must be greater than 5 seconds
#define TEMP_RATE                 (1*60*1000)
// Length of time in milliseconds LEDS stay on after pushbutton input
#define LED_ON_TIME               (5*60*1000) 

/******************************************************************
 * Gate Defines
 *   Here is where you change the basic gate settings
 ******************************************************************/
// this defines which way the linear actuator has to move to open
//   GATE_PULL_TO_OPEN or GATE_PUSH_TO_OPEN
#define GATE_DIRECTION            GATE_PULL_TO_OPEN

/******************************************************************
 * Gate Input Type Defines
 *   Here is where you change the board inputs to specific types
 *   Look in hal.h for IN_Type_Enum for valid input types
 ******************************************************************/
#define BOARD_IN1                 IN_TYPE_PUSHBUTTON_NO
#define BOARD_IN2                 IN_TYPE_NONE
#define BOARD_IN3                 IN_TYPE_NONE
#define BOARD_IN4                 IN_TYPE_NONE
#define BOARD_IN5                 IN_TYPE_PUSHBUTTON_NO
#define BOARD_IN6                 IN_TYPE_OPEN_NO
#define BOARD_IN7                 IN_TYPE_NONE
#define BOARD_IN8                 IN_TYPE_NONE
#define BOARD_IN9                 IN_TYPE_NONE
#define BOARD_IN10                IN_TYPE_NONE
#define BOARD_IN11                IN_TYPE_OPEN_PHOTOEYE_10K
#define BOARD_IN12                IN_TYPE_NONE

/******************************************************************
 * Gate Output Type Defines
 *   Here is where you change the board outputs to specific types
 ******************************************************************/

/******************************************************************
 * Home Assistant MQTT Defines
 ******************************************************************/
#define HASS_PREFIX               "homeassistant"
#define HASS_GATE_NAME            "new_front_gate"    // should be BOARD_NAME with no whitespace

// HASS defines below here should not be modified
#define HASS_AVAIL_TOPIC          HASS_PREFIX "/cover/" HASS_GATE_NAME "/avail"
#define HASS_PAYLOAD_AVAIL        "online"
#define HASS_PAYLOAD_NOT_AVAIL    "offline"

#define HASS_GATE_CONFIG_TOPIC    HASS_PREFIX "/cover/" HASS_GATE_NAME "/gate/config"
#define HASS_GATE_STATE_TOPIC     HASS_PREFIX "/cover/" HASS_GATE_NAME "/gate/state"
#define HASS_GATE_COMMAND_TOPIC   HASS_PREFIX "/cover/" HASS_GATE_NAME "/gate/set"
#define HASS_GATE_CONFIG          "{ \"name\": \"" BOARD_NAME "\", \"cmd_t\": \"" HASS_GATE_COMMAND_TOPIC \
                                  "\", \"stat_t\": \"" HASS_GATE_STATE_TOPIC "\", \"qos\": 1, \"retain\": false" \
                                  ", \"avty_t\": \"" HASS_AVAIL_TOPIC "\" }"
                                  
#define HASS_TEMP_CONFIG_TOPIC    HASS_PREFIX "/sensor/" HASS_GATE_NAME "/temperature/config"
#define HASS_TEMP_STATE_TOPIC     HASS_PREFIX "/sensor/" HASS_GATE_NAME "/temperature/state"
#define HASS_TEMP_CONFIG          "{ \"name\": \"" BOARD_NAME " Temperature\", \"stat_t\": \"" HASS_TEMP_STATE_TOPIC \
                                  "\", \"unit_of_meas\": \"°C\", \"avty_t\": \"" HASS_AVAIL_TOPIC "\" }"
                                  
#define HASS_RSSI_CONFIG_TOPIC    HASS_PREFIX "/sensor/" HASS_GATE_NAME "/rssi/config"
#define HASS_RSSI_STATE_TOPIC     HASS_PREFIX "/sensor/" HASS_GATE_NAME "/rssi/state"
#define HASS_RSSI_CONFIG          "{ \"name\": \"" BOARD_NAME " RSSI\", \"stat_t\": \"" HASS_RSSI_STATE_TOPIC \
                                  "\", \"unit_of_meas\": \"dBm\", \"avty_t\": \"" HASS_AVAIL_TOPIC "\" }"
                                  
#define HASS_STATUS_CONFIG_TOPIC  HASS_PREFIX "/sensor/" HASS_GATE_NAME "/status/config"
#define HASS_STATUS_STATE_TOPIC   HASS_PREFIX "/sensor/" HASS_GATE_NAME "/status/state"
#define HASS_STATUS_CONFIG        "{ \"name\": \"" BOARD_NAME " Status\", \"stat_t\": \"" HASS_STATUS_STATE_TOPIC \
                                  "\", \"avty_t\": \"" HASS_AVAIL_TOPIC "\" }"

/******************************************************************
 * Board Defines
 ******************************************************************/
/******************************************************************
 * Pin definitions
 ******************************************************************/
#define PIN_PB_SWITCH             PIN_A1  // Pushbutton (open/stop/close) switch input
#define PIN_LED_SWITCH            0       // LED enable switch input
#define PIN_LED_ENABLE            PIN_A3  // LED enable output
#define PIN_EXTEND_LIMIT          4       // Motor Extend Limit
#define PIN_RETRACT_LIMIT         5       // Motor Retract Limit
#define PIN_INPUT_CHANGED         PIN_A0  // I2C expander input changed input
#define PIN_MOTOR_EN              3       // Motor Enable
#define PIN_MOTOR_EXT_H           1       // Motor Extend High
#define PIN_MOTOR_EXT_L           PIN_A5  // Motor Extend Low
#define PIN_MIS                   PIN_A6  // Motor current sense PA07 (A6 on schematic)

// STATUS LEDs
#define STATUS_GREEN_LED          6
#define STATUS_RED_LED            7

// Logging/Printing defines
#ifdef ENABLE_SERIAL
#define Print(...)                Serial.print(__VA_ARGS__)
#define Println(...)              Serial.println(__VA_ARGS__)
#define Log(...)                  {WiFiRTC.printTimeHMS24Hr();Serial.print(' ');Serial.print(__VA_ARGS__);}
#define Logln(...)                {WiFiRTC.printTimeHMS24Hr();Serial.print(' ');Serial.println(__VA_ARGS__);}
#else
#define Print(...)
#define Println(...)
#define Log(...)
#define Logln(...)
#endif

/******************************************************************
 * RTOS Delay functions
 ******************************************************************/
inline void delayUs(int us) {
  vTaskDelay(us / portTICK_PERIOD_US );  
}

inline void delayMs(int ms) {
  vTaskDelay((ms * 1000) / portTICK_PERIOD_US);  
}

inline void delayMsUntil(TickType_t *previousWakeTime, int ms) {
  vTaskDelayUntil(previousWakeTime, (ms * 1000) / portTICK_PERIOD_US);  
}

#endif // WIFI_GATE_CONTROL_BOARD_H
