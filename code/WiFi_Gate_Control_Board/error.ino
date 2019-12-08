/*
  Error module for WiFi Gate Control Board.

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
#include "hal.h"
#include "WDTZero.h"
#include "error.h"

/******************************************************************
 * Public Methods
 ******************************************************************/

/******************************************************************
 * Constructor
 ******************************************************************/
ErrorClass::ErrorClass() {
  m_error = ERROR_NONE;         // set error in object
  HAL.ledSetStatus(STATUS_LED_GOOD); // Status LED to green color
}

/******************************************************************
 * Set the current error
 *   Will automatically set the Status LED based on error value.
 *   User can disable serial message
 ******************************************************************/
void ErrorClass::setError(uint8_t error, uint8_t displayErrorMessage) {
  // output error info
  if (displayErrorMessage) {
    switch (error) {
      case ERROR_WIFI_DISCON:
        Logln("Error: WiFi disconnected");
        break;
      case ERROR_MQTT_DISCON:
        Logln("Error: MQTT disconnected");
        break;
      case ERROR_NTP_ERROR:
        Logln("Error: No response from NTP server");
        break;
      case ERROR_12VOUT_FAULT:
        Logln("Error: Switched 12V fault");
        break;
      case ERROR_GATE_LIMIT:
        Logln("Error: Open and close limit together");
        break;
      case ERROR_OPEN_PE_FAULT:
        Logln("Error: Open PhotoEye input monitoring error");
        break;
      case ERROR_CLOSE_PE_FAULT:
        Logln("Error: Close PhotoEye input monitoring error");
        break;
      case ERROR_EDGE_FAULT:
        Logln("Error: Edge input monitoring error");
        break;
      case ERROR_SAFETY_FAULT:
        Logln("Error: Safety input monitoring error");
        break;
      case ERROR_SHADOW_FAULT:
        Logln("Error: Shadow input monitoring error");
        break;
      case ERROR_INPUT1_FAULT:
        Logln("Error: Input 1 monitoring error");
        break;
      case ERROR_INPUT2_FAULT:
        Logln("Error: Input 2 monitoring error");
        break;
    }
  }
  // update member error
  m_error = error;
  if (error == ERROR_NONE) {
    HAL.ledSetStatus(STATUS_LED_GOOD); // Status LED to green color
  } else {
    HAL.ledSetStatus(STATUS_LED_ERROR); // Status LED to red color
  }
}

/******************************************************************
 * Set the current error and halt
 *   Will automatically flash the Status LED based on error value.
 *   This method will never return and is used for hard faults.
 ******************************************************************/
void ErrorClass::setHaltError(uint8_t error) {
  uint8_t i;

  WiFi.end();
  // hard error force a halt in processor
  delay(100);                 // wait for I2C in HAL to finish
  // disable LP5024 and LED power supply
  digitalWrite(PIN_LED_ENABLE, LOW);
  delay(1);
  // enable LP5024 and LED power supply
  digitalWrite(PIN_LED_ENABLE, HIGH);
  HAL.ledSetStatus(STATUS_LED_ERROR);    // turn Status LED red
  // loop forever
  while (true) {
    WDTZero.reset();
    // flash status LED for error number of times
    for (i = 0; i < error; i++) {
      // turn Staus red LED on
      HAL.ledSetStatus(STATUS_LED_ERROR);
      delay(250);
      HAL.ledSetStatus(STATUS_LED_OFF);
      delay(250);
    }
    delay(1250);
  }
}

/******************************************************************
 * Get the current error
 *   Will clear the m_errorChanged flag
 ******************************************************************/
uint8_t ErrorClass::getError(void) {
  return m_error;
}

/******************************************************************
 * Private Methods
 ******************************************************************/

/******************************************************************
 * Preinstantiate Objects
 ******************************************************************/
ErrorClass ERR = ErrorClass();
