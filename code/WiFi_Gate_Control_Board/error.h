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
#ifndef ERROR_H
#define ERROR_H

/******************************************************************
 * Error enumeration
 ******************************************************************/
enum Error_Enum {
  ERROR_NONE = 0,                   // error free!
  ERROR_HAL_HALT = 4,               // I2C hardware problem
  ERROR_WIFI_HALT,                  // WINC1500 module not detected

  ERROR_WIFI_DISCON,                // not connected to WiFi server
  ERROR_MQTT_DISCON,                // not connected to MQTT server
  ERROR_NTP_ERROR,                  // no response from NTP server

  ERROR_12VOUT_FAULT,               // 12V Switched outputs over current
  ERROR_GATE_LIMIT,                 // Open and Close limit active at the same time
  ERROR_OPEN_PE_FAULT,              // 10k fault on Open PhotoEye input
  ERROR_CLOSE_PE_FAULT,             // 10k fault on Close PhotoEye input
  ERROR_EDGE_FAULT,                 // 10k fault on Edge input
  ERROR_SAFETY_FAULT,               // 10k fault on Safety input
  ERROR_SHADOW_FAULT,               // 10k fault on Shadow input
  ERROR_INPUT1_FAULT,               // 10k fault on Input 1 input
  ERROR_INPUT2_FAULT,               // 10k fault on Input 2 input
};

class ErrorClass {
public:
  ErrorClass(void);                 // default constructor
  void setError(uint8_t error, uint8_t displayErrorMessage = true);     // set a new error
  void setHaltError(uint8_t error); // set an error that will halt system
  uint8_t getError(void);           // get the current error

private:
  uint8_t m_error;                  // current error state
};

extern ErrorClass ERR;
#endif // ERROR_H
