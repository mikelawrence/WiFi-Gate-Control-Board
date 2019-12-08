/*
  Input/Output/LED Hardware Abstraction Layer for WiFi Gate Control Board.

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
#ifndef HAL_H
#define HAL_H

#include "WDTZero.h"
#include "WiFi_Gate_Control_Board.h"

#define INPUT_COUNT       13                  // Number of inputs for the gate
                                              // Matches number names in IN_Enum enumeration
#define LED_COUNT         23                  // Number of LEDs connected to LP5024

#define BOARD_IN_COUNT    12                  // Number of inputs on the board
#define DEBOUNCE_COUNT    BOARD_IN_COUNT*2+5  // Number of debounced inputs

/******************************************************************
 * Enumeration of input type must start at 0 and be squential
 ******************************************************************/
enum IN_Type_Enum {IN_TYPE_NONE = 0,          // Not used
                   IN_TYPE_OPEN_LIMIT_NO,     // Motor Open limit, Normally Closed
                   IN_TYPE_CLOSE_LIMIT_NO,    // Motor Closed limit, Normally Closed
                   IN_TYPE_PUSHBUTTON_NO,     // Pushbutton or Open/Close, Normally Open
                   IN_TYPE_PUSHBUTTON_NC,     // Pushbutton or Open/Close, Normally Closed
                   IN_TYPE_OPEN_NO,           // Open, Normally Open
                   IN_TYPE_OPEN_NC,           // Open, Normally Closed
                   IN_TYPE_CLOSE_NO,          // Close, Normally Open
                   IN_TYPE_CLOSE_NC,          // Close, Normally Closed
                   IN_TYPE_STOP_NO,           // Stop, Normally Open
                   IN_TYPE_STOP_NC,           // Stop, Normally Closed
                   IN_TYPE_OPEN_PHOTOEYE_NO,  // PhotoEye Open, Normally Open
                   IN_TYPE_OPEN_PHOTOEYE_NC,  // PhotoEye Open, Normally Closed
                   IN_TYPE_OPEN_PHOTOEYE_10K, // PhotoEye Open, Normally Open and 10k monitored
                   IN_TYPE_CLOSE_PHOTOEYE_NO, // PhotoEye Close, Normally Open
                   IN_TYPE_CLOSE_PHOTOEYE_NC, // PhotoEye Close, Normally Closed
                   IN_TYPE_CLOSE_PHOTOEYE_10K,// PhotoEye Close, Normally Open and 10k monitored
                   IN_TYPE_EDGE_NO,           // Edge, Normally Open
                   IN_TYPE_EDGE_NC,           // Edge, Normally Closed
                   IN_TYPE_EDGE_10K,          // Edge, Normally Open and 10k monitored
                   IN_TYPE_SAFETY_NO,         // Safety, Normally Open
                   IN_TYPE_SAFETY_NC,         // Safety, Normally Closed
                   IN_TYPE_SAFETY_10K,        // Safety, Normally Open and 10k monitored
                   IN_TYPE_SHADOW_NO,         // Shadow, Normally Open
                   IN_TYPE_SHADOW_NC,         // Shadow, Normally Closed
                   IN_TYPE_SHADOW_10K,        // Shadow, Normally Open and 10k monitored
                   IN_TYPE_INPUT1_NO,         // Input 1, Normally Open
                   IN_TYPE_INPUT1_NC,         // Input 1, Normally Closed
                   IN_TYPE_INPUT1_10K,        // Input 1, Normally Open and 10k monitored
                   IN_TYPE_INPUT2_NO,         // Input 2, Normally Open
                   IN_TYPE_INPUT2_NC,         // Input 2, Normally Closed
                   IN_TYPE_INPUT2_10K};       // Input 2, Normally Open and 10k monitored

/******************************************************************
 * Enumeration of input index must start at 0 and be squential
 ******************************************************************/
enum IN_Enum {IN_OPEN_LIMIT = 0,              // Motor Open Limit, Normally Open
              IN_CLOSE_LIMIT,                 // Motor Close Limit, Normally Open
              IN_PUSHBUTTON,                  // Pushbutton or Open/Close, Normally Open
              IN_OPEN,                        // Open, Normally Open
              IN_CLOSE,                       // Close, Normally Open
              IN_STOP,                        // Stop, Normally Open
              IN_OPEN_PHOTOEYE,               // PhotoEye Open, Normally Open
              IN_CLOSE_PHOTOEYE,              // PhotoEye Close, Normally Open
              IN_EDGE,                        // Edge, Normally Open
              IN_SAFETY,                      // Safety, Normally Open
              IN_SHADOW,                      // Shadow, Normally Open
              IN_INPUT1,                      // Input 1, Normally Open
              IN_INPUT2};                     // Input 2, Normally Open

// enumeration of input states, must start at 0 and be squential
enum IN_State_Enum {IN_STATE_INACTIVE = 0,
                    IN_STATE_ACTIVE,
                    IN_STATE_ERROR};

/******************************************************************
 * Enumeration of outputs
 ******************************************************************/
enum OUT_Enum {OUT_PHOTOEYE,                  // PhotoEye
               OUT_ALARM,                     // Alarm 12V
               OUT_OUT1,                      // Output 1
               OUT_OUT2};                     // Output 2           

/******************************************************************
 * Enumeration of outputs states
 ******************************************************************/
enum OUT_State_Enum {OUT_STATE_ON,            // for Power outputs (12V on)
                     OUT_STATE_OFF,           // for Power outputs (12V off)
                     OUT_STATE_OPEN,          // for Relay outputs (contact open)
                     OUT_STATE_CLOSED};       // for Relay outputs (contact closed)
                    

// enumeration of motor directions
enum MOTOR_Dir_Enum {MOTOR_DIR_OPEN,          // Open Motor direction
                     MOTOR_DIR_CLOSE};        // Close Motor direction

// enumeration of Status LED colors
enum STATUS_LED_Enum {STATUS_LED_OFF,         // Status LED is off
                      STATUS_LED_GOOD,        // Status LED is green
                      STATUS_LED_ERROR};      // Status LED is red

// enumeration of Motor LED colors
enum MOTOR_LED_Enum {MOTOR_LED_OFF,           // Motor LED is off
                     MOTOR_LED_OPENING,       // Motor LED is green
                     MOTOR_LED_CLOSING};      // Motor LED is red

class HalClass {
public:
  HalClass();
  bool begin(void);                           // initialize everything
  void loop(void);                            // must be called as schedule task
  // input methods
  uint8_t inputRead(uint8_t input);           // returns the current state of the specified input
  // output methods
  void outputSetPower(int output, int state); // turns output's +12V power on/off
  void outputSetRelay(int output, int state); // turns output's relay on/off
  // LED methods
  void ledSetStatus(int state);               // sets the Status LED to specified color
  void ledSetHalt(int state);                 // sets the Halt LED to ON or OFF
  void ledSetMotor(int state);                // sets the Motor LED to specified color
  // gate methods
  bool gateIsOpening(void);                   // returns true when gate is opening
  bool gateIsClosing(void);                   // returns true when gate is closing
  bool gateIsStopped(void);                   // returns true when gate is stopped
  void gateOpen(void);                        // starts the process of the opening the gate
  void gateClose(void);                       // starts the process of the closing the gate
  void gateStop(void);                        // stops the gate

private:
  uint8_t m_ledEnabled;                       // true when LEDs are enabled
  uint8_t m_ledPwmValue[LED_COUNT];           // PWM values for each individual LED
  TickType_t m_ledOffMillis;                  // ms in which the LEDs should be turned off
  
  uint8_t m_dbState[DEBOUNCE_COUNT];          // debounce current state, true=pressed
  uint8_t m_dbLastState[DEBOUNCE_COUNT];      // debounce previous state
  uint8_t m_dbChanged[DEBOUNCE_COUNT];        // debounce state changed since last read
  TickType_t m_dbLastChange[DEBOUNCE_COUNT];  // debounce time of last state change (ms)

  uint8_t m_inState[INPUT_COUNT];             // state of each input type
  uint8_t m_inLastState[INPUT_COUNT];         // last state of each input type

  uint8_t m_outState;                         // state of each output
  uint8_t m_outStateChanged;                  // when true output state was changed

  uint8_t m_gateOpening;                      // when true gate is opening
  uint8_t m_gateClosing;                      // when true gate is closing
};

extern HalClass HAL;
#endif // HAL_H
