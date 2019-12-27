/*
  State Machine module for WiFi Gate Control Board. 

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
#include "state.h"
#include "WiFi_Gate_Control_Board.h"

/******************************************************************
 * Local variables
 ******************************************************************/

/******************************************************************
 * Public Methods
 ******************************************************************/

/******************************************************************
 * Initialize State Machine class
 ******************************************************************/
void StateClass::begin() {
  uint32_t temp;

  // we want this task to be notified when the input has changed
  HAL.inputSetChangedNotify(xTaskGetCurrentTaskHandle());
 }

/******************************************************************
 * Handle State Machine
 *   Designed for use with FreeRTOS.
 ******************************************************************/
void StateClass::loop(void) {
  static uint8_t lastPBState = HAL.inputGet(IN_PUSHBUTTON);
  uint8_t newPBState;
  static uint8_t evenOdd = false;
  uint32_t result;

  // wait for input changed notificaton
  result = ulTaskNotifyTake(pdTRUE, 10);
  if (result == 1) {
    // input has changed
    newPBState = HAL.inputGet(IN_PUSHBUTTON);
    if ((lastPBState == IN_STATE_INACTIVE) && (newPBState == IN_STATE_ACTIVE)) {
      if (evenOdd) {
        HAL.gateOpen();
      } else {
        HAL.gateClose();
      }
      evenOdd = !evenOdd;
    }
    lastPBState = newPBState;
  }

}

/******************************************************************
   Preinstantiate Objects
 ******************************************************************/
StateClass STATE = StateClass();
