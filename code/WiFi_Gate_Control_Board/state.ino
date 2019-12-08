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
volatile uint32_t calSample;
volatile uint32_t calSampleCnt;
volatile uint8_t calSampleInProgress;

/******************************************************************
 * Public Methods
 ******************************************************************/

// /******************************************************************
//  * Prepares the motor module to open the motor
//  ******************************************************************/
// void State::motorOpen(void) {
//   taskENTER_CRITICAL();
// //  m_MotorOpen = true;
// //  m_MotorClose = false;
//   taskEXIT_CRITICAL();
// }

// /******************************************************************
//  * Prepares the motor module to close the motor
//  ******************************************************************/
// void State::motorClose(void) {
//   taskENTER_CRITICAL();
// //  m_MotorOpen = false;
// //  m_MotorClose = true;
//   taskEXIT_CRITICAL();
// }

// /******************************************************************
//  * Prepares the motor module to stop the motor
//  ******************************************************************/
// void State::motorStop(void) {
//   taskENTER_CRITICAL();
// //  m_MotorOpen = true;
// //  m_MotorClose = false;
//   taskEXIT_CRITICAL();
// }

// /******************************************************************
//  * Kicks off an interrupt based calibration sample average.
//  * Interrupt will average 100 ADC sample results.
//  * ADC is already set to a 1024 average so the average will take 
//  * 1.024 seconds and will comprise 102,400 samples.
//  ******************************************************************/
// void State::calSampleStart(void) {
//   calSample = 0;                              // accumulations starts at zero
//   calSampleCnt = 0;                           // no samples yet
//   calSampleInProgress = true;                 // calibration sample is now in progress
//   REG_ADC_INTENSET = ADC_INTENSET_RESRDY;     // enable ADC result ready interrupt
//   while (ADC->STATUS.bit.SYNCBUSY);           // wait for synchronization
//   REG_ADC_CTRLA = ADC_CTRLA_ENABLE;           // enable ADC
//   while (ADC->STATUS.bit.SYNCBUSY);           // wait for synchronization
// }

/******************************************************************
 * Initialize State Machine class
 ******************************************************************/
void StateClass::begin() {
  uint32_t temp;

 }

/******************************************************************
 * Handle State Machine
 *   Designed for use with FreeRTOS.
 ******************************************************************/
void StateClass::loop(void) {
  delayMs(100);
//  if (dir == MOTOR_DIR_OPEN) {
//    // open the gate
//    #if GATE_DIRECTION == GATE_PUSH_TO_OPEN
//    // Motor should extend to open
//    digitalWrite(PIN_MOTOR_EXT_H, HIGH);
//    digitalWrite(PIN_MOTOR_EXT_L, LOW);
//    #else
//    // Motor should retract to open
//    digitalWrite(PIN_MOTOR_EXT_H, LOW);
//    digitalWrite(PIN_MOTOR_EXT_L, HIGH);
//    #endif
//  } else {
//    // close the gate
//    #if GATE_DIRECTION == GATE_PUSH_TO_OPEN
//    // Motor should retract to close
//    digitalWrite(PIN_MOTOR_EXT_H, LOW);
//    digitalWrite(PIN_MOTOR_EXT_L, HIGH);
//    #else
//    // Motor should extend to close
//    digitalWrite(PIN_MOTOR_EXT_H, HIGH);
//    digitalWrite(PIN_MOTOR_EXT_L, LOW);
//    #endif
//  }
//  digitalWrite(PIN_MOTOR_EN, HIGH);           // Motor is ON
}

/******************************************************************
   Preinstantiate Objects
 ******************************************************************/
StateClass STATE = StateClass();
