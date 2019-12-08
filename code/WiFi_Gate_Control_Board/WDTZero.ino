/*
  Watch Dog Timer (WDT) module for Arduino Zero (SAMD processor).
  WDT clock is GCLK2 (

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
#include "WDTZero.h"
#include "WiFi_Gate_Control_Board.h"

/******************************************************************
 * Public Methods
 ******************************************************************/
/******************************************************************
 * Initialize WDTZero Class
 ******************************************************************/
bool WDTZeroClass::begin() {
  #ifdef ENABLE_WATCHDOG
    
  // The OSCULP32K (Internal low-power 32.768kHz oscillator) is always 
  // enabled just after reset. This is the Watchdog Timer clock source.
  // Set up the generic clock (GCLK2) to clock the watchdog timer at 256Hz
  REG_GCLK_GENDIV = GCLK_GENDIV_ID(2) |               // Select GCLK2
                    GCLK_GENDIV_DIV(4);               // 32.768kHz / 2^(6 + 1) = 256Hz
                                                      //   requires GCLK_GENCTRL_DIVSEL = 1
  while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization
  REG_GCLK_GENCTRL = GCLK_GENCTRL_ID(2) |             // Select GCLK2
                     GCLK_GENCTRL_GENEN |             // Enable GCLK2
                     GCLK_GENCTRL_DIVSEL |            // Set to divide by 2^(GENDIV.DIV + 1)
                     GCLK_GENCTRL_SRC_OSCULP32K;      // Clock source is Ultra Low Power Oscillator (OSCULP32K)
  while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization
  
  // Feed GCLK2 to Watchdog Timer (WDT)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_ID_WDT |            // select WDT clock mux
                     GCLK_CLKCTRL_CLKEN |             // Enable clock mux
                     GCLK_CLKCTRL_GEN(2);             // clock source is GCLK2
  while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization

  // Set the Watchdog timeout by selecting one of the following
//  REG_WDT_CONFIG = WDT_CONFIG_PER_16K;                // Set the WDT reset timeout to 16384 clock cycles or 16 seconds
//  REG_WDT_CONFIG = WDT_CONFIG_PER_8K;                 // Set the WDT reset timeout to 8192 clock cycles or 8 seconds
  REG_WDT_CONFIG = WDT_CONFIG_PER_4K;                 // Set the WDT reset timeout to 4096 clock cycles or 4 seconds
//  REG_WDT_CONFIG = WDT_CONFIG_PER_2K;                 // Set the WDT reset timeout to 2048 clock cycles or 2 seconds
//  REG_WDT_CONFIG = WDT_CONFIG_PER_1K;                 // Set the WDT reset timeout to 1024 clock cycles or 1 seconds
//  REG_WDT_CONFIG = WDT_CONFIG_PER_512;                // Set the WDT reset timeout to 512 clock cycles or 1/2 second
//  REG_WDT_CONFIG = WDT_CONFIG_PER_256;                // Set the WDT reset timeout to 512 clock cycles or 1/4 second
//  REG_WDT_CONFIG = WDT_CONFIG_PER_128;                // Set the WDT reset timeout to 512 clock cycles or 1/8 second
  while(WDT->STATUS.bit.SYNCBUSY);                    // Wait for synchronization
  
  REG_WDT_CTRL = WDT_CTRL_ENABLE;                     // Enable the WDT in normal mode
  while(WDT->STATUS.bit.SYNCBUSY);                    // Wait for synchronization
  return true;                                        // true means WDT was enabled
  #else
  return false;                                       // true means WDT was NOT enabled
  #endif
}

/******************************************************************
 * Reset Watchdog timer
 ******************************************************************/
void WDTZeroClass::reset(void) {
  #ifdef ENABLE_WATCHDOG
  if (!WDT->STATUS.bit.SYNCBUSY)                      // Check if the WDT registers are synchronized
    REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;              // Clear the watchdog timer
  #endif
}

/******************************************************************
 * Single instance of WDTZeroClass
 ******************************************************************/
WDTZeroClass WDTZero;
