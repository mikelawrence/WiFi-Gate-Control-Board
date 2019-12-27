/* 
  Input/Output/LED/Motor Hardware Abstraction Layer for WiFi Gate Control Board.

  There are three I2C devices on this board.
    TCA6424A 24-bit, I2C IO Expander used for the 12 independent inputs
      Address (010 0010)
    TCA6408A 8-bit, I2C IO Expander used to control the 12V switched output supplies
      and read the Error Status from the Infineon High-Side switch
      Address (010 0000)
    LP5024 24-channel, I2C LED driver used to drive the majority of the status LEDs
      Address (010 1000)

  The TCA6424A and TCA6408A IO Expanders have the INT pin connected to the
    SAMD microprocessor so input changes can cause an interrupt.

  The motor section of the this board consists of two Infineon IFX007T Half-Bridges.
  The bridges are wired to support three signals for control, enable (MEN) and
  direction signals (MEXTH and MEXTL). See schematic for truth table.

  The IS outputs are connected in parallel since the half-bridge only outputs
  current sense when the high side switch is on. The ADC input used for the current
  sense is set to automatically sample and average so that new averaged samples occur
  about 1000 times per second. Motor current sense is hard coded to A6/PA07.

  This module supports interrupts on Extend and Retract limit inputs so the motor
  can be turned off instantly.

  The ADC clock is changed from the Arduino defaults. 
  Do not use any Arduino ADC access functions

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
#include <I2C_DMAC.h>
#include "hal.h"

// Motor calibration defines (result from ADC will be 0 = 0A and 1500 = 15.0A)
// copied from results of calibration using electronic load
#define CAL_EXTEND_OFFSET       584
#define CAL_EXTEND_GAIN         2503
#define CAL_RETRACT_OFFSET      583
#define CAL_RETRACT_GAIN        2406

// I2C addresses
#define TMP100_ADDR       0x48    // I2C Address for Temperature Sensor
#define TCA6424_ADDR      0x22    // I2C Address for TCA6424 IO Expander
#define TCA6408_ADDR      0x20    // I2C Address for TCA6408 IO Expander
#define LP5024_ADDR       0x28    // I2C Address for LP5024 LED Driver

// special debounce indexes
#define DEBOUNCE_PB_INDEX           (DEBOUNCE_COUNT-5)
#define DEBOUNCE_LED_INDEX          (DEBOUNCE_COUNT-4)
#define DEBOUNCE_OPEN_LIMIT_INDEX   (DEBOUNCE_COUNT-3)
#define DEBOUNCE_CLOSE_LIMIT_INDEX  (DEBOUNCE_COUNT-2)
#define DEBOUNCE_12VOUT_FAULT_INDEX (DEBOUNCE_COUNT-1)

// LED PWM values
#define LED_PWM_ON        0xFF    // Intensity for LED on
#define LED_PWM_OFF       0x00    // Intensity for LED off

// Macros to help input and switch debounce
#define SET_DBSTATE(value, pos, var) (value ? var |= (1 << pos) : var &= ~(1 << pos))
#define GET_DBSTATE(pos, var) ((var & (1 << pos)) != 0)
#define WAS_PRESSED(pos)  (m_dbState[pos] && m_dbChanged[pos])
#define WAS_RELEASED(pos) (!m_dbState[pos] && m_dbChanged[pos])

// enumeration defining the LED position in arrays
enum LED_Pos_Enum {LED_INPUT2_GREEN = 0,      // Input 2 active red
                   LED_INPUT2_RED,            // Input 2 10k error red
                   LED_INPUT1_GREEN,          // Input 1 active green
                   LED_INPUT1_RED,            // Input 1 10k error red
                   LED_SHADOW_GREEN,          // Shadow active green
                   LED_SHADOW_RED,            // Shadow 10k error red
                   LED_SAFETY_GREEN,          // Safety active green
                   LED_SAFETY_RED,            // Safety 10k error red
                   LED_EDGE_GREEN,            // Edge active green
                   LED_EDGE_RED,              // Edge 10k error red
                   LED_CLOSE_PHOTOEYE_GREEN,  // Close PhotoEye active green
                   LED_CLOSE_PHOTOEYE_RED,    // Close PhotoEye 10k error red
                   LED_OPEN_PHOTOEYE_GREEN,   // Open PhotoEye active green
                   LED_OPEN_PHOTOEYE_RED,     // Open PhotoEye 10k error red
                   LED_STOP,                  // Gate Stop active green
                   LED_CLOSE,                 // Gate close active green
                   LED_OPEN,                  // Gate open active green
                   LED_PUSHBUTTON,            // Gate pushbutton or open/close active green
                   LED_CLOSE_LIMIT,           // Gate close limit active green
                   LED_OPEN_LIMIT,            // Gate open limit active green
                   LED_MOTOR_GREEN,           // Motor opening green
                   LED_MOTOR_RED,             // Motor closing red
                   LED_HALT
                  };                 // Halt active red

#define OUT_POS_12VOUT_FAULT    0x04      // +12V outputs over current position
#define OUT_POS_12V_PHOTOEYE_EN 0x01      // PhotoEye +12V out enable position
#define OUT_POS_12V_ALARM_EN    0x02      // Alarm +12V out enable position
#define OUT_POS_12V_OUT1_EN     0x08      // Output 1 +12V out enable position
#define OUT_POS_12V_OUT2_EN     0x10      // Output 2 +12V out enable position
#define OUT_POS_RELAY_OUT1_EN   0x20      // Output 1 relay enable position
#define OUT_POS_RELAY_OUT2_EN   0x40      // Output 2 relay enable position

// maps Motor Current Input Arduino pin to SAM pin info
#define MIS_SAM_PORT            g_APinDescription[PIN_MOTOR_MIS].ulPort
#define MIS_SAM_PIN             g_APinDescription[PIN_MOTOR_MIS].ulPin
#define MIS_SAM_PINMASK         (1ul << MIS_SAM_PIN)

// maps physical input to input type (change in WiFi_Gate_Control_Board.h)
const uint8_t m_inMapping[BOARD_IN_COUNT] = {BOARD_IN1, BOARD_IN2, BOARD_IN3, BOARD_IN4, 
                                             BOARD_IN5, BOARD_IN6, BOARD_IN7, BOARD_IN8, 
                                             BOARD_IN9, BOARD_IN10, BOARD_IN11, BOARD_IN12};

/******************************************************************
 * Local variables not part of HAL class
 ******************************************************************/
volatile uint32_t adcSample;              // ADC sample accumulator 
volatile uint32_t adcSampleCnt;           // ADC sample count
// these local variables are set by pin change interrupts
uint8_t dbMelimRead;                      // last read state of motor extend limit
uint8_t dbMrlimRead;                      // last read state of motor retract limit

/******************************************************************
   I2C_DMAC error variable, incremented each time i2cErrorCallback is called
 ******************************************************************/
uint8_t i2cError;

/******************************************************************
   I2C_DMAC error callback function
 ******************************************************************/
void i2cErrorCallback(void) {
  ++i2cError;
}

/******************************************************************
   Public Methods
 ******************************************************************/

/******************************************************************
   Constructor
 ******************************************************************/
HalClass::HalClass() {

}

/******************************************************************
   Initialize the I2C board class
 ******************************************************************/
bool HalClass::begin() {
  TickType_t currentTick;
  uint8_t i;
  uint8_t tempData[3];
  uint32_t curInput, temp;
 
  // configure Status LED outputs
  pinMode(STATUS_GREEN_LED, OUTPUT);
  digitalWrite(STATUS_GREEN_LED, LOW);  // Green Status LED pin is an output and currently on for good status
  pinMode(STATUS_RED_LED, OUTPUT);
  digitalWrite(STATUS_RED_LED, HIGH);   // Red Status LED pin is an output and currently off

  Print("  Initializing Motor Interface...");
   // setup motor control pins
  digitalWrite(PIN_MOTOR_EN, LOW);                    // Motor is OFF
  pinMode(PIN_MOTOR_EN, OUTPUT);                      // Motor Enable is an output
  digitalWrite(PIN_MOTOR_EXT_H, LOW);                 // Half-bridge direction is low-side
  pinMode(PIN_MOTOR_EXT_H, OUTPUT);                   // Motor Extend High is an output
  digitalWrite(PIN_MOTOR_EXT_L, LOW);                 // Half-bridge direction is low-side
  pinMode(PIN_MOTOR_EXT_L, OUTPUT);                   // Motor Extend Low is an output

  // setup extend/retract limit inputs
  pinMode(PIN_EXTEND_LIMIT, INPUT);                   // Motor Extend Limit is an input
  attachInterrupt(PIN_EXTEND_LIMIT, onMELIMChange, CHANGE); // Interrupt handler 
  pinMode(PIN_RETRACT_LIMIT, INPUT);                  // Motor Retract Limit is an input
  attachInterrupt(PIN_RETRACT_LIMIT, onMRLIMChange, CHANGE);

  // Initialize Motor Current Sense (MIS) pin
  // Make MIS pin an input
  PORT->Group[MIS_SAM_PORT].DIRCLR.reg = MIS_SAM_PINMASK;
  // Enable port multiplexing on MIS pin
  PORT->Group[MIS_SAM_PORT].PINCFG[MIS_SAM_PIN].bit.PMUXEN = 1;
  // Select Peripheral B (Analog) on MIS pin
  temp = PORT->Group[MIS_SAM_PORT].PMUX[MIS_SAM_PIN / 2].reg;
  if (MIS_SAM_PIN / 2 & 1) {
    // even pin group
    temp &= ~PORT_PMUX_PMUXE_Msk;
    temp |= PORT_PMUX_PMUXE_B;
  } else {
    // odd pin group
    temp &= ~PORT_PMUX_PMUXO_Msk;
    temp |= PORT_PMUX_PMUXO_B;
  }
  PORT->Group[MIS_SAM_PORT].PMUX[MIS_SAM_PIN / 2].reg = temp;

  // Arduino startup configures ADC clock source to GCLK0 (48MHz)
  // Instead we are going configure GCLK3 as the source for the ADC
  // Setup clock GCLK3 for 8MHz
  REG_GCLK_GENDIV = GCLK_GENDIV_ID(3) |               // select GCLK3
                    GCLK_GENDIV_DIV(6);               // 48MHz / 6 = 8MHz
  while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization
  REG_GCLK_GENCTRL = GCLK_GENCTRL_ID(3) |             // select GCLK3
                     GCLK_GENCTRL_GENEN |             // enable GCLK3
                     GCLK_GENCTRL_SRC_DFLL48M;        // clock source is DFLL48M
  while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization

  // enable and feed GCLK3 to ADC
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_ID_ADC |            // select ADC clock mux
                     GCLK_CLKCTRL_CLKEN |             // enable clock mux
                     GCLK_CLKCTRL_GEN(3);             // clock source is GCLK3
  while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization

  // configure the ADC to external reference (3.3V),
  // 12-bits, (8 clocks per sample), 250kHz sample rate,
  // 256 sample averaging with result still at 12 bits
  // effective sample rate is 976.6 Hz (1.024ms period)
  REG_ADC_CTRLA = 0;                                  // disable the ADC
  REG_ADC_REFCTRL = ADC_REFCTRL_REFSEL_AREFA;         // select VREFA (PA03 on schematic)
  REG_ADC_CTRLB = ADC_CTRLB_PRESCALER_DIV4 |          // clock divide 4 (8MHz / 4 = 2MHz)
                  ADC_CTRLB_RESSEL_16BIT |            // 16-bit resolution (required for averaging)
                  ADC_CTRLB_CORREN |                  // enable gain correction
                  ADC_CTRLB_FREERUN;                  // ADC is in free running mode
  while (ADC->STATUS.bit.SYNCBUSY);                   // wait for synchronization
  // digital correction disabled
  // single-ended mode
  // right adjusted result
  REG_ADC_SAMPCTRL = ADC_SAMPCTRL_SAMPLEN(2);         // 2 extra half clocks of sampling
  REG_ADC_AVGCTRL = ADC_AVGCTRL_SAMPLENUM_256 |       // 256 sample average (auto right shift 4)
                    ADC_AVGCTRL_ADJRES(4);            // right shift average 4 (result is 12-bits)
  REG_ADC_INPUTCTRL = ADC_INPUTCTRL_GAIN_1X |         // No input gain
                      ADC_INPUTCTRL_MUXNEG_IOGND |    // ADC negative input IOGND
                      ADC_INPUTCTRL_MUXPOS(MIS_SAM_PIN);  // ADC positive input MIS pin
  while (ADC->STATUS.bit.SYNCBUSY);                   // wait for synchronization

  // configure the ADC for windowing mode
  REG_ADC_WINCTRL = ADC_WINCTRL_WINMODE_MODE1;        // enable window mode (ADC result > WINLT)
  while (ADC->STATUS.bit.SYNCBUSY);                   // wait for synchronization
  REG_ADC_WINLT = 3196;                               // set lower threshold
  while (ADC->STATUS.bit.SYNCBUSY);                   // wait for synchronization

  // configure ADC interrupts
  NVIC_EnableIRQ(ADC_IRQn);                           // enable ADC interrupts
  NVIC_SetPriority(ADC_IRQn, 0);                      // set priority of the interrupt
  Println("Success");

  // init I2C interface all devices support 400kHz
  I2C.begin(400000, REG_ADDR_8BIT, PIO_SERCOM_ALT);
  I2C.attachDmacErrorCallback(i2cErrorCallback);
  I2C.attachSercomErrorCallback(i2cErrorCallback);
  i2cError = 0;           // no current I2C errors

  // I2C Expanders can alert the micro when input changes.
  // TCA6424A and TCA6408A INT pins are connected together
  pinMode(PIN_INPUT_CHANGED, INPUT);
  // init LED Enable output
  pinMode(PIN_LED_ENABLE, OUTPUT);
  digitalWrite(PIN_LED_ENABLE, HIGH);
  // wait a bit after enabling LP5024 chip
  vNopDelayMS(2);

  // init TCA6424A 24-bit I2C IO Expander
  Print("  Initializing TCA6424A 24-bit I2C IO Expander...");
  I2C.writeByte(TCA6424_ADDR, 0x08, 0x00);  // Port 0 All inputs are non-inverting
  I2C.writeByte(TCA6424_ADDR, 0x09, 0x00);  // Port 1 All inputs are non-inverting
  I2C.writeByte(TCA6424_ADDR, 0x0A, 0x00);  // Port 2 All inputs are non-inverting
  I2C.writeByte(TCA6424_ADDR, 0x0C, 0xFF);  // Port 0 All pins are inputs
  I2C.writeByte(TCA6424_ADDR, 0x0D, 0xFF);  // Port 1 All pins are inputs
  I2C.writeByte(TCA6424_ADDR, 0x0E, 0xFF);  // Port 2 All pins are inputs
  // check for I2C errors
  if (i2cError) {
    Println("Failed");
    return (false);
  } else {
    Println("Success");
  }

  // init TCA6408A 8-bit I2C IO Expander
  Print("  Initializing TCA6408A 8-bit I2C IO Expander...");
  I2C.writeByte(TCA6408_ADDR, 0x01, 0x00);  // All outputs are default off (low)
  I2C.writeByte(TCA6408_ADDR, 0x02, 0x00);  // All inputs are non-inverting
  I2C.writeByte(TCA6408_ADDR, 0x03, 0x04);  // P2 is an input, all others outputs
  m_outState = 0;                           // no outputs are on
  m_outStateChanged = false;;               // outputs have not changed
  // check for I2C errors
  if (i2cError) {
    Println("Failed");
    return (false);
  } else {
    Println("Success");
  }

  // set LED PWM values to default values
  for (i = 0; i < LED_COUNT; i++) {
    m_ledPwmValue[i] = LED_PWM_OFF;
  }
  // init LP5024 24-bit I2C LED driver
  Print("  Initializing LP5024 24-bit I2C LED Driver...");
  I2C.writeByte(LP5024_ADDR, 0x00, 0x40);   // Enable LP5024
  // write default LED PWM values to LP5024
  I2C.writeBytes(LP5024_ADDR, 0x10, m_ledPwmValue, LED_COUNT);
  while(I2C.writeBusy);                     // Wait for write to finish
  // check for I2C errors
  if (i2cError) {
    Println("Failed");
    return (false);
  } else {
    Println("Success");
  }

  // init TMP100 I2C Temperature Sensor
  Print("  Initializing TMP100 I2C Temperature Sensor...");
  I2C.writeByte(TMP100_ADDR, 0x01, 0x00);   // enable device
  I2C.writeByte(TMP100_ADDR, 0x01, 0x60);   // continuous conversion, 12-bit temperature
  // check for I2C errors
  if (i2cError) {
    Println("Failed");
  } else {
    Println("Success");
  }

  // Set LEDs to automatically turn off in a few minutes
  m_ledEnabled = true;
  m_ledOffMillis = xTaskGetTickCount() + LED_ON_TIME;

  // set current state of inputs
  I2C.readBytes(TCA6424_ADDR, 0x80, tempData, 3);
  while(I2C.readBusy);                      // Wait for read to finish
  curInput = (tempData[2] << 16) | (tempData[1] << 8) | tempData[0];
  // set the Open/Close switch current state
  SET_DBSTATE(!digitalRead(PIN_PB_SWITCH), DEBOUNCE_PB_INDEX, curInput);
  // set the LED Enable switch current state
  SET_DBSTATE(!digitalRead(PIN_LED_SWITCH), DEBOUNCE_LED_INDEX, curInput);
  // read the current motor retract limit state
  dbMrlimRead = digitalRead(PIN_RETRACT_LIMIT);
  // read the current motor extend Limit state
  dbMelimRead = digitalRead(PIN_EXTEND_LIMIT);
  // setup switch and input debounce
  currentTick = xTaskGetTickCount();
  for (i = 0; i < DEBOUNCE_COUNT; i++) {
    m_dbState[i] = GET_DBSTATE(i, curInput);
    m_dbLastState[i] = m_dbState[i];
    m_dbChanged[i] = false;
    m_dbLastChange[i] = currentTick;
  }

  // init gate input states
  for (i = 0; i < INPUT_COUNT; i++) {
    m_inState[i] = IN_STATE_INACTIVE;
    m_inLastState[i] = IN_STATE_INACTIVE;
  }

  // success
  return (true);
}

/******************************************************************
 * Periodically handle devices.
 *   Designed for use with FreeRTOS.
 ******************************************************************/
void HalClass::loop() {
  static TickType_t lastUpdate;
  TickType_t currentTick;
  uint32_t curInput;
  uint8_t cur10kError;
  uint8_t tempData[INPUT_COUNT];
  uint8_t i;
  uint8_t newError;
  uint8_t inputChanged;

  // always reset Watchdog Timer at the beginning of this loop
  WDTZero.reset();

  // time to update outputs?
  if (m_outStateChanged) {
    I2C.writeByte(TCA6408_ADDR, 0x01, m_outState);
    m_outStateChanged = false;      // change has been sent
  }

  // debounce switches
  currentTick = xTaskGetTickCount();
  // read current state from inputs
  I2C.readBytes(TCA6424_ADDR, 0x80, tempData, 3);
  while(I2C.readBusy);                // Wait for read to finish
  curInput = (tempData[2] << 16) | (tempData[1] << 8) | tempData[0];
  // set the Open/Close switch current state
  SET_DBSTATE(!digitalRead(PIN_PB_SWITCH), DEBOUNCE_PB_INDEX, curInput);
  // set the LED Enable switch current state
  SET_DBSTATE(!digitalRead(PIN_LED_SWITCH), DEBOUNCE_LED_INDEX, curInput);
  // set the Open and Close Limit current state
  if (GATE_DIRECTION == GATE_PULL_TO_OPEN) {
    // retract to open
    SET_DBSTATE(!dbMrlimRead, DEBOUNCE_OPEN_LIMIT_INDEX, curInput);
    SET_DBSTATE(!dbMelimRead, DEBOUNCE_CLOSE_LIMIT_INDEX, curInput);
  } else {
    // extend to close
    SET_DBSTATE(!dbMelimRead, DEBOUNCE_OPEN_LIMIT_INDEX, curInput);
    SET_DBSTATE(!dbMrlimRead, DEBOUNCE_CLOSE_LIMIT_INDEX, curInput);
  }
  // read over current in ITS4075Q High-Side Power Switch
  I2C.readByte(TCA6408_ADDR, 0x00);   // Read the input port
  while(I2C.readBusy);                // Wait for read to finish
  // set the overcurrent current state
  SET_DBSTATE(!(I2C.getData() & OUT_POS_12VOUT_FAULT), DEBOUNCE_12VOUT_FAULT_INDEX, curInput);
  // handle debouncing
  for (i = 0; i < DEBOUNCE_COUNT; i++) {
    if (currentTick - m_dbLastChange[i] < DEBOUNCE_TIME) {
      m_dbChanged[i] = false;         // not changed
      // debounce DEBOUNCE_12VOUT_FAULT_INDEX special
      // if sampled high anytime in the debounce time then input is still active
      if ((i == DEBOUNCE_12VOUT_FAULT_INDEX) && (GET_DBSTATE(i, curInput))) {
        m_dbLastChange[i] = currentTick;
      }
    } else {
      // debounce time elapsed, make last state equal current state
      m_dbLastState[i] = m_dbState[i];
      // make current state equal to just read state
      m_dbState[i] = GET_DBSTATE(i, curInput);
      // determine if changed
      m_dbChanged[i] = (m_dbState[i] != m_dbLastState[i]);
      // update change time
      if (m_dbChanged[i]) {
        m_dbLastChange[i] = currentTick;
      }
    }
  }

  // process inputs and LEDs
  for (i = 0; i < INPUT_COUNT; i++) {
    // default value for new input state is inactive
    tempData[i] = IN_STATE_INACTIVE;
  }

  // set LED PWM values to OFF (not including Motor and Halt)
  for (i = 0; i < LED_COUNT - 2; i++) {
    if ((i != LED_MOTOR_GREEN) && 
        (i != LED_MOTOR_RED) &&
        (i != LED_HALT)) {
      m_ledPwmValue[i] = LED_PWM_OFF;
    }
  }

  // update open limit input and LEDs
  if (m_dbState[DEBOUNCE_OPEN_LIMIT_INDEX]) {
    tempData[IN_OPEN_LIMIT] = IN_STATE_ACTIVE;
    m_ledPwmValue[LED_OPEN_LIMIT] = LED_PWM_ON;
  }
  // update close limit input and LEDs
  if (m_dbState[DEBOUNCE_CLOSE_LIMIT_INDEX]) {
    tempData[IN_CLOSE_LIMIT] = IN_STATE_ACTIVE;
    m_ledPwmValue[LED_CLOSE_LIMIT] = LED_PWM_ON;
  }
  // update pushbutton open/close input and LEDs
  if (m_dbState[DEBOUNCE_PB_INDEX]) {
    tempData[IN_PUSHBUTTON] = IN_STATE_ACTIVE;
    m_ledPwmValue[LED_PUSHBUTTON] = LED_PWM_ON;
  }

  // start with no error
  newError = ERROR_NONE;
  // process board inputs into correct gate inputs
  // also apply update LED PWM values
  for (i = 0; i < BOARD_IN_COUNT; i++) {
    // get current states
    curInput = m_dbState[i * 2 + 1];
    cur10kError = m_dbState[i * 2];
    // action is based on input type
    switch (m_inMapping[i]) {
      case IN_TYPE_OPEN_LIMIT_NO:     // Motor Open limit, Normally Open
        if (!curInput) {
          tempData[IN_OPEN_LIMIT] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_OPEN_LIMIT] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_CLOSE_LIMIT_NO:    // Motor Close limit, Normally Open
        if (!curInput) {
          tempData[IN_CLOSE_LIMIT] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_CLOSE_LIMIT] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_PUSHBUTTON_NO:     // Pushbutton or Open/Close, Normally Open
        if (!curInput) {
          tempData[IN_PUSHBUTTON] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_PUSHBUTTON] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_PUSHBUTTON_NC:     // Pushbutton or Open/Close, Normally Closed
        if (curInput) {
          tempData[IN_PUSHBUTTON] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_PUSHBUTTON] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_OPEN_NO:           // Open, Normally Open
        if (!curInput) {
          tempData[IN_OPEN] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_OPEN] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_OPEN_NC:           // Open, Normally Closed
        if (curInput) {
          tempData[IN_OPEN] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_OPEN] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_CLOSE_NO:          // Close, Normally Open
        if (!curInput) {
          tempData[IN_CLOSE] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_CLOSE] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_CLOSE_NC:          // Close, Normally Closed
        if (curInput) {
          tempData[IN_CLOSE] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_CLOSE] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_STOP_NO:           // Stop, Normally Open
        if (!curInput) {
          tempData[IN_STOP] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_STOP] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_STOP_NC:           // Stop, Normally Closed
        if (curInput) {
          tempData[IN_STOP] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_STOP] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_OPEN_PHOTOEYE_NO:  // PhotoEye Open, Normally Open
        if (!curInput) {
          tempData[IN_OPEN_PHOTOEYE] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_OPEN_PHOTOEYE_GREEN] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_OPEN_PHOTOEYE_NC:  // PhotoEye Open, Normally Closed
        if (curInput) {
          tempData[IN_OPEN_PHOTOEYE] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_OPEN_PHOTOEYE_GREEN] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_OPEN_PHOTOEYE_10K: // PhotoEye Open, Normally Open and 10k monitored
        if (!curInput) {
          tempData[IN_OPEN_PHOTOEYE] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_OPEN_PHOTOEYE_GREEN] = LED_PWM_ON;
        } else if (cur10kError && (tempData[IN_OPEN_PHOTOEYE] != IN_STATE_ACTIVE)) {
          newError = ERROR_OPEN_PE_FAULT;
          tempData[IN_OPEN_PHOTOEYE] = IN_STATE_ERROR;
          m_ledPwmValue[LED_OPEN_PHOTOEYE_RED] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_CLOSE_PHOTOEYE_NO: // PhotoEye Close, Normally Open
        if (!curInput) {
          tempData[IN_CLOSE_PHOTOEYE] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_CLOSE_PHOTOEYE_GREEN] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_CLOSE_PHOTOEYE_NC: // PhotoEye Close, Normally Closed
        if (curInput) {
          tempData[IN_CLOSE_PHOTOEYE] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_CLOSE_PHOTOEYE_GREEN] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_CLOSE_PHOTOEYE_10K:// PhotoEye Close, Normally Open and 10k monitored
        if (!curInput) {
          tempData[IN_CLOSE_PHOTOEYE] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_CLOSE_PHOTOEYE_GREEN] = LED_PWM_ON;
        } else if (cur10kError && (tempData[IN_CLOSE_PHOTOEYE] != IN_STATE_ACTIVE)) {
          newError = ERROR_CLOSE_PE_FAULT;
          tempData[IN_CLOSE_PHOTOEYE] = IN_STATE_ERROR;
          m_ledPwmValue[LED_CLOSE_PHOTOEYE_RED] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_EDGE_NO:           // Edge, Normally Open
        if (!curInput) {
          tempData[IN_EDGE] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_EDGE_GREEN] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_EDGE_NC:           // Edge, Normally Closed
        if (curInput) {
          tempData[IN_EDGE] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_EDGE_GREEN] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_EDGE_10K:          // Edge, Normally Open and 10k monitored
        if (!curInput) {
          tempData[IN_EDGE] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_EDGE_GREEN] = LED_PWM_ON;
        } else if (cur10kError && (tempData[IN_EDGE] != IN_STATE_ACTIVE)) {
          newError = ERROR_EDGE_FAULT;
          tempData[IN_EDGE] = IN_STATE_ERROR;
          m_ledPwmValue[LED_EDGE_RED] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_SAFETY_NO:         // Safety, Normally Open
        if (!curInput) {
          tempData[IN_SAFETY] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_SAFETY_GREEN] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_SAFETY_NC:         // Safety, Normally Closed
        if (curInput) {
          tempData[IN_SAFETY] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_SAFETY_GREEN] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_SAFETY_10K:        // Safety, Normally Open and 10k monitored
        if (!curInput) {
          tempData[IN_SAFETY] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_SAFETY_GREEN] = LED_PWM_ON;
        } else if (cur10kError && (tempData[IN_SAFETY] != IN_STATE_ACTIVE)) {
          newError = ERROR_SAFETY_FAULT;
          tempData[IN_SAFETY] = IN_STATE_ERROR;
          m_ledPwmValue[LED_SAFETY_RED] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_SHADOW_NO:         // Shadow, Normally Open
        if (!curInput) {
          tempData[IN_SHADOW] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_SHADOW_GREEN] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_SHADOW_NC:         // Shadow, Normally Closed
        if (curInput) {
          tempData[IN_SHADOW] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_SHADOW_GREEN] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_SHADOW_10K:        // Shadow, Normally Open and 10k monitored
        if (!curInput) {
          tempData[IN_SHADOW] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_SHADOW_GREEN] = LED_PWM_ON;
        } else if (cur10kError && (tempData[IN_SHADOW] != IN_STATE_ACTIVE)) {
          newError = ERROR_SHADOW_FAULT;
          tempData[IN_SHADOW] = IN_STATE_ERROR;
          m_ledPwmValue[LED_SHADOW_RED] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_INPUT1_NO:         // Input 1, Normally Open
        if (!curInput) {
          tempData[IN_INPUT1] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_INPUT1_GREEN] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_INPUT1_NC:         // Input 1, Normally Closed
        if (curInput) {
          tempData[IN_INPUT1] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_INPUT1_GREEN] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_INPUT1_10K:        // Input 1, Normally Open and 10k monitored
        if (!curInput) {
          tempData[IN_INPUT1] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_INPUT1_GREEN] = LED_PWM_ON;
        } else if (cur10kError && (tempData[IN_INPUT1] != IN_STATE_ACTIVE)) {
          newError = ERROR_INPUT1_FAULT;
          tempData[IN_INPUT1] = IN_STATE_ERROR;
          m_ledPwmValue[LED_INPUT1_RED] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_INPUT2_NO:         // Input 2, Normally Open
        if (!curInput) {
          tempData[IN_INPUT2] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_INPUT2_GREEN] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_INPUT2_NC:         // Input 2, Normally Closed
        if (curInput) {
          tempData[IN_INPUT2] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_INPUT2_GREEN] = LED_PWM_ON;
        }
        break;
      case IN_TYPE_INPUT2_10K:        // Input 2, Normally Open and 10k monitored
        if (!curInput) {
          tempData[IN_INPUT2] = IN_STATE_ACTIVE;
          m_ledPwmValue[LED_INPUT2_GREEN] = LED_PWM_ON;
        } else if (cur10kError && (tempData[IN_INPUT2] != IN_STATE_ACTIVE)) {
          newError = ERROR_INPUT2_FAULT;
          tempData[IN_INPUT2] = IN_STATE_ERROR;
          m_ledPwmValue[LED_INPUT2_RED] = LED_PWM_ON;
        }
        break;
      default:                        // Not used
        break;
    }
  }

  taskENTER_CRITICAL();
  // update gate input current and last states, also handle input changed
  inputChanged = false;
  for (i = 0; i < INPUT_COUNT; i++) {
    m_inLastState[i] = m_inState[i];
    // current state is updated all at once here due to scheduler task switching
    m_inState[i] = tempData[i];
    if (m_inLastState[i] != m_inState[i]) {
      // input changed
      inputChanged = true;
    }
  }
  taskEXIT_CRITICAL();

  // notify tasks waiting for input changed
  if (inputChanged) {
    if (m_inChangedNotifyTask != NULL) {
      // we have a task to notify of input changed
      xTaskNotifyGive(m_inChangedNotifyTask);
    }
  }

  // handle 10k monitoring errors
  if (newError == ERROR_NONE) {
    // new error free, was previous error a 10k fault
    if ((ERR.getError() >= ERROR_OPEN_PE_FAULT) &&
        (ERR.getError() <= ERROR_INPUT2_FAULT)) {
      // 10k error cleared so update Error
      ERR.setError(ERROR_NONE);
    }
  } if (newError > ERR.getError()) {
    // out error is a higher priority error
    ERR.setError(newError);
  }
  
  // time to change LEDs enabled state?
  if WAS_PRESSED(DEBOUNCE_LED_INDEX) {
    if (m_ledEnabled) {
      // LEDs are currently ON, time turn OFF
      Logln("LEDs turned off");
      // disable LP5024 and LED power supply
      digitalWrite(PIN_LED_ENABLE, LOW);
      // flag LEDs OFF
      m_ledEnabled = false;
    } else {
      // LEDs are currently OFF, time to turn ON
      Logln("LEDs turned on");
      // enable LP5024 and LED power supply
      digitalWrite(PIN_LED_ENABLE, HIGH);
      // wait a bit after enabling LP5024 chip
      delayMs(2);
      // Enable LP5024 via I2C
      I2C.writeByte(LP5024_ADDR, 0x00, 0x40);
      // Set LEDs to automatically turn off in a few minutes
      m_ledOffMillis = xTaskGetTickCount() + LED_ON_TIME;
      // flag LEDs ON
      m_ledEnabled = true;
    }
  }

  // update or turn LEDs off
  if (m_ledEnabled) {
    if (xTaskGetTickCount() > m_ledOffMillis) {
      Logln("LEDs automatically disabled");
      // disable LP5024 and LED power supply
      digitalWrite(PIN_LED_ENABLE, LOW);
      // flag LEDs OFF
      m_ledEnabled = false;
    } else {
      // time to update LEDs, write current LED PWM values to LP5024
      I2C.writeBytes(LP5024_ADDR, 0x10, m_ledPwmValue, LED_COUNT);
      delayMs(1);                       // this will take about 650us to complete
      while(I2C.writeBusy);             // wait for write to finish
    }
  }

  // handle fault ITS4075Q fault
  if (m_dbState[DEBOUNCE_12VOUT_FAULT_INDEX]) {
    // active 12V fault error
    if (ERROR_12VOUT_FAULT > ERR.getError()) {
      // over current error is a higher priority
      ERR.setError(ERROR_12VOUT_FAULT);
    }
  } else {
    // no error, was previous error over current
    if (ERR.getError() == ERROR_12VOUT_FAULT) {
      ERR.setError(ERROR_NONE);
    }
  }

  // handle Open Limit and Close Limit active simultaneously
  if (m_inState[IN_OPEN_LIMIT] && m_inState[IN_CLOSE_LIMIT]) {
    // error
    if (ERROR_GATE_LIMIT > ERR.getError()) {
      // over current error is a higher priority
      ERR.setError(ERROR_GATE_LIMIT);
    }
  } else {
    // no error, was previous error open and close limit
    if (ERR.getError() == ERROR_GATE_LIMIT) {
      ERR.setError(ERROR_NONE);
    }
  }

  // if (xTaskGetTickCount() - lastUpdate >= 5000) {
  //   Print("melimCount=");
  //   Print(melimCount);
  //   Print(", mrlimCount=");
  //   Println(mrlimCount);
  //   lastUpdate += 5000;
  // }
}

/******************************************************************
 * Periodically handle the motor.
 *   Designed for use with FreeRTOS.
 ******************************************************************/
void HalClass::motorLoop(void) {
  TickType_t currentTick;
  uint32_t retracting;

  // switch to calibration mode?
  #ifdef ENABLE_CALIBRATION_MODE
  motorCal();
  return;
  #endif

  // motor is doing nothing, check for time to do something
  if (!m_gateOpening && !m_gateClosing) {
    // nothing to do
    return;
  }
  // check for time to open and gate is already open all the way
  if (m_gateOpening && m_inState[IN_OPEN_LIMIT] == IN_STATE_ACTIVE) {
    // cannot open an already open gate
    Logln("Cannot open an already open gate.");
    m_gateOpening = false;
    m_gateClosing = false;
    return;
  }
  // check for time to close and gate is already closed
  if (m_gateClosing && m_inState[IN_CLOSE_LIMIT] == IN_STATE_ACTIVE) {
    // cannot close an already closed gate
    Logln("Cannot close an already closed gate.");
    m_gateOpening = false;
    m_gateClosing = false;
    return;
  }

  if (m_gateOpening) {
    // we are opening the gate
    Logln("Starting Gate Open");
    ledSetMotor(MOTOR_LED_OPENING);             // set motor LED color to opening
    if (GATE_DIRECTION == GATE_PULL_TO_OPEN) {
      retracting = true;                        // gate retracts to open
    } else {
      retracting = false;                       // gate extends to open
    }
  } else if (m_gateClosing) {
    // we are closing the gate
    Logln("Starting Gate Close");
    ledSetMotor(MOTOR_LED_CLOSING);             // set motor LED color to closing
    if (GATE_DIRECTION == GATE_PULL_TO_OPEN) {
      retracting = false;                       // gate extends to close
    } else {
      retracting = true;                        // gate retracts to close
    }
  }

  // // time to sample motor sense current offset and configure ADC calibration
  // // enable half-bridges and set both to lowside active
  // // this prevents the motor from moving but still activates motor current sense
  // // which is necessary to measure it's offset
  // digitalWrite(PIN_MOTOR_EXT_H, HIGH);          // set to extend
  // digitalWrite(PIN_MOTOR_EXT_L, LOW);
  // digitalWrite(PIN_MOTOR_EN, HIGH);             // enable motor for offset measurement
  // adcSample = 0;                                // accumulations starts at zero
  // adcSampleCnt = 0;                             // 0 sample cout mean interrupt will ignore samples
  // REG_ADC_GAINCORR = 2048;                      // no gain correction
  // REG_ADC_OFFSETCORR = 0;                       // no offset correction
  // REG_ADC_CTRLA = ADC_CTRLA_ENABLE;             // enable ADC
  // while (ADC->STATUS.bit.SYNCBUSY);             // wait for synchronization
  // REG_ADC_INTENSET = ADC_INTENSET_RESRDY;       // enable ADC result ready interrupt
  // while (ADC->STATUS.bit.SYNCBUSY);             // wait for synchronization
  // delayMs(10);                                  // let the offset current stabilize
  // adcSampleCnt = 1000;                            // prep # of samples in accumulation
  // while (adcSampleCnt != 0)
  //   delayMs(1);                                 // wait unitl all samples are accumulated
  // adcSampleExtend = adcSample / 1000;             // complete ADC sample average
  // Log("Offset Correction Extend: ");
  // Println(adcSampleExtend);
  // digitalWrite(PIN_MOTOR_EXT_H, LOW);
  // delayMs(10);                                  // let the offset current stabilize
  // digitalWrite(PIN_MOTOR_EXT_L, HIGH);          // set both to retract
  // digitalWrite(PIN_MOTOR_EN, HIGH);             // enable motor for offset measurement
  // delayMs(10);                                  // let the offset current stabilize
  // adcSample = 0;                                // accumulations starts at zero
  // adcSampleCnt = 1000;                            // prep # of samples in accumulation
  // while (adcSampleCnt != 0)
  //   delayMs(1);                                 // wait unitl all samples are accumulated
  // adcSampleRetract = adcSample / 1000;            // complete ADC sample average
  // Log("Offset Correction Retract: ");
  // Println(adcSampleRetract);
  // // REG_ADC_OFFSETCORR = adcSample;               // correct for current offset
  
  // time to move the gate
  REG_ADC_CTRLA = ADC_CTRLA_ENABLE;             // enable ADC
  while (ADC->STATUS.bit.SYNCBUSY);             // wait for synchronization
  REG_ADC_INTENSET = ADC_INTENSET_RESRDY;       // enable ADC result ready interrupt
  while (ADC->STATUS.bit.SYNCBUSY);             // wait for synchronization
  delayMs(3);                                   // ignore the first few ADC samples
  if (retracting) {
    // we are retracting the gate
    REG_ADC_OFFSETCORR = CAL_RETRACT_OFFSET;    // use extend offset correction
    REG_ADC_GAINCORR = CAL_RETRACT_GAIN;        // use extend gain correction
    digitalWrite(PIN_MOTOR_EXT_L, HIGH);        // set motor to retract
    digitalWrite(PIN_MOTOR_EXT_H, LOW);
  } else {
    // we are extending the gate
    REG_ADC_OFFSETCORR = CAL_EXTEND_OFFSET;     // use extend offset correction
    REG_ADC_GAINCORR = CAL_EXTEND_GAIN;         // use extend gain correction
    digitalWrite(PIN_MOTOR_EXT_L, LOW);         // set motor to extend
    digitalWrite(PIN_MOTOR_EXT_H, HIGH);
  }

  REG_ADC_INTENCLR = ADC_INTENCLR_RESRDY;       // disable ADC result ready interrupt
  while (ADC->STATUS.bit.SYNCBUSY);             // wait for synchronization
  REG_ADC_CTRLA = 0;                            // disable ADC
  while (ADC->STATUS.bit.SYNCBUSY);             // wait for synchronization

  ledSetMotor(MOTOR_LED_OFF);                   // set motor LED off
  m_gateOpening = false;
  m_gateClosing = false;
  digitalWrite(PIN_MOTOR_EN, LOW);
}

/******************************************************************
   Read current input
 ******************************************************************/
uint8_t HalClass::inputGet(uint8_t input) {
  return m_inState[input];
}

/******************************************************************
 * Set Output's 12V Power on/off
 ******************************************************************/
void HalClass::outputSetPower(int output, int state) {
  switch (output) {
    case OUT_PHOTOEYE:
      if (state) {
        m_outState |= OUT_POS_12V_PHOTOEYE_EN;
      } else {
        m_outState &= OUT_POS_12V_PHOTOEYE_EN;
      }
      break;
    case OUT_ALARM:
      if (state) {
        m_outState |= OUT_POS_12V_ALARM_EN;
      } else {
        m_outState &= OUT_POS_12V_ALARM_EN;
      }
      break;
    case OUT_OUT1:
      if (state) {
        m_outState |= OUT_POS_12V_OUT1_EN;
      } else {
        m_outState &= OUT_POS_12V_OUT1_EN;
      }
      break;
    case OUT_OUT2:
      if (state) {
        m_outState |= OUT_POS_12V_OUT2_EN;
      } else {
        m_outState &= OUT_POS_12V_OUT2_EN;
      }
      break;
  }
  m_outStateChanged = true;   // outputs need to be sent to TCA6408
}

/******************************************************************
 * Set Relay's switch open/closed
 ******************************************************************/
void HalClass::outputSetRelay(int output, int state) {
  switch (output) {
    case OUT_OUT1:
      if (state) {
        m_outState |= OUT_POS_RELAY_OUT1_EN;
      } else {
        m_outState &= OUT_POS_RELAY_OUT1_EN;
      }
      break;
    case OUT_OUT2:
      if (state) {
        m_outState |= OUT_POS_RELAY_OUT2_EN;
      } else {
        m_outState &= OUT_POS_RELAY_OUT2_EN;
      }
      break;
  }
  m_outStateChanged = true;   // outputs need to be sent to TCA6408
}

/******************************************************************
 * Set Status LED specified color
 ******************************************************************/
void HalClass::ledSetStatus(int state) {
  switch (state) {
    case STATUS_LED_GOOD:
      digitalWrite(STATUS_GREEN_LED, HIGH); // Green Status LED on
      digitalWrite(STATUS_RED_LED, LOW);    // Red Status LED off
      break;
    case STATUS_LED_ERROR:
      digitalWrite(STATUS_GREEN_LED, LOW);  // Green Status LED off
      digitalWrite(STATUS_RED_LED, HIGH);   // Red Status LED on
      break;
    default:
      digitalWrite(STATUS_GREEN_LED, LOW);  // Green Status LED off
      digitalWrite(STATUS_RED_LED, LOW);    // Red Status LED off
      break;
  }
}

/******************************************************************
 * Set Halt LED to specifed state (on or off)
 ******************************************************************/
void HalClass::ledSetHalt(int state) {
  if (state)
    m_ledPwmValue[LED_HALT] = LED_PWM_ON;
  else
    m_ledPwmValue[LED_HALT] = LED_PWM_OFF;
}

/******************************************************************
 * Set Motor LED to specified color
 ******************************************************************/
void HalClass::ledSetMotor(int state) {
  switch (state) {
    case MOTOR_LED_OPENING:
      m_ledPwmValue[LED_MOTOR_GREEN] = LED_PWM_ON; // Green LED on
      m_ledPwmValue[LED_MOTOR_RED] = LED_PWM_OFF; // Red LED off
      break;
    case MOTOR_LED_CLOSING:
      m_ledPwmValue[LED_MOTOR_GREEN] = LED_PWM_OFF; // Green LED off
      m_ledPwmValue[LED_MOTOR_RED] = LED_PWM_ON; // Red LED on
      break;
    default:
      m_ledPwmValue[LED_MOTOR_GREEN] = LED_PWM_OFF; // Green LED off
      m_ledPwmValue[LED_MOTOR_RED] = LED_PWM_OFF; // Red LED off
      break;
  }
}

/******************************************************************
 * Returns true when gate is opening
 ******************************************************************/
bool HalClass::gateIsOpening(void) {
  return m_gateOpening;
}

/******************************************************************
 * Returns true when gate is closing
 ******************************************************************/
bool HalClass::gateIsClosing(void) {
  return m_gateClosing;
}

/******************************************************************
 * Returns true when gate is stopped
 ******************************************************************/
bool HalClass::gateIsStopped(void) {
  bool result;
  taskENTER_CRITICAL();
  result = !m_gateOpening && !m_gateClosing;
  taskEXIT_CRITICAL();
  return result;
}

/******************************************************************
 * starts the process of the opening the gate
 ******************************************************************/
void HalClass::gateOpen(void) {
  taskENTER_CRITICAL();
  m_gateOpening = true;
  m_gateClosing = false;
  taskEXIT_CRITICAL();
}

/******************************************************************
 * starts the process of the closing the gate
 ******************************************************************/
void HalClass::gateClose(void) {
  taskENTER_CRITICAL();
  m_gateOpening = false;
  m_gateClosing = true;
  taskEXIT_CRITICAL();
}

/******************************************************************
 * Stops the gate
 ******************************************************************/
void HalClass::gateStop(void) {
  taskENTER_CRITICAL();
  m_gateOpening = false;
  m_gateClosing = false;
  ledSetMotor(MOTOR_LED_OFF);                   // Set motor LED color
  digitalWrite(PIN_MOTOR_EN, LOW);              // Motor is now off
  digitalWrite(PIN_MOTOR_EXT_H, LOW);
  digitalWrite(PIN_MOTOR_EXT_L, LOW);
  taskEXIT_CRITICAL();
}

/******************************************************************
 * Configure the task to notify when the input has changed.
 * If set to NULL no task will be notified
 ******************************************************************/
void HalClass::inputSetChangedNotify(TaskHandle_t task) {
  m_inChangedNotifyTask = task;
}

/******************************************************************
 * Private Methods
 ******************************************************************/
/******************************************************************
 * When built with ENABLE_CALIBRATION_MODE defined this function 
 *   is called instead of motorLoop(). Serial interface will 
 *   automatically be enabled because it is needed for prompting 
 *   the user throughout the calibration process.
 ******************************************************************/
void HalClass::motorCal(void) {
  uint8_t lastPBState, newPBState;
  uint32_t adcExtendOffset, adcRetractOffset, 
           adcExtendSample, adcRetractSample, 
           adcExtendGain, adcRetractGain,
           result;
  
  ERR.setError(ERROR_NONE);
  HAL.inputSetChangedNotify(xTaskGetCurrentTaskHandle());

  // Sample motor extend sense current offset
  Println("");
  Logln("Entering Calibration");
  Logln("Measure Motor offset current");
  Logln("  We are going to measure the no-load offset current.");
  Logln("  Connect electronic load to motor output and set Constant Current mode at 0 amps.");
  Logln("  Note this test will apply motor control in both directions. Make sure the");
  Logln("  electronic load is connected through a full-wave bridge rectifier.")
  Logln("Press OPEN/CLOSE button to continue...");

  // wait for OPEN/CLOSE button to be pressed
  lastPBState = HAL.inputGet(IN_PUSHBUTTON);
  while (true) {
    result = ulTaskNotifyTake(pdTRUE, 10);
    if (result == 1) {
      // input has changed
      newPBState = HAL.inputGet(IN_PUSHBUTTON);
      if ((lastPBState == IN_STATE_INACTIVE) && (newPBState == IN_STATE_ACTIVE)) {
        // pushbutton pressed
        break;
      }
    }
  }

  Log("Measuring extend offset current...");
  digitalWrite(PIN_MOTOR_EXT_H, HIGH);          // set motor to extend
  digitalWrite(PIN_MOTOR_EXT_L, LOW);
  digitalWrite(PIN_MOTOR_EN, HIGH);             // enable motor for offset measurement
  ledSetMotor(MOTOR_LED_CLOSING);
  adcSampleCnt = 0;                             // 0 sample cout mean interrupt will ignore samples
  REG_ADC_GAINCORR = 2048;                      // no gain correction
  REG_ADC_OFFSETCORR = 0;                       // no offset correction
  REG_ADC_CTRLA = ADC_CTRLA_ENABLE;             // enable ADC
  while (ADC->STATUS.bit.SYNCBUSY);             // wait for synchronization
  REG_ADC_INTENSET = ADC_INTENSET_RESRDY;       // enable ADC result ready interrupt
  while (ADC->STATUS.bit.SYNCBUSY);             // wait for synchronization
  delayMs(500);                                 // let the offset current stabilize
  adcSample = 0;                                // accumulations starts at zero
  adcSampleCnt = 1000;                          // prep # of samples in accumulation
  while (adcSampleCnt != 0)
    delayMs(1);                                 // wait until all samples are accumulated
  adcExtendOffset = adcSample / 1000;           // complete ADC sample average
  Println(adcExtendOffset);
  Log("Measuring retract offset current...");
  digitalWrite(PIN_MOTOR_EN, LOW);              // disable motor
  digitalWrite(PIN_MOTOR_EXT_H, LOW);           // set motor to retract
  digitalWrite(PIN_MOTOR_EXT_L, HIGH);
  digitalWrite(PIN_MOTOR_EN, HIGH);             // enable motor for offset measurement
  ledSetMotor(MOTOR_LED_OPENING);
  delayMs(500);                                 // let the offset current stabilize
  adcSample = 0;                                // accumulations starts at zero
  adcSampleCnt = 1000;                          // prep # of samples in accumulation
  while (adcSampleCnt != 0)
    delayMs(1);                                 // wait until all samples are accumulated
  adcRetractOffset = adcSample / 1000;          // complete ADC sample average
  Println(adcRetractOffset);

  digitalWrite(PIN_MOTOR_EN, LOW);              // disable motor
  ledSetMotor(MOTOR_LED_OFF);                   // set motor LED off
  delayMs(100);
  
  Println("");
  Logln("Measure Motor 15 amp load current.")
  Logln("  We are going to measure the full-load current.");
  Logln("  Set Electronic Load to Constant Current mode at 15 amps.");
  Logln("  Again this test will apply motor control in both directions.")
  Logln("Press OPEN/CLOSE button to continue...");

  // wait for OPEN/CLOSE button to be pressed
  lastPBState = HAL.inputGet(IN_PUSHBUTTON);
  while (true) {
    result = ulTaskNotifyTake(pdTRUE, 10);
    if (result == 1) {
      // input has changed
      newPBState = HAL.inputGet(IN_PUSHBUTTON);
      if ((lastPBState == IN_STATE_INACTIVE) && (newPBState == IN_STATE_ACTIVE)) {
        // pushbutton pressed
        break;
      }
    }
  }

  Log("Measuring extend 15 amp current...");
  digitalWrite(PIN_MOTOR_EXT_H, HIGH);          // set motor to extend
  digitalWrite(PIN_MOTOR_EXT_L, LOW);
  digitalWrite(PIN_MOTOR_EN, HIGH);             // enable motor for offset measurement
  ledSetMotor(MOTOR_LED_CLOSING);
  delayMs(500);                                 // let the offset current stabilize
  adcSample = 0;                                // accumulations starts at zero
  adcSampleCnt = 1000;                          // prep # of samples in accumulation
  while (adcSampleCnt != 0)
    delayMs(1);                                 // wait until all samples are accumulated
  adcExtendSample = adcSample / 1000;           // complete ADC sample average
  Println(adcExtendSample);
  Log("Measuring retract 15 amp current...");
  digitalWrite(PIN_MOTOR_EN, LOW);              // disable motor
  digitalWrite(PIN_MOTOR_EXT_H, LOW);           // set motor to retract
  digitalWrite(PIN_MOTOR_EXT_L, HIGH);
  digitalWrite(PIN_MOTOR_EN, HIGH);             // enable motor for offset measurement
  ledSetMotor(MOTOR_LED_OPENING);
  delayMs(500);                                 // let the offset current stabilize
  adcSample = 0;                                // accumulations starts at zero
  adcSampleCnt = 1000;                          // prep # of samples in accumulation
  while (adcSampleCnt != 0)
    delayMs(1);                                 // wait until all samples are accumulated
  adcRetractSample = adcSample / 1000;          // complete ADC sample average
  Println(adcRetractSample);
 
  // turn motor off
  digitalWrite(PIN_MOTOR_EN, LOW);              // disable motor
  ledSetMotor(MOTOR_LED_OFF);                   // set motor LED off
  delayMs(100);

  // compute calibration values
  adcExtendGain = 2048 * 1500 / (adcExtendSample - adcExtendOffset);
  adcRetractGain = 2048 * 1500 / (adcRetractSample - adcRetractOffset);
  // add device specific temperature and aging factor
  adcExtendGain = (uint32_t) (1.006 * (double) adcExtendGain);
  adcRetractGain = (uint32_t) (1.006 * (double) adcRetractGain);
  Logln("Add the following 4 lines to the top of hal.ino file.")
  Print("#define CAL_EXTEND_OFFSET       "); Println(adcExtendOffset);
  Print("#define CAL_EXTEND_GAIN         "); Println(adcExtendGain);
  Print("#define CAL_RETRACT_OFFSET      "); Println(adcRetractOffset);
  Print("#define CAL_RETRACT_GAIN        "); Println(adcRetractGain);

  // Println("");
  // Logln("Measure Motor 30 amp load current.")
  // Logln("  We are going to measure the 30 amp current.");
  // Logln("  Set Electronic Load to Constant Current mode at 30 amps.");
  // Logln("  Again this test will apply motor control in both directions.")
  // Logln("Press OPEN/CLOSE button to continue...");

  // // wait for OPEN/CLOSE button to be pressed
  // lastPBState = HAL.inputGet(IN_PUSHBUTTON);
  // while (true) {
  //   result = ulTaskNotifyTake(pdTRUE, 10);
  //   if (result == 1) {
  //     // input has changed
  //     newPBState = HAL.inputGet(IN_PUSHBUTTON);
  //     if ((lastPBState == IN_STATE_INACTIVE) && (newPBState == IN_STATE_ACTIVE)) {
  //       // pushbutton pressed
  //       break;
  //     }
  //   }
  // }

  // Log("Measuring extend 15 amp current...");
  // REG_ADC_OFFSETCORR = adcExtendOffset;         // extend offset correction
  // REG_ADC_GAINCORR = adcExtendGain;             // extend gain correction
  // digitalWrite(PIN_MOTOR_EXT_H, HIGH);          // set motor to extend
  // digitalWrite(PIN_MOTOR_EXT_L, LOW);
  // digitalWrite(PIN_MOTOR_EN, HIGH);             // enable motor for offset measurement
  // ledSetMotor(MOTOR_LED_CLOSING);
  // delayMs(500);                                  // let the offset current stabilize
  // adcSample = 0;                                // accumulations starts at zero
  // adcSampleCnt = 1000;                          // prep # of samples in accumulation
  // while (adcSampleCnt != 0)
  //   delayMs(1);                                 // wait until all samples are accumulated
  // Println(adcSample / 1000);
  // Log("Measuring retract 15 amp current...");
  // REG_ADC_OFFSETCORR = adcRetractOffset;        // retract offset correction
  // REG_ADC_GAINCORR = adcRetractGain;            // retract gain correction
  // digitalWrite(PIN_MOTOR_EN, LOW);              // disable motor
  // digitalWrite(PIN_MOTOR_EXT_H, LOW);           // set motor to retract
  // digitalWrite(PIN_MOTOR_EXT_L, HIGH);
  // digitalWrite(PIN_MOTOR_EN, HIGH);             // enable motor for offset measurement
  // ledSetMotor(MOTOR_LED_OPENING);
  // delayMs(500);                                  // let the offset current stabilize
  // adcSample = 0;                                // accumulations starts at zero
  // adcSampleCnt = 1000;                          // prep # of samples in accumulation
  // while (adcSampleCnt != 0)
  //   delayMs(1);                                 // wait until all samples are accumulated
  // Println(adcSample / 1000);

  // // turn motor off
  // digitalWrite(PIN_MOTOR_EN, LOW);              // disable motor
  // ledSetMotor(MOTOR_LED_OFF);                   // set motor LED off
  // delayMs(100);

  Println("");
  Logln("Calibration is complete.");
  Logln("This application has stopped and will no longer function.");
  Logln("Rebuild application without calibration enabled to operate normally.");
  
  // We are done with the ADC
  REG_ADC_INTENCLR = ADC_INTENCLR_RESRDY;       // disable ADC result ready interrupt
  while (ADC->STATUS.bit.SYNCBUSY);             // wait for synchronization
  REG_ADC_CTRLA = 0;                            // disable ADC
  while (ADC->STATUS.bit.SYNCBUSY);             // wait for synchronization

  while(true) {
    WDTZero.reset();
  }
}

/******************************************************************
 * Interrupt Handlers
 ******************************************************************/
/******************************************************************
 * This is the ADC interrupt service routine (ISR)
 * When a calibration sample is in progress it will be result ready 
 * interrupts that call this interrupt handler.
 * When a normal over current detect cycle it will be window monitor 
 * interrupts that call this interrupt handler.
 ******************************************************************/
void ADC_Handler() {
  uint32_t result;
  if (REG_ADC_INTFLAG & ADC_INTFLAG_WINMON) {
    // this was a window monitor interrupt
  } else {
    // this was a result ready interrupt
    result = REG_ADC_RESULT;                    // read the last result
    if (adcSampleCnt > 0) {
      // update accumulation only if there are more samples to add
      adcSample += result;                      // add last result to accumulator
      adcSampleCnt--;                           // one more sample added to average
    }
  }
}

/******************************************************************
 * This is the MELIM interrupt service routine (ISR)
 * This is called anytime the MELIM pin changes.
 ******************************************************************/
void onMELIMChange() {
  // melimCount++;
  dbMelimRead = digitalRead(PIN_EXTEND_LIMIT);    // get current extend limit state
  if (GATE_DIRECTION  == GATE_PULL_TO_OPEN) {
    // gate motor extends when closing
    if (HAL.gateIsClosing() && !dbMelimRead) {
      // motor is closing and extend limit switch just went active, shut off motor
      HAL.gateStop();
    }
  } else {
    // gate motor extends when opening
    if (HAL.gateIsOpening() && !dbMelimRead) {
      // motor is opening and extend limit switch just went active, shut off motor
      HAL.gateStop();
    }
  }
}

/******************************************************************
 * This is the MRLIM interrupt service routine (ISR)
 * This is called anytime the MELIM pin changes.
 ******************************************************************/
void onMRLIMChange() {
  // mrlimCount++;
  dbMrlimRead = digitalRead(PIN_RETRACT_LIMIT);    // get current extend limit state
  if (GATE_DIRECTION  == GATE_PULL_TO_OPEN) {
    // gate motor retracts when opening
    if (HAL.gateIsOpening() && !dbMrlimRead) {
      // motor is opening and retract limit switch just went active, shut off motor
      HAL.gateStop();
    }
  } else {
    // gate motor retracts when closing
    if (HAL.gateIsClosing() && !dbMrlimRead) {
      // motor is closing and retract limit switch just went active, shut off motor
      HAL.gateStop();
    }
  }
}

/******************************************************************
   Preinstantiate Objects
 ******************************************************************/
HalClass HAL = HalClass();
