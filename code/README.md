# WiFi Gate Control Board Sketch
The WiFi Gate Control Board Sketch is written in C++. It replaces the functionality of a US Automatic Gate Control Board while adding IOT and Automation via WiFi. The sketch is a discoverable cover and sensor for [Home Assistant](https://home-assistant.io/), an open-source home automation platform running on Python. [MQTT](http://mqtt.org/), a machine-to-machine (M2M)/"Internet of Things" connectivity protocol, is the basis of communication with Home Assistant.

## Status
This software is in the development stage.
* Uses FreeRTOS to prevent Arduino's WiFi101 library from blocking gate functionality when there are network problems.
* Network is fully functional including MQTT connectivity.
* Hardware Access Layer (HAL) supports the 12 board inputs including mapping to gate inputs.
* Error module supports hard faults which stop the board from doing anything and general errors with Status LED indication.
* Basic Home Assistant automatic configuration via MQTT is in place but does nothing yet.

# Setup
## Sketch Setup
Build defines in [WiFi_Gate_Control_Board.h](WiFi_Gate_Control_Board/WiFi_Gate_Control_Board.h) control how the sketch is built.

```c
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
 * Application defines
 ******************************************************************/
// Name of this board, each board should be unique
// Used as MQTT Client ID, HASS Name, and OTA Name
#define BOARD_NAME                "New Front Gate"
// input debounce time in milliseconds
#define DEBOUNCE_TIME             100
// temperature measurement time in milliseconds, Must be greater than 5 seconds
#define TEMP_RATE                 (1*60*1000)
// Length of time in milliseconds LEDS stay on after pushbutton input
#define LED_ON_TIME               (5*60*1000)
// NTP server used to update local time, can be IP Address or name
#define NTP_SERVER                "pool.ntp.org"
//#define NTP_SERVER                "time.nist.gov"
// timezone difference from GMT in hours (Standard Time difference)
#define TZDIFF                    -6

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
```

* When "ENABLE_WATCHDOG" is defined the Watchdog Timer is enabled with a 16 second timeout. This will reset the ARM processor is something goes wrong.
* When "ENABLE_OTA_UPDATES" is defined the sketch can be updated using Arduino's Over-The-Air Update capability.
* When "ENABLE_SERIAL" is defined then status information is sent out the serial connection which in this case is the USB port. Otherwise the serial port won't even be enabled.
* When "ENABLE_WIFI_LOW_POWER" is defined the WINC1500 module is set to Low Power Mode. This can drop the current requirements of the module to a third or even less. Low Power Mode reduces the transmit frequency to the beacon interval and may also cause the module to hang occasionally.
* When "ENABLE_CALIBRATION_MODE" is defined the sketch is put into calibration mode. This is an allows the user to calibrate the Motor Current Sense ADC input at 15 amps and 25 C.
* "BOARD_NAME" names the board so multiple instances of this board can on the WiFi network at the same time. It is used is several places including Home Assistant Name, OTA Name, and MQTT Client ID. This is a string.
* "DEBOUNCE_TIME" is how long to ignore contact changes on the input before accepting them as valid. This is an integer and the units are milliseconds.
* "TEMP_RATE" is how often the temperature should be sampled and updated via MQTT.  This is an integer and the units are milliseconds. The time must be greater than 5000 milliseconds.
* "NTP_SERVER" is name of the NTP server used to synchronize time. This can be a DNS name like "pool.ntp.org" or an IP address 192.168.1.123.
* "TZDIFF" is the UTC to actual time zone difference in hours.
* "LED_ON_TIME" is how long the LED's on the board will stay on after the LED button is pushed.
* 'GATE_DIRECTION' is a define used to set the direction of the gate. Either "GATE_PULL_TO_OPEN" or "GATE_PUSH_TO_OPEN".
* 'BOARD_IN1-BOARD_IN12' defines the type of input for each of the generic inputs on the board.
  * IN_TYPE_NONE means the input is not active.
  * IN_TYPE_OPEN_LIMIT_NO means normally open switch Open Limit input.
  * IN_TYPE_CLOSE_LIMIT_NO means normally open switch Close Limit input.
  * IN_TYPE_PUSHBUTTON_NO means normally open switch Open/Stop/Close input (pushbutton toggle).
  * IN_TYPE_PUSHBUTTON_NC means normally closed switch Open/Stop/Close input (pushbutton toggle).
  * IN_TYPE_OPEN_NO means normally open switch Open input.
  * IN_TYPE_OPEN_NC means normally closed switch Open input.
  * IN_TYPE_CLOSE_NO means normally open switch Close input.
  * IN_TYPE_CLOSE_NC means normally closed switch Close input.
  * IN_TYPE_STOP_NO means normally open switch Stop input.
  * IN_TYPE_STOP_NC means normally closed switch Stop input.
  * IN_TYPE_OPEN_PHOTOEYE_NO means normally open PhotoEye open input.
  * IN_TYPE_OPEN_PHOTOEYE_NC means normally closed PhotoEye open input.
  * IN_TYPE_OPEN_PHOTOEYE_10K means normally open PhotoEye open input with 10k monitoring.
  * IN_TYPE_CLOSE_PHOTOEYE_NO means normally open PhotoEye close input.
  * IN_TYPE_CLOSE_PHOTOEYE_NC means normally closed PhotoEye close input.
  * IN_TYPE_CLOSE_PHOTOEYE_10K means normally open PhotoEye close input with 10k monitoring.
  * IN_TYPE_EDGE_NO means normally open Edge input.
  * IN_TYPE_EDGE_NC means normally closed Edge input.
  * IN_TYPE_EDGE_10K means normally open Edge input with 10k monitoring.
  * IN_TYPE_SAFETY_NO means normally open Safety input.
  * IN_TYPE_SAFETY_NC means normally closed Safety input.
  * IN_TYPE_SAFETY_10K means normally open Safety input with 10k monitoring.
  * IN_TYPE_SHADOW_NO means normally open Shadow input.
  * IN_TYPE_SHADOW_NC means normally closed Shadow input.
  * IN_TYPE_SHADOW_10K means normally open Shadow input with 10k monitoring.
  * IN_TYPE_INPUT1_NO means normally open Input 1 input.
  * IN_TYPE_INPUT1_NC means normally closed Input 1 input.
  * IN_TYPE_INPUT1_10K means normally open Input 1 input with 10k monitoring.
  * IN_TYPE_INPUT2_NO means normally open Input 2 input.
  * IN_TYPE_INPUT2_NC means normally closed Input 2 input.
  * IN_TYPE_INPUT2_10K means normally open Input 2 input with 10k monitoring.
* "HASS_PREFIX" is the Home Assistant MQTT Discovery Prefix as defined in your system. This is a string.
* "HASS_GATE_NAME" is used in the MQTT topics to identify the cover and sensor to Home Assistant. Home Assistant calls this the node id. It is a string and must not contain special characters including a space.

The arduino_secrets.h file is not included on Github. You must create and edit it to meet your configuration.

```c
/******************************************************************
 * arduino_secrets.h
 ******************************************************************/
 /******************************************************************
  * WiFI Setup
  ******************************************************************/
 #define SECRET_SSID             "SSID of your WiFi network"
 #define SECRET_PASSWORD         "Password for your WIFi network"

 /******************************************************************
  * MQTT Setup
  ******************************************************************/
 #define MQTT_SERVER             "mqtt.local"
 #define MQTT_SERVERPORT         1883
 #define MQTT_USERNAME           "MQTT username"
 #define MQTT_PASSWORD           "MQTT password"

 /******************************************************************
  * OTA Setup
  ******************************************************************/
 #define OTA_PASSWORD            "OTA password"
```

## Home Assistant Setup
ToDo.

## Calibration
ToDo.
