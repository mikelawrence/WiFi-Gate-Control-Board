/*
  Network Module for WiFi Gate Control Board.

  Handles connections to WiFi and MQTT.

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
#include <WiFi101.h>
#include <WiFi101OTA.h>
#include <MQTT.h>               // https://github.com/256dpi/arduino-mqtt
#include "error.h"
#include "network.h"
#include "WiFi_Gate_Control_Board.h"

/******************************************************************
 * Defines, constants and enumerations
 ******************************************************************/
// time in milliseconds that a WiFi connection is unresponsive before reconnecting
#define WIFI_CONNECTION_RETRY_TIME 5*60*1000UL
// time in milliseconds between MQTT connection attempts
#define MQTT_CONNECTION_DELAY_TIME 30*1000UL

/******************************************************************
 * Error enumeration
 ******************************************************************/
enum Network_Status_Enum {
  NET_BOTH_CONNECTED = 0,         // WiFi and MQTT are both connected
  NET_WIFI_NOT_CONNECTED,         // WiFi is not connected
  NET_MQTT_NOT_CONNECTED,         // WiFi is connected but MQTT is not
};

/******************************************************************
 * Prototypes
 ******************************************************************/
void messageReceived(MQTTClient *client, char topic[], char payload[], int payload_length);

/******************************************************************
 * Public Methods
 ******************************************************************/

/******************************************************************
   Constructor
 ******************************************************************/
NetworkClass::NetworkClass() {
}

/******************************************************************
   Initialize the Network Class.
 ******************************************************************/
void NetworkClass::begin() {
  // WiFi is not connected by default
  ERR.setError(ERROR_WIFI_DISCON, false);

  // Turn on WINC1500 WiFi module and connect to network
  WiFi.begin(SECRET_SSID, SECRET_PASSWORD);
  Logln("Connecting to WiFi Network: " SECRET_SSID);

  // MQTT setup
  m_mqtt.begin(MQTT_SERVER, MQTT_SERVERPORT, m_net);    // initialize mqtt object
  m_mqtt.setOptions(65, true, 5000);                    // keep Alive, Clean Session, Timeout
  m_mqtt.setWill(HASS_AVAIL_TOPIC, HASS_PAYLOAD_NOT_AVAIL, true, 1); // Set MQTT Will to offline
  m_mqtt.onMessageAdvanced(messageReceived);            // topic received handler
}

/******************************************************************
 * Periodically handle network and operations.
 *   Designed for use with FreeRTOS.
 ******************************************************************/
void NetworkClass::loop() {  
  uint8_t connectionStatus;

  // we are connected to both WiFi and MQTT
  WiFiRTC.loop();                 // keep track of time

  // check connections
  connectionStatus = connect();
  if (connectionStatus == NET_WIFI_NOT_CONNECTED) {
    // WiFi is not connected, update error
    if (ERROR_WIFI_DISCON > ERR.getError()) {
      // WiFi is not connected is a higher priority
      ERR.setError(ERROR_WIFI_DISCON);
    }
    // ignore the rest of this loop
    return;
  } else if (connectionStatus == NET_MQTT_NOT_CONNECTED) {
    // MQTT is not connected
    if (ERROR_MQTT_DISCON > ERR.getError()) {
      // MQTT is not connected is a higher priority
      ERR.setError(ERROR_MQTT_DISCON);
    }
    // ignore the rest of this loop
    return;
  } else {
    // no error, was previous error WiFi or MQTT not connected
    if ((ERR.getError() == ERROR_WIFI_DISCON) || 
        (ERR.getError() == ERROR_MQTT_DISCON)) {
      ERR.setError(ERROR_NONE);
    }
  }

  #ifdef ENABLE_OTA_UPDATES
  // check for WiFi OTA updates
  WiFiOTA.poll();
  #endif

  // MQTT client loop call
  m_mqtt.loop();
}

/******************************************************************
   Private Methods
 ******************************************************************/
/******************************************************************
 * Verify/Make WiFi and MQTT connections
 ******************************************************************/
uint8_t NetworkClass::connect() {
  static uint32_t lastWiFiRetryTime = xTaskGetTickCount();
  static uint32_t lastMQTTRetryTime = lastWiFiRetryTime - (MQTT_CONNECTION_DELAY_TIME + 5);
  static uint32_t lastDisconnectTime;
  byte mac[6];
  IPAddress ip;
  
  int32_t status = WiFi.status();
  
  if (status != WL_CONNECTED) {
    // Wifi is disconnected
    m_wifiDisconnectOccurred = true;                  // we were disconnected
    if (abs(xTaskGetTickCount() - lastDisconnectTime) > 4*WIFI_CONNECTION_RETRY_TIME) {
      // something went wrong with lastDisconnectTime, reset
      lastDisconnectTime = xTaskGetTickCount() - WIFI_CONNECTION_RETRY_TIME - 10;
    }
    if (xTaskGetTickCount() - lastDisconnectTime > WIFI_CONNECTION_RETRY_TIME) {
      // it's been too long since we were last connected
      WiFi.end();                                     // turn off WiFi module
      // Turn on WINC1500 WiFi module and connect to network again
      WiFi.begin(SECRET_SSID, SECRET_PASSWORD);
      // Reconnect in another 15 minutes
      lastDisconnectTime = xTaskGetTickCount();
      Logln("Reconnecting to WiFi Network: " SECRET_SSID);
    }
    return NET_WIFI_NOT_CONNECTED;                    // return with WiFi not connected
  }
  
  // update last time we were disconnected to now because WiFi is connected
  lastDisconnectTime = xTaskGetTickCount();
  
  if (m_wifiDisconnectOccurred && (status == WL_CONNECTED)) {
    // WiFi just connected
    Logln("Connected to WiFi Network: " SECRET_SSID);
    
    // we have detected that we just connected
    m_wifiDisconnectOccurred = false;                 // so we won't print network stats until next reconnect
    m_wifiConnectOccurred = true;                     // so MQTT publishing will know that Wifi just connected
    
    #ifdef ENABLE_SERIAL
    // Display MAC Address
    WiFi.macAddress(mac);
    Log("  MAC Address: ");
    for (int i = 5; i != 0; i--) {
      if (mac[i] < 16) Print("0");
      Print(mac[i], HEX);
      Print(":");
    }
    if (mac[0] < 16) Print("0");
    Println(mac[0], HEX);
    
    // Display IP Address
    ip = WiFi.localIP();
    Log("  IP Address: ");
    Println(ip);
    #endif
    
    // Enable WiFi Low Power Mode
    #ifdef ENABLE_WIFI_LOW_POWER
    WiFi.lowPowerMode();
    Logln("  Low Power Mode enabled");
    #else
    Logln("  Low Power Mode disabled");
    #endif
    
    #ifdef ENABLE_OTA_UPDATES
    // start the WiFi OTA library with internal based storage
    WiFiOTA.begin(BOARD_NAME, OTA_PASSWORD, InternalStorage);
    Logln("  WiFi OTA updates enabled");
    #else
    Logln("  WiFi OTA updates disabled");
    #endif

    // start the WiFi Real Time Clock
    WiFiRTC.begin(TZDIFF, NTP_SERVER);
  }

  // WiFi is connected, see if MQTT is connected
  if (!m_mqtt.connected()) {
    if (xTaskGetTickCount() - lastWiFiRetryTime >= WIFI_CONNECTION_RETRY_TIME) {
      // it's been too long since we had an MQTT connection, turn off WiFi module
      WiFi.end(); 
      // Turn on WINC1500 WiFi module and connect to network again
      WiFi.begin(SECRET_SSID, SECRET_PASSWORD);
      Logln("Too long without MQTT server: " MQTT_SERVER ", Reconnecting to WiFi Network: " SECRET_SSID);
      lastMQTTRetryTime = xTaskGetTickCount();                   // restart MQTT retry time
      lastWiFiRetryTime = lastMQTTRetryTime;          // restart WiFi retry time
      m_wifiDisconnectOccurred = true;                // indicate we actually disconnected for a bit
      return NET_WIFI_NOT_CONNECTED;                  // Now WIFI is not connected
    }
//    if (abs(xTaskGetTickCount() - lastMQTTRetryTime) > 4*MQTT_CONNECTION_DELAY_TIME) {
//      // something went wrong with lastMQTTRetryTime, reset
//      lastMQTTRetryTime = xTaskGetTickCount() - MQTT_CONNECTION_DELAY_TIME - 1;
//    }
    // we are not currently connected to MQTT Server
    if (xTaskGetTickCount() - lastMQTTRetryTime <= MQTT_CONNECTION_DELAY_TIME) {
      // not time to retry MQTT connection
      return NET_MQTT_NOT_CONNECTED;                  // MQTT is not connected
    }
    // time to retry server connection
    lastMQTTRetryTime = xTaskGetTickCount();                     // restart MQTT retry time
    Logln("Connecting to MQTT server");
    if (m_mqtt.connect(BOARD_NAME, MQTT_USERNAME, MQTT_PASSWORD)) {
      // successfully connected to MQTT server
      Logln("Connected to MQTT Server: " MQTT_SERVER);
    } else {
      // failed to connect to MQTT server
      Logln("Failed to connect to MQTT Server: " MQTT_SERVER);
      return NET_MQTT_NOT_CONNECTED;                  // MQTT is not connected
    }
        
    // Subscribe to Home Assistant command topic
    if (!m_mqtt.subscribe(HASS_GATE_COMMAND_TOPIC)) {
      Logln("  Failed to subscribe MQTT topic '" HASS_GATE_COMMAND_TOPIC "'");
    } else {
      Logln("  Subscribed to MQTT topic '" HASS_GATE_COMMAND_TOPIC "'");
    }
    
    // set Home Assistant Availibility Topic to available
    if (!m_mqtt.publish(HASS_AVAIL_TOPIC, HASS_PAYLOAD_AVAIL, true, 1)) {
      Logln("  Failed to publish MQTT topic '" HASS_AVAIL_TOPIC "'");
    } else {
      Logln("  Published MQTT topic '" HASS_AVAIL_TOPIC "', value '" HASS_PAYLOAD_AVAIL "'");
    }

    // Publish Home Assistant gate config topic
    if (!m_mqtt.publish(HASS_GATE_CONFIG_TOPIC, HASS_GATE_CONFIG, true, 1)) {
      Logln("  Failed to publish MQTT topic '" HASS_GATE_CONFIG_TOPIC "'");
    } else {
      Logln("  Published MQTT topic '" HASS_GATE_CONFIG_TOPIC "', value '" HASS_GATE_CONFIG "'");
    }
    
    // Publish Home Assistant temperature config topic
    if (!m_mqtt.publish(HASS_TEMP_CONFIG_TOPIC, HASS_TEMP_CONFIG, true, 1)) {
      Logln("  Failed to publish MQTT topic '" HASS_TEMP_CONFIG_TOPIC "'");
    } else {
      Logln("  Published MQTT topic '" HASS_TEMP_CONFIG_TOPIC "', value '" HASS_TEMP_CONFIG "'");
    }
    
    // Publish Home Assistant RSSI config topic
    if (!m_mqtt.publish(HASS_RSSI_CONFIG_TOPIC, HASS_RSSI_CONFIG, true, 1)) {
      Logln("  Failed to publish MQTT topic '" HASS_RSSI_CONFIG_TOPIC "'");
    } else {
      Logln("  Published MQTT topic '" HASS_RSSI_CONFIG_TOPIC "', value '" HASS_RSSI_CONFIG "'");
    }
    
    // Publish Home Assistant status config topic
    if (!m_mqtt.publish(HASS_STATUS_CONFIG_TOPIC, HASS_STATUS_CONFIG, true, 1)) {
      Logln("  Failed to publish MQTT topic '" HASS_STATUS_CONFIG_TOPIC "'");
    } else {
      Logln("  Published MQTT topic '" HASS_STATUS_CONFIG_TOPIC "', value '" HASS_STATUS_CONFIG "'");
    }
    
    if (m_resetOccurred) {                              // Hardware reset occurred
      // reset just recently occurred
      if (!m_mqtt.publish(HASS_STATUS_STATE_TOPIC, "Reset Hardware", true, 1)) {
        Logln("  Failed to publish MQTT topic '" HASS_STATUS_STATE_TOPIC "'");
      } else {
        Logln("  Published MQTT topic '" HASS_STATUS_STATE_TOPIC "', value 'Reset Hardware'");
      }
    } else if (m_wifiConnectOccurred) {                 // WiFi Connected
      // Wifi just connected
      if (!m_mqtt.publish(HASS_STATUS_STATE_TOPIC, "Reset WiFi Connect", true, 1)) {
        Logln("  Failed to publish MQTT topic '" HASS_STATUS_STATE_TOPIC "'");
      } else {
        Logln("  Published MQTT topic '" HASS_STATUS_STATE_TOPIC "', value 'Reset WiFi Connect'");
      }
    }
    
    // clear the connect reason flags
    m_resetOccurred = false;
    m_wifiConnectOccurred = false;

    // we had to reconnect to MQTT Broker make sure at least one false is returned
    //return NET_MQTT_NOT_CONNECTED;                      // MQTT is not connected
  }
  
  lastMQTTRetryTime = xTaskGetTickCount();                         // be ready for next MQTT retry time
  lastWiFiRetryTime = lastMQTTRetryTime;                // be ready for next WiFi retry time
    
  // if we got here then both WiFi and MQTT are connected
  return NET_BOTH_CONNECTED;
}

/******************************************************************
 * MQTT subscribed message received
 ******************************************************************/
void messageReceived(MQTTClient *client, char topic[], char payload[], int payload_length) {
//  if (strcmp(topic, HASS_GATE_COMMAND_TOPIC) == 0) {
//    
//  }
}

/******************************************************************
   Preinstantiate Objects
 ******************************************************************/
NetworkClass NET = NetworkClass();
