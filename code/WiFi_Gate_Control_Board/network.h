/*
  Network Module for WiFi Gate Control Board.

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
#ifndef NETWORK_H
#define NETWORK_H
#include <MQTT.h>
#include "WiFi_Gate_Control_Board.h"

class NetworkClass {
public:
  NetworkClass();
  void begin(void);                       // initialize everything
  void loop(void);                        // must be called as schedule task

private:
  WiFiClient m_net;                       // Network
  MQTTClient m_mqtt = MQTTClient(1024);   // MQTT CLient
  bool m_resetOccurred = true;            // when true a reset occurred recently
  bool m_wifiDisconnectOccurred = true;   // when true WiFi was recently disconnected from the network
  bool m_wifiConnectOccurred = false;     // when true WiFi was recently connected to the network
  uint8_t connect(void);                  // connect to to WiFi and MQTT
};

extern NetworkClass NET;
#endif // NETWORK_H
