/*
MIT LICENCE

Copyright 2025 Michel Gauvin (michel.gauvin@gmail.com) www.mikesbrewshop.com

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WebSocketsServer.h>
#include "storage.h"
#include "wifi_setup.h"
#include "web_server.h"
#include "fermenter_control.h"
#include "relay_control.h"
#include "read_ds18b20_temp.h"

#define ONE_WIRE_BUS 4  // Pin for OneWire bus

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
DNSServer dns;

// Create a WebSocket server object on port 3333
WebSocketsServer webSocket(3333);

// Storage object to save and restore setpoints and fermenter status
Storage storage;

// Temperature sensor setup
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// PID Constants
const double Kp_small = 70, Ki_small = 1, Kd_small = 0;
const double Kp_large = 150, Ki_large = 1, Kd_large = 0;

// Fermenter setup
Fermenter fermenters[4] = {
    {1, 0, 0, 0, false, false, false, 0, PID(&fermenters[0].input, &fermenters[0].output, &fermenters[0].setpoint, Kp_small, Ki_small, Kd_small, REVERSE), MovingAveragePlus<float>(5), 0, 0, {0x28, 0xFF, 0x0B, 0xF4, 0x6F, 0x18, 0x01, 0x79}},
    {2, 0, 0, 0, false, false, false, 0, PID(&fermenters[1].input, &fermenters[1].output, &fermenters[1].setpoint, Kp_large, Ki_large, Kd_large, REVERSE), MovingAveragePlus<float>(5), 0, 0, {0x28, 0xFF, 0x3F, 0xE2, 0x69, 0x18, 0x03, 0xBE}},
    {3, 0, 0, 0, false, false, false, 0, PID(&fermenters[2].input, &fermenters[2].output, &fermenters[2].setpoint, Kp_large, Ki_large, Kd_large, REVERSE), MovingAveragePlus<float>(5), 0, 0, {0x28, 0xFF, 0x96, 0xE7, 0x6F, 0x18, 0x01, 0x92}},
    {4, 0, 0, 0, false, false, false, 0, PID(&fermenters[3].input, &fermenters[3].output, &fermenters[3].setpoint, Kp_small, Ki_small, Kd_small, REVERSE), MovingAveragePlus<float>(5), 0, 0, {0x28, 0xFF, 0x34, 0xF8, 0x6F, 0x18, 0x01, 0x80}}
};

// Timing variables
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
const long interval = 1000; // Interval between temperature readings (milliseconds)

void setup() {
    Serial.begin(115200);
    Serial.println("SETUP");

    // Initialize relays
    setupRelays();

    // Initialize storage
    storage.begin();

    // Initialize WiFi
    setupWiFi();

    // Initialize Web Server
    setupWebServer();

    // Mount file system
    if (!LittleFS.begin(true)) {
        Serial.println("Failed to mount LittleFS");
        return;
    }

    // Initialize PID controllers
    setupPID(fermenters, 4);
}

void loop() {
    currentMillis = millis();
    
    while ((currentMillis - previousMillis) < interval) {
        // Handle WebSocket events
        webSocket.loop();
        delay(10);
        currentMillis = millis();
    }
    
    previousMillis = currentMillis;
    
    // Read temperatures and update PID control
    readTemperatures(fermenters, 4, sensors);
    handlePIDLoop(fermenters, 4);
    sendData();
}
