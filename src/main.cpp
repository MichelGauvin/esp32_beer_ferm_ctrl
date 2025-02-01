/*
  Fermentation PID temperature Controller (x4)
  Made by Michel Gauvin
  www.mikesbrewshop.com

  TODO:
  - Reset the PID when it is turn off:
      PID.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
      PID.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
      PID.SetOutputLimits(PIDMinimum, PIDMaximum);  // Set the limits back to normal
  - Synchroniser avec NTP la date et l'heure.
  TODO: Il faudrait ajouter une facon de pouvoir reconfigurer le wifi en appuyant sur un bouton lors du redémarrage, se mettre en mode portal pour un temps
          indéfini.

  BUG:
  
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

#define ONE_WIRE_BUS 4 //

// Create AsyncWebServer object on port 80
// This is to publish the webpage
AsyncWebServer server(80);
DNSServer dns;
// Create a WebSocket server object on port 3333
WebSocketsServer webSocket = WebSocketsServer(3333);
//Create storage object to save and read setpoint and fermenter status.
//This will assure to put the controller in the same state (setpoint and fermenter status) when reboot
//after power outage or anything else.
Storage storage;
// For temperature reading
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
// PID Constants and Variables
// Petit fermenteur
double Kp = 70, Ki = 1, Kd = 0;
// Gros fermenteur 2 double Kp = 150, Ki = 1, Kd = 0; // J'ai mis le proportionelle a 150 pour le gros fermenteur.
double Kp2 = 150, Ki2 = 1, Kd2 = 0;
//Setup fermenters
Fermenter fermenters[4] = {
    {1, 0, 0, 0, false, false, false, 0, PID(&fermenters[0].input, &fermenters[0].output, &fermenters[0].setpoint, Kp, Ki, Kd, REVERSE), MovingAveragePlus<float>(5), 0, 0, {0x28, 0xFF, 0x0B, 0xF4, 0x6F, 0x18, 0x01, 0x79}},
    {2, 0, 0, 0, false, false, false, 0, PID(&fermenters[1].input, &fermenters[1].output, &fermenters[1].setpoint, Kp2, Ki2, Kd2, REVERSE), MovingAveragePlus<float>(5), 0, 0, {0x28, 0xFF, 0x3F, 0xE2, 0x69, 0x18, 0x03, 0xBE}},
    {3, 0, 0, 0, false, false, false, 0, PID(&fermenters[2].input, &fermenters[2].output, &fermenters[2].setpoint, Kp2, Ki2, Kd2, REVERSE), MovingAveragePlus<float>(5), 0, 0, {0x28, 0xFF, 0x96, 0xE7, 0x6F, 0x18, 0x01, 0x92}},
    {4, 0, 0, 0, false, false, false, 0, PID(&fermenters[3].input, &fermenters[3].output, &fermenters[3].setpoint, Kp, Ki, Kd, REVERSE), MovingAveragePlus<float>(5), 0, 0, {0x28, 0xFF, 0x34, 0xF8, 0x6F, 0x18, 0x01, 0x80}}};
// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
const long interval = 1000; // interval between temperature reading (milliseconds)

void setup()
{
  // Init relays
  setupRelays();
  // Init storage
  storage.begin();
  // Init serial communication
  Serial.begin(115200);
  Serial.println(" SETUP ");
  // Init WiFi
  setupWiFi();
  // Init Web Server
  setupWebServer();
  // Mount file system to have access to index.html
  if (!LittleFS.begin(true)) {
          Serial.println("Failed to mount LittleFS");
          return;
      }
  //Init PID
  setupPID(fermenters, 4);
}

void loop()
{
  currentMillis = millis();
  while ((currentMillis - previousMillis) < interval)
  {
    // The code in this while loop executes between two temperatures measurements.
    // Handle WebSocket events
    webSocket.loop();
    delay(10);
    currentMillis = millis();
  }
  previousMillis = currentMillis;
  readTemperatures(fermenters, 4, sensors);
  handlePIDLoop(fermenters, 4);
  sendData();
}