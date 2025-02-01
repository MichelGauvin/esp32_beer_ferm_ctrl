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
  - Tester ce qui arrive si on perd le Wifi:
    * Testé: Le code continu de s'exécuter
    * Testé: Si le réseau déjà configuré n'est pas disponible le code va attendre 2 mins pour la configuration du portal, si pas de nouvelle config
      continuer d'exécuter le code sans internet. Si le réseau revient la connexion wifi sera rétablie.
    TODO: Il faudrait ajouter une facon de pouvoir reconfigurer le wifi en appuyant sur un bouton lors du redémarrage, se mettre en mode portal pour un temps
          indéfini.

  BUG:
  
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <MovingAveragePlus.h>
#include <SPI.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ESPAsyncWiFiManager.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>
#include <esp_system.h>
#include <storage.h>
#include "wifi_setup.h"
#include "web_server.h"
#include "fermenter_control.h"

#define RELAY1 16 //
#define RELAY2 17 //
#define RELAY3 18 //
#define RELAY4 19 //

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

//************************ PID DEFINITION****************************
// PID Constants and Variables
// Petit fermenteur
double Kp = 70, Ki = 1, Kd = 0;
// Gros fermenteur 2 double Kp = 150, Ki = 1, Kd = 0; // J'ai mis le proportionelle a 150 pour le gros fermenteur.
double Kp2 = 150, Ki2 = 1, Kd2 = 0;

//Setup fermenters
Fermenter fermenters[4] = {
    {1, 0, 0, 0, false, false, false, 0, PID(&fermenters[0].input, &fermenters[0].output, &fermenters[0].setpoint, Kp, Ki, Kd, REVERSE), MovingAveragePlus<float>(5), 0, 0, {0x28, 0xFF, 0x0B, 0xF4, 0x6F, 0x18, 0x01, 0x79}, RELAY1},
    {2, 0, 0, 0, false, false, false, 0, PID(&fermenters[1].input, &fermenters[1].output, &fermenters[1].setpoint, Kp2, Ki2, Kd2, REVERSE), MovingAveragePlus<float>(5), 0, 0, {0x28, 0xFF, 0x3F, 0xE2, 0x69, 0x18, 0x03, 0xBE}, RELAY2},
    {3, 0, 0, 0, false, false, false, 0, PID(&fermenters[2].input, &fermenters[2].output, &fermenters[2].setpoint, Kp2, Ki2, Kd2, REVERSE), MovingAveragePlus<float>(5), 0, 0, {0x28, 0xFF, 0x96, 0xE7, 0x6F, 0x18, 0x01, 0x92}, RELAY3},
    {4, 0, 0, 0, false, false, false, 0, PID(&fermenters[3].input, &fermenters[3].output, &fermenters[3].setpoint, Kp, Ki, Kd, REVERSE), MovingAveragePlus<float>(5), 0, 0, {0x28, 0xFF, 0x34, 0xF8, 0x6F, 0x18, 0x01, 0x80}, RELAY4}};

const int pid_T = 30; // PID computation interval in seconds
const int WindowSize = 300;
//************************ END PID **********************************

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

//*********** FIRMWARE VARIABLES DEFINITIONS **************************
const long interval = 1000; // interval between temperature reading (milliseconds)

//*********** END FIRMWARE VARIABLES DEFINITIONS **********************

void setup()
{

  //************************ RELAY SETUP ******************************
  // We need to configure the pinMode to Ferm1_Input_PULLUP first
  // It avoids having a LOW state when configuring the pinMode to Ferm1_Output.
  // IMPORTANT: WE INVERSE THE RELAY LOGIC BY DOING THIS
  // Issue #3 mikesbrewshop.com
  pinMode(RELAY1, INPUT_PULLUP);
  pinMode(RELAY2, INPUT_PULLUP);
  pinMode(RELAY3, INPUT_PULLUP);
  pinMode(RELAY4, INPUT_PULLUP);
  // delay(100);
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);

  for (int i = 0; i < 4; ++i)
  {
    digitalWrite(fermenters[i].relayPin, HIGH);
    fermenters[i].pumpStatus = false;
  }

  //************************ END RELAY SETUP ******************************

  // Initialize storage
  storage.begin();
  //************************ SERIAL COMM SETUP ************************
  Serial.begin(115200);
  Serial.println(" SETUP ");
  //************************ END SERIAL COMM SETUP ********************

  //************************ WIFI SETUP *******************************
  setupWiFi();
  //************************ WEBSERVER SETUP *******************************
  setupWebServer();
  //************************ START LITTLEFS SETUP *********************
  
  if (!LittleFS.begin(true)) {
          Serial.println("Failed to mount LittleFS");
          return;
      }

      Serial.println("Files in LittleFS:");
      File root = LittleFS.open("/");
      File file = root.openNextFile();

      while (file) {
          Serial.print("FILE: ");
          Serial.print(file.name());
          Serial.print("\tSIZE: ");
          Serial.println(file.size());
          file = root.openNextFile();
      }  
  //************************ END LITTLEFS SETUP ***********************

  //************************ PID SETUP ********************************
  for (int i = 0; i < 4; ++i)
  {
    fermenters[i].setpoint = storage.readSetpoint(i);
    fermenters[i].fermStatus = storage.readFermStatus(i);
  }

  for (auto &fermenter : fermenters)
  {
    fermenter.pid.SetMode(AUTOMATIC);
    fermenter.windowStartTime = millis();
  }

  // Tell the PID to range between 0 and the full window size
  fermenters[0].pid.SetOutputLimits(0, WindowSize / 10);  // Limit maximum Ferm1_Output to 30 seconds every 5 minutes
  fermenters[1].pid.SetOutputLimits(0, WindowSize / 1);  // Limit maximum Ferm2_Output to 300 seconds every 5 minutes
  fermenters[2].pid.SetOutputLimits(0, WindowSize / 1); // Limit maximum Ferm3_Output to 30 seconds every 5 minutes
  fermenters[3].pid.SetOutputLimits(0, WindowSize / 10); // Limit maximum Ferm4_Output to 30 seconds every 5 minutes

  //************************ END PID SETUP*****************************

  //************************ TEMPERATURE READING SETUP ********************
  // Make asynchronous call to requestTempeature()
  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures(); 
  //************************ END TEMPERATURE READING SETUP ****************
}

// Function to update next pump cycle time
void updateNextPumpCycleTime()
{
  for (int i = 0; i < 4; ++i)
  {
    if (fermenters[i].fermStatus == true)
    {
      fermenters[i].nextPumpCycleTime = ((WindowSize * 1000 - (millis() - fermenters[i].windowStartTime))) / 1000;
    }
    else
    {
      fermenters[i].nextPumpCycleTime = 0;
      fermenters[i].output = 0;
    }
  }
}

// Function to handle the PID loop for a fermenter
void handleFermenterPIDLoop()
{
  for (int i = 0; i < 4; ++i)
  {
    if (fermenters[i].fermStatus)
    {

      if (fermenters[i].input != -127)
      {
        fermenters[i].input = fermenters[i].movingAvg.push(fermenters[i].input).get();

        // Compute PID output only every 30 seconds
        if (fermenters[i].pidIntervalCounter + 1 >= pid_T)
        {
          fermenters[i].pid.Compute();
          fermenters[i].pidIntervalCounter = 0;
          if (fermenters[i].output < 2)
          {
            fermenters[i].output = 0;
          }
        }
        else
        {
          fermenters[i].pidIntervalCounter++;
        }

        // Turn the output pin on/off based on PID output
        if (millis() - fermenters[i].windowStartTime > WindowSize * 1000)
        {
          fermenters[i].windowStartTime += WindowSize * 1000;
        }
        if (fermenters[i].output * 1000 < millis() - fermenters[i].windowStartTime)
        {
          digitalWrite(fermenters[i].relayPin, HIGH);
          fermenters[i].pumpStatus = false;
        }
        else
        {
          digitalWrite(fermenters[i].relayPin, LOW);
          fermenters[i].pumpStatus = true;
        }
      }
      else { //Make sure we adjust the windowStartTime even if the temp reading is -127.
      if (millis() - fermenters[i].windowStartTime > WindowSize * 1000)
        {
          fermenters[i].windowStartTime += WindowSize * 1000;
        }
        }
    }
    else
    {
      fermenters[i].windowStartTime = millis();
      // I need to find a way to managed the Next Pump Cycle Time when the temp reading is -127.
    }
  }
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

  // Read temperatures
  for (int i = 0; i < 4; ++i)
  {
    fermenters[i].input = sensors.getTempC(fermenters[i].tc);
  }

  // Calling requestTemperatures() here makes sure we leave enough time for the temperature conversion (1 second).
  sensors.requestTemperatures();

  // Execute all fermenters pid
  handleFermenterPIDLoop();

  updateNextPumpCycleTime();

  // Send data to websocket
  if (WiFi.status() == WL_CONNECTED) {
  for (int i = 0; i < 4; ++i)
  {
    String json = fermenters[i].toJson();
    webSocket.broadcastTXT(json);
  }
  }
  else{
    printf("No internet.\n");
  }
}