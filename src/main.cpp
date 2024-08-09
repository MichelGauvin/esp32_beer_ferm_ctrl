/*
  Arduino PID fermentation temperature Controller (x4)
  Made by Michel Gauvin
  www.mikesbrewshop.com

  TODO:
  - Master pump control let the operator to force the pump ON for the desired duration via web command
  - Implement a cold crash process

  BUG:
  - Il y a un bug avec le windows start time, on devrait le resetter lorsqu'on met le PID à "ON"
  Il est seulement initialisé lors du démarrage de l'application.
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <MovingAveragePlus.h>
#include <SPI.h>
#include <PID_v1.h>
#include "ESP_EEPROM.h"
#include <ESP8266WiFi.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ESPAsyncWiFiManager.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>

#define RELAY1 12
#define RELAY2 13
#define RELAY3 14
#define RELAY4 15

#define ONE_WIRE_BUS 4

// These are the mandatory persistent values
// If the controller reboot, it will start from the last known state
#define EEPROM_ADDR_SETPOINT1 0
#define EEPROM_ADDR_SETPOINT2 8
#define EEPROM_ADDR_SETPOINT3 16
#define EEPROM_ADDR_SETPOINT4 24
#define EEPROM_ADDR_FERM1STATUS 32
#define EEPROM_ADDR_FERM2STATUS 34
#define EEPROM_ADDR_FERM3STATUS 36
#define EEPROM_ADDR_FERM4STATUS 38

// const char* ssid = "Michel";
// const char* password = "Mikego20";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
DNSServer dns;

//************************ TEMPERATURE DEFINITION********************
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

const uint8_t TC1[8] = {0x28, 0x80, 0x5E, 0xD0, 0x4F, 0x20, 0x01, 0x33};
const uint8_t TC2[8] = {0x28, 0xFC, 0x9E, 0x03, 0x00, 0x00, 0x80, 0x00}; // N'est pas branché
const uint8_t TC3[8] = {0x28, 0xFF, 0x3F, 0xE2, 0x69, 0x18, 0x03, 0xBE}; //
const uint8_t TC4[8] = {0x28, 0xFF, 0x96, 0xE7, 0x6F, 0x18, 0x01, 0x92};
const uint8_t TC5[8] = {0x28, 0xFF, 0x34, 0xF8, 0x6F, 0x18, 0x01, 0x80};

bool read_tc;
//************************ END TEMPERATURE DEFINITION ***************

//************************ PID DEFINITION****************************
// PID Constants and Variables

// Petit fermenteur
double Kp = 70, Ki = 1, Kd = 0;
// Gros fermenteur 2 double Kp = 150, Ki = 1, Kd = 0; // J'ai mis le proportionelle a 150 pour le gros fermenteur.
double Kp2 = 150, Ki2 = 1, Kd2 = 0;

struct Fermenter {
  int fermenterId;
  double setpoint;
  double input;
  double output;
  bool fermStatus;
  bool pumpStatus;
  float nextPumpCycleTime;
  PID pid;
  MovingAveragePlus<float> movingAvg;
  unsigned long windowStartTime;
  int pidIntervalCounter;
  const uint8_t tc[8];
  int relayPin;

  // Method to serialize to JSON
  String toJson()
  {
    StaticJsonDocument<200> doc; // Adjust the size as per your data complexity
    doc["fermenterId"] = fermenterId;
    doc["setpoint"] = setpoint;
    doc["tc1TempC"] = input;
    doc["output"] = output;
    doc["nextPumpCycleTime"] = nextPumpCycleTime;
    doc["pumpStatus"] = pumpStatus;
    doc["fermStatus"] = fermStatus;

    String jsonOutput;
    serializeJson(doc, jsonOutput);
    return jsonOutput;
  };
};

Fermenter fermenters[4] = {
  {1, 0, 0, 0, false, false, 0, PID(&fermenters[0].input, &fermenters[0].output, &fermenters[0].setpoint, Kp, Ki, Kd, REVERSE), MovingAveragePlus<float>(5), 0, 0, {0x28, 0x80, 0x5E, 0xD0, 0x4F, 0x20, 0x01, 0x33}, RELAY1},
  {2, 0, 0, 0, false, false, 0, PID(&fermenters[1].input, &fermenters[1].output, &fermenters[1].setpoint, Kp2, Ki2, Kd2, REVERSE), MovingAveragePlus<float>(5), 0, 0, TC3[8], RELAY2},
  {3, 0, 0, 0, false, false, 0, PID(&fermenters[2].input, &fermenters[2].output, &fermenters[2].setpoint, Kp, Ki, Kd, REVERSE), MovingAveragePlus<float>(5), 0, 0, TC4[8], RELAY3},
  {4, 0, 0, 0, false, false, 0, PID(&fermenters[3].input, &fermenters[3].output, &fermenters[3].setpoint, Kp, Ki, Kd, REVERSE), MovingAveragePlus<float>(5), 0, 0, TC5[8], RELAY4}
};

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

//*********** Create a WebSocket server object on port 81 *************
WebSocketsServer webSocket = WebSocketsServer(3333);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", num);
    break;
  case WStype_CONNECTED:
  {
    IPAddress ip = webSocket.remoteIP(num);
    Serial.printf("[%u] Connection from ", num);
    Serial.println(ip.toString());

    // Send a welcome message to the client
    StaticJsonDocument<200> doc;
    doc["message"] = "Welcome to the WebSocket server!";
    String json;
    serializeJson(doc, json);
    webSocket.sendTXT(num, json);
  }
  break;
  case WStype_TEXT:
    Serial.printf("[%u] Received text: %s\n", num, payload);

    // Echo the received message back to the client
    webSocket.sendTXT(num, payload);
    break;
  }
}
//*********** END WebSocket server object on port 81 ******************

void updateStatus(const char *key, const char *str_value, bool &status, double &output, int &pid_i, int eepromAddress)
{
  if (str_value == NULL)
  {
    printf("Error: str_value is NULL\n");
    return;
  }

  if (strcmp(str_value, "ON") == 0)
  {
    status = true;
  }
  else
  {
    status = false;
    output = 0;
    pid_i = 0;
  }

  EEPROM.put(eepromAddress, status);
  EEPROM.commit();
}

void updateSetpoint(const char *key, const char *str_value, double &setpoint, int eepromAddress)
{
  if (str_value == NULL)
  {
    printf("Error: str_value is NULL\n");
    return;
  }

  char *end;
  double double_value = strtod(str_value, &end);
  if (*end != '\0')
  {
    printf("Conversion error, non-convertible part: %s\n", end);
  }
  setpoint = double_value;
  Serial.print(eepromAddress);
  EEPROM.put(eepromAddress, double_value);
  EEPROM.commit();
}

void setup()
{
  EEPROM.begin(64);

  //************************ SERIAL COMM SETUP ************************
  Serial.begin(115200);
  Serial.println(" SETUP ");
  //************************ END SERIAL COMM SETUP ********************

  //************************ WIFI SETUP *******************************
  AsyncWiFiManager wifiManager(&server, &dns);
  wifiManager.autoConnect("AutoConnectAP");
  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  /*
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  */
  //************************ END WIFI SETUP ***************************

  //************************ START LITTLEFS SETUP *********************
  if (!LittleFS.begin())
  {
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }

  Dir dir = LittleFS.openDir("");
  while (dir.next())
  {
    Serial.println("Here the files:\r");
    Serial.println(dir.fileName());
    Serial.println(dir.fileSize());
  }
  //************************ END LITTLEFS SETUP ***********************

  //************************ START Route for root / web page *********/
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/index.html"); });

  //************************ END Route for root / web page ************

  // Cette fonction gère les nouveaux setpoint et le démarrage et l'arrêt du fermenteur.
  server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
            {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, data);

    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
      return;
    }

    for (JsonPair kv : doc.as<JsonObject>()) {
    const char* key_value = kv.key().c_str();
    const char* str_value = kv.value().as<const char*>();
    Serial.print(key_value);
    Serial.print(" : ");
    Serial.print(str_value);
    Serial.print("\n");

      if (strcmp(key_value, "ferm1_Status") == 0) {
          updateStatus(key_value, str_value, fermenters[0].fermStatus, fermenters[0].output, fermenters[0].pidIntervalCounter, EEPROM_ADDR_FERM1STATUS);
      } else if (strcmp(key_value, "ferm1_Setpoint") == 0) {
          updateSetpoint(key_value, str_value, fermenters[0].setpoint, EEPROM_ADDR_SETPOINT1);
      } else if (strcmp(key_value, "ferm2_Status") == 0) {
          updateStatus(key_value, str_value, fermenters[1].fermStatus, fermenters[1].output, fermenters[1].pidIntervalCounter, EEPROM_ADDR_FERM2STATUS);
      } else if (strcmp(key_value, "ferm2_Setpoint") == 0) {
          updateSetpoint(key_value, str_value, fermenters[1].setpoint, EEPROM_ADDR_SETPOINT2);
      } else if (strcmp(key_value, "ferm3_Status") == 0) {
          updateStatus(key_value, str_value, fermenters[2].fermStatus, fermenters[2].output, fermenters[2].pidIntervalCounter, EEPROM_ADDR_FERM3STATUS);
      } else if (strcmp(key_value, "ferm3_Setpoint") == 0) {
          updateSetpoint(key_value, str_value, fermenters[2].setpoint, EEPROM_ADDR_SETPOINT3);
      } else if (strcmp(key_value, "ferm4_Status") == 0) {
          updateStatus(key_value, str_value, fermenters[3].fermStatus, fermenters[3].output, fermenters[3].pidIntervalCounter, EEPROM_ADDR_FERM4STATUS);
      } else if (strcmp(key_value, "ferm4_Setpoint") == 0) {
          updateSetpoint(key_value, str_value, fermenters[3].setpoint, EEPROM_ADDR_SETPOINT4);
      }
    }
    
    request->send(200, "application/json", "{\"status\":\"success\",\"message\":\"JSON data received\"}"); });

  // ******************************************************************
  server.begin();

  //************************ PID SETUP ********************************
  // Restore setpoints and status from EEPROM
  for (int i = 0; i < 4; ++i) {
    EEPROM.get(EEPROM_ADDR_SETPOINT1 + (i * 8), fermenters[i].setpoint);
    EEPROM.get(EEPROM_ADDR_FERM1STATUS + (i * 2), fermenters[i].fermStatus);
  }

  for (auto& fermenter : fermenters) {
    fermenter.pid.SetMode(AUTOMATIC);
    fermenter.windowStartTime = millis();
  }

  // Tell the PID to range between 0 and the full window size
  fermenters[0].pid.SetOutputLimits(0, WindowSize / 10); // Limit maximum Ferm1_Output to 30 seconds every 5 minutes
  fermenters[1].pid.SetOutputLimits(0, WindowSize / 1);  // Limit maximum Ferm2_Output to 300 seconds every 5 minutes
  fermenters[2].pid.SetOutputLimits(0, WindowSize / 10); // Limit maximum Ferm3_Output to 30 seconds every 5 minutes
  fermenters[3].pid.SetOutputLimits(0, WindowSize / 10); // Limit maximum Ferm4_Output to 30 seconds every 5 minutes

  
  //************************ END PID SETUP*****************************

  //************************ RELAY SETUP ******************************
  // We need to configure the pinMode to Ferm1_Input_PULLUP first
  // It avoids having a HIGH state when configuring the pinMode to Ferm1_Output.
  // IMPORTANT: WE INVERSE THE RELAY LOGIC BY DOING THIS
  // Issue #3 mikesbrewshop.com
  pinMode(RELAY1, INPUT_PULLUP);
  pinMode(RELAY1, OUTPUT);

  pinMode(RELAY2, INPUT_PULLUP);
  pinMode(RELAY2, OUTPUT);

  pinMode(RELAY3, INPUT_PULLUP);
  pinMode(RELAY3, OUTPUT);

  pinMode(RELAY4, INPUT_PULLUP);
  pinMode(RELAY4, OUTPUT);
  //************************ END RELAY SETUP ******************************

  //************************ TEMPERATURE READING SETUP ********************
  // Make asynchronous call to requestTempeature()
  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();

  ////************************ Start the WebSocket server *******************
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println("WebSocket server started.");

  //************************ END TEMPERATURE READING SETUP ****************
}

// Function to update next pump cycle time
void updateNextPumpCycleTime()
{
  for (int i = 0; i < 4; ++i) {
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
  for (int i = 0; i < 4; ++i) {
    if (fermenters[i].fermStatus) {
          
      if (fermenters[i].input != -127) {

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
    }
    else
    {
      fermenters[i].windowStartTime = millis();
      //I need to find a way to managed the Next Pump Cycle Time when the temp reading is -127.
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
    double test = sensors.getTempC(TC1);
    Serial.print("Temp: ");
    Serial.print(test);
    Serial.print("\n");
  }

  // Calling requestTemperatures() here makes sure we leave enough time for the temperature conversion (1 second).
  sensors.requestTemperatures();
  
  // Execute all fermenters pid
  handleFermenterPIDLoop();

  updateNextPumpCycleTime();

  // Send data to websocket
  for (int i = 0; i < 4; ++i)
  {
    String json = fermenters[i].toJson();
    webSocket.broadcastTXT(json);
  }
}