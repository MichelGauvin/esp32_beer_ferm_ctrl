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
#include <Preferences.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ESPAsyncWiFiManager.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>
#include <esp_system.h>

#define RELAY1 16 //
#define RELAY2 17 //
#define RELAY3 18 //
#define RELAY4 19 //

#define ONE_WIRE_BUS 4 //

//Preferences handling
Preferences preferences;

#define PREF_NAMESPACE "ferment"

// Setpoint keys
const char* SETPOINT_KEYS[] = {
  "setpoint1",
  "setpoint2",
  "setpoint3",
  "setpoint4"
};

const char* FERMSTATUS_KEYS[] = {
  "fermStatus1",
  "fermStatus2",
  "fermStatus3",
  "fermStatus4"
};

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
DNSServer dns;

//************************ TEMPERATURE DEFINITION********************
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//************************ END TEMPERATURE DEFINITION ***************

//************************ PID DEFINITION****************************
// PID Constants and Variables

// Petit fermenteur
double Kp = 70, Ki = 1, Kd = 0;
// Gros fermenteur 2 double Kp = 150, Ki = 1, Kd = 0; // J'ai mis le proportionelle a 150 pour le gros fermenteur.
double Kp2 = 150, Ki2 = 1, Kd2 = 0;

struct Fermenter
{
  int fermenterId;
  double setpoint;
  double input;
  double output;
  bool fermStatus;
  bool pumpStatus;
  bool coldCrash;
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
    doc["coldCrash"] = coldCrash;

    String jsonOutput;
    serializeJson(doc, jsonOutput);
    return jsonOutput;
  };
};

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

// Functions to read, write setpoint and status value on disk
double getSetpoint(int index) {
  return preferences.getDouble(SETPOINT_KEYS[index], 20.0);
}

bool getFermStatus(int index) {
  return preferences.getBool(FERMSTATUS_KEYS[index], false);
}

int saveSetpoint(int index, double value) {
  if (index < 0 || index >= 4) {
    Serial.println("Error: Invalid setpoint index");
    return -1; // Return error
  }
  if (isnan(value)) {
    Serial.println("Error: Invalid value for setpoint");
    return -1; // Return error
  }
  preferences.putDouble(SETPOINT_KEYS[index], value);
  return 0;
}

int saveFermStatus(int index, bool value) {
  if (index < 0 || index >= 4) {
    Serial.println("Error: Invalid setpoint index");
    return -1; // Return error
  }  
  preferences.putBool(FERMSTATUS_KEYS[index], value);
  return 0;
}

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

void setPumpColdCrash(const char *str_value, bool &coldCrash, uint8_t relayPin, int fermIndex)
{
  printf("Cold crash %s, %d\n", str_value, relayPin);
  if (str_value == NULL)
  {
    printf("Error: str_value is NULL\n");
    return;
  }

  if (strcmp(str_value, "ON") == 0)
  {
    coldCrash = true;
    digitalWrite(relayPin, LOW);
    fermenters[fermIndex].pumpStatus = true;
  }
  else
  {
    coldCrash = false;
    digitalWrite(relayPin, HIGH);
    fermenters[fermIndex].pumpStatus = false;
  }
}

void updateStatus(const char *key, const char *str_value, bool &status, double &output, int &pid_i, int fermIndex)
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
  saveFermStatus(fermIndex, status);
}

void updateSetpoint(const char *key, const char *str_value, double &setpoint, int fermIndex)
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
  saveSetpoint(fermIndex, setpoint);
}

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

  // Initialize Preferences
  preferences.begin(PREF_NAMESPACE, false);

  //************************ SERIAL COMM SETUP ************************
  Serial.begin(115200);
  Serial.println(" SETUP ");
  //************************ END SERIAL COMM SETUP ********************

  //************************ WIFI SETUP *******************************
  //************************ WIFI SETUP *******************************
  AsyncWiFiManager wifiManager(&server, &dns);
  // Set the configuration portal timeout to 5 seconds
  wifiManager.setConfigPortalTimeout(120); // Timeout in seconds
  wifiManager.autoConnect("AutoConnectAP");
  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  /*
  WiFi.begin(ssid, password);

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Déconnecté du Wi-Fi. Tentative de reconnexion...");
    WiFi.begin(ssid, password); // Reconnexion
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 5) {
      delay(500);
      attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Reconnecté au Wi-Fi !");
      Serial.print("Adresse IP : ");
      Serial.println(WiFi.localIP());
    }
  }
  */
  //************************ WIFI SETUP *******************************

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
    int fermIndex = -1;

    // Determine the fermenter index based on the key_value
    if (strcmp(key_value, "ferm1_Status") == 0 || strcmp(key_value, "ferm1_Setpoint") == 0 || strcmp(key_value, "ferm1_ColdCrash") == 0) {
        fermIndex = 0;
    } else if (strcmp(key_value, "ferm2_Status") == 0 || strcmp(key_value, "ferm2_Setpoint") == 0 || strcmp(key_value, "ferm2_ColdCrash") == 0) {
        fermIndex = 1;
    } else if (strcmp(key_value, "ferm3_Status") == 0 || strcmp(key_value, "ferm3_Setpoint") == 0 || strcmp(key_value, "ferm3_ColdCrash") == 0) {
        fermIndex = 2;
    } else if (strcmp(key_value, "ferm4_Status") == 0 || strcmp(key_value, "ferm4_Setpoint") == 0 || strcmp(key_value, "ferm4_ColdCrash") == 0) {
        fermIndex = 3;
    }

    // If a valid fermenter index is found, process the status or setpoint
    if (fermIndex != -1) {
        if (strstr(key_value, "Status") != NULL) {
      // If we start or stop the fermenter we set the pump to OFF, the code will determine the later state
      // This allows us to stop the pump immediately when we stop the fermenter.
            updateStatus(key_value, str_value, fermenters[fermIndex].fermStatus, fermenters[fermIndex].output, fermenters[fermIndex].pidIntervalCounter, fermIndex);
            digitalWrite(fermenters[fermIndex].relayPin, HIGH);
            fermenters[fermIndex].pumpStatus = false;
        } else if (strstr(key_value, "Setpoint") != NULL) {
            updateSetpoint(key_value, str_value, fermenters[fermIndex].setpoint, fermIndex);
        } else if (strstr(key_value, "ColdCrash") != NULL) {
            setPumpColdCrash(str_value, fermenters[fermIndex].coldCrash, fermenters[fermIndex].relayPin, fermIndex);
        }
    }

    }
    
    request->send(200, "application/json", "{\"status\":\"success\",\"message\":\"JSON data received\"}"); });

  // ******************************************************************
  server.begin();

  //************************ PID SETUP ********************************
  for (int i = 0; i < 4; ++i)
  {
    fermenters[i].setpoint = getSetpoint(i);
    fermenters[i].fermStatus = getFermStatus(i);
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

  ////************************ Start the WebSocket server *******************
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println("WebSocket server started.");

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