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
#include <MovingAverage.h>
#include <SPI.h>
#include <PID_v1.h>
#include "EEPROM.h"
#include <ESP8266WiFi.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ESPAsyncWiFiManager.h>
#include <ArduinoJson.h>

#define RELAY1 12
#define RELAY2 13
#define RELAY3 14
#define RELAY4 15

#define ONE_WIRE_BUS 4

// These are the mandatory persistent values
// If the controller reboot, it will start from the last known state
#define EEPROM_ADDR_SETPOINT1 0
#define EEPROM_ADDR_SETPOINT2 4
#define EEPROM_ADDR_SETPOINT3 8
#define EEPROM_ADDR_SETPOINT4 12
#define EEPROM_ADDR_FERM1STATUS 16
#define EEPROM_ADDR_FERM2STATUS 17
#define EEPROM_ADDR_FERM3STATUS 18
#define EEPROM_ADDR_FERM4STATUS 19

//Test pour recevoir un paramètre.
const char* PARAM_INPUT_1 = "input1";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
DNSServer dns;

//************************ PID DEFINITION *************************
// Define Variables we'll be connecting to
double ferm1_Setpoint, ferm1_Input, ferm1_Output;
double ferm2_Setpoint, ferm2_Input, ferm2_Output;
double ferm3_Setpoint, ferm3_Input, ferm3_Output;
double ferm4_Setpoint, ferm4_Input, ferm4_Output;

bool ferm1_Status, ferm2_Status, ferm3_Status, ferm4_Status;

// Specify the links and initial tuning parameters
// double Kp = 220, Ki = 2.5, Kd = 0;

// Petit fermenteur
double Kp = 70, Ki = 1, Kd = 0;
// Gros fermenteur 2 double Kp = 150, Ki = 1, Kd = 0; // J'ai mis le proportionelle a 150 pour le gros fermenteur.
double Kp2 = 150, Ki2 = 1, Kd2 = 0;

PID ferm1_pid(&ferm1_Input, &ferm1_Output, &ferm1_Setpoint, Kp, Ki, Kd, REVERSE);
PID ferm2_pid(&ferm2_Input, &ferm2_Output, &ferm2_Setpoint, Kp2, Ki2, Kd2, REVERSE);
PID ferm3_pid(&ferm3_Input, &ferm3_Output, &ferm3_Setpoint, Kp, Ki, Kd, REVERSE);
PID ferm4_pid(&ferm4_Input, &ferm4_Output, &ferm4_Setpoint, Kp, Ki, Kd, REVERSE);

unsigned WindowSize = 300; // In seconds

unsigned long ferm1_windowStartTime;
unsigned long ferm2_windowStartTime;
unsigned long ferm3_windowStartTime;
unsigned long ferm4_windowStartTime;

int ferm1_pid_i = 0;  // Iterator for interval execution
int ferm2_pid_i = 0;  // Iterator for interval execution
int ferm3_pid_i = 0;  // Iterator for interval execution
int ferm4_pid_i = 0;  // Iterator for interval execution
const int pid_T = 30; // PID computation interval in seconds
//************************ END PID **********************************

//************************ TEMPERATURE DEFINITION********************
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

uint8_t TC1[8] = {0x28, 0x80, 0x5E, 0xD0, 0x4F, 0x20, 0x01, 0x33};
uint8_t TC2[8] = {0x28, 0xFC, 0x9E, 0x03, 0x00, 0x00, 0x80, 0x00}; // N'est pas branché
uint8_t TC3[8] = {0x28, 0xFF, 0x3F, 0xE2, 0x69, 0x18, 0x03, 0xBE}; //
uint8_t TC4[8] = {0x28, 0xFF, 0x96, 0xE7, 0x6F, 0x18, 0x01, 0x92};
uint8_t TC5[8] = {0x28, 0xFF, 0x34, 0xF8, 0x6F, 0x18, 0x01, 0x80};

MovingAverage<float, 8> ferm1MovAvg;
MovingAverage<float, 8> ferm2MovAvg;
MovingAverage<float, 8> ferm3MovAvg;
MovingAverage<float, 8> ferm4MovAvg;
unsigned long tc_read_interval_millis = 1000;
unsigned long tc_read_last_millis;
bool read_tc;
//************************ END TEMPERATURE DEFINITION ***************

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

//*********** COMMUNICATION DATA STRUCTURE DEFINITION ***************
/**
  Create a data structure for transmitting and receiving data
  This allows many variables to be easily sent and received in a single transmission
  See http://www.cplusplus.com/doc/tutorial/structures/
*/

struct FermenterData
{
  long identifier;
  float setpoint;
  float tc1TempC;
  float tc2TempC;
  double output;
  float nextPumpCycleTime;
  bool pumpStatus;
  bool fermStatus;

  // Method to serialize to JSON
  String toJson() {
    StaticJsonDocument<200> doc; // Adjust the size as per your data complexity
    doc["setpoint"] = setpoint;
    doc["tc1TempC"] = tc1TempC;
    doc["tc2TempC"] = tc2TempC;
    doc["output"] = output;
    doc["nextPumpCycleTime"] = nextPumpCycleTime;
    doc["pumpStatus"] = pumpStatus;
    doc["fermStatus"] = fermStatus;
    
    String output;
    serializeJson(doc, output);
    return output;
  }
};
FermenterData ferm1Data;
FermenterData ferm2Data;
FermenterData ferm3Data;
FermenterData ferm4Data;

struct ControlData
{
  double ferm1_setpoint;
  double ferm2_setpoint;
  double ferm3_setpoint;
  double ferm4_setpoint;
  bool ferm1_status;
  bool ferm2_status;
  bool ferm3_status;
  bool ferm4_status;
};

ControlData controlData;

//*********** END COMMUNICATION DATA STRUCTURE DEFINITION *************

//*********** FIRMWARE VARIABLES DEFINITIONS **************************
const long interval = 1000; // interval between temperature reading (milliseconds)

float TC1_tempC = 10;
float TC2_tempC = 10;
float TC3_tempC = 10;
float TC4_tempC = 10;
float TC5_tempC = 10;
float TC6_tempC = 10;

float ferm1AvgTemp = 0;
float ferm2AvgTemp = 0;
float ferm3AvgTemp = 0;
float ferm4AvgTemp = 0;

int i = 0;
//*********** END FIRMWARE VARIABLES DEFINITIONS **********************

//
// Payload
//

const int min_payload_size = 4;
const int max_payload_size = 32;
const int payload_size_increments_by = 1;
int next_payload_size = min_payload_size;

char receive_payload[max_payload_size + 1]; // +1 to allow room for a terminating NULL char

void setup()
{
  //************************ SERIAL COMM SETUP ************************
  Serial.begin(115200);
  Serial.println(" SETUP ");
  //************************ END SERIAL COMM SETUP ********************

  //************************ WIFI SETUP *******************************
  AsyncWiFiManager wifiManager(&server, &dns);
  wifiManager.autoConnect("AutoConnectAP");
  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
  //************************ END WIFI SETUP ***************************

  //************************ START LITTLEFS SETUP *********************
  if (!LittleFS.begin ()) {
    Serial.println ("An Error has occurred while mounting LittleFS");
    return;
  }

  Dir dir = LittleFS.openDir ("");
    while (dir.next ()) {
      Serial.println ("Here the files:\r");
      Serial.println (dir.fileName ());
      Serial.println (dir.fileSize ());
    }
  //************************ END LITTLEFS SETUP ***********************

  //************************ START Route for root / web page *********/
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(LittleFS, "/index.html");
  });

  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest * request) {
    char str[20];
    sprintf(str, "%f", ferm1Data.tc1TempC);
    String temp1 = String(str);
    request->send_P(200, "text/plain", temp1.c_str());
  });
  //************************ END Route for root / web page ************

  // Send a GET request to <ESP_IP>/get?input1=<inputMessage> *********
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    String inputParam;
    
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      inputParam = PARAM_INPUT_1;
    }
    
    Serial.println(inputMessage);
    
    request->send(200, "text/html", "HTTP GET request sent to your ESP on input field (" 
                                     + inputParam + ") with value: " + inputMessage +
                                     "<br><a href=\"/\">Return to Home Page</a>");
    
  });

  server.on("/ferm1data", HTTP_GET, [](AsyncWebServerRequest *request) {
  String json = "{\"setpoint\": " + String(ferm1Data.setpoint, 1) + 
                ", \"tc1TempC\": " + String(ferm1Data.tc1TempC, 1) +
                ", \"output\": " + String(ferm1Data.output) +
                ", \"nextPumpCycleTime\": " + String(ferm1Data.nextPumpCycleTime) +
                ", \"pumpStatus\": " + String(ferm1Data.pumpStatus) + "}";
  request->send(200, "application/json", json);
});
  // ******************************************************************
  server.begin();

  //************************ PID SETUP ********************************
  ferm1_windowStartTime = millis();
  ferm2_windowStartTime = millis();
  ferm3_windowStartTime = millis();
  ferm4_windowStartTime = millis();

  // initialize the variables we're linked to

  // Pour modifier manuellement le status et le setpoint des fermenteurs
  // dans le cas ou le RPI ne fonctionne pas.
  ferm1_Status = true;
  ferm2_Status = true;
  ferm3_Status = true;
  ferm4_Status = true;

  ferm1_Setpoint = 20;
  ferm2_Setpoint = 20;
  ferm3_Setpoint = 20;
  ferm4_Setpoint = 20;

  EEPROM.put(EEPROM_ADDR_SETPOINT1, ferm1_Setpoint);
  EEPROM.put(EEPROM_ADDR_SETPOINT2, ferm2_Setpoint);
  EEPROM.put(EEPROM_ADDR_SETPOINT3, ferm3_Setpoint);
  EEPROM.put(EEPROM_ADDR_SETPOINT4, ferm4_Setpoint);

  EEPROM.put(EEPROM_ADDR_FERM1STATUS, ferm1_Status);
  EEPROM.put(EEPROM_ADDR_FERM2STATUS, ferm2_Status);
  EEPROM.put(EEPROM_ADDR_FERM3STATUS, ferm3_Status);
  EEPROM.put(EEPROM_ADDR_FERM4STATUS, ferm4_Status);

  // Fin de la configuration manuelle.

  EEPROM.get(EEPROM_ADDR_SETPOINT1, ferm1_Setpoint);
  EEPROM.get(EEPROM_ADDR_SETPOINT2, ferm2_Setpoint);
  EEPROM.get(EEPROM_ADDR_SETPOINT3, ferm3_Setpoint);
  EEPROM.get(EEPROM_ADDR_SETPOINT4, ferm4_Setpoint);

  EEPROM.get(EEPROM_ADDR_FERM1STATUS, ferm1_Status);
  EEPROM.get(EEPROM_ADDR_FERM2STATUS, ferm2_Status);
  EEPROM.get(EEPROM_ADDR_FERM3STATUS, ferm3_Status);
  EEPROM.get(EEPROM_ADDR_FERM4STATUS, ferm4_Status);

  // tell the PID to range between 0 and the full window size
  ferm1_pid.SetOutputLimits(0, WindowSize / 10); // Limit maximum Ferm1_Output to 30 seconds every 5 minutes
  ferm2_pid.SetOutputLimits(0, WindowSize / 1);  // Limit maximum Ferm2_Output to 300 seconds every 5 minutes
  ferm3_pid.SetOutputLimits(0, WindowSize / 10); // Limit maximum Ferm3_Output to 30 seconds every 5 minutes
  ferm4_pid.SetOutputLimits(0, WindowSize / 10); // Limit maximum Ferm4_Output to 30 seconds every 5 minutes

  // turn the PID on
  ferm1_pid.SetMode(AUTOMATIC);
  ferm2_pid.SetMode(AUTOMATIC);
  ferm3_pid.SetMode(AUTOMATIC);
  ferm4_pid.SetMode(AUTOMATIC);

  //************************ END PID SETUP*****************************
  Serial.println(" SETUP_1 ");
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
  tc_read_last_millis = millis();
  read_tc = false;

  //************************ END TEMPERATURE READING SETUP ****************
  Serial.println(" ENDSETUP ");
}

void loop()
{

  currentMillis = millis();

  while ((currentMillis - previousMillis) < interval)
  {
    // Execute all other code between two temperature measurement.
    // Put the radio in listening mode, and verify for new ferm1_Setpoint.
    delay(500);
    currentMillis = millis();

    // Read TC every tc_read_interval_millis = X ms
    if ((millis() - tc_read_last_millis) >= tc_read_interval_millis)
    {
      read_tc = true;
      tc_read_last_millis = millis();
    }
  }

  previousMillis = currentMillis;

  ferm1Data.setpoint = ferm1_Setpoint;
  ferm2Data.setpoint = ferm2_Setpoint;
  ferm3Data.setpoint = ferm3_Setpoint;
  ferm4Data.setpoint = ferm4_Setpoint;

  if (read_tc == true)
  {
    ferm1Data.tc1TempC = sensors.getTempC(TC1);
    ferm1Data.tc2TempC = sensors.getTempC(TC2);
    ferm2Data.tc1TempC = sensors.getTempC(TC4); // On utilise ce TC pour le ferm2 puisque l'autre ne fonctionne plus bien.
    ferm3Data.tc1TempC = sensors.getTempC(TC3);
    ferm4Data.tc1TempC = sensors.getTempC(TC5);
    read_tc = false;
    // Calling requestTemperatures() here makes sure we leave enough time for the temperature conversion (1 second).
    sensors.requestTemperatures();
  }

  //************************ FERMENTER 1 PID LOOP *************************
  if (ferm1_Status == true)
  {
    if (ferm1Data.tc1TempC == -127)
    {
      // Do nothing
    }
    else
    {

      ferm1AvgTemp = ferm1MovAvg.add(ferm1Data.tc1TempC);
      ferm1_Input = ferm1AvgTemp;

      /************************************************
        Compute PID Ferm1_Output only every 30 seconds
      ************************************************/
      if (ferm1_pid_i + 1 >= pid_T)
      {
        ferm1_pid.Compute();
        ferm1_pid_i = 0;
        if (ferm1_Output < 2)
        {
          ferm1_Output = 0;
        }
      }
      else
      {
        ferm1_pid_i++;
      }

      /************************************************
        turn the ferm1_Output pin on/off based on pid ferm1_Output
      ************************************************/
      if (millis() - ferm1_windowStartTime > WindowSize * 1000)
      { // time to shift the Relay Window
        ferm1_windowStartTime += WindowSize * 1000;
      }
      if (ferm1_Output * 1000 < millis() - ferm1_windowStartTime)
      {
        digitalWrite(RELAY1, 1);
        ferm1Data.pumpStatus = false;
      }
      else
      {
        digitalWrite(RELAY1, 0);
        ferm1Data.pumpStatus = true;
      }
    }
  }
  else{
    ferm1_windowStartTime = millis();
  }
  //************************ END FERMENTER 1 PID LOOP *********************

  //************************ FERMENTER 2 PID LOOP *************************
  if (ferm2_Status == true)
  {
    if (ferm2Data.tc1TempC == -127)
    {
      // Do nothing
    }
    else
    {

      ferm2AvgTemp = ferm2MovAvg.add(ferm2Data.tc1TempC);
      ferm2_Input = ferm2AvgTemp;

      /************************************************
        Compute PID ferm2_Output only every 30 seconds
      ************************************************/
      if (ferm2_pid_i + 1 >= pid_T)
      {
        ferm2_pid.Compute();
        ferm2_pid_i = 0;
        if (ferm2_Output < 2)
        {
          ferm2_Output = 0;
        }
      }
      else
      {
        ferm2_pid_i++;
      }

      /************************************************
        turn the ferm1_Output pin on/off based on pid ferm1_Output
      ************************************************/
      if (millis() - ferm2_windowStartTime > WindowSize * 1000)
      { // time to shift the Relay Window
        ferm2_windowStartTime += WindowSize * 1000;
      }
      if (ferm2_Output * 1000 < millis() - ferm2_windowStartTime)
      {
        digitalWrite(RELAY2, 1);
        ferm2Data.pumpStatus = false;
      }
      else
      {
        digitalWrite(RELAY2, 0);
        ferm2Data.pumpStatus = true;
      }
    }
  }
  else{
    ferm2_windowStartTime = millis();
  }
  //************************ END FERMENTER 2 PID LOOP *********************

  //************************ FERMENTER 3 PID LOOP *************************
  if (ferm3_Status == true)
  {
    if (ferm3Data.tc1TempC == -127)
    {
      // Do nothing
    }
    else
    {

      ferm3AvgTemp = ferm3MovAvg.add(ferm3Data.tc1TempC);
      ferm3_Input = ferm3AvgTemp;

      /************************************************
        Compute PID Ferm3_Output only every 30 seconds
      ************************************************/
      if (ferm3_pid_i + 1 >= pid_T)
      {
        ferm3_pid.Compute();
        ferm3_pid_i = 0;
        if (ferm3_Output < 2)
        {
          ferm3_Output = 0;
        }
      }
      else
      {
        ferm3_pid_i++;
      }

      /************************************************
        turn the ferm3_Output pin on/off based on pid ferm3_Output
      ************************************************/
      if (millis() - ferm3_windowStartTime > WindowSize * 1000)
      { // time to shift the Relay Window
        ferm3_windowStartTime += WindowSize * 1000;
      }
      if (ferm3_Output * 1000 < millis() - ferm3_windowStartTime)
      {
        digitalWrite(RELAY3, 1);
        ferm3Data.pumpStatus = false;
      }
      else
      {
        digitalWrite(RELAY3, 0);
        ferm3Data.pumpStatus = true;
      }
    }
  }
  else{
    ferm3_windowStartTime = millis();
  }
  //************************ END FERMENTER 3 PID LOOP *********************

  //************************ FERMENTER 4 PID LOOP *************************
  if (ferm4_Status == true)
  {
    if (ferm4Data.tc1TempC == -127)
    {
      // Do nothing
    }
    else
    {

      ferm4AvgTemp = ferm4MovAvg.add(ferm4Data.tc1TempC);
      ferm4_Input = ferm4AvgTemp;

      /************************************************
        Compute PID ferm4_Output only every 30 seconds
      ************************************************/
      if (ferm4_pid_i + 1 >= pid_T)
      {
        ferm4_pid.Compute();
        ferm4_pid_i = 0;
        if (ferm4_Output < 2)
        {
          ferm4_Output = 0;
        }
      }
      else
      {
        ferm4_pid_i++;
      }

      /************************************************
        turn the ferm4_Output pin on/off based on pid ferm4_Output
      ************************************************/
      if (millis() - ferm4_windowStartTime > WindowSize * 1000)
      { // time to shift the Relay Window
        ferm4_windowStartTime += WindowSize * 1000;
      }
      if (ferm4_Output * 1000 < millis() - ferm4_windowStartTime)
      {
        digitalWrite(RELAY4, 1);
        ferm4Data.pumpStatus = false;
      }
      else
      {
        digitalWrite(RELAY4, 0);
        ferm4Data.pumpStatus = true;
      }
    }
  }
  else{
    ferm4_windowStartTime = millis();
  }
  //************************ END FERMENTER 4 PID LOOP *********************

  //
  //   Send data to RPI every X seconds
  //

  ferm1Data.output = ferm1_Output;
  ferm2Data.output = ferm2_Output;
  ferm3Data.output = ferm3_Output;
  ferm4Data.output = ferm4_Output;

  // The +500 is to round to nearest integer
  if (ferm1_Status == true)
  {
    ferm1Data.nextPumpCycleTime = ((WindowSize * 1000 - (millis() - ferm1_windowStartTime)) + 500) / 1000;
  }
  else
  {
    ferm1Data.nextPumpCycleTime = 0;
    ferm1Data.output = 0;
    // TODO If pump is "ON" turn it "OFF".
    // This will also check for a master pump control in the future, the logic will also be implemented in the pid loop
    // Master pump control let the operator to force the pump ON for the desired duration via web command
  }
  if (ferm2_Status == true)
  {
    ferm2Data.nextPumpCycleTime = (WindowSize * 1000 - (millis() - ferm2_windowStartTime)) / 1000;
  }
  else
  {
    ferm2Data.nextPumpCycleTime = 0;
    ferm2Data.output = 0;
    // TODO If pump is "ON" turn it "OFF".
    // This will also check for a master pump control in the future, the logic will also be implemented in the pid loop
    // Master pump control let the operator to force the pump ON for the desired duration
  }
  if (ferm3_Status == true)
  {
    ferm3Data.nextPumpCycleTime = (WindowSize * 1000 - (millis() - ferm3_windowStartTime)) / 1000;
  }
  else
  {
    ferm3Data.nextPumpCycleTime = 0;
    ferm3Data.output = 0;
    // TODO If pump is "ON" turn it "OFF".
    // This will also check for a master pump control in the future, the logic will also be implemented in the pid loop
    // Master pump control let the operator to force the pump ON for the desired duration
  }
  if (ferm4_Status == true)
  {
    ferm4Data.nextPumpCycleTime = (WindowSize * 1000 - (millis() - ferm4_windowStartTime)) / 1000;
  }
  else
  {
    ferm4Data.nextPumpCycleTime = 0;
    ferm4Data.output = 0;
    // TODO If pump is "ON" turn it "OFF".
    // This will also check for a master pump control in the future, the logic will also be implemented in the pid loop
    // Master pump control let the operator to force the pump ON for the desired duration
  }

  // First, stop listening so we can talk.
  // radio.stopListening();

  // Send data for the 4 fermenters
  /*
  String message1 = "Fermenteur 1: Setpoint = " + (String)ferm1Data.setpoint + "\tTC1= " + (String)ferm1Data.tc1TempC + "\tOutput = " + (String)ferm1Data.output + "\tnextPumpCycleTime = " + ferm1Data.nextPumpCycleTime + "\tpumpStatus = " + (String)ferm1Data.pumpStatus;
  String message2 = "Fermenteur 2: Setpoint = " + (String)ferm2Data.setpoint + "\tTC2= " + (String)ferm2Data.tc1TempC + "\tOutput = " + (String)ferm2Data.output + "\tnextPumpCycleTime = " + ferm2Data.nextPumpCycleTime + "\tpumpStatus = " + (String)ferm2Data.pumpStatus;
  String message3 = "Fermenteur 3: Setpoint = " + (String)ferm3Data.setpoint + "\tTC3= " + (String)ferm3Data.tc1TempC + "\tOutput = " + (String)ferm3Data.output + "\tnextPumpCycleTime = " + ferm3Data.nextPumpCycleTime + "\tpumpStatus = " + (String)ferm3Data.pumpStatus;
  String message4 = "Fermenteur 4: Setpoint = " + (String)ferm4Data.setpoint + "\tTC4= " + (String)ferm4Data.tc1TempC + "\tOutput = " + (String)ferm4Data.output + "\tnextPumpCycleTime = " + ferm4Data.nextPumpCycleTime + "\tpumpStatus = " + (String)ferm4Data.pumpStatus;

  String message = "****************************\r\n" + message1 + "\r\n" + message2 + "\r\n" + message3 + "\r\n" + message4;
  Serial.println(message);
*/
/*
  EEPROM.put(EEPROM_ADDR_SETPOINT1, controlData.ferm1_setpoint);
  EEPROM.put(EEPROM_ADDR_SETPOINT2, controlData.ferm2_setpoint);
  EEPROM.put(EEPROM_ADDR_SETPOINT3, controlData.ferm3_setpoint);
  EEPROM.put(EEPROM_ADDR_SETPOINT4, controlData.ferm4_setpoint);
  EEPROM.put(EEPROM_ADDR_FERM1STATUS, controlData.ferm1_status);
  EEPROM.put(EEPROM_ADDR_FERM2STATUS, controlData.ferm2_status);
  EEPROM.put(EEPROM_ADDR_FERM3STATUS, controlData.ferm3_status);
  EEPROM.put(EEPROM_ADDR_FERM4STATUS, controlData.ferm4_status);
  EEPROM.get(EEPROM_ADDR_SETPOINT1, ferm1_Setpoint);
  EEPROM.get(EEPROM_ADDR_SETPOINT2, ferm2_Setpoint);
  EEPROM.get(EEPROM_ADDR_SETPOINT3, ferm3_Setpoint);
  EEPROM.get(EEPROM_ADDR_SETPOINT4, ferm4_Setpoint);
  EEPROM.get(EEPROM_ADDR_FERM1STATUS, ferm1_Status);
  EEPROM.get(EEPROM_ADDR_FERM2STATUS, ferm2_Status);
  EEPROM.get(EEPROM_ADDR_FERM3STATUS, ferm3_Status);
  EEPROM.get(EEPROM_ADDR_FERM4STATUS, ferm4_Status);
  ferm1Data.fermStatus = ferm1_Status;
  ferm2Data.fermStatus = ferm2_Status;
  ferm3Data.fermStatus = ferm3_Status;
  ferm4Data.fermStatus = ferm4_Status;
  */
}
