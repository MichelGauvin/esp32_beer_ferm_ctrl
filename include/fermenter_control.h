#ifndef FERMENTER_CONTROL_H
#define FERMENTER_CONTROL_H

#include <Arduino.h>
#include "storage.h"
#include <PID_v1.h>
#include <MovingAveragePlus.h>
#include <ArduinoJson.h>

#define pid_T 30 // PID computation interval in seconds
#define WindowSize 300 // Each 5 minutes the pump starts for a duration dertermined by the PID

extern Storage storage;
extern struct Fermenter fermenters[4];  // Array of fermenters (assuming 4 fermenters)

void setPumpColdCrash(const char* str_value, int fermIndex);
void updateStatus(const char* key, const char* str_value, int fermIndex);
void updateSetpoint(const char* key, const char* str_value, int fermIndex);
void setupPID(Fermenter* fermenters, int fermenterCount);
void updateNextPumpCycleTime();
void handlePIDLoop(Fermenter* fermenters, int fermenterCount);

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

#endif // FERMENTER_CONTROL_H
