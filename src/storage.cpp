#include "storage.h"

// Definition of keys and namespace
const char* Storage::SETPOINT_KEYS[4] = { "setpoint1", "setpoint2", "setpoint3", "setpoint4" };
const char* Storage::FERMSTATUS_KEYS[4] = { "fermStatus1", "fermStatus2", "fermStatus3", "fermStatus4" };
const char* Storage::PREF_NAMESPACE = "ferment";

Storage::Storage() {
  // Constructor, no specific initialization here
}

Storage::~Storage() {
  // Destructor, may close preferences if necessary
}

void Storage::begin() {
  preferences.begin(PREF_NAMESPACE, false);  // Initialize preferences
}

double Storage::readSetpoint(int index) {
  return preferences.getDouble(SETPOINT_KEYS[index], 25.0);  // Default value 25.0
}

bool Storage::readFermStatus(int index) {
  return preferences.getBool(FERMSTATUS_KEYS[index], false);  // Default value false
}

int Storage::saveSetpoint(int index, double value) {
  if (index < 0 || index >= 4) {
    Serial.println("Error: invalid setpoint index");
    return -1;  // Error
  }
  if (isnan(value)) {
    Serial.println("Error: invalid value for setpoint");
    return -1;  // Error
  }
  preferences.putDouble(SETPOINT_KEYS[index], value);
  return 0;
}

int Storage::saveFermStatus(int index, bool value) {
  if (index < 0 || index >= 4) {
    Serial.println("Error: invalid setpoint index");
    return -1;  // Error
  }
  preferences.putBool(FERMSTATUS_KEYS[index], value);
  return 0;
}
