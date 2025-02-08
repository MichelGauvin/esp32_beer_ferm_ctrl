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
