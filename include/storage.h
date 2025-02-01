#ifndef STORAGE_H
#define STORAGE_H

#include <Preferences.h>

class Storage {
public:
  Storage();
  ~Storage();

  void begin();
  double readSetpoint(int index);
  bool readFermStatus(int index);
  int saveSetpoint(int index, double value);
  int saveFermStatus(int index, bool value);

private:
  Preferences preferences;
  static const char* SETPOINT_KEYS[4];
  static const char* FERMSTATUS_KEYS[4];
  static const char* PREF_NAMESPACE;
};

#endif