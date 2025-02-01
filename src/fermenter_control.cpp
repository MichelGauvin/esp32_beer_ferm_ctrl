#include "fermenter_control.h"
#include <ArduinoJson.h>
#include "storage.h"
#include "relay_control.h"

extern Storage storage;
extern struct Fermenter fermenters[4];

// Function to set the cold crash status and pump status
void setPumpColdCrash(const char* str_value, int fermIndex)
{
  if (str_value == NULL)
  {
    printf("Error: str_value is NULL\n");
    return;
  }


  bool coldCrashStatus = (strcmp(str_value, "ON") == 0);
  fermenters[fermIndex].coldCrash = coldCrashStatus;
  
  // Control the pump based on cold crash status
  if (coldCrashStatus)
  {
    updateRelayStatus(fermIndex, false);
    fermenters[fermIndex].pumpStatus = true;
  }
  else
  {
    updateRelayStatus(fermIndex, true);
    fermenters[fermIndex].pumpStatus = false;
  }
}

// Function to update fermenter status
void updateStatus(const char* key, const char* str_value, int fermIndex)
{
  if (str_value == NULL)
  {
    printf("Error: str_value is NULL\n");
    return;
  }

  bool fermStatus = (strcmp(str_value, "ON") == 0);
  fermenters[fermIndex].fermStatus = fermStatus;

  if (!fermStatus)
  {
    fermenters[fermIndex].output = 0;
    fermenters[fermIndex].pidIntervalCounter = 0;
  }

  storage.saveFermStatus(fermIndex, fermStatus);
}

// Function to update setpoint for the fermenter
void updateSetpoint(const char* key, const char* str_value, int fermIndex)
{
  if (str_value == NULL)
  {
    printf("Error: str_value is NULL\n");
    return;
  }

  char* end;
  double double_value = strtod(str_value, &end);

  // Check for conversion errors
  if (*end != '\0')
  {
    printf("Conversion error, non-convertible part: %s\n", end);
    return;
  }

  fermenters[fermIndex].setpoint = double_value;
  storage.saveSetpoint(fermIndex, double_value);
}
