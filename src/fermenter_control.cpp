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

void setupPID(Fermenter* fermenters, int fermenterCount) {
  for (int i = 0; i < fermenterCount; ++i) {
    fermenters[i].pid.SetMode(AUTOMATIC);
    fermenters[i].windowStartTime = millis();
    fermenters[i].pid.SetOutputLimits(0, WindowSize / 10); // Adjust as needed
    fermenters[i].setpoint = storage.readSetpoint(i);
    fermenters[i].fermStatus = storage.readFermStatus(i);
    // Tell the PID to range between 0 and the full window size
    fermenters[0].pid.SetOutputLimits(0, WindowSize / 10);  // Limit maximum Ferm1_Output to 30 seconds every 5 minutes
    fermenters[1].pid.SetOutputLimits(0, WindowSize / 1);  // Limit maximum Ferm2_Output to 300 seconds every 5 minutes
    fermenters[2].pid.SetOutputLimits(0, WindowSize / 1); // Limit maximum Ferm3_Output to 30 seconds every 5 minutes
    fermenters[3].pid.SetOutputLimits(0, WindowSize / 10); // Limit maximum Ferm4_Output to 30 seconds every 5 minutes
  }
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

void handlePIDLoop(Fermenter* fermenters, int fermenterCount) {
  for (int i = 0; i < fermenterCount; ++i) {
    if (fermenters[i].fermStatus) {
      if (fermenters[i].input != -127) {
        fermenters[i].input = fermenters[i].movingAvg.push(fermenters[i].input).get();

        if (fermenters[i].pidIntervalCounter + 1 >= pid_T) {
          fermenters[i].pid.Compute();
          fermenters[i].pidIntervalCounter = 0;
          if (fermenters[i].output < 2) fermenters[i].output = 0;
        } else {
          fermenters[i].pidIntervalCounter++;
        }

        if (millis() - fermenters[i].windowStartTime > WindowSize * 1000) {
          fermenters[i].windowStartTime += WindowSize * 1000;
        }

        if (fermenters[i].output * 1000 < millis() - fermenters[i].windowStartTime) {
          updateRelayStatus(i, true);
        } else {
          updateRelayStatus(i, false);
        }
      }
    } else {
      fermenters[i].windowStartTime = millis();
    }
  }
  updateNextPumpCycleTime();
}