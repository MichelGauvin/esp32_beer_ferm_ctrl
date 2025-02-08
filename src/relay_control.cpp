#include "relay_control.h"
#include <Arduino.h>
#include "fermenter_control.h"

extern struct Fermenter fermenters[4];

const int relayPins[4] = {RELAY1, RELAY2, RELAY3, RELAY4};

void updateRelayStatus(int fermenterIndex, bool status) {
  digitalWrite(relayPins[fermenterIndex], status ? LOW : HIGH);
}

void setupRelays() {
  for (int i = 0; i < 4; ++i) {
    pinMode(relayPins[i], INPUT_PULLUP);
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], HIGH);
    fermenters[i].pumpStatus = false;
  }
}
