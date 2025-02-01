#ifndef RELAY_CONTROL_H
#define RELAY_CONTROL_H

#define RELAY1 16
#define RELAY2 17
#define RELAY3 18
#define RELAY4 19

void setupRelays();
void updateRelayStatus(int fermenterIndex, bool status);

extern struct Fermenter fermenters[4];

#endif
