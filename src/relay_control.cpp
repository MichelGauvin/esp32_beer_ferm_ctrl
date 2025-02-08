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
