#include "read_ds18b20_temp.h"
#include "fermenter_control.h"

extern OneWire oneWire;
extern DallasTemperature sensors;
extern struct Fermenter fermenters[4];  // Array of fermenters (assuming 4 fermenters)

void readTemperatures(Fermenter fermenters[], int count, DallasTemperature &sensors) {
    for (int i = 0; i < count; ++i) {
        fermenters[i].input = sensors.getTempC(fermenters[i].tc);
    }
    // Demande de conversion de température
    sensors.requestTemperatures();
}