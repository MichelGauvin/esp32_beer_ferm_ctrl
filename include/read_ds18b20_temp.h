// read_ds18b20_temp.h
#ifndef READ_DS18B20_TEMP_H
#define READ_DS18B20_TEMP_H

#include <OneWire.h>
#include <DallasTemperature.h>
#include "fermenter_control.h"

#define ONE_WIRE_BUS 4

extern OneWire oneWire;
extern DallasTemperature sensors;

void readTemperatures(Fermenter fermenters[], int count, DallasTemperature &sensors);

#endif // TEMPERATURE_READING_H