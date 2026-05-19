#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include <stdbool.h>

bool initNTCTemperatureSensor(void);
float readNTCTemperature(void);

#endif