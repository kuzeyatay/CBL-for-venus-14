#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include <stdbool.h>

bool initVL53L0X(void);
int readVL53L0XDistance(void);

float scanObjectWidth(void);

#endif