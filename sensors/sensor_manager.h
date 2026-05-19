#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "robot_types.h"

sensor_data_t readSensorData(void);
sensor_data_t readReliableSensorData(void);

#endif