#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <stdbool.h>

#include "robot_types.h"

void tcs3200Init(void);
void tcs3200DebugOutPin(void);

bool isTCS3200Calibrated(void);
bool tcs3200LoadCalibrationFromBaseStation(const char *message);
void tcs3200SendCalibrationReading(const char *color_name);

sample_color_t classifyTCS3200Color(void);
bool tcs3200DetectBlackTape(void);

#endif