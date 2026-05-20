#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <stdbool.h>

#include "robot_types.h"

void tcs3200Init(void);
void tcs3200DebugOutPin(void);

bool isTCS3200Calibrated(void);

bool tcs3200LoadCalibrationFromBaseStation(const char *message);

bool tcs3200LoadManualCalibration(
    double white_r, double white_g, double white_b, double white_c,
    double black_r, double black_g, double black_b, double black_c,
    double red_r,   double red_g,   double red_b,   double red_c,
    double green_r, double green_g, double green_b, double green_c,
    double blue_r,  double blue_g,  double blue_b,  double blue_c);

void tcs3200SendCalibrationReading(const char *color_name);

sample_color_t classifyTCS3200Color(void);
bool tcs3200DetectBlackTape(void);

#endif