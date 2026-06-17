#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>

int speedFromCmPerSec(float cm_per_sec);

void setGyroTurnFeedbackEnabled(bool enabled);
void recalibrateGyroIfReady(void);
void runGyroScaleCalibration(int full_rotations, float step_deg, float speed_cm_s);
void moveWithRamp(float distance_cm, float speed_cm_s);
void move(float distance_cm, float speed_cm_s);
void turn(float angle_deg, float speed_cm_s);
void moveTo(float target_x_cm, float target_y_cm, float speed_cm_s);
void setSpeed(float speed_cm_s);
void dance(void);

#endif
