#ifndef MOTION_H
#define MOTION_H

int speedFromCmPerSec(float cm_per_sec);

void moveWithRamp(float distance_cm, float speed_cm_s);
void move(float distance_cm, float speed_cm_s);
void turn(float angle_deg, float speed_cm_s);
void moveTo(float target_x_cm, float target_y_cm, float speed_cm_s);
void setSpeed(float speed_cm_s);
void dance(void);

#endif