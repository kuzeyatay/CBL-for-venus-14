#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "constants/robot_types.h"

void odometryInit(float x, float y, float yaw);
void updatePoseAfterMove(float distance_cm);
void updatePoseAfterTurn(float angle_deg);

pose_t getPose(void);
void printPose(void);

float degToRad(float degrees);
float normalizeAngle(float angle);

#endif