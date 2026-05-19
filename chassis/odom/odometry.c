#include <stdio.h>
#include <math.h>

#include "config.h"
#include "odometry.h"

static pose_t my_pose;

void odometryInit(float x, float y, float yaw) {
    my_pose.x = x;
    my_pose.y = y;
    my_pose.yaw = normalizeAngle(yaw);
}

pose_t getPose(void) {
    return my_pose;
}

float degToRad(float degrees) {
    return degrees * PI / 180.0;
}

float normalizeAngle(float angle) {
    while (angle >= 360.0) {
        angle -= 360.0;
    }

    while (angle < 0.0) {
        angle += 360.0;
    }

    return angle;
}

void updatePoseAfterMove(float distance_cm) {
    float yaw_rad = degToRad(my_pose.yaw);

    my_pose.x = my_pose.x + distance_cm * cos(yaw_rad);
    my_pose.y = my_pose.y + distance_cm * sin(yaw_rad);
}

void updatePoseAfterTurn(float angle_deg) {
    my_pose.yaw = normalizeAngle(my_pose.yaw + angle_deg);
}

void printPose(void) {
    printf("Pose: x = %.2f cm, y = %.2f cm, yaw = %.2f degrees\n",
           my_pose.x,
           my_pose.y,
           my_pose.yaw);
}