#include <libpynq.h>
#include <stepper.h>
#include <math.h>
#include <stdlib.h>

#include "config.h"
#include "odometry.h"
#include "motion.h"

int speedFromCmPerSec(float cm_per_sec)
{
    int speed_value = round(
        1.0 / (cm_per_sec * STEPS_PER_CM * SECONDS_PER_SPEED_UNIT));

    if (speed_value < STEPPER_SPEED_MIN_VALUE)
    {
        speed_value = STEPPER_SPEED_MIN_VALUE;
    }

    if (speed_value > STEPPER_SPEED_MAX_VALUE)
    {
        speed_value = STEPPER_SPEED_MAX_VALUE;
    }

    return speed_value;
}

void moveWithRamp(float distance_cm, float speed_cm_s)
{
    int steps = round(distance_cm * STEPS_PER_CM);

    int stepper_speed = speedFromCmPerSec(speed_cm_s);
    stepper_set_speed(stepper_speed, stepper_speed);
    stepper_steps(steps, steps);

    while (!stepper_steps_done())
    {
        // wait
    }

    sleep_msec(500);
    updatePoseAfterMove(distance_cm);
}

void move(float distance_cm, float speed_cm_s)
{
    int steps = round(distance_cm * STEPS_PER_CM);

    int stepper_speed = speedFromCmPerSec(speed_cm_s);
    stepper_set_speed(stepper_speed, stepper_speed);
    stepper_steps(steps, steps);

    while (!stepper_steps_done())
    {
        // wait
    }

    sleep_msec(500);
    updatePoseAfterMove(distance_cm);
}

void turn(float angle_deg, float speed_cm_s)
{
    angle_deg = fmodf(angle_deg, 360);

    if (angle_deg > 180)
    {
        angle_deg -= 360;
    }

    float radians = angle_deg * PI / 180.0;
    float distance = TURN_RADIUS * radians;
    int steps = round(distance * STEPS_PER_CM);

    int stepper_speed = speedFromCmPerSec(speed_cm_s);
    stepper_set_speed(stepper_speed, stepper_speed);

    stepper_steps(-steps, steps);

    while (!stepper_steps_done())
    {
        // wait
    }

    sleep_msec(500);
    updatePoseAfterTurn(angle_deg);
}

void moveTo(float target_x_cm, float target_y_cm, float speed_cm_s)
{
    pose_t pose = getPose();

    float delta_x = target_x_cm - pose.x;
    float delta_y = target_y_cm - pose.y;

    float angle_to_target = atan2(delta_y, delta_x);
    float angle_to_turn = angle_to_target - degToRad(pose.yaw);

    angle_to_turn = angle_to_turn * 180.0 / PI;

    turn(angle_to_turn, speed_cm_s);

    float distance_to_target = sqrt(delta_x * delta_x + delta_y * delta_y);

    moveWithRamp(distance_to_target, speed_cm_s);
}

void setSpeed(float speed_cm_s)
{
    int stepper_speed = speedFromCmPerSec(speed_cm_s);
    stepper_set_speed(stepper_speed, stepper_speed);
}

void dance(void)
{
    for (int i = 0; i < 5; i++)
    {
        turn(60, 25);
        turn(-60, 25);
    }
}