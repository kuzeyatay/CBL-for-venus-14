#include <libpynq.h>
#include <stepper.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "gyro_sensor.h"
#include "odometry.h"
#include "motion.h"

static float normalizeTurnCommand(float angle_deg);
static int turnStepsForAngle(float angle_deg);
static bool waitForStepperSteps(bool update_gyro);
static void turnOpenLoop(float angle_deg, float speed_cm_s);
static void turnWithGyroFeedback(float angle_deg, float speed_cm_s);

static bool gyro_turn_feedback_enabled = false;

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

void setGyroTurnFeedbackEnabled(bool enabled)
{
    gyro_turn_feedback_enabled = enabled;
}

void recalibrateGyroIfReady(void)
{
#if GYRO_TURN_ENABLED
    if (!USE_MOCK_SENSORS && isMPU6886Initialized()) {
        updateMPU6886GyroBias(100);
    }
#endif
}

static float normalizeTurnCommand(float angle_deg)
{
    angle_deg = fmodf(angle_deg, 360.0f);

    if (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }

    if (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static int turnStepsForAngle(float angle_deg)
{
    float radians = fabsf(angle_deg) * PI / 180.0f;
    float distance = TURN_RADIUS * radians;
    int steps = (int)roundf(distance * STEPS_PER_CM);

    if (steps < 1) {
        steps = 1;
    }

    return steps;
}

static bool waitForStepperSteps(bool update_gyro)
{
    bool gyro_ok = true;

    while (!stepper_steps_done()) {
        if (update_gyro && !updateMPU6886Yaw()) {
            gyro_ok = false;
        }

        sleep_msec(GYRO_TURN_UPDATE_INTERVAL_MS);
    }

    if (update_gyro && !updateMPU6886Yaw()) {
        gyro_ok = false;
    }

    return gyro_ok;
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
    angle_deg = normalizeTurnCommand(angle_deg);

    if (fabsf(angle_deg) <= GYRO_TURN_TOLERANCE_DEG) {
        return;
    }

#if GYRO_TURN_ENABLED
    if (!USE_MOCK_SENSORS) {
        if (gyro_turn_feedback_enabled && isMPU6886Initialized()) {
            turnWithGyroFeedback(angle_deg, speed_cm_s);
            return;
        }

        printf("GYRO TURN required but unavailable; cancelled %.2f deg turn\n",
               angle_deg);
        return;
    }
#endif

    printf("GYRO TURN mock fallback: open-loop %.2f deg\n",
           angle_deg);
    turnOpenLoop(angle_deg, speed_cm_s);
}

static void turnOpenLoop(float angle_deg, float speed_cm_s)
{
    int direction = angle_deg >= 0.0f ? 1 : -1;
    int steps = turnStepsForAngle(angle_deg);

    int stepper_speed = speedFromCmPerSec(speed_cm_s);
    stepper_set_speed(stepper_speed, stepper_speed);

    stepper_steps(-direction * steps, direction * steps);
    (void)waitForStepperSteps(false);

    sleep_msec(500);
    updatePoseAfterTurn(angle_deg);
}

static void turnWithGyroFeedback(float angle_deg, float speed_cm_s)
{
    int direction = angle_deg >= 0.0f ? 1 : -1;
    float target_abs_deg = fabsf(angle_deg);
    float measured_abs_deg = 0.0f;
    float previous_abs_deg = 0.0f;
    int stale_chunks = 0;
    int iteration = 0;

    /* At normal speed a small turn finishes in < 5 ms, giving the 2 ms
     * gyro poller only 1-2 samples — not enough to integrate the
     * rotation.  Clamp to a slow speed so there are at least ~14
     * samples per degree. */
    if (target_abs_deg < GYRO_SMALL_TURN_THRESHOLD_DEG &&
        speed_cm_s > GYRO_SMALL_TURN_MAX_SPEED_CM_S) {
        speed_cm_s = GYRO_SMALL_TURN_MAX_SPEED_CM_S;
    }

    int stepper_speed = speedFromCmPerSec(speed_cm_s);
    stepper_set_speed(stepper_speed, stepper_speed);

    resetMPU6886Yaw();

    if (!updateMPU6886Yaw()) {
        printf("GYRO TURN failed to prime gyro; cancelled %.2f deg turn\n",
               angle_deg);
        return;
    }

    sleep_msec(GYRO_TURN_UPDATE_INTERVAL_MS);

    if (!updateMPU6886Yaw()) {
        printf("GYRO TURN failed to read gyro; cancelled %.2f deg turn\n",
               angle_deg);
        return;
    }

    while (measured_abs_deg < target_abs_deg - GYRO_TURN_TOLERANCE_DEG &&
           iteration < GYRO_TURN_MAX_ITERATIONS) {
        float remaining_deg = target_abs_deg - measured_abs_deg;
        float chunk_deg = remaining_deg;
        int steps = 0;

        if (chunk_deg > GYRO_TURN_MAX_CHUNK_DEG) {
            chunk_deg = GYRO_TURN_MAX_CHUNK_DEG;
        }

        if (chunk_deg < GYRO_TURN_MIN_CHUNK_DEG) {
            chunk_deg = GYRO_TURN_MIN_CHUNK_DEG;
        }

        steps = turnStepsForAngle(chunk_deg);
        stepper_steps(-direction * steps, direction * steps);

        if (!waitForStepperSteps(true)) {
            printf("GYRO TURN read failure during feedback loop\n");
            measured_abs_deg += chunk_deg;
            break;
        }

        measured_abs_deg = fabsf(getMPU6886YawDeg());

        if (measured_abs_deg <= previous_abs_deg + 0.05f) {
            stale_chunks++;
        } else {
            stale_chunks = 0;
        }

        if (stale_chunks >= GYRO_TURN_MAX_STALE_CHUNKS) {
            printf("GYRO TURN stopped: yaw not changing after %d chunks\n",
                   stale_chunks);
            break;
        }

        previous_abs_deg = measured_abs_deg;
        iteration++;
    }

    sleep_msec(GYRO_TURN_SETTLE_MS);
    (void)updateMPU6886Yaw();

    measured_abs_deg = fabsf(getMPU6886YawDeg());

    printf("GYRO TURN requested=%.2f deg, measured=%.2f deg, error=%.2f deg, iterations=%d\n",
           angle_deg,
           (float)direction * measured_abs_deg,
           target_abs_deg - measured_abs_deg,
           iteration);

    updatePoseAfterTurn((float)direction * measured_abs_deg);
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
