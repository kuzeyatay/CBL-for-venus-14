#include <libpynq.h>
#include <stepper.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "config.h"
#include "gyro_sensor.h"
#include "odometry.h"
#include "motion.h"

static float normalizeTurnCommand(float angle_deg);
static int turnStepsForAngle(float angle_deg);
static bool waitForStepperSteps(bool update_gyro);
static void settleWithGyroTracking(int duration_ms, bool track_yaw);
static void runStraightMove(float distance_cm, float speed_cm_s);
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
        if (update_gyro) {
            /* No sleep while tracking yaw: each update is a multi-ms I2C
             * burst that paces the loop by itself, and sampling as fast as
             * possible minimizes the integration loss that the yaw scale
             * constant otherwise has to paper over. */
            if (!updateMPU6886Yaw()) {
                gyro_ok = false;
            }
        } else {
            sleep_msec(GYRO_TURN_UPDATE_INTERVAL_MS);
        }
    }

    if (update_gyro && !updateMPU6886Yaw()) {
        gyro_ok = false;
    }

    return gyro_ok;
}

static uint64_t motionNowUs(void)
{
    struct timeval tv;

    gettimeofday(&tv, NULL);

    return ((uint64_t)tv.tv_sec * 1000000ULL) + (uint64_t)tv.tv_usec;
}

/*
 * Settle wait that keeps integrating the gyro.  A plain sleep followed by a
 * single update hands the integrator one long gap bridging "spinning" and
 * "stopped"; averaging the rates at the gap's ends then credits half the
 * final rotation rate across the whole gap — about +7 phantom degrees per
 * 15-degree turn at scan speed.  Polling continuously keeps every
 * integration step short, so the deceleration is tracked as it happens.
 */
static void settleWithGyroTracking(int duration_ms, bool track_yaw)
{
    if (!track_yaw) {
        sleep_msec(duration_ms);
        return;
    }

    uint64_t end_us = motionNowUs() + (uint64_t)duration_ms * 1000ULL;

    while (motionNowUs() < end_us) {
        (void)updateMPU6886Yaw();  /* the I2C transaction paces the loop */
    }
}

/*
 * Straight move with gyro yaw tracking.
 *
 * The wheels are commanded equally, but slip and unequal wheel diameters
 * still rotate the robot.  Integrating the gyro during the move measures
 * that drift so the pose stays honest: the measured yaw change is split
 * around the translation (midpoint integration) so the distance is
 * projected along the average heading of the move.
 */
static void runStraightMove(float distance_cm, float speed_cm_s)
{
    int steps = round(distance_cm * STEPS_PER_CM);
    int stepper_speed = speedFromCmPerSec(speed_cm_s);
    bool track_yaw = false;
    float yaw_change_deg = 0.0f;

#if GYRO_TURN_ENABLED
    track_yaw = !USE_MOCK_SENSORS &&
                gyro_turn_feedback_enabled &&
                isMPU6886Initialized();
#endif

    if (track_yaw) {
        resetMPU6886Yaw();

        if (!updateMPU6886Yaw()) {
            printf("MOVE failed to prime gyro; yaw drift untracked for %.2f cm move\n",
                   distance_cm);
            track_yaw = false;
        }
    }

    stepper_set_speed(stepper_speed, stepper_speed);
    stepper_steps(steps, steps);

    if (!waitForStepperSteps(track_yaw) && track_yaw) {
        printf("MOVE gyro read failure mid-move; drift estimate may be partial\n");
    }

    settleWithGyroTracking(500, track_yaw);

    if (track_yaw) {
        yaw_change_deg = getMPU6886YawDeg();

        if (fabsf(yaw_change_deg) > GYRO_TURN_TOLERANCE_DEG) {
            printf("MOVE drift: yaw changed %.2f deg over %.2f cm move\n",
                   yaw_change_deg,
                   distance_cm);
        }
    }

    updatePoseAfterTurn(yaw_change_deg * 0.5f);
    updatePoseAfterMove(distance_cm);
    updatePoseAfterTurn(yaw_change_deg * 0.5f);
}

void moveWithRamp(float distance_cm, float speed_cm_s)
{
    runStraightMove(distance_cm, speed_cm_s);
}

void move(float distance_cm, float speed_cm_s)
{
    runStraightMove(distance_cm, speed_cm_s);
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
    bool stall_retry_done = false;

    if (target_abs_deg < GYRO_SMALL_TURN_THRESHOLD_DEG &&
        speed_cm_s > GYRO_SMALL_TURN_MAX_SPEED_CM_S) {
        speed_cm_s = GYRO_SMALL_TURN_MAX_SPEED_CM_S;
    }

    if (target_abs_deg >= GYRO_LARGE_TURN_THRESHOLD_DEG &&
        speed_cm_s > GYRO_LARGE_TURN_MAX_SPEED_CM_S) {
        speed_cm_s = GYRO_LARGE_TURN_MAX_SPEED_CM_S;
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
            float remaining = target_abs_deg - measured_abs_deg;

            if (!stall_retry_done &&
                remaining > GYRO_TURN_TOLERANCE_DEG &&
                speed_cm_s > GYRO_SMALL_TURN_MAX_SPEED_CM_S) {
                printf("GYRO TURN stall at %.2f/%.2f deg, retrying slowly\n",
                       measured_abs_deg, target_abs_deg);
                speed_cm_s = GYRO_SMALL_TURN_MAX_SPEED_CM_S;
                stepper_speed = speedFromCmPerSec(speed_cm_s);
                stepper_set_speed(stepper_speed, stepper_speed);
                stale_chunks = 0;
                stall_retry_done = true;
                continue;
            }

            printf("GYRO TURN stopped: yaw not changing after %d chunks\n",
                   stale_chunks);
            break;
        }

        previous_abs_deg = measured_abs_deg;
        iteration++;
    }

    settleWithGyroTracking(GYRO_TURN_SETTLE_MS, true);

    measured_abs_deg = fabsf(getMPU6886YawDeg());

    printf("GYRO TURN requested=%.2f deg, measured=%.2f deg, error=%.2f deg, iterations=%d\n",
           angle_deg,
           (float)direction * measured_abs_deg,
           target_abs_deg - measured_abs_deg,
           iteration);

    updatePoseAfterTurn((float)direction * measured_abs_deg);
}

/*
 * Interactive gyro yaw-scale calibration.  Call from main() instead of the
 * mission run, with the robot on the floor and room to spin in place.
 * Console prompts only — the base station is not involved.
 *
 * Procedure:
 *   1. Keep the robot still while the gyro bias is calibrated.
 *   2. Mark the robot's heading (tape on the floor along the chassis edge),
 *      press Enter, and let it spin full_rotations turns counterclockwise.
 *   3. Read the leftover angle between mark and chassis: positive when the
 *      robot rotated PAST the mark in the spin direction, negative when it
 *      stopped short.  Type it in when prompted.
 *   4. Copy the printed GYRO_Z_SCALE_CORRECTION into constants/config.h.
 *
 * Run it with the mission's dominant step size (SCAN_360_STEP_DEG) and
 * DEFAULT_SPEED so the calibration sees the same turn dynamics.  A fixed
 * gyro mounting tilt scales every reading by the same cos(tilt) factor, so
 * it is absorbed into this constant too.  The spin leaves the odometry pose
 * rotated — restart the program before running a mission.
 */
void runGyroScaleCalibration(int full_rotations, float step_deg, float speed_cm_s)
{
    char line[64];
    float commanded_total_deg = 0.0f;
    float commanded_so_far_deg = 0.0f;
    float measured_total_deg = 0.0f;
    float offset_deg = 0.0f;
    float physical_total_deg = 0.0f;
    float current_scale = getMPU6886GyroZScaleCorrection();
    float new_scale = 0.0f;

    if (!isMPU6886Initialized()) {
        printf("GYRO CAL: gyro not initialized — aborting.\n");
        return;
    }

    if (full_rotations < 1) {
        full_rotations = 1;
    }

    if (full_rotations > 10) {
        full_rotations = 10;
    }

    if (step_deg < 5.0f) {
        step_deg = 5.0f;
    }

    if (step_deg > 90.0f) {
        step_deg = 90.0f;
    }

    commanded_total_deg = (float)full_rotations * 360.0f;

    printf("GYRO CAL: %d rotation(s), %.1f deg steps, %.1f cm/s, current scale %.4f\n",
           full_rotations, step_deg, speed_cm_s, current_scale);
    printf("GYRO CAL: keep the robot still — calibrating bias...\n");
    updateMPU6886GyroBias(GYRO_CALIBRATION_SAMPLES);
    printf("GYRO CAL: mark the robot's heading with tape, then press Enter to spin.\n");
    (void)fgets(line, sizeof(line), stdin);

    while (commanded_so_far_deg < commanded_total_deg - 0.01f) {
        float chunk_deg = commanded_total_deg - commanded_so_far_deg;

        if (chunk_deg > step_deg) {
            chunk_deg = step_deg;
        }

        turn(chunk_deg, speed_cm_s);

        /* turn() resets the yaw integrator on entry and leaves the measured
         * (scale-corrected) turn angle in it. */
        measured_total_deg += getMPU6886YawDeg();
        commanded_so_far_deg += chunk_deg;
    }

    printf("GYRO CAL: spin done. commanded=%.1f deg, gyro measured=%.2f deg\n",
           commanded_total_deg, measured_total_deg);
    printf("GYRO CAL: read the angle between the tape mark and the chassis heading.\n");
    printf("GYRO CAL: positive = rotated past the mark, negative = stopped short.\n");
    printf("GYRO CAL: enter offset in degrees (e.g. 12 or -8.5): ");
    fflush(stdout);

    if (fgets(line, sizeof(line), stdin) == NULL ||
        sscanf(line, "%f", &offset_deg) != 1) {
        printf("GYRO CAL: no valid offset entered — aborting without a result.\n");
        return;
    }

    if (fabsf(measured_total_deg) < 90.0f) {
        printf("GYRO CAL: measured total %.2f deg is implausible — aborting.\n",
               measured_total_deg);
        return;
    }

    if (measured_total_deg < 0.0f) {
        printf("GYRO CAL: measured total is negative — the gyro Z axis is inverted\n");
        printf("GYRO CAL: (flipped mount).  The scale below carries that sign; copy it\n");
        printf("GYRO CAL: with the minus sign, it also fixes straight-move drift signs.\n");
    }

    /* The gyro accounted measured_total_deg (signed) for a physical rotation
     * of commanded_total_deg + offset.  The corrected scale is the current
     * scale stretched by physical / measured; dividing by a negative
     * measured total flips the sign for an inverted axis automatically. */
    physical_total_deg = commanded_total_deg + offset_deg;
    new_scale = current_scale * physical_total_deg / measured_total_deg;

    printf("GYRO CAL: physical=%.1f deg, measured=%.2f deg\n",
           physical_total_deg, measured_total_deg);
    printf("GYRO CAL: set GYRO_Z_SCALE_CORRECTION to %.4f  (currently %.4f)\n",
           new_scale, current_scale);
    printf("GYRO CAL: copy it into constants/config.h and rebuild.\n");
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
