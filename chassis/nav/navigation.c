#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "config.h"
#include "robot_types.h"
#include "odometry.h"
#include "motion.h"
#include "sensor_manager.h"
#include "communication.h"
#include "navigation.h"
#include "distance_sensor.h"
#include "color_sensor.h"
#include "temperature_sensor.h"

static const char *colorToString(sample_color_t color);
static void markCellAhead(cell_status_t status);
static void reportFieldEvent(field_event_t event);

static bool mission_running = false;
static cell_status_t map_grid[MAP_SIZE][MAP_SIZE];

static bool isUsableDistance(int distance_mm)
{
    return distance_mm > 0 &&
           distance_mm != VL53L0X_INVALID_DISTANCE_MM &&
           distance_mm <= VL53L0X_MAX_REASONABLE_MM;
}

static void moveToDistanceFromObject(int target_distance_mm)
{
    int distance_mm = readVL53L0XDistance();

    if (!isUsableDistance(distance_mm)) {
        printf("Sample approach failed: invalid distance reading = %d mm\n",
               distance_mm);
        return;
    }

    int error_mm = distance_mm - target_distance_mm;

    if (abs(error_mm) <= SAMPLE_APPROACH_TOLERANCE_MM) {
        printf("Already close enough for color reading: %d mm\n", distance_mm);
        return;
    }

    float move_cm = error_mm / 10.0f;

    if (move_cm > SAMPLE_APPROACH_MAX_CM) {
        move_cm = SAMPLE_APPROACH_MAX_CM;
    }

    if (move_cm < -SAMPLE_APPROACH_MAX_CM) {
        move_cm = -SAMPLE_APPROACH_MAX_CM;
    }

    printf("Moving %.2f cm to reach %d mm from sample. Current distance = %d mm\n",
           move_cm,
           target_distance_mm,
           distance_mm);

    moveWithRamp(move_cm, SAMPLE_APPROACH_SPEED_CM_S);
    sendPoseUpdate();
}

static void finalizeRockSampleAtCloseRange(field_event_t *event)
{
    moveToDistanceFromObject(SAMPLE_COLOR_DISTANCE_MM);

    event->distance_mm = readVL53L0XDistance();

    if (isTCS3200Calibrated()) {
        event->color = classifyTCS3200Color();
    } else {
        printf("Cannot classify rock color: TCS3200 is not calibrated.\n");
        event->color = COLOR_UNKNOWN;
    }

    event->temperature_c = readNTCTemperature();

    printf("Final rock sample reading: distance=%d mm, color=%s, temp=%.2f C\n",
           event->distance_mm,
           colorToString(event->color),
           event->temperature_c);
}

static bool scanAfterForwardMoveAndApproachObject(void)
{
    float half_scan_deg = POST_MOVE_SCAN_TOTAL_DEG / 2.0f;
    float current_angle_deg = 0.0f;

    float best_angle_deg = 0.0f;
    int best_distance_mm = VL53L0X_INVALID_DISTANCE_MM;
    bool found_object = false;

    float black_tape_angle_deg = 0.0f;
    bool found_black_tape = false;
    bool tape_scan_enabled = isTCS3200Calibrated();

    printf("Post-move scan started: total %.1f degrees\n", POST_MOVE_SCAN_TOTAL_DEG);

    if (!tape_scan_enabled)
    {
        printf("Post-move tape scan disabled: TCS3200 is not calibrated.\n");
    }

    turn(-half_scan_deg, POST_MOVE_SCAN_SPEED_CM_S);
    current_angle_deg = -half_scan_deg;

    while (current_angle_deg <= half_scan_deg + 0.01f)
    {
        int distance_mm = readVL53L0XDistance();
        bool black_tape_here = false;

        if (tape_scan_enabled)
        {
            black_tape_here = tcs3200DetectBlackTape();

            if (black_tape_here && !found_black_tape)
            {
                found_black_tape = true;
                black_tape_angle_deg = current_angle_deg;
            }
        }

        bool object_candidate =
            isUsableDistance(distance_mm) &&
            distance_mm <= FRONT_OBJECT_THRESHOLD_MM &&
            !black_tape_here;

        printf(
            "Post-move scan: angle=%.1f deg, distance=%d mm, black_tape=%s, object_candidate=%s\n",
            current_angle_deg,
            distance_mm,
            black_tape_here ? "yes" : "no",
            object_candidate ? "yes" : "no"
        );

        if (object_candidate)
        {
            if (!found_object || distance_mm < best_distance_mm)
            {
                found_object = true;
                best_distance_mm = distance_mm;
                best_angle_deg = current_angle_deg;
            }
        }

        if (current_angle_deg >= half_scan_deg)
        {
            break;
        }

        float step_deg = POST_MOVE_SCAN_STEP_DEG;

        if (current_angle_deg + step_deg > half_scan_deg)
        {
            step_deg = half_scan_deg - current_angle_deg;
        }

        turn(step_deg, POST_MOVE_SCAN_SPEED_CM_S);
        current_angle_deg += step_deg;
    }

    /*
     * Safety priority:
     * If black tape was seen anywhere during the scan, avoid it first.
     * Do not approach an object when black tape was detected in the same scan.
     */
    if (found_black_tape)
    {
        printf(
            "Post-move scan found black tape: angle=%.1f deg. Avoiding instead of approaching.\n",
            black_tape_angle_deg
        );

        /*
         * Face the direction where the tape was detected, so markCellAhead()
         * marks the unsafe cell in the detected tape direction.
         */
        turn(black_tape_angle_deg - current_angle_deg, POST_MOVE_SCAN_SPEED_CM_S);

        field_event_t tape_event;
        tape_event.type = FIELD_BLACK_TAPE;
        tape_event.distance_mm = VL53L0X_INVALID_DISTANCE_MM;
        tape_event.width_cm = 0.0f;
        tape_event.color = COLOR_BLACK;
        tape_event.sample_size_cm = 0;
        tape_event.temperature_c = 0.0f;

        markCellAhead(CELL_UNSAFE);
        reportFieldEvent(tape_event);

        moveWithRamp(-REVERSE_DISTANCE_CM, DEFAULT_SPEED);
        turn(AVOID_TURN_DEG, DEFAULT_SPEED);

        sendPoseUpdate();
        return false;
    }

    if (found_object)
    {
        printf(
            "Post-move scan found object: angle=%.1f deg, distance=%d mm\n",
            best_angle_deg,
            best_distance_mm
        );

        turn(best_angle_deg - current_angle_deg, POST_MOVE_SCAN_SPEED_CM_S);

        int current_distance_mm = readVL53L0XDistance();

        if (isUsableDistance(current_distance_mm))
        {
            best_distance_mm = current_distance_mm;
        }

        int approach_mm = best_distance_mm - POST_MOVE_SCAN_STOP_DISTANCE_MM;

        if (approach_mm > 0)
        {
            float approach_cm = approach_mm / 10.0f;

            if (approach_cm > POST_MOVE_SCAN_APPROACH_MAX_CM)
            {
                approach_cm = POST_MOVE_SCAN_APPROACH_MAX_CM;
            }

            printf("Moving %.2f cm toward scanned object.\n", approach_cm);
            moveWithRamp(approach_cm, DEFAULT_SPEED);
        }
        else
        {
            printf("Scanned object is already close enough. Not moving forward.\n");
        }

        sendPoseUpdate();
        return true;
    }

    printf("Post-move scan found no object and no black tape. Returning to original heading.\n");
    turn(-current_angle_deg, POST_MOVE_SCAN_SPEED_CM_S);
    sendPoseUpdate();

    return false;
}

void startMission(void) {
    mission_running = true;
    sendStatusUpdate("navigating", "none");
}

void stopMission(void) {
    mission_running = false;
    sendStatusUpdate("stopped", "none");
}

bool isMissionRunning(void) {
    return mission_running;
}

static void initMap(void) {
    for (int x = 0; x < MAP_SIZE; x++) {
        for (int y = 0; y < MAP_SIZE; y++) {
            map_grid[x][y] = CELL_UNKNOWN;
        }
    }

    map_grid[MAP_CENTER][MAP_CENTER] = CELL_EXPLORED;
}

static bool poseToGridCell(float x_cm, float y_cm, int *grid_x, int *grid_y) {
    int gx = MAP_CENTER + (int)round(x_cm / CELL_SIZE_CM);
    int gy = MAP_CENTER + (int)round(y_cm / CELL_SIZE_CM);

    if (gx < 0 || gx >= MAP_SIZE || gy < 0 || gy >= MAP_SIZE) {
        return false;
    }

    *grid_x = gx;
    *grid_y = gy;
    return true;
}

static void markCurrentCell(cell_status_t status) {
    pose_t pose = getPose();

    int gx, gy;

    if (poseToGridCell(pose.x, pose.y, &gx, &gy)) {
        map_grid[gx][gy] = status;
        sendCellUpdate(gx, gy, status);
    }
}

static void markCellAhead(cell_status_t status) {
    pose_t pose = getPose();

    float yaw_rad = degToRad(pose.yaw);

    float ahead_x = pose.x + CELL_SIZE_CM * cos(yaw_rad);
    float ahead_y = pose.y + CELL_SIZE_CM * sin(yaw_rad);

    int gx, gy;

    if (poseToGridCell(ahead_x, ahead_y, &gx, &gy)) {
        map_grid[gx][gy] = status;
        sendCellUpdate(gx, gy, status);
    }
}

static const char *colorToString(sample_color_t color) {
    switch (color) {
        case COLOR_WHITE: return "white";
        case COLOR_BLACK: return "black";
        case COLOR_RED: return "red";
        case COLOR_GREEN: return "green";
        case COLOR_BLUE: return "blue";
        default: return "unknown";
    }
}

static const char *eventToString(field_event_type_t event) {
    switch (event) {
        case FIELD_CLEAR: return "clear";
        case FIELD_BLACK_TAPE: return "black_tape";
        case FIELD_HILL: return "hill";
        case FIELD_ROCK_SAMPLE: return "rock_sample";
        case FIELD_SENSOR_FAULT: return "sensor_fault";
        default: return "unknown";
    }
}

static bool estimateSampleSize(float width_cm, int *sample_size_cm)
{
    /*
     * These thresholds are empirical for this robot.
     * The VL53L0X width scan underestimates real cube width because the robot
     * rotates around its wheel axis, not around the distance sensor itself.
     */

    if (width_cm >= 1.5f && width_cm < 3.4f)
    {
        *sample_size_cm = 3;
        return true;
    }

    if (width_cm >= 3.4f && width_cm <= 7.5f)
    {
        *sample_size_cm = 6;
        return true;
    }

    return false;
}

static field_event_t interpretSensorData(sensor_data_t data)
{
    field_event_t event;

    event.type = FIELD_CLEAR;
    event.distance_mm = data.front_distance_mm;
    event.width_cm = data.estimated_width_cm;
    event.color = data.object_color;
    event.sample_size_cm = 0;
    event.temperature_c = data.temperature_c;

    if (!data.valid)
    {
        event.type = FIELD_SENSOR_FAULT;
        return event;
    }

    if (data.black_tape_detected)
    {
        event.type = FIELD_BLACK_TAPE;
        return event;
    }

    if (data.front_object_detected &&
        data.front_distance_mm <= FRONT_OBJECT_THRESHOLD_MM)
    {
        int sample_size = 0;

        if (estimateSampleSize(data.estimated_width_cm, &sample_size))
        {
            event.type = FIELD_ROCK_SAMPLE;
            event.sample_size_cm = sample_size;
            event.color = COLOR_UNKNOWN;
            return event;
        }

        event.type = FIELD_HILL;
        return event;
    }

    return event;
}

static void reportFieldEvent(field_event_t event) {
    printf(
        "REPORT: robot=%s, event=%s, distance=%d mm, width=%.2f cm, color=%s, size=%d cm, temp=%.2f C\n",
        ROBOT_ID,
        eventToString(event.type),
        event.distance_mm,
        event.width_cm,
        colorToString(event.color),
        event.sample_size_cm,
        event.temperature_c
    );

    sendFieldEventUpdate(event);
}

static void handleFieldEvent(field_event_t event) {
    printf("Navigation received event: %s\n", eventToString(event.type));

    switch (event.type) {
        case FIELD_CLEAR:
            markCurrentCell(CELL_EXPLORED);

            moveWithRamp(FORWARD_INCREMENT_CM, DEFAULT_SPEED);
            markCurrentCell(CELL_EXPLORED);
            sendPoseUpdate();

            /*
             * After every normal forward movement, scan around the robot.
             * If black tape is visible, avoid it.
             * If no black tape is visible but a close object is visible, approach it.
             */
            scanAfterForwardMoveAndApproachObject();

            break;

        case FIELD_BLACK_TAPE:
            markCellAhead(CELL_UNSAFE);
            reportFieldEvent(event);
            moveWithRamp(-REVERSE_DISTANCE_CM, DEFAULT_SPEED);
            turn(AVOID_TURN_DEG, DEFAULT_SPEED);
            sendPoseUpdate();
            break;

        case FIELD_HILL:
            markCellAhead(CELL_HILL);
            reportFieldEvent(event);
            moveWithRamp(-REVERSE_DISTANCE_CM, DEFAULT_SPEED);
            turn(AVOID_TURN_DEG, DEFAULT_SPEED);
            sendPoseUpdate();
            break;

        case FIELD_ROCK_SAMPLE:
            markCellAhead(CELL_SAMPLE);

            /*
             * The object has already been classified as a rock sample by width.
             * Now move close enough for the TCS3200 color sensor, then classify color.
             */
            finalizeRockSampleAtCloseRange(&event);

            reportFieldEvent(event);

            moveWithRamp(-REVERSE_DISTANCE_CM, DEFAULT_SPEED);
            turn(SAMPLE_AVOID_TURN_DEG, DEFAULT_SPEED);
            sendPoseUpdate();

            break;

        case FIELD_SENSOR_FAULT:
            reportFieldEvent(event);
            turn(AVOID_TURN_DEG, DEFAULT_SPEED);
            sendStatusUpdate("fault", "sensor_fault");
            break;

        default:
            sendErrorMessage("unknown_event");
            break;
    }
}

void runNavigation(void) {
    initMap();

    printf("Starting autonomous navigation...\n");

    mission_running = true;

    for (int step = 0; step < MAX_NAVIGATION_STEPS && mission_running; step++) {
        pollESP32Messages();

        printf("\n--- Navigation step %d ---\n", step);
        printPose();

        sensor_data_t sensor_data = readReliableSensorData();
        field_event_t event = interpretSensorData(sensor_data);

        handleFieldEvent(event);
    }

    printf("Navigation finished.\n");
}