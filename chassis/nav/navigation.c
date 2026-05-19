#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "config.h"
#include "robot_types.h"
#include "odometry.h"
#include "motion.h"
#include "sensor_manager.h"
#include "communication.h"
#include "navigation.h"

static bool mission_running = false;
static cell_status_t map_grid[MAP_SIZE][MAP_SIZE];

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

static bool isValidSampleColor(sample_color_t color) {
    return color == COLOR_WHITE ||
           color == COLOR_BLACK ||
           color == COLOR_RED ||
           color == COLOR_GREEN ||
           color == COLOR_BLUE;
}

static bool estimateSampleSize(float width_cm, int *sample_size_cm) {
    if (fabs(width_cm - 3.0) <= 1.5) {
        *sample_size_cm = 3;
        return true;
    }

    if (fabs(width_cm - 6.0) <= 1.5) {
        *sample_size_cm = 6;
        return true;
    }

    return false;
}

static field_event_t interpretSensorData(sensor_data_t data) {
    field_event_t event;

    event.type = FIELD_CLEAR;
    event.distance_mm = data.front_distance_mm;
    event.width_cm = data.estimated_width_cm;
    event.color = data.object_color;
    event.sample_size_cm = 0;
    event.temperature_c = data.temperature_c;

    if (!data.valid) {
        event.type = FIELD_SENSOR_FAULT;
        return event;
    }

    if (data.black_tape_detected && !data.front_object_detected) {
        event.type = FIELD_BLACK_TAPE;
        return event;
    }

    if (data.front_object_detected &&
        data.front_distance_mm <= FRONT_OBJECT_THRESHOLD_MM) {

        int sample_size = 0;

        if (isValidSampleColor(data.object_color) &&
            estimateSampleSize(data.estimated_width_cm, &sample_size)) {

            event.type = FIELD_ROCK_SAMPLE;
            event.sample_size_cm = sample_size;
            return event;
        }

        event.type = FIELD_HILL;
        return event;
    }

    event.type = FIELD_CLEAR;
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