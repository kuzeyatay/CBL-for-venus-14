#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "robot_types.h"
#include "odometry.h"
#include "navigation.h"
#include "mission_frame.h"
#include "communication.h"

static const char *cellStatusToString(cell_status_t status)
{
    switch (status) {
        case CELL_UNKNOWN: return "unknown";
        case CELL_RESERVED: return "reserved";
        case CELL_EXPLORED: return "explored";
        case CELL_UNSAFE: return "unsafe";
        case CELL_HILL: return "hill";
        case CELL_SAMPLE: return "sample";
        default: return "unknown";
    }
}

static const char *eventTypeToString(field_event_type_t event_type)
{
    switch (event_type) {
        case FIELD_CLEAR: return "clear";
        case FIELD_BLACK_TAPE: return "black_tape";
        case FIELD_HILL: return "hill";
        case FIELD_ROCK_SAMPLE: return "rock_sample";
        case FIELD_SENSOR_FAULT: return "sensor_fault";
        default: return "unknown";
    }
}

static const char *colorToString(sample_color_t color)
{
    switch (color) {
        case COLOR_WHITE: return "white";
        case COLOR_BLACK: return "black";
        case COLOR_RED: return "red";
        case COLOR_GREEN: return "green";
        case COLOR_BLUE: return "blue";
        default: return "unknown";
    }
}

bool initESP32UART(void)
{
    /*
     * TODO: Replace this placeholder with real libpynq UART initialization.
     * The rest of the communication module already builds the text payloads
     * that should be wrapped as: [4 bytes payload_length][payload bytes].
     */
    return false;
}

bool sendPayloadToESP32(const char *payload)
{
    if (payload == NULL) {
        return false;
    }

    printf("UART OUT: %s\n", payload);
    return true;
}

bool receivePayloadFromESP32(char *buffer, int buffer_size)
{
    (void)buffer;
    (void)buffer_size;
    return false;
}

bool pollESP32Messages(void)
{
    char payload[UART_PAYLOAD_MAX_SIZE];

    if (!receivePayloadFromESP32(payload, sizeof(payload))) {
        return false;
    }

    handleBaseStationMessage(payload);
    return true;
}

void handleBaseStationMessage(const char *message)
{
    if (message == NULL) {
        return;
    }

    if (strcmp(message, "START_MISSION") == 0) {
        startMission();
        return;
    }

    if (strcmp(message, "STOP_MISSION") == 0) {
        stopMission();
        return;
    }

    if (strcmp(message, "PING") == 0) {
        sendPayloadToESP32("PONG," ROBOT_ID);
        return;
    }

    printf("Unknown base station message: %s\n", message);
    sendErrorMessage("unknown_command");
}

void sendPoseUpdate(void)
{
    pose_t local_pose = getPose();
    pose_t global_pose = missionFrameLocalPoseToGlobal(local_pose);
    int global_cell_x = 0;
    int global_cell_y = 0;
    char payload[UART_PAYLOAD_MAX_SIZE];

    missionFrameLocalPoseToGlobalCell(local_pose, &global_cell_x, &global_cell_y);

    snprintf(payload,
             sizeof(payload),
             "POSE,%s,%.2f,%.2f,%.2f,%d,%d",
             ROBOT_ID,
             global_pose.x,
             global_pose.y,
             global_pose.yaw,
             global_cell_x,
             global_cell_y);

    sendPayloadToESP32(payload);
}

void sendFieldEventUpdate(field_event_t event)
{
    pose_t local_pose = getPose();
    pose_t global_pose = missionFrameLocalPoseToGlobal(local_pose);
    float local_event_x = local_pose.x;
    float local_event_y = local_pose.y;
    float global_event_x = 0.0f;
    float global_event_y = 0.0f;
    int global_cell_x = 0;
    int global_cell_y = 0;
    char payload[UART_PAYLOAD_MAX_SIZE];

    if (event.distance_mm > 0) {
        float distance_cm = (float)event.distance_mm / 10.0f;
        float yaw_rad = degToRad(local_pose.yaw);

        local_event_x += distance_cm * cosf(yaw_rad);
        local_event_y += distance_cm * sinf(yaw_rad);
    } else {
        float yaw_rad = degToRad(local_pose.yaw);

        local_event_x += CELL_SIZE_CM * cosf(yaw_rad);
        local_event_y += CELL_SIZE_CM * sinf(yaw_rad);
    }

    missionFrameLocalPointToGlobal(local_event_x,
                                    local_event_y,
                                    &global_event_x,
                                    &global_event_y);

    global_cell_x = (int)roundf(global_event_x / CELL_SIZE_CM);
    global_cell_y = (int)roundf(global_event_y / CELL_SIZE_CM);

    snprintf(payload,
             sizeof(payload),
             "EVENT,%s,%s,%.2f,%.2f,%.2f,%d,%d,%d,%.2f,%s,%d,%.2f",
             ROBOT_ID,
             eventTypeToString(event.type),
             global_event_x,
             global_event_y,
             global_pose.yaw,
             global_cell_x,
             global_cell_y,
             event.distance_mm,
             event.width_cm,
             colorToString(event.color),
             event.sample_size_cm,
             event.temperature_c);

    sendPayloadToESP32(payload);
}

void sendCellUpdate(int local_grid_x, int local_grid_y, cell_status_t status)
{
    int global_cell_x = 0;
    int global_cell_y = 0;
    char payload[UART_PAYLOAD_MAX_SIZE];

    missionFrameLocalGridIndexToGlobalCell(local_grid_x,
                                            local_grid_y,
                                            &global_cell_x,
                                            &global_cell_y);

    snprintf(payload,
             sizeof(payload),
             "CELL_UPDATE,%s,%d,%d,%s",
             ROBOT_ID,
             global_cell_x,
             global_cell_y,
             cellStatusToString(status));

    sendPayloadToESP32(payload);
}

void sendStatusUpdate(const char *state, const char *error_code)
{
    char payload[UART_PAYLOAD_MAX_SIZE];

    if (state == NULL) {
        state = "unknown";
    }

    if (error_code == NULL) {
        error_code = "none";
    }

    snprintf(payload,
             sizeof(payload),
             "STATUS,%s,%s,-1,0,%s",
             ROBOT_ID,
             state,
             error_code);

    sendPayloadToESP32(payload);
}

void sendErrorMessage(const char *error_code)
{
    char payload[UART_PAYLOAD_MAX_SIZE];

    if (error_code == NULL) {
        error_code = "unknown_error";
    }

    snprintf(payload,
             sizeof(payload),
             "ERROR,%s,%s",
             ROBOT_ID,
             error_code);

    sendPayloadToESP32(payload);
}
