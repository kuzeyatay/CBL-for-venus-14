#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <libpynq.h>

#include "config.h"
#include "robot_types.h"
#include "odometry.h"
#include "navigation.h"
#include "mission_frame.h"
#include "communication.h"

#define ESP32_UART UART0
#define ESP32_UART_RX_PIN IO_AR0
#define ESP32_UART_TX_PIN IO_AR1
#define ESP32_UART_READY_PIN IO_AR3

#define ESP32_TX_READY_TIMEOUT_MS 50
#define ESP32_RX_BYTE_TIMEOUT_MS 10

static bool esp32_uart_initialized = false;

static bool waitForESP32Ready(void)
{
    for (int elapsed_ms = 0;
         elapsed_ms < ESP32_TX_READY_TIMEOUT_MS;
         elapsed_ms++) {
        if (gpio_get_level(ESP32_UART_READY_PIN) != GPIO_LEVEL_LOW) {
            return true;
        }

        sleep_msec(1);
    }

    return false;
}

static void drainESP32UART(void)
{
    while (uart_has_data(ESP32_UART)) {
        (void)uart_recv(ESP32_UART);
    }
}

static bool readUARTByteWithTimeout(uint8_t *byte)
{
    if (byte == NULL) {
        return false;
    }

    for (int elapsed_ms = 0;
         elapsed_ms < ESP32_RX_BYTE_TIMEOUT_MS;
         elapsed_ms++) {
        if (uart_has_data(ESP32_UART)) {
            *byte = (uint8_t)uart_recv(ESP32_UART);
            return true;
        }

        sleep_msec(1);
    }

    return false;
}

static const char *cellStatusToString(cell_status_t status)
{
    switch (status) {
        case CELL_UNKNOWN: return "unknown";
        case CELL_SCANNED_CLEAR: return "scanned_clear";
        case CELL_RESERVED: return "reserved";
        case CELL_EXPLORED: return "explored";
        case CELL_UNSAFE: return "unsafe";
        case CELL_TAPE: return "tape";
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
    switchbox_set_pin(ESP32_UART_RX_PIN, SWB_UART0_RX);
    switchbox_set_pin(ESP32_UART_TX_PIN, SWB_UART0_TX);
    gpio_set_direction(ESP32_UART_READY_PIN, GPIO_DIR_INPUT);

    uart_init(ESP32_UART);
    uart_reset_fifos(ESP32_UART);

    esp32_uart_initialized = true;
    printf("ESP32 UART initialized: RX=AR0, TX=AR1, ready=AR3\n");
    return true;
}

bool sendPayloadToESP32(const char *payload)
{
    uint32_t len = 0;

    if (payload == NULL) {
        return false;
    }

    printf("UART OUT: %s\n", payload);

    if (!esp32_uart_initialized) {
        printf("UART warning: ESP32 UART not initialized; payload not sent.\n");
        return false;
    }

    len = (uint32_t)strlen(payload);

    if (len == 0 || len >= UART_PAYLOAD_MAX_SIZE) {
        printf("UART warning: invalid payload length %u\n", (unsigned int)len);
        return false;
    }

    if (!waitForESP32Ready()) {
        printf("UART warning: ESP32 not ready on AR3; skipped payload.\n");
        return false;
    }

    uart_send(ESP32_UART, (uint8_t)(len & 0xFF));
    uart_send(ESP32_UART, (uint8_t)((len >> 8) & 0xFF));
    uart_send(ESP32_UART, (uint8_t)((len >> 16) & 0xFF));
    uart_send(ESP32_UART, (uint8_t)((len >> 24) & 0xFF));

    for (uint32_t i = 0; i < len; i++) {
        uart_send(ESP32_UART, (uint8_t)payload[i]);
    }

    return true;
}

bool receivePayloadFromESP32(char *buffer, int buffer_size)
{
    uint32_t len = 0;

    if (!esp32_uart_initialized || buffer == NULL || buffer_size <= 1) {
        return false;
    }

    if (!uart_has_data(ESP32_UART)) {
        return false;
    }

    for (int i = 0; i < 4; i++) {
        uint8_t byte = 0;

        if (!readUARTByteWithTimeout(&byte)) {
            printf("UART warning: timeout while reading payload length.\n");
            drainESP32UART();
            return false;
        }

        len |= (uint32_t)byte << (i * 8);
    }

    if (len == 0 || len >= (uint32_t)buffer_size) {
        printf("UART warning: invalid incoming payload length %u\n",
               (unsigned int)len);
        drainESP32UART();
        return false;
    }

    for (uint32_t i = 0; i < len; i++) {
        uint8_t byte = 0;

        if (!readUARTByteWithTimeout(&byte)) {
            printf("UART warning: timeout while reading payload body.\n");
            drainESP32UART();
            return false;
        }

        buffer[i] = (char)byte;
    }

    buffer[len] = '\0';
    printf("UART IN: %s\n", buffer);
    return true;
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

    if (strcmp(message, "START_MISSION") == 0 ||
        strncmp(message, "START_MISSION,", 14) == 0) {
        startMission();
        sendPayloadToESP32("MISSION_STARTED," ROBOT_ID);
        return;
    }

    if (strcmp(message, "STOP_MISSION") == 0 ||
        strncmp(message, "STOP_MISSION,", 13) == 0) {
        stopMission();
        return;
    }

    if (strcmp(message, "PING") == 0 ||
        strncmp(message, "PING,", 5) == 0) {
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

void sendScanRayUpdate(float yaw, int distance_mm)
{
    char payload[UART_PAYLOAD_MAX_SIZE];

    snprintf(payload,
             sizeof(payload),
             "SCAN_RAY,%s,%.2f,%d",
             ROBOT_ID,
             yaw,
             distance_mm);

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

void sendNavLog(const char *level, const char *format, ...)
{
    char message[UART_PAYLOAD_MAX_SIZE / 2];
    char payload[UART_PAYLOAD_MAX_SIZE];
    va_list args;

    if (level == NULL) {
        level = "INFO";
    }

    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);

    snprintf(payload,
             sizeof(payload),
             "LOG,%s,%s,%s",
             ROBOT_ID,
             level,
             message);

    sendPayloadToESP32(payload);
}
