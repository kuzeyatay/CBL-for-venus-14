#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdbool.h>

typedef struct {
    float x;
    float y;
    float yaw;
} pose_t;

typedef enum {
    COLOR_UNKNOWN,
    COLOR_WHITE,
    COLOR_BLACK,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE
} sample_color_t;

typedef enum {
    FIELD_CLEAR,
    FIELD_BLACK_TAPE,
    FIELD_HILL,
    FIELD_ROCK_SAMPLE,
    FIELD_SENSOR_FAULT
} field_event_type_t;

typedef enum {
    CELL_UNKNOWN,
    CELL_RESERVED,
    CELL_EXPLORED,
    CELL_UNSAFE,
    CELL_HILL,
    CELL_SAMPLE
} cell_status_t;

typedef struct {
    bool valid;
    bool black_tape_detected;
    bool front_object_detected;
    int front_distance_mm;
    float estimated_width_cm;
    sample_color_t object_color;
    float temperature_c;
} sensor_data_t;

typedef struct {
    field_event_type_t type;
    int distance_mm;
    float width_cm;
    sample_color_t color;
    int sample_size_cm;
    float temperature_c;
} field_event_t;

#endif
