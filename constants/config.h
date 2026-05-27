#ifndef CONFIG_H
#define CONFIG_H

/*
 * Robot identity
 *
 * Use the same code on both robots, but change these two constants before
 * compiling/flashing each robot.
 *
 * Back-to-back mission convention:
 *   - ROBOT_START_FRONT robot physically points toward the shared +X half.
 *   - ROBOT_START_BACK  robot physically points toward the shared -X half.
 *
 * Internally both robots still use local odometry with yaw = 0 as "forward".
 * The mission_frame module converts local coordinates to the shared GUI frame.
 */
#define ROBOT_ID "R81"                 /* Use "R81" for robot 1 and "R83" for robot 2. */
#define ROBOT_START_FRONT 1
#define ROBOT_START_BACK  2
#define ROBOT_START_ORIENTATION ROBOT_START_FRONT

/*
 * Shared-frame start geometry.
 * The shared GUI origin is the midpoint between the two robots at startup.
 * With START_OFFSET_CELLS = 1:
 *   FRONT robot starts at global cell (+1, 0).
 *   BACK  robot starts at global cell (-1, 0).
 * The global x = 0 column is the no-entry divider/buffer column.
 */
#define ROBOT_START_OFFSET_CELLS 1
#define PARTITION_BUFFER_CELLS 0
#define EXPLORATION_USE_STATIC_PARTITION 1

#define PI 3.14159265359f

/* Stepper/geometry constants */
#define WHEEL_DIAMETER_CM 8.0f
#define STEPS_PER_ROTATION 1600.0f
#define STEPS_PER_CM (STEPS_PER_ROTATION / (PI * WHEEL_DIAMETER_CM))
#define WHEEL_BASE_CM 12.5f
#define TURN_RADIUS (WHEEL_BASE_CM / 2.0f)

#define STEPPER_SPEED_MIN_VALUE 3024
#define STEPPER_SPEED_MAX_VALUE 65535
#define DEFAULT_SPEED 15.0f
#define SECONDS_PER_SPEED_UNIT 0.00000009f

/* 1 = scripted fake readings, 0 = real sensors. */
#define USE_MOCK_SENSORS 0

/* Grid exploration constants */
#define MAP_SIZE 21
#define MAP_CENTER (MAP_SIZE / 2)
#define CELL_SIZE_CM 20.0f
#define FORWARD_INCREMENT_CM 5.0f
#define REVERSE_DISTANCE_CM 6.0f
#define AVOID_TURN_DEG 90.0f
#define SAMPLE_AVOID_TURN_DEG 60.0f
#define MAX_NAVIGATION_STEPS 80

/* Sensor/navigation thresholds */
#define FRONT_OBJECT_THRESHOLD_MM 115
#define SENSOR_RETRY_COUNT 3

#define WIDTH_SCAN_STEP_DEG 2.0f
#define WIDTH_SCAN_MAX_DEG 55.0f
#define WIDTH_SCAN_TURN_SPEED 8.0f
#define WIDTH_SCAN_DISTANCE_MARGIN_MM 20
#define MOVE_AFTER_SCAN_CM 5.5f
#define WIDTH_SCAN_AVERAGE_COUNT 4

#define VL53L0X_INVALID_DISTANCE_MM -1
#define VL53L0X_MAX_REASONABLE_MM 2000

/* TCS3200 pins */
#define PIN_S0 IO_AR4
#define PIN_S1 IO_AR5
#define PIN_S2 IO_AR6
#define PIN_S3 IO_AR7
#define PIN_OUT IO_AR8

#define SAMPLE_TIME_MS 150
#define AVERAGE_SAMPLE_COUNT 8
#define SETTLE_TIME_MS 30

/* UART payload buffer */
#define UART_PAYLOAD_MAX_SIZE 256

/* Sample approach and post-move scan constants */
#define SAMPLE_COLOR_DISTANCE_MM 55
#define SAMPLE_APPROACH_TOLERANCE_MM 5
#define SAMPLE_APPROACH_MAX_CM 20.0f
#define SAMPLE_APPROACH_SPEED_CM_S 5.0f

#define POST_MOVE_SCAN_TOTAL_DEG 60.0f
#define POST_MOVE_SCAN_STEP_DEG 10.0f
#define POST_MOVE_SCAN_SPEED_CM_S 8.0f
#define POST_MOVE_SCAN_MAX_DISTANCE_MM 600
#define POST_MOVE_SCAN_STOP_DISTANCE_MM FRONT_OBJECT_THRESHOLD_MM
#define POST_MOVE_SCAN_APPROACH_MAX_CM FORWARD_INCREMENT_CM

#ifndef HILL_PHYSICAL_SIZE_CM
#define HILL_PHYSICAL_SIZE_CM 30.0f
#endif

#ifndef HILL_FOOTPRINT_MARGIN_CM
#define HILL_FOOTPRINT_MARGIN_CM 7.0f
#endif

#ifndef SAMPLE_FOOTPRINT_MARGIN_CM
#define SAMPLE_FOOTPRINT_MARGIN_CM 2.0f
#endif

#ifndef BLACK_TAPE_FOOTPRINT_WIDTH_CM
#define BLACK_TAPE_FOOTPRINT_WIDTH_CM 14.0f
#endif

#ifndef BLACK_TAPE_FOOTPRINT_DEPTH_CM
#define BLACK_TAPE_FOOTPRINT_DEPTH_CM 7.0f
#endif

#ifndef BLACK_TAPE_FOOTPRINT_DISTANCE_CM
#define BLACK_TAPE_FOOTPRINT_DISTANCE_CM 7.0f
#endif

#endif
