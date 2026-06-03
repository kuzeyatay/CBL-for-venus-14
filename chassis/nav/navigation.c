#include <math.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>

#include "config.h"
#include "robot_types.h"
#include "odometry.h"
#include "motion.h"
#include "sensor_manager.h"
#include "communication.h"
#include "navigation.h"
#include "mission_frame.h"
#include "distance_sensor.h"
#include "color_sensor.h"
#include "temperature_sensor.h"

static bool nav_log_timestamps_enabled = false;
static double nav_log_start_msec = 0.0;

static double navNowMsec(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1000000.0;
}

static void navPrintf(const char *format, ...)
{
    va_list args;

    if (nav_log_timestamps_enabled) {
        double elapsed_sec = (navNowMsec() - nav_log_start_msec) / 1000.0;

        fprintf(stdout, "[NAV +%8.3fs] ", elapsed_sec);
    }

    va_start(args, format);
    vfprintf(stdout, format, args);
    va_end(args);
}

#define printf(...) navPrintf(__VA_ARGS__)

typedef struct {
    int x;
    int y;
} grid_cell_t;

typedef enum {
    NAV_DIR_POS_X = 0,
    NAV_DIR_POS_Y,
    NAV_DIR_NEG_X,
    NAV_DIR_NEG_Y,
    NAV_DIR_COUNT
} nav_direction_t;

typedef enum {
    NAV_STATE_INIT,
    NAV_STATE_MARK_CURRENT,
    NAV_STATE_SCAN_360,
    NAV_STATE_UPDATE_MAP,
    NAV_STATE_CHOOSE_TARGET,
    NAV_STATE_MOVE_TO_TARGET_CENTER,
    NAV_STATE_INVESTIGATE_OBJECT,
    NAV_STATE_BACKTRACK,
    NAV_STATE_SIDE_COMPLETE,
    NAV_STATE_ERROR
} nav_state_t;

typedef struct {
    int clear_count;
    int object_count;
    int tape_count;
    int min_distance_mm;
} scan_evidence_t;

typedef struct {
    int display_distance_mm;
    int best_close_distance_mm;
    int close_reading_count;
    int invalid_reading_count;
} distance_window_t;

static const char *cellStatusToString(cell_status_t status);
static const char *colorToString(sample_color_t color);
static const char *directionName(nav_direction_t direction);
static const char *eventToString(field_event_type_t event);
static const char *navStateToString(nav_state_t state);

static bool isInsideMap(int x, int y);
static bool isTerminalCellStatus(cell_status_t status);
static bool isBlockedCellStatus(cell_status_t status);
static bool isCellAdjacent(grid_cell_t a, grid_cell_t b);
static bool isRobotFootprintClearForTarget(grid_cell_t cell,
                                           nav_direction_t travel_direction);
static bool isUsableDistance(int distance_mm);
static bool isCloseFrontObjectDistance(int distance_mm);
static bool distanceWindowHasCloseObject(distance_window_t window,
                                         int required_close_readings);
static bool poseToGridCell(float x_cm, float y_cm, int *grid_x, int *grid_y);
static bool poseToCurrentGridCell(grid_cell_t *cell);
static bool shouldTreatBlackReadingAsTape(bool black_detected,
                                          bool close_front_object_detected);
static bool estimateSampleSize(float width_cm, int *sample_size_cm);
static bool pushDfsCell(grid_cell_t cell);
static bool isValidTargetCell(grid_cell_t cell,
                              nav_direction_t travel_direction);
static bool isValidBacktrackCell(grid_cell_t cell);

static float cellCenterXCm(grid_cell_t cell);
static float cellCenterYCm(grid_cell_t cell);
static float directionToYaw(nav_direction_t direction);
static float shortestAngleDelta(float target_yaw_deg, float current_yaw_deg);

static distance_window_t readDistanceWindow(int reading_count);

static grid_cell_t adjacentCell(grid_cell_t cell, nav_direction_t direction);

static nav_direction_t directionFromYaw(float yaw_deg);

static void transitionTo(nav_state_t next_state, const char *reason);
static void initMap(void);
static void resetNavigationRuntime(void);
static void setMapCellStatus(int x, int y, cell_status_t status);
static void resetScanEvidence(void);
static void turnToYaw(float target_yaw_deg, float speed_cm_s);
static void turnTowardPoint(float target_x_cm, float target_y_cm, float speed_cm_s);
static void returnToPoseApprox(pose_t saved_pose);
static void reverseToCellCenter(grid_cell_t cell, float final_yaw_deg);
static void moveToDistanceFromObject(int target_distance_mm);
static void finalizeRockSampleAtCloseRange(field_event_t *event);
static void reportFieldEvent(field_event_t event);
static void markHillBlockAroundCell(grid_cell_t center_cell);
static void markSampleAtCell(grid_cell_t cell);
static void markTapeAtCell(grid_cell_t cell);
static void markCurrentCellState(void);
static void runScan360(void);
static void updateMapFromScan(void);
static void chooseNextTarget(void);
static void investigateObjectCandidate(void);
static void handleMovementObject(grid_cell_t affected_cell,
                                 grid_cell_t return_cell,
                                 float return_yaw_deg);
static void moveToTargetCenter(void);
static void handleBacktrack(void);
static void handleSideComplete(void);
static void handleNavigationError(void);

static field_event_t interpretSensorData(sensor_data_t data);

static bool mission_running = false;
static bool map_initialized = false;
static cell_status_t map_grid[MAP_SIZE][MAP_SIZE];

static nav_state_t current_state = NAV_STATE_INIT;
static grid_cell_t current_cell = {MAP_CENTER, MAP_CENTER};
static grid_cell_t target_cell = {MAP_CENTER, MAP_CENTER};
static grid_cell_t object_candidate_cell = {MAP_CENTER, MAP_CENTER};
static int object_candidate_dir = -1;

static scan_evidence_t scan_evidence[NAV_DIR_COUNT];

static grid_cell_t dfs_stack[MAP_SIZE * MAP_SIZE];
static int dfs_stack_top = 0;

static const char *cellStatusToString(cell_status_t status)
{
    switch (status) {
        case CELL_UNKNOWN: return "unknown";
        case CELL_SCANNED_CLEAR: return "scanned_clear";
        case CELL_RESERVED: return "reserved";
        case CELL_EXPLORED: return "explored";
        case CELL_UNSAFE: return "unsafe";
        case CELL_HILL: return "hill";
        case CELL_SAMPLE: return "sample";
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

static const char *directionName(nav_direction_t direction)
{
    switch (direction) {
        case NAV_DIR_POS_X: return "+x";
        case NAV_DIR_POS_Y: return "+y";
        case NAV_DIR_NEG_X: return "-x";
        case NAV_DIR_NEG_Y: return "-y";
        default: return "unknown";
    }
}

static const char *eventToString(field_event_type_t event)
{
    switch (event) {
        case FIELD_CLEAR: return "clear";
        case FIELD_BLACK_TAPE: return "black_tape";
        case FIELD_HILL: return "hill";
        case FIELD_ROCK_SAMPLE: return "rock_sample";
        case FIELD_SENSOR_FAULT: return "sensor_fault";
        default: return "unknown";
    }
}

static const char *navStateToString(nav_state_t state)
{
    switch (state) {
        case NAV_STATE_INIT: return "NAV_STATE_INIT";
        case NAV_STATE_MARK_CURRENT: return "NAV_STATE_MARK_CURRENT";
        case NAV_STATE_SCAN_360: return "NAV_STATE_SCAN_360";
        case NAV_STATE_UPDATE_MAP: return "NAV_STATE_UPDATE_MAP";
        case NAV_STATE_CHOOSE_TARGET: return "NAV_STATE_CHOOSE_TARGET";
        case NAV_STATE_MOVE_TO_TARGET_CENTER: return "NAV_STATE_MOVE_TO_TARGET_CENTER";
        case NAV_STATE_INVESTIGATE_OBJECT: return "NAV_STATE_INVESTIGATE_OBJECT";
        case NAV_STATE_BACKTRACK: return "NAV_STATE_BACKTRACK";
        case NAV_STATE_SIDE_COMPLETE: return "NAV_STATE_SIDE_COMPLETE";
        case NAV_STATE_ERROR: return "NAV_STATE_ERROR";
        default: return "NAV_STATE_UNKNOWN";
    }
}

static bool isInsideMap(int x, int y)
{
    return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE;
}

static bool isTerminalCellStatus(cell_status_t status)
{
    return status == CELL_UNSAFE ||
           status == CELL_HILL ||
           status == CELL_SAMPLE;
}

static bool isBlockedCellStatus(cell_status_t status)
{
    return status == CELL_UNSAFE ||
           status == CELL_HILL ||
           status == CELL_SAMPLE;
}

static bool isCellAdjacent(grid_cell_t a, grid_cell_t b)
{
    int dx = a.x - b.x;
    int dy = a.y - b.y;

    if (dx < 0) {
        dx = -dx;
    }

    if (dy < 0) {
        dy = -dy;
    }

    return dx + dy == 1;
}

static bool isRobotFootprintClearForTarget(grid_cell_t cell,
                                           nav_direction_t travel_direction)
{
    int forward_x = 0;
    int forward_y = 0;
    int side_x = 0;
    int side_y = 0;

    switch (travel_direction) {
        case NAV_DIR_POS_X:
            forward_x = 1;
            side_y = 1;
            break;

        case NAV_DIR_POS_Y:
            forward_y = 1;
            side_x = 1;
            break;

        case NAV_DIR_NEG_X:
            forward_x = -1;
            side_y = 1;
            break;

        case NAV_DIR_NEG_Y:
            forward_y = -1;
            side_x = 1;
            break;

        default:
            break;
    }

    for (int along = -ROBOT_FOOTPRINT_MARGIN_CELLS;
         along <= ROBOT_FOOTPRINT_MARGIN_CELLS;
         along++) {
        for (int side = -ROBOT_FOOTPRINT_SIDE_MARGIN_CELLS;
             side <= ROBOT_FOOTPRINT_SIDE_MARGIN_CELLS;
             side++) {
            int x = cell.x + along * forward_x + side * side_x;
            int y = cell.y + along * forward_y + side * side_y;

            if (!isInsideMap(x, y)) {
                return false;
            }

            if (!missionFrameLocalGridIndexIsInAssignedHalf(x, y)) {
                return false;
            }

            if (isBlockedCellStatus(map_grid[x][y])) {
                printf("TARGET footprint blocked: target=(%d,%d), dir=%s, blocked=(%d,%d) status=%s\n",
                       cell.x,
                       cell.y,
                       directionName(travel_direction),
                       x,
                       y,
                       cellStatusToString(map_grid[x][y]));
                return false;
            }
        }
    }

    return true;
}

static bool isUsableDistance(int distance_mm)
{
    return distance_mm > 0 &&
           distance_mm != VL53L0X_INVALID_DISTANCE_MM &&
           distance_mm <= VL53L0X_MAX_REASONABLE_MM;
}

static bool isCloseFrontObjectDistance(int distance_mm)
{
    return isUsableDistance(distance_mm) &&
           distance_mm >= FRONT_OBJECT_MIN_DISTANCE_MM &&
           distance_mm <= FRONT_OBJECT_MAX_DISTANCE_MM;
}

static bool distanceWindowHasCloseObject(distance_window_t window,
                                         int required_close_readings)
{
    return window.close_reading_count >= required_close_readings;
}

static bool poseToGridCell(float x_cm, float y_cm, int *grid_x, int *grid_y)
{
    int gx = MAP_CENTER + (int)roundf(x_cm / CELL_SIZE_CM);
    int gy = MAP_CENTER + (int)roundf(y_cm / CELL_SIZE_CM);

    if (!isInsideMap(gx, gy)) {
        return false;
    }

    *grid_x = gx;
    *grid_y = gy;
    return true;
}

static bool poseToCurrentGridCell(grid_cell_t *cell)
{
    pose_t pose = getPose();
    int grid_x = 0;
    int grid_y = 0;

    if (!poseToGridCell(pose.x, pose.y, &grid_x, &grid_y)) {
        return false;
    }

    cell->x = grid_x;
    cell->y = grid_y;
    return true;
}

static bool shouldTreatBlackReadingAsTape(bool black_detected,
                                          bool close_front_object_detected)
{
    /*
     * A black rock sample can also make the TCS3200 return "black".
     * Therefore, black is only interpreted as floor tape when there is no
     * close object in front of the robot according to the VL53L0X.
     */
    return black_detected && !close_front_object_detected;
}

static bool estimateSampleSize(float width_cm, int *sample_size_cm)
{
    /*
     * These thresholds are empirical for this robot.
     * The VL53L0X width scan underestimates real cube width because the robot
     * rotates around its wheel axis, not around the distance sensor itself.
     */

    if (width_cm >= 1.5f && width_cm < 3.4f) {
        *sample_size_cm = 3;
        return true;
    }

    if (width_cm >= 3.4f && width_cm <= 7.5f) {
        *sample_size_cm = 6;
        return true;
    }

    return false;
}

static bool pushDfsCell(grid_cell_t cell)
{
    if (dfs_stack_top > 0 &&
        dfs_stack[dfs_stack_top - 1].x == cell.x &&
        dfs_stack[dfs_stack_top - 1].y == cell.y) {
#if NAV_DEBUG_DFS
        printf("DFS PUSH skipped duplicate cell=(%d,%d), stack_size=%d\n",
               cell.x,
               cell.y,
               dfs_stack_top);
#endif
        return true;
    }

    if (dfs_stack_top >= (int)(sizeof(dfs_stack) / sizeof(dfs_stack[0]))) {
        printf("DFS PUSH failed: stack full at %d entries\n", dfs_stack_top);
        return false;
    }

    dfs_stack[dfs_stack_top] = cell;
    dfs_stack_top++;

#if NAV_DEBUG_DFS
    printf("DFS PUSH cell=(%d,%d), stack_size=%d\n",
           cell.x,
           cell.y,
           dfs_stack_top);
#endif

    return true;
}

static bool isValidTargetCell(grid_cell_t cell,
                              nav_direction_t travel_direction)
{
    if (!isInsideMap(cell.x, cell.y)) {
        return false;
    }

    if (!missionFrameLocalGridIndexIsInAssignedHalf(cell.x, cell.y)) {
        return false;
    }

    if (map_grid[cell.x][cell.y] != CELL_SCANNED_CLEAR) {
        return false;
    }

    return isRobotFootprintClearForTarget(cell, travel_direction);
}

static bool isValidBacktrackCell(grid_cell_t cell)
{
    if (!isInsideMap(cell.x, cell.y)) {
        return false;
    }

    if (!missionFrameLocalGridIndexIsInAssignedHalf(cell.x, cell.y)) {
        return false;
    }

    if (map_grid[cell.x][cell.y] != CELL_EXPLORED) {
        return false;
    }

    return isCellAdjacent(current_cell, cell);
}

static float cellCenterXCm(grid_cell_t cell)
{
    return (float)(cell.x - MAP_CENTER) * CELL_SIZE_CM;
}

static float cellCenterYCm(grid_cell_t cell)
{
    return (float)(cell.y - MAP_CENTER) * CELL_SIZE_CM;
}

static float directionToYaw(nav_direction_t direction)
{
    switch (direction) {
        case NAV_DIR_POS_X: return 0.0f;
        case NAV_DIR_POS_Y: return 90.0f;
        case NAV_DIR_NEG_X: return 180.0f;
        case NAV_DIR_NEG_Y: return 270.0f;
        default: return 0.0f;
    }
}

static float shortestAngleDelta(float target_yaw_deg, float current_yaw_deg)
{
    float delta = normalizeAngle(target_yaw_deg - current_yaw_deg);

    if (delta > 180.0f) {
        delta -= 360.0f;
    }

    return delta;
}

static distance_window_t readDistanceWindow(int reading_count)
{
    distance_window_t window;

    window.display_distance_mm = VL53L0X_INVALID_DISTANCE_MM;
    window.best_close_distance_mm = VL53L0X_INVALID_DISTANCE_MM;
    window.close_reading_count = 0;
    window.invalid_reading_count = 0;

    for (int sample = 0; sample < reading_count; sample++) {
        int sample_distance_mm = readVL53L0XDistance();

        if (!isUsableDistance(sample_distance_mm)) {
            window.invalid_reading_count++;
            continue;
        }

        window.display_distance_mm = sample_distance_mm;

        if (isCloseFrontObjectDistance(sample_distance_mm)) {
            window.close_reading_count++;

            if (window.best_close_distance_mm == VL53L0X_INVALID_DISTANCE_MM ||
                sample_distance_mm < window.best_close_distance_mm) {
                window.best_close_distance_mm = sample_distance_mm;
            }
        }
    }

    if (window.best_close_distance_mm != VL53L0X_INVALID_DISTANCE_MM) {
        window.display_distance_mm = window.best_close_distance_mm;
    }

    return window;
}

static grid_cell_t adjacentCell(grid_cell_t cell, nav_direction_t direction)
{
    grid_cell_t adjacent = cell;

    switch (direction) {
        case NAV_DIR_POS_X:
            adjacent.x++;
            break;
        case NAV_DIR_POS_Y:
            adjacent.y++;
            break;
        case NAV_DIR_NEG_X:
            adjacent.x--;
            break;
        case NAV_DIR_NEG_Y:
            adjacent.y--;
            break;
        default:
            break;
    }

    return adjacent;
}

static nav_direction_t directionFromYaw(float yaw_deg)
{
    float yaw = normalizeAngle(yaw_deg);

    if (yaw >= 315.0f || yaw < 45.0f) {
        return NAV_DIR_POS_X;
    }

    if (yaw < 135.0f) {
        return NAV_DIR_POS_Y;
    }

    if (yaw < 225.0f) {
        return NAV_DIR_NEG_X;
    }

    return NAV_DIR_NEG_Y;
}

static void transitionTo(nav_state_t next_state, const char *reason)
{
#if NAV_DEBUG_STATE
    printf("NAV STATE: %s -> %s, reason=%s\n",
           navStateToString(current_state),
           navStateToString(next_state),
           reason == NULL ? "none" : reason);
#endif

    current_state = next_state;
}

static void initMap(void)
{
    for (int x = 0; x < MAP_SIZE; x++) {
        for (int y = 0; y < MAP_SIZE; y++) {
            map_grid[x][y] = CELL_UNKNOWN;
        }
    }

    map_initialized = true;
    printf("Map initialized: size=%d, center=%d, cell_size=%.2f cm\n",
           MAP_SIZE,
           MAP_CENTER,
           CELL_SIZE_CM);
}

static void resetNavigationRuntime(void)
{
    dfs_stack_top = 0;
    current_cell.x = MAP_CENTER;
    current_cell.y = MAP_CENTER;
    target_cell = current_cell;
    object_candidate_cell = current_cell;
    object_candidate_dir = -1;
    resetScanEvidence();
}

static void setMapCellStatus(int x, int y, cell_status_t status)
{
    if (!isInsideMap(x, y)) {
        printf("MAP CELL ignored outside map: (%d,%d) -> %s\n",
               x,
               y,
               cellStatusToString(status));
        return;
    }

    cell_status_t previous = map_grid[x][y];

    if (previous == status) {
        printf("MAP CELL (%d,%d): %s -> %s (unchanged)\n",
               x,
               y,
               cellStatusToString(previous),
               cellStatusToString(status));
        sendCellUpdate(x, y, status);
        return;
    }

    if (isTerminalCellStatus(previous)) {
        printf("MAP CELL protected: (%d,%d) stays %s, rejected %s\n",
               x,
               y,
               cellStatusToString(previous),
               cellStatusToString(status));
        return;
    }

    if (status == CELL_SCANNED_CLEAR && previous == CELL_EXPLORED) {
        printf("MAP CELL weak clear ignored: (%d,%d) stays %s\n",
               x,
               y,
               cellStatusToString(previous));
        return;
    }

    map_grid[x][y] = status;

    printf("MAP CELL (%d,%d): %s -> %s\n",
           x,
           y,
           cellStatusToString(previous),
           cellStatusToString(status));
    sendCellUpdate(x, y, status);
}

static void resetScanEvidence(void)
{
    for (int dir = 0; dir < NAV_DIR_COUNT; dir++) {
        scan_evidence[dir].clear_count = 0;
        scan_evidence[dir].object_count = 0;
        scan_evidence[dir].tape_count = 0;
        scan_evidence[dir].min_distance_mm = VL53L0X_INVALID_DISTANCE_MM;
    }
}

static void turnToYaw(float target_yaw_deg, float speed_cm_s)
{
    pose_t pose = getPose();
    float delta = shortestAngleDelta(target_yaw_deg, pose.yaw);

    if (fabsf(delta) <= GYRO_TURN_FINE_SKIP_DEG) {
        printf("TURN TO YAW skipped tiny correction target=%.2f, current=%.2f, delta=%.2f\n",
               normalizeAngle(target_yaw_deg),
               pose.yaw,
               delta);
        return;
    }

    turn(delta, speed_cm_s);
}

static void turnTowardPoint(float target_x_cm, float target_y_cm, float speed_cm_s)
{
    pose_t pose = getPose();
    float delta_x = target_x_cm - pose.x;
    float delta_y = target_y_cm - pose.y;
    float target_yaw = atan2f(delta_y, delta_x) * 180.0f / PI;

    turnToYaw(target_yaw, speed_cm_s);
}

static void returnToPoseApprox(pose_t saved_pose)
{
    pose_t pose = getPose();
    float delta_x = saved_pose.x - pose.x;
    float delta_y = saved_pose.y - pose.y;
    float distance_cm = sqrtf(delta_x * delta_x + delta_y * delta_y);

    if (distance_cm > CELL_CENTER_REACHED_TOLERANCE_CM) {
        printf("RETURN CELL CENTER: distance=%.2f cm\n", distance_cm);
        turnTowardPoint(saved_pose.x, saved_pose.y, DEFAULT_SPEED);
        moveWithRamp(distance_cm, DEFAULT_SPEED);
        sendPoseUpdate();
    }

    turnToYaw(saved_pose.yaw, DEFAULT_SPEED);
    sendPoseUpdate();
}

static void reverseToCellCenter(grid_cell_t cell, float final_yaw_deg)
{
    pose_t pose = getPose();
    pose_t target_pose;
    float delta_x = 0.0f;
    float delta_y = 0.0f;
    float distance_cm = 0.0f;
    float target_yaw = 0.0f;
    float reverse_yaw = 0.0f;
    float reverse_alignment_error = 0.0f;

    target_pose.x = cellCenterXCm(cell);
    target_pose.y = cellCenterYCm(cell);
    target_pose.yaw = final_yaw_deg;

    delta_x = target_pose.x - pose.x;
    delta_y = target_pose.y - pose.y;
    distance_cm = sqrtf(delta_x * delta_x + delta_y * delta_y);

    if (distance_cm > CELL_CENTER_REACHED_TOLERANCE_CM) {
        target_yaw = atan2f(delta_y, delta_x) * 180.0f / PI;
        reverse_yaw = normalizeAngle(target_yaw + 180.0f);
        reverse_alignment_error =
            fabsf(shortestAngleDelta(reverse_yaw, pose.yaw));

        if (reverse_alignment_error <= REVERSE_TO_CELL_ALIGNMENT_TOLERANCE_DEG) {
            printf("REVERSE CELL CENTER: distance=%.2f cm, target=(%d,%d), alignment_error=%.2f deg\n",
                   distance_cm,
                   cell.x,
                   cell.y,
                   reverse_alignment_error);
            moveWithRamp(-distance_cm, DEFAULT_SPEED);
            sendPoseUpdate();
        } else {
            printf("REVERSE CELL CENTER fallback: alignment_error=%.2f deg, using point return\n",
                   reverse_alignment_error);
            returnToPoseApprox(target_pose);
            return;
        }
    }

    turnToYaw(final_yaw_deg, DEFAULT_SPEED);
    sendPoseUpdate();
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
    int abs_error_mm = error_mm < 0 ? -error_mm : error_mm;

    if (abs_error_mm <= SAMPLE_APPROACH_TOLERANCE_MM) {
        printf("Already close enough for color reading: %d mm\n", distance_mm);
        return;
    }

    float move_cm = (float)error_mm / 10.0f;

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

static void reportFieldEvent(field_event_t event)
{
    printf("REPORT: robot=%s, event=%s, distance=%d mm, width=%.2f cm, color=%s, size=%d cm, temp=%.2f C\n",
           ROBOT_ID,
           eventToString(event.type),
           event.distance_mm,
           event.width_cm,
           colorToString(event.color),
           event.sample_size_cm,
           event.temperature_c);

    sendFieldEventUpdate(event);
}

static void markHillBlockAroundCell(grid_cell_t center_cell)
{
    float hill_cells = HILL_PHYSICAL_SIZE_CM / CELL_SIZE_CM;
    int base_radius = (int)ceilf((hill_cells - 1.0f) / 2.0f);

    if (base_radius < 1) {
        base_radius = 1;
    }

    int radius = base_radius + HILL_FOOTPRINT_MARGIN_CELLS;

    printf("MARK HILL center=(%d,%d), radius=%d cells, physical_size=%.2f cm\n",
           center_cell.x,
           center_cell.y,
           radius,
           HILL_PHYSICAL_SIZE_CM);

    for (int dx = -radius; dx <= radius; dx++) {
        for (int dy = -radius; dy <= radius; dy++) {
            int x = center_cell.x + dx;
            int y = center_cell.y + dy;

            if (!isInsideMap(x, y)) {
                continue;
            }

            if (!missionFrameLocalGridIndexIsInAssignedHalf(x, y)) {
                continue;
            }

            if (x == current_cell.x && y == current_cell.y) {
                printf("MARK HILL skip current cell=(%d,%d)\n", x, y);
                continue;
            }

            if (map_grid[x][y] == CELL_EXPLORED) {
                printf("MARK HILL skip explored cell=(%d,%d)\n", x, y);
                continue;
            }

            setMapCellStatus(x, y, CELL_HILL);
        }
    }
}

static void markSampleAtCell(grid_cell_t cell)
{
    printf("MARK SAMPLE cell=(%d,%d)\n", cell.x, cell.y);
    setMapCellStatus(cell.x, cell.y, CELL_SAMPLE);
}

static void markTapeAtCell(grid_cell_t cell)
{
    printf("MARK UNSAFE tape cell=(%d,%d)\n", cell.x, cell.y);
    setMapCellStatus(cell.x, cell.y, CELL_UNSAFE);
}

static void markCurrentCellState(void)
{
    grid_cell_t cell;
    pose_t pose = getPose();
    int global_cell_x = 0;
    int global_cell_y = 0;

    if (!poseToCurrentGridCell(&cell)) {
        transitionTo(NAV_STATE_ERROR, "current pose outside local map");
        return;
    }

    current_cell = cell;

    if (!missionFrameLocalGridIndexIsInAssignedHalf(current_cell.x,
                                                     current_cell.y)) {
        transitionTo(NAV_STATE_ERROR, "current cell outside assigned half");
        return;
    }

    if (!isTerminalCellStatus(map_grid[current_cell.x][current_cell.y])) {
        setMapCellStatus(current_cell.x, current_cell.y, CELL_EXPLORED);
    } else {
        printf("Current cell is terminal and will not be overwritten: (%d,%d)=%s\n",
               current_cell.x,
               current_cell.y,
               cellStatusToString(map_grid[current_cell.x][current_cell.y]));
    }

    missionFrameLocalGridIndexToGlobalCell(current_cell.x,
                                            current_cell.y,
                                            &global_cell_x,
                                            &global_cell_y);

    printf("MARK CURRENT local=(%d,%d), global=(%d,%d), yaw=%.2f\n",
           current_cell.x,
           current_cell.y,
           global_cell_x,
           global_cell_y,
           pose.yaw);

    sendPoseUpdate();
    transitionTo(NAV_STATE_SCAN_360, "current cell marked explored");
}

static void runScan360(void)
{
    pose_t start_pose = getPose();
    float original_yaw = start_pose.yaw;
    float scanned_deg = 0.0f;
    bool tape_scan_enabled = isTCS3200Calibrated();

    resetScanEvidence();
    recalibrateGyroIfReady();

#if NAV_DEBUG_SCAN360
    printf("SCAN360 START yaw=%.2f, step=%.2f, tape_scan=%s\n",
           original_yaw,
           SCAN_360_STEP_DEG,
           tape_scan_enabled ? "enabled" : "disabled");
#endif

    while (scanned_deg <= 360.0f + 0.01f) {
        pose_t pose = getPose();
        float display_yaw = pose.yaw;
        bool is_heading_check_ray = scanned_deg >= 360.0f - 0.01f;
        nav_direction_t direction = directionFromYaw(pose.yaw);
        distance_window_t distance_window =
            readDistanceWindow(SCAN_DISTANCE_READINGS_PER_ANGLE);
        bool close_object =
            distanceWindowHasCloseObject(distance_window,
                                         SCAN_ANGLE_OBJECT_MIN_CLOSE_READINGS);
        bool raw_black = false;
        bool black_tape = false;
        bool clear = false;

        if (tape_scan_enabled) {
            raw_black = tcs3200DetectBlackTape();
        }

        black_tape = shouldTreatBlackReadingAsTape(raw_black, close_object);
        /*
         * V1 only classifies adjacent cells.  A VL53 out-of-range reading means
         * there is no close object in that direction, so it is useful clear
         * evidence on an empty field instead of "no information".
         */
        clear = !close_object && !black_tape;

        if (!is_heading_check_ray) {
            scan_evidence_t *evidence = &scan_evidence[direction];

            if (close_object) {
                evidence->object_count++;

                if (evidence->min_distance_mm == VL53L0X_INVALID_DISTANCE_MM ||
                    distance_window.best_close_distance_mm < evidence->min_distance_mm) {
                    evidence->min_distance_mm = distance_window.best_close_distance_mm;
                }
            } else if (black_tape) {
                evidence->tape_count++;
            } else if (clear) {
                evidence->clear_count++;
            }
        }

#if NAV_DEBUG_SCAN360
        if (is_heading_check_ray) {
            display_yaw = 360.0f;
        }

        printf("SCAN360 RAY yaw=%.2f, dir=%s, distance=%d, close_reads=%d/%d, invalid_reads=%d, raw_black=%s, tape=%s, object=%s, clear=%s, counted=%s\n",
               display_yaw,
               directionName(direction),
               distance_window.display_distance_mm,
               distance_window.close_reading_count,
               SCAN_DISTANCE_READINGS_PER_ANGLE,
               distance_window.invalid_reading_count,
               raw_black ? "yes" : "no",
               black_tape ? "yes" : "no",
               close_object ? "yes" : "no",
               clear ? "yes" : "no",
               is_heading_check_ray ? "no" : "yes");
#endif

        if (is_heading_check_ray) {
            break;
        }

        float remaining_deg = 360.0f - scanned_deg;
        float step_deg = SCAN_360_STEP_DEG;

        if (step_deg > remaining_deg) {
            step_deg = remaining_deg;
        }

        scanned_deg += step_deg;

        if (scanned_deg >= 360.0f - 0.01f) {
            scanned_deg = 360.0f;
            turnToYaw(original_yaw, DEFAULT_SPEED);
        } else {
            turn(step_deg, DEFAULT_SPEED);
        }
    }

    pose_t completed_pose = getPose();
    printf("SCAN360 COMPLETE original_yaw=%.2f, final_yaw=%.2f\n",
           original_yaw,
           completed_pose.yaw);
    odometryInit(completed_pose.x, completed_pose.y, original_yaw);
    sendPoseUpdate();

#if NAV_DEBUG_SCAN360
    for (int dir = 0; dir < NAV_DIR_COUNT; dir++) {
        scan_evidence_t evidence = scan_evidence[dir];

        printf("SCAN360 EVIDENCE dir=%s, clear=%d, object=%d, tape=%d, min_distance=%d\n",
               directionName((nav_direction_t)dir),
               evidence.clear_count,
               evidence.object_count,
               evidence.tape_count,
               evidence.min_distance_mm);
    }
#endif

    transitionTo(NAV_STATE_UPDATE_MAP, "360 scan complete");
}

static void updateMapFromScan(void)
{
    int best_object_distance_mm = VL53L0X_MAX_REASONABLE_MM + 1;

    object_candidate_dir = -1;
    object_candidate_cell = current_cell;

    for (int dir = 0; dir < NAV_DIR_COUNT; dir++) {
        nav_direction_t direction = (nav_direction_t)dir;
        grid_cell_t cell = adjacentCell(current_cell, direction);
        scan_evidence_t evidence = scan_evidence[dir];
        bool has_object = evidence.object_count >= SCAN_OBJECT_MIN_READINGS;
        bool has_tape = evidence.tape_count >= SCAN_TAPE_MIN_READINGS;

        if (!isInsideMap(cell.x, cell.y)) {
            printf("SCAN360 EVIDENCE ignored outside map dir=%s cell=(%d,%d)\n",
                   directionName(direction),
                   cell.x,
                   cell.y);
            continue;
        }

        if (!missionFrameLocalGridIndexIsInAssignedHalf(cell.x, cell.y)) {
            printf("SCAN360 EVIDENCE ignored outside assigned half dir=%s cell=(%d,%d)\n",
                   directionName(direction),
                   cell.x,
                   cell.y);
            continue;
        }

        cell_status_t status = map_grid[cell.x][cell.y];

        if (status == CELL_EXPLORED || isTerminalCellStatus(status)) {
            printf("SCAN360 EVIDENCE ignored existing status dir=%s cell=(%d,%d) status=%s\n",
                   directionName(direction),
                   cell.x,
                   cell.y,
                   cellStatusToString(status));
            continue;
        }

        if (has_tape && !has_object) {
            markTapeAtCell(cell);
        } else if (has_object) {
            printf("SCAN360 EVIDENCE object candidate dir=%s cell=(%d,%d) min_distance=%d\n",
                   directionName(direction),
                   cell.x,
                   cell.y,
                   evidence.min_distance_mm);

            if (evidence.min_distance_mm > 0 &&
                evidence.min_distance_mm < best_object_distance_mm) {
                best_object_distance_mm = evidence.min_distance_mm;
                object_candidate_dir = dir;
                object_candidate_cell = cell;
            }
        } else if (evidence.clear_count >= SCAN_CLEAR_MIN_READINGS) {
            printf("MARK SCANNED_CLEAR cell=(%d,%d), dir=%s\n",
                   cell.x,
                   cell.y,
                   directionName(direction));
            setMapCellStatus(cell.x, cell.y, CELL_SCANNED_CLEAR);
        }
    }

    if (object_candidate_dir >= 0) {
        transitionTo(NAV_STATE_INVESTIGATE_OBJECT,
                     "strongest adjacent object candidate selected");
    } else {
        transitionTo(NAV_STATE_CHOOSE_TARGET,
                     "no adjacent object candidate after map update");
    }
}

static void chooseNextTarget(void)
{
    for (int dir = 0; dir < NAV_DIR_COUNT; dir++) {
        nav_direction_t direction = (nav_direction_t)dir;
        grid_cell_t candidate = adjacentCell(current_cell, direction);

        if (!isValidTargetCell(candidate, direction)) {
            continue;
        }

        if (!pushDfsCell(current_cell)) {
            transitionTo(NAV_STATE_ERROR, "DFS stack full");
            return;
        }

        target_cell = candidate;
        printf("MOVE TARGET CENTER selected dir=%s, target=(%d,%d)\n",
               directionName(direction),
               target_cell.x,
               target_cell.y);
        setMapCellStatus(target_cell.x, target_cell.y, CELL_RESERVED);
        transitionTo(NAV_STATE_MOVE_TO_TARGET_CENTER,
                     "deterministic adjacent scanned_clear target");
        return;
    }

    transitionTo(NAV_STATE_BACKTRACK, "no adjacent scanned_clear target");
}

static void investigateObjectCandidate(void)
{
    if (object_candidate_dir < 0) {
        transitionTo(NAV_STATE_CHOOSE_TARGET, "no object candidate to investigate");
        return;
    }

    if (!isInsideMap(object_candidate_cell.x, object_candidate_cell.y)) {
        transitionTo(NAV_STATE_CHOOSE_TARGET, "object candidate outside map");
        return;
    }

    if (isTerminalCellStatus(map_grid[object_candidate_cell.x][object_candidate_cell.y])) {
        transitionTo(NAV_STATE_CHOOSE_TARGET,
                     "object candidate already classified as terminal");
        return;
    }

    pose_t saved_pose = getPose();
    nav_direction_t direction = (nav_direction_t)object_candidate_dir;

    printf("INVESTIGATE OBJECT dir=%s, cell=(%d,%d)\n",
           directionName(direction),
           object_candidate_cell.x,
           object_candidate_cell.y);

    turnToYaw(directionToYaw(direction), DEFAULT_SPEED);

    distance_window_t confirm_window =
        readDistanceWindow(INVESTIGATE_DISTANCE_READINGS_PER_CONFIRMATION);

    if (!distanceWindowHasCloseObject(confirm_window,
                                      INVESTIGATE_OBJECT_MIN_CLOSE_READINGS)) {
        printf("Object confirmation failed: close_reads=%d/%d, invalid_reads=%d, distance=%d\n",
               confirm_window.close_reading_count,
               INVESTIGATE_DISTANCE_READINGS_PER_CONFIRMATION,
               confirm_window.invalid_reading_count,
               confirm_window.display_distance_mm);
        setMapCellStatus(object_candidate_cell.x,
                         object_candidate_cell.y,
                         CELL_SCANNED_CLEAR);
        returnToPoseApprox(saved_pose);
        object_candidate_dir = -1;
        transitionTo(NAV_STATE_CHOOSE_TARGET,
                     "object candidate rejected by distance window");
        return;
    }

    sensor_data_t sensor_data = readReliableSensorData();
    field_event_t event = interpretSensorData(sensor_data);

    switch (event.type) {
        case FIELD_ROCK_SAMPLE:
            finalizeRockSampleAtCloseRange(&event);
            reportFieldEvent(event);
            markSampleAtCell(object_candidate_cell);
            returnToPoseApprox(saved_pose);
            break;

        case FIELD_HILL:
            if (event.width_cm < HILL_PHYSICAL_SIZE_CM) {
                event.width_cm = HILL_PHYSICAL_SIZE_CM;
            }

            reportFieldEvent(event);
            markHillBlockAroundCell(object_candidate_cell);
            returnToPoseApprox(saved_pose);
            break;

        case FIELD_BLACK_TAPE:
            reportFieldEvent(event);
            markTapeAtCell(object_candidate_cell);
            returnToPoseApprox(saved_pose);
            break;

        case FIELD_SENSOR_FAULT:
            printf("Object confirmation failed: sensor fault\n");
            reportFieldEvent(event);
            setMapCellStatus(object_candidate_cell.x,
                             object_candidate_cell.y,
                             CELL_SCANNED_CLEAR);
            returnToPoseApprox(saved_pose);
            break;

        case FIELD_CLEAR:
        default:
            printf("Object confirmation failed: event=%s\n",
                   eventToString(event.type));
            setMapCellStatus(object_candidate_cell.x,
                             object_candidate_cell.y,
                             CELL_SCANNED_CLEAR);
            returnToPoseApprox(saved_pose);
            break;
    }

    object_candidate_dir = -1;
    transitionTo(NAV_STATE_CHOOSE_TARGET, "object investigation complete");
}

static void handleMovementObject(grid_cell_t affected_cell,
                                 grid_cell_t return_cell,
                                 float return_yaw_deg)
{
    distance_window_t confirm_window =
        readDistanceWindow(MOVE_DISTANCE_READINGS_PER_CHECK);

    if (!distanceWindowHasCloseObject(confirm_window,
                                      MOVE_OBJECT_MIN_CLOSE_READINGS)) {
        printf("MOVE TARGET CENTER object rejected: close_reads=%d/%d, invalid_reads=%d, distance=%d affected_cell=(%d,%d)\n",
               confirm_window.close_reading_count,
               MOVE_DISTANCE_READINGS_PER_CHECK,
               confirm_window.invalid_reading_count,
               confirm_window.display_distance_mm,
               affected_cell.x,
               affected_cell.y);
        printf("MOVE TARGET CENTER target reset to unknown to avoid immediate retry after ambiguous stop\n");
        setMapCellStatus(affected_cell.x,
                         affected_cell.y,
                         CELL_UNKNOWN);
        reverseToCellCenter(return_cell, return_yaw_deg);
        return;
    }

    sensor_data_t sensor_data = readReliableSensorData();
    field_event_t event = interpretSensorData(sensor_data);

    printf("MOVE TARGET CENTER object safety event=%s affected_cell=(%d,%d)\n",
           eventToString(event.type),
           affected_cell.x,
           affected_cell.y);

    switch (event.type) {
        case FIELD_ROCK_SAMPLE:
            finalizeRockSampleAtCloseRange(&event);
            reportFieldEvent(event);
            markSampleAtCell(affected_cell);
            break;

        case FIELD_HILL:
            if (event.width_cm < HILL_PHYSICAL_SIZE_CM) {
                event.width_cm = HILL_PHYSICAL_SIZE_CM;
            }

            reportFieldEvent(event);
            markHillBlockAroundCell(affected_cell);
            break;

        case FIELD_BLACK_TAPE:
            reportFieldEvent(event);
            markTapeAtCell(affected_cell);
            break;

        case FIELD_SENSOR_FAULT:
            reportFieldEvent(event);
            printf("MOVE TARGET CENTER target reset to unknown after sensor fault during entry\n");
            setMapCellStatus(affected_cell.x,
                             affected_cell.y,
                             CELL_UNKNOWN);
            break;

        case FIELD_CLEAR:
        default:
            printf("Movement object confirmation returned %s; target not explored.\n",
                   eventToString(event.type));
            printf("MOVE TARGET CENTER target reset to unknown after inconsistent entry reading\n");
            setMapCellStatus(affected_cell.x,
                             affected_cell.y,
                             CELL_UNKNOWN);
            break;
    }

    reverseToCellCenter(return_cell, return_yaw_deg);
}

static void moveToTargetCenter(void)
{
    grid_cell_t return_cell = current_cell;
    pose_t start_pose = getPose();
    float target_x_cm = cellCenterXCm(target_cell);
    float target_y_cm = cellCenterYCm(target_cell);

    printf("MOVE TARGET CENTER target=(%d,%d), target_xy=(%.2f,%.2f), from=(%d,%d)\n",
           target_cell.x,
           target_cell.y,
           target_x_cm,
           target_y_cm,
           return_cell.x,
           return_cell.y);

    turnTowardPoint(target_x_cm, target_y_cm, DEFAULT_SPEED);

    while (mission_running) {
        pose_t pose = getPose();
        float delta_x = target_x_cm - pose.x;
        float delta_y = target_y_cm - pose.y;
        float distance_cm = sqrtf(delta_x * delta_x + delta_y * delta_y);

        if (distance_cm <= CELL_CENTER_REACHED_TOLERANCE_CM) {
            distance_window_t final_window =
                readDistanceWindow(MOVE_DISTANCE_READINGS_PER_CHECK);
            bool final_close_object =
                distanceWindowHasCloseObject(final_window,
                                             MOVE_OBJECT_MIN_CLOSE_READINGS);
            bool final_raw_black = false;
            bool final_black_tape = false;

            if (isTCS3200Calibrated()) {
                final_raw_black = tcs3200DetectBlackTape();
            }

            final_black_tape =
                shouldTreatBlackReadingAsTape(final_raw_black, final_close_object);

            if (final_black_tape) {
                field_event_t event;

                event.type = FIELD_BLACK_TAPE;
                event.distance_mm = final_window.display_distance_mm;
                event.width_cm = 0.0f;
                event.color = COLOR_BLACK;
                event.sample_size_cm = 0;
                event.temperature_c = 0.0f;

                printf("MOVE TARGET CENTER final tape safety stop target=(%d,%d), distance=%d, close_reads=%d/%d, invalid_reads=%d\n",
                       target_cell.x,
                       target_cell.y,
                       final_window.display_distance_mm,
                       final_window.close_reading_count,
                       MOVE_DISTANCE_READINGS_PER_CHECK,
                       final_window.invalid_reading_count);
                reportFieldEvent(event);
                markTapeAtCell(target_cell);
                reverseToCellCenter(return_cell, start_pose.yaw);
                current_cell = return_cell;
                transitionTo(NAV_STATE_CHOOSE_TARGET,
                             "returned to already-scanned cell after target tape");
                return;
            }

            current_cell = target_cell;
            setMapCellStatus(target_cell.x, target_cell.y, CELL_EXPLORED);
            sendPoseUpdate();
            transitionTo(NAV_STATE_MARK_CURRENT, "target cell center reached");
            return;
        }

        distance_window_t front_window =
            readDistanceWindow(MOVE_DISTANCE_READINGS_PER_CHECK);
        bool close_object =
            distanceWindowHasCloseObject(front_window,
                                         MOVE_OBJECT_MIN_CLOSE_READINGS);
        bool raw_black = false;
        bool black_tape = false;

        if (isTCS3200Calibrated()) {
            raw_black = tcs3200DetectBlackTape();
        }

        black_tape = shouldTreatBlackReadingAsTape(raw_black, close_object);

        if (black_tape) {
            field_event_t event;

            event.type = FIELD_BLACK_TAPE;
            event.distance_mm = front_window.display_distance_mm;
            event.width_cm = 0.0f;
            event.color = COLOR_BLACK;
            event.sample_size_cm = 0;
            event.temperature_c = 0.0f;

            printf("MOVE TARGET CENTER tape safety stop target=(%d,%d), distance=%d, close_reads=%d/%d, invalid_reads=%d\n",
                   target_cell.x,
                   target_cell.y,
                   front_window.display_distance_mm,
                   front_window.close_reading_count,
                   MOVE_DISTANCE_READINGS_PER_CHECK,
                   front_window.invalid_reading_count);
            reportFieldEvent(event);
            markTapeAtCell(target_cell);
            reverseToCellCenter(return_cell, start_pose.yaw);
            current_cell = return_cell;
            transitionTo(NAV_STATE_CHOOSE_TARGET,
                         "returned to already-scanned cell after movement tape");
            return;
        }

        if (close_object) {
            printf("MOVE TARGET CENTER object safety stop target=(%d,%d), distance=%d, close_reads=%d/%d, invalid_reads=%d, raw_black=%s\n",
                   target_cell.x,
                   target_cell.y,
                   front_window.display_distance_mm,
                   front_window.close_reading_count,
                   MOVE_DISTANCE_READINGS_PER_CHECK,
                   front_window.invalid_reading_count,
                   raw_black ? "yes" : "no");
            handleMovementObject(target_cell, return_cell, start_pose.yaw);
            current_cell = return_cell;
            transitionTo(NAV_STATE_CHOOSE_TARGET,
                         "returned to already-scanned cell after movement object");
            return;
        }

        float step_cm = distance_cm;

        if (step_cm > FORWARD_INCREMENT_CM) {
            step_cm = FORWARD_INCREMENT_CM;
        }

        moveWithRamp(step_cm, DEFAULT_SPEED);
        sendPoseUpdate();
        pollESP32Messages();
    }
}

static void handleBacktrack(void)
{
    while (dfs_stack_top > 0) {
        grid_cell_t candidate;

        dfs_stack_top--;
        candidate = dfs_stack[dfs_stack_top];

#if NAV_DEBUG_DFS
        printf("DFS BACKTRACK pop cell=(%d,%d), stack_size=%d\n",
               candidate.x,
               candidate.y,
               dfs_stack_top);
#endif

        if (!isValidBacktrackCell(candidate)) {
            printf("DFS BACKTRACK skipped invalid cell=(%d,%d)\n",
                   candidate.x,
                   candidate.y);
            continue;
        }

        target_cell = candidate;
        printf("DFS BACKTRACK target=(%d,%d)\n",
               target_cell.x,
               target_cell.y);
        transitionTo(NAV_STATE_MOVE_TO_TARGET_CENTER,
                     "valid adjacent explored backtrack cell");
        return;
    }

    transitionTo(NAV_STATE_SIDE_COMPLETE, "DFS stack empty");
}

static void handleSideComplete(void)
{
    int unknown_count = 0;
    int scanned_clear_count = 0;
    int explored_count = 0;
    int blocked_count = 0;

    for (int x = 0; x < MAP_SIZE; x++) {
        for (int y = 0; y < MAP_SIZE; y++) {
            if (!missionFrameLocalGridIndexIsInAssignedHalf(x, y)) {
                continue;
            }

            if (map_grid[x][y] == CELL_UNKNOWN) {
                unknown_count++;
            } else if (map_grid[x][y] == CELL_SCANNED_CLEAR) {
                scanned_clear_count++;
            } else if (map_grid[x][y] == CELL_EXPLORED) {
                explored_count++;
            } else if (isBlockedCellStatus(map_grid[x][y])) {
                blocked_count++;
            }
        }
    }

    printf("SIDE COMPLETE SUMMARY unknown=%d, scanned_clear=%d, explored=%d, blocked=%d, dfs_stack_size=%d\n",
           unknown_count,
           scanned_clear_count,
           explored_count,
           blocked_count,
           dfs_stack_top);

    sendStatusUpdate("side_complete", "none");
    mission_running = false;
}

static void handleNavigationError(void)
{
    printf("Navigation error: stopping mission\n");
    sendErrorMessage("navigation_error");
    sendStatusUpdate("fault", "navigation_error");
    mission_running = false;
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

    if (!data.valid) {
        event.type = FIELD_SENSOR_FAULT;
        return event;
    }

    bool close_front_object =
        data.front_object_detected &&
        isCloseFrontObjectDistance(data.front_distance_mm);

    bool confirmed_black_tape =
        shouldTreatBlackReadingAsTape(data.black_tape_detected, close_front_object);

    if (data.black_tape_detected && close_front_object) {
        printf("Black reading ignored as tape because a close front object is present: distance=%d mm, width=%.2f cm\n",
               data.front_distance_mm,
               data.estimated_width_cm);
    }

    /*
     * A black object in front of the robot must not be classified as tape.
     * Therefore, object handling is allowed before black-tape handling when
     * VL53L0X confirms a close front object.
     */
    if (close_front_object) {
        int sample_size = 0;

        if (data.estimated_width_cm <= 0.0f) {
            printf("Close object rejected: invalid width %.2f cm at distance=%d mm\n",
                   data.estimated_width_cm,
                   data.front_distance_mm);
            event.type = FIELD_SENSOR_FAULT;
            return event;
        }

        if (estimateSampleSize(data.estimated_width_cm, &sample_size)) {
            event.type = FIELD_ROCK_SAMPLE;
            event.sample_size_cm = sample_size;
            event.color = COLOR_UNKNOWN;
            return event;
        }

        event.type = FIELD_HILL;
        return event;
    }

    if (confirmed_black_tape) {
        event.type = FIELD_BLACK_TAPE;
        return event;
    }

    return event;
}

void startMission(void)
{
    mission_running = true;
    current_state = NAV_STATE_INIT;
    sendStatusUpdate("navigating", "none");
}

void stopMission(void)
{
    mission_running = false;
    sendStatusUpdate("stopped", "none");
}

bool isMissionRunning(void)
{
    return mission_running;
}

void runNavigation(void)
{
    int fsm_cycles = 0;
    int max_fsm_cycles = MAP_SIZE * MAP_SIZE * 20;

    if (!mission_running) {
        startMission();
    }

    nav_log_timestamps_enabled = false;
    printf("Starting V1 scan-based cell exploration...\n");
    nav_log_start_msec = navNowMsec();
    nav_log_timestamps_enabled = true;

    while (mission_running) {
        pollESP32Messages();

        if (!mission_running) {
            break;
        }

        if (fsm_cycles > max_fsm_cycles) {
            transitionTo(NAV_STATE_ERROR, "FSM cycle limit exceeded");
        }

        switch (current_state) {
            case NAV_STATE_INIT:
                if (!map_initialized) {
                    initMap();
                }

                resetNavigationRuntime();
                mission_running = true;
                transitionTo(NAV_STATE_MARK_CURRENT, "navigation initialized");
                break;

            case NAV_STATE_MARK_CURRENT:
                markCurrentCellState();
                break;

            case NAV_STATE_SCAN_360:
                runScan360();
                break;

            case NAV_STATE_UPDATE_MAP:
                updateMapFromScan();
                break;

            case NAV_STATE_CHOOSE_TARGET:
                chooseNextTarget();
                break;

            case NAV_STATE_MOVE_TO_TARGET_CENTER:
                moveToTargetCenter();
                break;

            case NAV_STATE_INVESTIGATE_OBJECT:
                investigateObjectCandidate();
                break;

            case NAV_STATE_BACKTRACK:
                handleBacktrack();
                break;

            case NAV_STATE_SIDE_COMPLETE:
                handleSideComplete();
                break;

            case NAV_STATE_ERROR:
            default:
                handleNavigationError();
                break;
        }

        fsm_cycles++;
    }

    printf("Navigation finished.\n");
}
