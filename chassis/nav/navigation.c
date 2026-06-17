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
    float min_distance_yaw;
} scan_evidence_t;

typedef struct {
    int display_distance_mm;
    int best_close_distance_mm;
    int close_reading_count;
    int invalid_reading_count;
} distance_window_t;

typedef enum {
    APPROACH_OK,            /* verified at target distance within tolerance */
    APPROACH_NOT_CONVERGED, /* ran out of attempts, no push detected */
    APPROACH_PUSHING        /* forward motion stopped shrinking the distance:
                               the object is sliding in front of the robot */
} approach_result_t;

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
static grid_cell_t projectMeasuredObjectCell(int distance_mm,
                                             grid_cell_t fallback_cell);
static bool isKnownObjectCell(grid_cell_t cell);
static bool shouldTreatBlackReadingAsTape(bool black_detected,
                                          bool close_front_object_detected);
static bool estimateSampleSize(float width_cm, int *sample_size_cm);
static bool pushDfsCell(grid_cell_t cell);
static bool isValidTargetCell(grid_cell_t cell,
                              nav_direction_t travel_direction);
static bool isValidBacktrackCell(grid_cell_t cell);

static float cellCenterXCm(grid_cell_t cell);
static float cellCenterYCm(grid_cell_t cell);
static float shortestAngleDelta(float target_yaw_deg, float current_yaw_deg);

static distance_window_t readDistanceWindow(int reading_count);
static int readMedianUsableDistanceMm(int reading_count);

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
static approach_result_t moveToDistanceFromObject(int target_distance_mm);
static void finalizeRockSampleAtCloseRange(field_event_t *event);
static void reportFieldEvent(field_event_t event);
static void markHillBlockAroundCell(grid_cell_t center_cell);
static void markSampleAtCell(grid_cell_t cell);
static void markTapeAtCell(grid_cell_t cell);
static void markCurrentCellState(void);
static void runScan360(void);
static void updateMapFromScan(void);
static void chooseNextTarget(void);
static field_event_type_t investigateObjectAt(grid_cell_t candidate_cell,
                                              nav_direction_t direction,
                                              float candidate_yaw);
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
static double state_entry_msec = 0.0;
static bool map_initialized = false;
static cell_status_t map_grid[MAP_SIZE][MAP_SIZE];

static nav_state_t current_state = NAV_STATE_INIT;
static grid_cell_t current_cell = {MAP_CENTER, MAP_CENTER};
static grid_cell_t target_cell = {MAP_CENTER, MAP_CENTER};
static grid_cell_t object_candidate_cells[NAV_DIR_COUNT];
static int object_candidate_dirs[NAV_DIR_COUNT];
static float object_candidate_yaws[NAV_DIR_COUNT];
static int object_candidate_count = 0;
static int object_candidate_index = 0;

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
        case CELL_TAPE: return "tape";
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
           status == CELL_TAPE ||
           status == CELL_HILL ||
           status == CELL_SAMPLE;
}

static bool isBlockedCellStatus(cell_status_t status)
{
    return status == CELL_UNSAFE ||
           status == CELL_TAPE ||
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

/*
 * Cell containing the object surface the distance sensor is looking at: the
 * current pose projected forward by the measured distance.  The scan
 * candidate cell is only "the next cell in that direction" and falls short
 * when the object sits further away, which would anchor the hill footprint
 * one or more cells toward the robot, away from where the EVENT icon lands
 * (sendFieldEventUpdate uses this same projection).
 */
static grid_cell_t projectMeasuredObjectCell(int distance_mm,
                                             grid_cell_t fallback_cell)
{
    pose_t pose = getPose();
    float distance_cm =
        distance_mm > 0 ? (float)distance_mm / 10.0f : CELL_SIZE_CM;
    float yaw_rad = pose.yaw * PI / 180.0f;
    float object_x_cm = pose.x + distance_cm * cosf(yaw_rad);
    float object_y_cm = pose.y + distance_cm * sinf(yaw_rad);
    grid_cell_t cell = fallback_cell;
    int grid_x = 0;
    int grid_y = 0;

    if (poseToGridCell(object_x_cm, object_y_cm, &grid_x, &grid_y)) {
        cell.x = grid_x;
        cell.y = grid_y;
    } else {
        printf("PROJECT OBJECT CELL outside map: x=%.2f, y=%.2f, fallback=(%d,%d)\n",
               object_x_cm,
               object_y_cm,
               fallback_cell.x,
               fallback_cell.y);
    }

    return cell;
}

/*
 * True when the cell the distance beam lands on already belongs to a
 * classified object.  Hills get one cell of slack in every direction: the
 * 2x2 hill footprint is anchored from a noisy distance projection, so a
 * repeat sighting of the same hill routinely lands one cell off the marked
 * block — which is exactly how a clipped hill edge gets re-measured with a
 * tiny width and re-reported as a brand-new rock sample.
 */
static bool isKnownObjectCell(grid_cell_t cell)
{
    if (!isInsideMap(cell.x, cell.y)) {
        return false;
    }

    if (isTerminalCellStatus(map_grid[cell.x][cell.y])) {
        return true;
    }

    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            int x = cell.x + dx;
            int y = cell.y + dy;

            if (isInsideMap(x, y) && map_grid[x][y] == CELL_HILL) {
                return true;
            }
        }
    }

    return false;
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

/*
 * Confirm a raw black floor reading before treating it as boundary tape.
 * Re-samples the floor at TAPE_CONFIRM_NUDGE_CM forward offsets: boundary
 * tape is wide enough to stay black across neighbouring positions, while an
 * A4 sheet seam or a shadow edge reads black at most once.  May leave the
 * robot up to (TAPE_CONFIRM_SAMPLE_COUNT - 1) nudges further forward; the
 * caller's pose-based loop replans from the updated odometry.
 */
static bool confirmBlackTapeReading(void)
{
    int black_count = 1;  /* the caller's triggering reading */

    for (int sample = 1; sample < TAPE_CONFIRM_SAMPLE_COUNT; sample++) {
        if (black_count >= TAPE_CONFIRM_MIN_BLACK) {
            break;  /* confirmed — stop creeping toward the tape */
        }

        moveWithRamp(TAPE_CONFIRM_NUDGE_CM, TAPE_CONFIRM_NUDGE_SPEED_CM_S);
        sendPoseUpdate();

        if (tcs3200DetectBlackTape()) {
            black_count++;
        }
    }

    printf("TAPE CONFIRM black=%d/%d (min %d)\n",
           black_count,
           TAPE_CONFIRM_SAMPLE_COUNT,
           TAPE_CONFIRM_MIN_BLACK);

    return black_count >= TAPE_CONFIRM_MIN_BLACK;
}

static bool estimateSampleSize(float width_cm, int *sample_size_cm)
{
    /*
     * These thresholds are empirical for this robot.
     * The VL53L0X width scan underestimates real cube width because the robot
     * rotates around its wheel axis, not around the distance sensor itself.
     */

    if (width_cm >= 0.5f && width_cm < 3.5f) {
        *sample_size_cm = 3;
        return true;
    }

    if (width_cm >= 3.5f && width_cm <= 7.5f) {
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

static int readMedianUsableDistanceMm(int reading_count)
{
    int sorted_readings[SAMPLE_APPROACH_READINGS];
    int valid_count = 0;

    if (reading_count > SAMPLE_APPROACH_READINGS) {
        reading_count = SAMPLE_APPROACH_READINGS;
    }

    for (int i = 0; i < reading_count; i++) {
        int distance_mm = readVL53L0XDistance();

        if (!isUsableDistance(distance_mm)) {
            continue;
        }

        int j = valid_count;

        while (j > 0 && sorted_readings[j - 1] > distance_mm) {
            sorted_readings[j] = sorted_readings[j - 1];
            j--;
        }

        sorted_readings[j] = distance_mm;
        valid_count++;
    }

    if (valid_count == 0) {
        return VL53L0X_INVALID_DISTANCE_MM;
    }

    /* Lower median: with an even count, prefer the closer reading so the
     * approach errs toward stopping short rather than ramming the object. */
    return sorted_readings[(valid_count - 1) / 2];
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

static double stateTimeoutSeconds(nav_state_t state)
{
    switch (state) {
        case NAV_STATE_SCAN_360:              return 90.0;
        case NAV_STATE_MOVE_TO_TARGET_CENTER: return 30.0;
        case NAV_STATE_BACKTRACK:             return 30.0;
        case NAV_STATE_INVESTIGATE_OBJECT:    return 120.0;
        case NAV_STATE_UPDATE_MAP:            return 15.0;
        case NAV_STATE_CHOOSE_TARGET:         return 15.0;
        case NAV_STATE_MARK_CURRENT:          return 15.0;
        case NAV_STATE_INIT:                  return 15.0;
        case NAV_STATE_SIDE_COMPLETE:         return 15.0;
        case NAV_STATE_ERROR:                 return 0.0;
        default:                              return 30.0;
    }
}

static nav_state_t watchdogRecoveryState(nav_state_t timed_out_state)
{
    if (timed_out_state == NAV_STATE_MOVE_TO_TARGET_CENTER ||
        timed_out_state == NAV_STATE_BACKTRACK) {
        return NAV_STATE_BACKTRACK;
    }

    return NAV_STATE_CHOOSE_TARGET;
}

static void transitionTo(nav_state_t next_state, const char *reason)
{
    sendNavLog("INFO", "%s -> %s: %s",
               navStateToString(current_state),
               navStateToString(next_state),
               reason == NULL ? "" : reason);

#if NAV_DEBUG_STATE
    printf("NAV STATE: %s -> %s, reason=%s\n",
           navStateToString(current_state),
           navStateToString(next_state),
           reason == NULL ? "none" : reason);
#endif

    current_state = next_state;
    state_entry_msec = navNowMsec();
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
    object_candidate_count = 0;
    object_candidate_index = 0;
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

    /* EXPLORED means the robot physically stood on the cell — strictly more
     * information than a scan result.  Downgrading it (e.g. an investigation
     * resetting a backtrack target to unknown) would also break backtracking,
     * which only travels through explored cells. */
    if (previous == CELL_EXPLORED &&
        (status == CELL_SCANNED_CLEAR || status == CELL_UNKNOWN)) {
        printf("MAP CELL weak %s ignored: (%d,%d) stays %s\n",
               cellStatusToString(status),
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
        scan_evidence[dir].min_distance_yaw = 0.0f;
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
        float target_yaw = atan2f(delta_y, delta_x) * 180.0f / PI;
        float reverse_yaw = normalizeAngle(target_yaw + 180.0f);
        float reverse_alignment_error =
            fabsf(shortestAngleDelta(reverse_yaw, pose.yaw));

        if (reverse_alignment_error <= REVERSE_TO_CELL_ALIGNMENT_TOLERANCE_DEG) {
            /* The saved point is directly behind the robot — the usual case
             * after approaching an object head-on.  Back straight out instead
             * of spinning 180 degrees, driving forward, and spinning back. */
            printf("RETURN CELL CENTER reverse: distance=%.2f cm, alignment_error=%.2f deg\n",
                   distance_cm,
                   reverse_alignment_error);
            moveWithRamp(-distance_cm, DEFAULT_SPEED);
            sendPoseUpdate();
        } else {
            printf("RETURN CELL CENTER forward: distance=%.2f cm, alignment_error=%.2f deg\n",
                   distance_cm,
                   reverse_alignment_error);
            turnTowardPoint(saved_pose.x, saved_pose.y, DEFAULT_SPEED);
            moveWithRamp(distance_cm, DEFAULT_SPEED);
            sendPoseUpdate();
        }
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
    odometryInit(target_pose.x, target_pose.y, final_yaw_deg);
    sendPoseUpdate();
}

/*
 * Closed-loop positioning at a sensing distance from the object in front.
 * Width classification is only valid at INVESTIGATE_APPROACH_DISTANCE_MM and
 * the color read at SAMPLE_COLOR_DISTANCE_MM, so the robot must actually
 * reach the requested distance: read, correct, re-read until within
 * tolerance instead of trusting one reading and one open-loop move.
 * Returns APPROACH_OK when the final verified distance is within tolerance,
 * APPROACH_PUSHING when forward motion stops shrinking the measured distance
 * (the object is sliding along in front of the robot — stop before shoving
 * it further), and APPROACH_NOT_CONVERGED otherwise.
 */
static approach_result_t moveToDistanceFromObject(int target_distance_mm)
{
    int last_distance_mm = VL53L0X_INVALID_DISTANCE_MM;
    int previous_reading_mm = VL53L0X_INVALID_DISTANCE_MM;
    float net_forward_since_reading_cm = 0.0f;
    int push_strike_count = 0;

    for (int attempt = 1; attempt <= SAMPLE_APPROACH_MAX_ATTEMPTS; attempt++) {
        int distance_mm = readMedianUsableDistanceMm(SAMPLE_APPROACH_READINGS);

        if (!isUsableDistance(distance_mm)) {
            printf("Sample approach %d/%d: invalid distance reading\n",
                   attempt, SAMPLE_APPROACH_MAX_ATTEMPTS);
            continue;
        }

        /* Approaches only start with the object inside the front-object
         * window. A reading far beyond it means the beam slipped off the
         * object onto the background; moving toward it would ram the object. */
        if (distance_mm > FRONT_OBJECT_MAX_DISTANCE_MM +
                              SAMPLE_APPROACH_BACKGROUND_MARGIN_MM) {
            printf("Sample approach %d/%d: background reading %d mm ignored\n",
                   attempt, SAMPLE_APPROACH_MAX_ATTEMPTS, distance_mm);
            continue;
        }

        /* Push detection: driving forward must shrink the reading by roughly
         * the distance driven.  An object sliding in front of the robot keeps
         * the reading nearly constant while the robot advances. */
        if (previous_reading_mm != VL53L0X_INVALID_DISTANCE_MM &&
            net_forward_since_reading_cm >= APPROACH_PUSH_MIN_FORWARD_CM) {
            int expected_drop_mm = (int)(net_forward_since_reading_cm * 10.0f);
            int actual_drop_mm = previous_reading_mm - distance_mm;

            if (actual_drop_mm * 2 < expected_drop_mm) {
                push_strike_count++;
                printf("Sample approach push strike %d/%d: moved %.1f cm, distance %d -> %d mm\n",
                       push_strike_count,
                       APPROACH_PUSH_MAX_STRIKES,
                       net_forward_since_reading_cm,
                       previous_reading_mm,
                       distance_mm);

                if (push_strike_count >= APPROACH_PUSH_MAX_STRIKES) {
                    sendNavLog("WARN", "Approach aborted at %dmm — object sliding (pushed)",
                               distance_mm);
                    return APPROACH_PUSHING;
                }
            } else {
                push_strike_count = 0;
            }
        }

        previous_reading_mm = distance_mm;
        net_forward_since_reading_cm = 0.0f;
        last_distance_mm = distance_mm;

        int error_mm = distance_mm - target_distance_mm;
        int abs_error_mm = error_mm < 0 ? -error_mm : error_mm;

        if (abs_error_mm <= SAMPLE_APPROACH_TOLERANCE_MM) {
            printf("Sample approach done: %d mm (target %d mm)\n",
                   distance_mm, target_distance_mm);
            return APPROACH_OK;
        }

        if (attempt == SAMPLE_APPROACH_MAX_ATTEMPTS) {
            break;
        }

        float move_cm = (float)error_mm / 10.0f;

        if (move_cm > SAMPLE_APPROACH_MAX_CM) {
            move_cm = SAMPLE_APPROACH_MAX_CM;
        }

        if (move_cm < -SAMPLE_APPROACH_MAX_CM) {
            move_cm = -SAMPLE_APPROACH_MAX_CM;
        }

        printf("Sample approach %d/%d: moving %.2f cm to reach %d mm, current %d mm\n",
               attempt,
               SAMPLE_APPROACH_MAX_ATTEMPTS,
               move_cm,
               target_distance_mm,
               distance_mm);

        moveWithRamp(move_cm, SAMPLE_APPROACH_SPEED_CM_S);
        net_forward_since_reading_cm += move_cm;
        sendPoseUpdate();
    }

    sendNavLog("WARN", "Approach to %dmm did not converge (last reading %dmm)",
               target_distance_mm, last_distance_mm);
    return APPROACH_NOT_CONVERGED;
}

static void finalizeRockSampleAtCloseRange(field_event_t *event)
{
    bool at_color_distance =
        moveToDistanceFromObject(SAMPLE_COLOR_DISTANCE_MM) == APPROACH_OK;

    event->distance_mm = readVL53L0XDistance();

    if (!at_color_distance) {
        /* Color read from the wrong distance returns the floor/background
         * color, which is worse than reporting no color at all. */
        printf("Color reading skipped: not at %d mm color distance\n",
               SAMPLE_COLOR_DISTANCE_MM);
        event->color = COLOR_UNKNOWN;
    } else if (isTCS3200Calibrated()) {
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
    /*
     * A ~30 cm hill spans roughly a 2x2 block of 15 cm cells.  A 2x2 has no
     * center cell, so anchor it on the detected cell and extend it away from
     * the robot (the hill lies beyond the detection point) plus one cell to
     * one lateral side.
     *
     * Forward must be a single cardinal axis, taken from the robot's yaw
     * (the robot faces the hill when this runs).  Deriving it from the
     * robot->hill cell difference breaks on diagonals: both components step
     * at once and the four cells land in a diamond with a hole in the middle
     * instead of a solid 2x2 square.
     */
    static const float cardinal_yaw_deg[NAV_DIR_COUNT] =
        {0.0f, 90.0f, 180.0f, 270.0f};

    pose_t pose = getPose();
    nav_direction_t facing = directionFromYaw(pose.yaw);
    float off_axis_deg = shortestAngleDelta(pose.yaw, cardinal_yaw_deg[facing]);
    int fwd_x = 0;
    int fwd_y = 0;

    switch (facing) {
        case NAV_DIR_POS_X: fwd_x = 1;  break;
        case NAV_DIR_POS_Y: fwd_y = 1;  break;
        case NAV_DIR_NEG_X: fwd_x = -1; break;
        case NAV_DIR_NEG_Y: fwd_y = -1; break;
        default:            fwd_x = 1;  break;
    }

    /* Lateral cell on the side of the cardinal axis the beam leans toward:
     * positive off-axis means the hill extends counterclockwise of forward
     * (forward rotated +90 degrees), negative means clockwise. */
    int perp_x = off_axis_deg >= 0.0f ? -fwd_y : fwd_y;
    int perp_y = off_axis_deg >= 0.0f ? fwd_x : -fwd_x;

    printf("MARK HILL 2x2 center=(%d,%d), yaw=%.2f, fwd=(%d,%d), perp=(%d,%d)\n",
           center_cell.x, center_cell.y, pose.yaw, fwd_x, fwd_y, perp_x, perp_y);

    for (int f = 0; f <= 1; f++) {
        for (int p = 0; p <= 1; p++) {
            int x = center_cell.x + f * fwd_x + p * perp_x;
            int y = center_cell.y + f * fwd_y + p * perp_y;

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
    printf("MARK TAPE cell=(%d,%d)\n", cell.x, cell.y);
    setMapCellStatus(cell.x, cell.y, CELL_TAPE);
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
    /* Arc end boundaries (absolute yaw) for each cardinal direction */
    static const float arc_end_abs[NAV_DIR_COUNT] = {45.0f, 135.0f, 225.0f, 315.0f};

    static int scan_count = 0;
    scan_count++;

    pose_t start_pose = getPose();
    float original_yaw = start_pose.yaw;
    float total_scan_deg = 360.0f;
    float scanned_deg = 0.0f;
    float sweep_start_yaw = original_yaw;
    bool tape_scan_enabled = isTCS3200Calibrated();

    /* Last hill classified during this sweep: later close-object rays near
     * that heading must read clearly closer to count as a new object. */
    bool hill_sighted = false;
    float hill_sighted_yaw = 0.0f;
    int hill_sighted_distance_mm = 0;

    /* Determine which direction arcs actually need scanning.
     * EXPLORED and terminal cells are definitively known — skip their arcs.
     * SCANNED_CLEAR is kept active because edge objects near its boundary
     * may be visible from the current position. */
    bool dir_active[NAV_DIR_COUNT];
    int dirs_needed = 0;

    for (int d = 0; d < NAV_DIR_COUNT; d++) {
        grid_cell_t adj = adjacentCell(current_cell, (nav_direction_t)d);
        dir_active[d] = false;

        if (!isInsideMap(adj.x, adj.y) ||
            !missionFrameLocalGridIndexIsInAssignedHalf(adj.x, adj.y)) {
            continue;
        }

        cell_status_t st = map_grid[adj.x][adj.y];

        if (st != CELL_EXPLORED && !isTerminalCellStatus(st)) {
            dir_active[d] = true;
            dirs_needed++;
        }
    }

    resetScanEvidence();
    recalibrateGyroIfReady();

    printf("SCAN START cell=(%d,%d) yaw=%.2f scan=%d dirs=%d/%d tape=%s\n",
           current_cell.x, current_cell.y, original_yaw,
           scan_count, dirs_needed, NAV_DIR_COUNT,
           tape_scan_enabled ? "yes" : "no");

    while (scanned_deg <= total_scan_deg + 0.01f) {
        pose_t pose = getPose();
        bool is_heading_check_ray = scanned_deg >= total_scan_deg - 0.01f;
        nav_direction_t direction = directionFromYaw(pose.yaw);

        /* If this direction's arc is not needed, physically skip ahead to the
         * next arc boundary rather than sweeping through empty space. */
        if (!is_heading_check_ray && !dir_active[direction]) {
            float abs_yaw = normalizeAngle(sweep_start_yaw + scanned_deg);
            float skip_deg = arc_end_abs[direction] - abs_yaw;

            if (skip_deg < 0.0f) {
                skip_deg += 360.0f;
            }

            if (skip_deg >= SCAN_360_STEP_DEG) {
                printf("SCAN SKIP dir=%s (%.1f deg)\n",
                       directionName(direction), skip_deg);

                float new_scanned = scanned_deg + skip_deg;

                if (new_scanned >= total_scan_deg - 0.01f) {
                    scanned_deg = total_scan_deg;
                    turnToYaw(original_yaw, DEFAULT_SPEED);
                } else {
                    turn(skip_deg, DEFAULT_SPEED);
                    scanned_deg = new_scanned;
                }
                continue;
            }
            /* Skip amount smaller than one step — fall through without
             * accumulating evidence, then take the normal step below. */
        }

        /* Take sensor readings */
        distance_window_t window =
            readDistanceWindow(SCAN_DISTANCE_READINGS_PER_ANGLE);
        bool close_object =
            distanceWindowHasCloseObject(window, SCAN_ANGLE_OBJECT_MIN_CLOSE_READINGS);
        bool raw_black = false;
        bool black_tape = false;
        bool clear = false;

        if (tape_scan_enabled) {
            raw_black = tcs3200DetectBlackTape();
        }

        black_tape = shouldTreatBlackReadingAsTape(raw_black, close_object);
        clear = !close_object && !black_tape;

        /* Active directions: classify objects on the spot, otherwise accumulate
         * tape/clear evidence for the post-scan map update. */
        if (!is_heading_check_ray && dir_active[direction]) {
            sendScanRayUpdate(pose.yaw, window.display_distance_mm);

            if (close_object) {
                grid_cell_t obj_cell = adjacentCell(current_cell, direction);
                grid_cell_t hit_cell =
                    projectMeasuredObjectCell(window.best_close_distance_mm,
                                              obj_cell);
                /* A hill classified earlier in this sweep is re-seen at a
                 * similar distance for many steps; a genuinely new object in
                 * front of it must read clearly closer than the hill did. */
                bool same_hill_by_distance =
                    hill_sighted &&
                    fabsf(shortestAngleDelta(pose.yaw, hill_sighted_yaw)) <=
                        HILL_RESIGHT_YAW_WINDOW_DEG &&
                    window.best_close_distance_mm >
                        hill_sighted_distance_mm - HILL_RESIGHT_MIN_DISTANCE_DROP_MM;

                if (isKnownObjectCell(hit_cell)) {
                    /* The beam landed on an already-classified object — a
                     * repeat sighting, not a new candidate.  Keep the arc
                     * active and take the normal step so a different object
                     * later in this arc (e.g. a sample one step past a hill)
                     * is still found. */
                    printf("SCAN360 known object re-sighted dir=%s hit=(%d,%d) yaw=%.2f — sweep continues\n",
                           directionName(direction), hit_cell.x, hit_cell.y, pose.yaw);
                    sendNavLog("INFO", "Known object re-sighted at yaw %.0f — skipped",
                               pose.yaw);
                } else if (same_hill_by_distance) {
                    printf("SCAN360 hill re-sighted by distance dir=%s yaw=%.2f dist=%d (hill %d mm @ yaw %.2f) — sweep continues\n",
                           directionName(direction), pose.yaw,
                           window.best_close_distance_mm,
                           hill_sighted_distance_mm, hill_sighted_yaw);
                    sendNavLog("INFO", "Hill re-sighted at yaw %.0f (%dmm vs %dmm) — skipped",
                               pose.yaw, window.best_close_distance_mm,
                               hill_sighted_distance_mm);
                } else {
                    if (isInsideMap(obj_cell.x, obj_cell.y) &&
                        missionFrameLocalGridIndexIsInAssignedHalf(obj_cell.x, obj_cell.y) &&
                        map_grid[obj_cell.x][obj_cell.y] != CELL_EXPLORED &&
                        !isTerminalCellStatus(map_grid[obj_cell.x][obj_cell.y])) {
                        printf("SCAN360 object seen dir=%s cell=(%d,%d) yaw=%.2f — investigating now\n",
                               directionName(direction), obj_cell.x, obj_cell.y, pose.yaw);

                        if (investigateObjectAt(obj_cell, direction, pose.yaw) ==
                                FIELD_HILL) {
                            hill_sighted = true;
                            hill_sighted_yaw = pose.yaw;
                            hill_sighted_distance_mm =
                                window.best_close_distance_mm;
                        }
                    }

                    /* This direction is classified — skip the rest of its arc and
                     * resume the sweep from where we left off. */
                    dir_active[direction] = false;
                    continue;
                }
            } else if (black_tape) {
                scan_evidence[direction].tape_count++;
            } else if (clear) {
                scan_evidence[direction].clear_count++;
            }
        }

#if NAV_DEBUG_SCAN360
        printf("SCAN360 RAY yaw=%.2f dir=%s active=%s dist=%d close=%d/%d invalid=%d black=%s tape=%s object=%s clear=%s counted=%s\n",
               pose.yaw,
               directionName(direction),
               dir_active[direction] ? "yes" : "no",
               window.display_distance_mm,
               window.close_reading_count,
               SCAN_DISTANCE_READINGS_PER_ANGLE,
               window.invalid_reading_count,
               raw_black    ? "yes" : "no",
               black_tape   ? "yes" : "no",
               close_object ? "yes" : "no",
               clear        ? "yes" : "no",
               is_heading_check_ray ? "no" : "yes");
#endif

        if (is_heading_check_ray) {
            break;
        }

        float remaining_deg = total_scan_deg - scanned_deg;
        float step_deg = SCAN_360_STEP_DEG;

        if (step_deg > remaining_deg) {
            step_deg = remaining_deg;
        }

        scanned_deg += step_deg;

        if (scanned_deg >= total_scan_deg - 0.01f) {
            scanned_deg = total_scan_deg;
            turnToYaw(original_yaw, DEFAULT_SPEED);
        } else {
            turn(step_deg, DEFAULT_SPEED);
        }
    }

    pose_t completed_pose = getPose();
    printf("SCAN COMPLETE cell=(%d,%d) original_yaw=%.2f final_yaw=%.2f\n",
           current_cell.x, current_cell.y, original_yaw, completed_pose.yaw);

    /* The robot scans in place, so it is physically at the current cell center.
     * A mid-scan object investigation drives forward and returns only
     * approximately, leaving odometry x/y biased toward the investigated cell.
     * Re-anchor to the known cell center so the next turnTowardPoint() does not
     * aim from a drifted position and send the robot off in the wrong (often
     * backward) direction. */
    odometryInit(cellCenterXCm(current_cell),
                 cellCenterYCm(current_cell),
                 original_yaw);
    sendPoseUpdate();

#if NAV_DEBUG_SCAN360
    for (int dir = 0; dir < NAV_DIR_COUNT; dir++) {
        scan_evidence_t e = scan_evidence[dir];
        printf("SCAN EVIDENCE dir=%s clear=%d object=%d tape=%d min_distance=%d\n",
               directionName((nav_direction_t)dir),
               e.clear_count, e.object_count, e.tape_count, e.min_distance_mm);
    }
#endif

    transitionTo(NAV_STATE_UPDATE_MAP, "360 scan complete");
}

static void updateMapFromScan(void)
{
    object_candidate_count = 0;
    object_candidate_index = 0;

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
                object_candidate_count < NAV_DIR_COUNT) {
                object_candidate_cells[object_candidate_count] = cell;
                object_candidate_dirs[object_candidate_count] = dir;
                object_candidate_yaws[object_candidate_count] = evidence.min_distance_yaw;
                object_candidate_count++;
            }
        } else if (evidence.clear_count >= SCAN_CLEAR_MIN_READINGS) {
            printf("MARK SCANNED_CLEAR cell=(%d,%d), dir=%s\n",
                   cell.x,
                   cell.y,
                   directionName(direction));
            setMapCellStatus(cell.x, cell.y, CELL_SCANNED_CLEAR);
        }
    }

    sendNavLog("INFO", "Scan at (%d,%d) complete: %d candidate(s) found",
               current_cell.x, current_cell.y, object_candidate_count);

    if (object_candidate_count > 0) {
        printf("SCAN360 found %d object candidate(s) to investigate\n",
               object_candidate_count);
        transitionTo(NAV_STATE_INVESTIGATE_OBJECT,
                     "object candidates found");
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

/*
 * Investigate a single object: turn to the heading where it was seen, confirm
 * it is really there, approach to classification distance, classify it, mark
 * the cell, and return to the pose we started from.  Called inline during the
 * 360 scan (the moment an object is seen) and from the post-scan candidate
 * queue.  Always leaves the robot back at its starting pose.  Returns the
 * event type acted on (FIELD_CLEAR when nothing was confirmed, FIELD_HILL
 * also when a duplicate hill sighting was suppressed) so the scan sweep can
 * track what it just saw.
 */
static field_event_type_t investigateObjectAt(grid_cell_t candidate_cell,
                                              nav_direction_t direction,
                                              float candidate_yaw)
{
    pose_t saved_pose = getPose();

    /* Aim at the exact heading where the object was seen, not the cardinal —
     * a sample offset within the cell appears a few degrees off-axis. */
    turnToYaw(candidate_yaw, DEFAULT_SPEED);

    distance_window_t confirm_window =
        readDistanceWindow(INVESTIGATE_DISTANCE_READINGS_PER_CONFIRMATION);

    if (!distanceWindowHasCloseObject(confirm_window,
                                      INVESTIGATE_OBJECT_MIN_CLOSE_READINGS)) {
        printf("Object confirmation failed: close_reads=%d/%d, invalid_reads=%d, distance=%d\n",
               confirm_window.close_reading_count,
               INVESTIGATE_DISTANCE_READINGS_PER_CONFIRMATION,
               confirm_window.invalid_reading_count,
               confirm_window.display_distance_mm);
        sendNavLog("WARN", "Object at %s rejected: only %d/%d close readings (dist=%dmm)",
                   directionName(direction),
                   confirm_window.close_reading_count,
                   INVESTIGATE_DISTANCE_READINGS_PER_CONFIRMATION,
                   confirm_window.display_distance_mm);
        setMapCellStatus(candidate_cell.x, candidate_cell.y, CELL_SCANNED_CLEAR);
        returnToPoseApprox(saved_pose);
        return FIELD_CLEAR;
    }

    if (moveToDistanceFromObject(INVESTIGATE_APPROACH_DISTANCE_MM) ==
            APPROACH_PUSHING) {
        /* The object slid along in front of the robot during the approach.
         * A width measured while pressed against it is garbage — a shoved
         * hill face reads a few cm wide and turns into a phantom rock
         * sample — so do not classify.  UNKNOWN keeps the cell off-limits
         * as a drive target but rescannable from another vantage. */
        sendNavLog("WARN", "Object at %s pushed during approach — not classified",
                   directionName(direction));
        setMapCellStatus(candidate_cell.x, candidate_cell.y, CELL_UNKNOWN);
        returnToPoseApprox(saved_pose);
        return FIELD_SENSOR_FAULT;
    }

    sensor_data_t sensor_data = readReliableSensorData();
    field_event_t event = interpretSensorData(sensor_data);

    if (event.type == FIELD_SENSOR_FAULT) {
        printf("Sensor fault on first attempt, retrying investigation\n");
        sendNavLog("WARN", "Sensor fault at %s — retrying", directionName(direction));
        sensor_data = readReliableSensorData();
        event = interpretSensorData(sensor_data);
    }

    sendNavLog("INFO", "Object at %s: %s (width=%.1fcm, dist=%dmm)",
               directionName(direction),
               eventToString(event.type),
               event.width_cm,
               event.distance_mm);

    /* Cell the beam actually hit at classification distance.  The object is
     * marked there, not at the scan candidate cell: an object seen through
     * an adjacent cell sits further away, and marking the candidate puts the
     * map entry (and later dedup matching) one or two cells off the object. */
    grid_cell_t hit_cell = candidate_cell;

    if (event.type == FIELD_ROCK_SAMPLE || event.type == FIELD_HILL) {
        hit_cell = projectMeasuredObjectCell(event.distance_mm, candidate_cell);

        /* A sighting that lands on (or one cell off) an already-marked
         * object is a repeat of something already classified and reported;
         * re-reporting it would double-count it — and a clipped hill edge
         * measures a small width, so the repeat typically arrives disguised
         * as a new rock sample. */
        if (isKnownObjectCell(hit_cell)) {
            printf("Duplicate sighting suppressed: %s hit=(%d,%d) overlaps classified object\n",
                   eventToString(event.type), hit_cell.x, hit_cell.y);
            sendNavLog("INFO", "Duplicate %s sighting at (%d,%d) suppressed",
                       eventToString(event.type), hit_cell.x, hit_cell.y);
            returnToPoseApprox(saved_pose);
            return event.type;
        }
    }

    switch (event.type) {
        case FIELD_ROCK_SAMPLE: {
            finalizeRockSampleAtCloseRange(&event);
            /* Override distance with the geometric distance to the hit cell
             * center so sendFieldEventUpdate projects the EVENT to the same
             * cell that markSampleAtCell marks (fixes icon/cell mismatch). */
            pose_t post_approach = getPose();
            float dx = cellCenterXCm(hit_cell) - post_approach.x;
            float dy = cellCenterYCm(hit_cell) - post_approach.y;
            event.distance_mm = (int)(sqrtf(dx * dx + dy * dy) * 10.0f);
            reportFieldEvent(event);
            markSampleAtCell(hit_cell);
            break;
        }

        case FIELD_HILL: {
            if (event.width_cm < HILL_PHYSICAL_SIZE_CM) {
                event.width_cm = HILL_PHYSICAL_SIZE_CM;
            }

            reportFieldEvent(event);
            markHillBlockAroundCell(hit_cell);
            break;
        }

        case FIELD_BLACK_TAPE:
            reportFieldEvent(event);
            markTapeAtCell(candidate_cell);
            break;

        case FIELD_SENSOR_FAULT:
            printf("Object confirmation failed: sensor fault\n");
            reportFieldEvent(event);
            /* A close object was confirmed right before the approach, so the
             * cell is not clear — it holds something that failed to classify.
             * SCANNED_CLEAR would make it a drive target and send the robot
             * into the object; UNKNOWN keeps it off-limits but rescannable. */
            setMapCellStatus(candidate_cell.x, candidate_cell.y, CELL_UNKNOWN);
            break;

        case FIELD_CLEAR:
        default:
            printf("Object confirmation failed: event=%s\n", eventToString(event.type));
            setMapCellStatus(candidate_cell.x, candidate_cell.y, CELL_UNKNOWN);
            break;
    }

    returnToPoseApprox(saved_pose);
    return event.type;
}

static void investigateObjectCandidate(void)
{
    while (object_candidate_index < object_candidate_count) {
        grid_cell_t candidate_cell = object_candidate_cells[object_candidate_index];
        nav_direction_t direction   = (nav_direction_t)object_candidate_dirs[object_candidate_index];
        float candidate_yaw         = object_candidate_yaws[object_candidate_index];

        if (isInsideMap(candidate_cell.x, candidate_cell.y) &&
            !isTerminalCellStatus(map_grid[candidate_cell.x][candidate_cell.y])) {
            investigateObjectAt(candidate_cell, direction, candidate_yaw);
        }

        object_candidate_index++;
    }

    transitionTo(NAV_STATE_CHOOSE_TARGET, "all object candidates investigated");
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
        printf("MOVE TARGET CENTER target reset to scanned_clear after ambiguous stop\n");
        setMapCellStatus(affected_cell.x,
                         affected_cell.y,
                         CELL_SCANNED_CLEAR);
        reverseToCellCenter(return_cell, return_yaw_deg);
        return;
    }

    /* The robot stops wherever the object first entered the front-object
     * window, which is usually closer than the width-classification
     * distance. Back up to it first, like the scan-360 investigation path,
     * so the width scan geometry matches the tuned thresholds. */
    if (moveToDistanceFromObject(INVESTIGATE_APPROACH_DISTANCE_MM) ==
            APPROACH_PUSHING) {
        sendNavLog("WARN", "Object at (%d,%d) pushed during approach — not classified",
                   affected_cell.x, affected_cell.y);
        setMapCellStatus(affected_cell.x, affected_cell.y, CELL_UNKNOWN);
        reverseToCellCenter(return_cell, return_yaw_deg);
        return;
    }

    sensor_data_t sensor_data = readReliableSensorData();
    field_event_t event = interpretSensorData(sensor_data);

    printf("MOVE TARGET CENTER object safety event=%s affected_cell=(%d,%d)\n",
           eventToString(event.type),
           affected_cell.x,
           affected_cell.y);

    /* Cell the beam actually hit at classification distance — objects are
     * marked there, not at the move target cell (the object that stopped the
     * move can sit one or two cells beyond the target). */
    grid_cell_t hit_cell = affected_cell;

    if (event.type == FIELD_ROCK_SAMPLE || event.type == FIELD_HILL) {
        hit_cell = projectMeasuredObjectCell(event.distance_mm, affected_cell);

        /* Same duplicate guard as the scan investigation: if the beam landed
         * on (or one cell off) an already-marked object, do not re-report it.
         * The cell goes back to unknown so it is never blindly entered but
         * stays eligible for a future rescan. */
        if (isKnownObjectCell(hit_cell)) {
            printf("MOVE TARGET CENTER duplicate sighting suppressed: %s hit=(%d,%d)\n",
                   eventToString(event.type), hit_cell.x, hit_cell.y);
            sendNavLog("INFO", "Duplicate %s sighting at (%d,%d) suppressed",
                       eventToString(event.type), hit_cell.x, hit_cell.y);
            setMapCellStatus(affected_cell.x, affected_cell.y, CELL_UNKNOWN);
            reverseToCellCenter(return_cell, return_yaw_deg);
            return;
        }
    }

    switch (event.type) {
        case FIELD_ROCK_SAMPLE: {
            finalizeRockSampleAtCloseRange(&event);
            /* Project the EVENT to the marked cell, like the scan path. */
            pose_t post_approach = getPose();
            float dx = cellCenterXCm(hit_cell) - post_approach.x;
            float dy = cellCenterYCm(hit_cell) - post_approach.y;
            event.distance_mm = (int)(sqrtf(dx * dx + dy * dy) * 10.0f);
            reportFieldEvent(event);
            markSampleAtCell(hit_cell);
            break;
        }

        case FIELD_HILL: {
            if (event.width_cm < HILL_PHYSICAL_SIZE_CM) {
                event.width_cm = HILL_PHYSICAL_SIZE_CM;
            }

            reportFieldEvent(event);
            markHillBlockAroundCell(hit_cell);
            break;
        }

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

    float total_moved_cm = 0.0f;

    while (mission_running) {
        pose_t pose = getPose();
        float delta_x = target_x_cm - pose.x;
        float delta_y = target_y_cm - pose.y;
        float distance_cm = sqrtf(delta_x * delta_x + delta_y * delta_y);

        if (total_moved_cm > CELL_SIZE_CM * 2.0f &&
            distance_cm > CELL_CENTER_REACHED_TOLERANCE_CM) {
            sendNavLog("WARN", "Overshot target (%d,%d) after %.1fcm — heading wrong, aborting",
                       target_cell.x, target_cell.y, total_moved_cm);
            printf("MOVE TARGET CENTER overshot: moved=%.2f cm, distance_remaining=%.2f cm — aborting\n",
                   total_moved_cm,
                   distance_cm);
            reverseToCellCenter(return_cell, start_pose.yaw);
            current_cell = return_cell;
            transitionTo(NAV_STATE_CHOOSE_TARGET,
                         "aborted — exceeded max movement without reaching target");
            return;
        }

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

            if (final_black_tape && !confirmBlackTapeReading()) {
                printf("MOVE TARGET CENTER black blip at target (%d,%d) rejected after confirmation\n",
                       target_cell.x,
                       target_cell.y);
                final_black_tape = false;
            }

            if (final_black_tape) {
                field_event_t event;
                /* Project the event to the target cell center (where the tape
                 * is) so the GUI icon lands on the same cell markTapeAtCell
                 * colors, instead of one cell further along the heading. */
                pose_t tape_pose = getPose();
                float tape_dx = cellCenterXCm(target_cell) - tape_pose.x;
                float tape_dy = cellCenterYCm(target_cell) - tape_pose.y;
                int tape_dist_mm =
                    (int)(sqrtf(tape_dx * tape_dx + tape_dy * tape_dy) * 10.0f);

                event.type = FIELD_BLACK_TAPE;
                event.distance_mm = tape_dist_mm > 0 ? tape_dist_mm : 1;
                event.width_cm = 0.0f;
                event.color = COLOR_BLACK;
                event.sample_size_cm = 0;
                event.temperature_c = 0.0f;

                sendNavLog("WARN", "Tape at target (%d,%d) — reversing to (%d,%d)",
                           target_cell.x, target_cell.y,
                           return_cell.x, return_cell.y);
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

        /* Gyro drift measured during previous increments shows up as a
         * heading error relative to the bearing to the target; correct it
         * here so the error cannot compound across increments. */
        if (distance_cm > FORWARD_INCREMENT_CM) {
            float bearing_deg = atan2f(delta_y, delta_x) * 180.0f / PI;
            float heading_error_deg = shortestAngleDelta(bearing_deg, pose.yaw);

            if (fabsf(heading_error_deg) > MOVE_REAIM_THRESHOLD_DEG) {
                printf("MOVE TARGET CENTER re-aim: heading error %.2f deg, %.2f cm from target\n",
                       heading_error_deg,
                       distance_cm);
                turnTowardPoint(target_x_cm, target_y_cm, DEFAULT_SPEED);
            }
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

        if (black_tape && !confirmBlackTapeReading()) {
            printf("MOVE TARGET CENTER black blip rejected after confirmation, resuming move to (%d,%d)\n",
                   target_cell.x,
                   target_cell.y);
            /* The confirmation nudges advanced the robot; restart the loop so
             * the remaining distance is recomputed before the next increment. */
            continue;
        }

        if (black_tape) {
            field_event_t event;
            /* Project the event to the target cell center (where the tape is)
             * so the GUI icon lands on the same cell markTapeAtCell colors. */
            pose_t tape_pose = getPose();
            float tape_dx = cellCenterXCm(target_cell) - tape_pose.x;
            float tape_dy = cellCenterYCm(target_cell) - tape_pose.y;
            int tape_dist_mm =
                (int)(sqrtf(tape_dx * tape_dx + tape_dy * tape_dy) * 10.0f);

            event.type = FIELD_BLACK_TAPE;
            event.distance_mm = tape_dist_mm > 0 ? tape_dist_mm : 1;
            event.width_cm = 0.0f;
            event.color = COLOR_BLACK;
            event.sample_size_cm = 0;
            event.temperature_c = 0.0f;

            sendNavLog("WARN", "Tape during move to (%d,%d) — reversing to (%d,%d)",
                       target_cell.x, target_cell.y,
                       return_cell.x, return_cell.y);
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
            int remaining_mm = (int)(distance_cm * 10.0f);

            /* An object further away than the end of this move (plus the
             * nose clearance) does not block it: stopping at the target
             * still leaves a gap.  A hill 20 cm beyond a backtrack target
             * must not strand the robot on the wrong side of the map. */
            if (front_window.best_close_distance_mm >
                    remaining_mm + MOVE_OBJECT_STOP_CLEARANCE_MM) {
                printf("MOVE TARGET CENTER object beyond target ignored: dist=%d mm, remaining=%d mm\n",
                       front_window.best_close_distance_mm,
                       remaining_mm);
            } else {
                sendNavLog("WARN", "Object during move to (%d,%d): dist=%dmm",
                           target_cell.x, target_cell.y,
                           front_window.display_distance_mm);
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
        }

        float step_cm = distance_cm;

        if (step_cm > FORWARD_INCREMENT_CM) {
            step_cm = FORWARD_INCREMENT_CM;
        }

        moveWithRamp(step_cm, DEFAULT_SPEED);
        total_moved_cm += step_cm;
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
        sendNavLog("INFO", "Backtracking to (%d,%d), %d cells remain in stack",
                   target_cell.x, target_cell.y, dfs_stack_top);
        transitionTo(NAV_STATE_MOVE_TO_TARGET_CENTER,
                     "valid adjacent explored backtrack cell");
        return;
    }

    sendNavLog("WARN", "DFS stack empty — no reachable explored cells");
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

    sendNavLog("INFO", "Side complete — explored: %d  blocked: %d  unknown: %d  clear: %d",
               explored_count, blocked_count, unknown_count, scanned_clear_count);
    sendStatusUpdate("side_complete", "none");
    mission_running = false;
}

static void handleNavigationError(void)
{
    printf("Navigation error: stopping mission\n");
    sendNavLog("ERROR", "Navigation error — mission stopped");
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
    state_entry_msec   = nav_log_start_msec;
    nav_log_timestamps_enabled = true;

    /*
     * Mission start delay: stay still for a few seconds so the operator can
     * step clear, then recalibrate the gyro bias while the robot is guaranteed
     * stationary before any movement begins.  STOP_MISSION still cancels here.
     */
    sendNavLog("INFO", "Mission starts in %ds — hold still, calibrating gyro",
               MISSION_START_DELAY_SEC);

    double start_delay_until = navNowMsec() + (double)MISSION_START_DELAY_SEC * 1000.0;

    while (mission_running && navNowMsec() < start_delay_until) {
        pollESP32Messages();
    }

    if (!mission_running) {
        printf("Mission cancelled during start delay.\n");
        return;
    }

    recalibrateGyroIfReady();
    sendNavLog("INFO", "Gyro calibrated — mission underway");

    while (mission_running) {
        pollESP32Messages();

        if (!mission_running) {
            break;
        }

        if (fsm_cycles > max_fsm_cycles) {
            transitionTo(NAV_STATE_ERROR, "FSM cycle limit exceeded");
        }

        /* Watchdog: recover if a state has run longer than its allowed budget */
        {
            double timeout_sec = stateTimeoutSeconds(current_state);
            double elapsed_sec = (navNowMsec() - state_entry_msec) / 1000.0;

            if (timeout_sec > 0.0 && elapsed_sec > timeout_sec) {
                nav_state_t recovery = watchdogRecoveryState(current_state);
                sendNavLog("ERROR", "Watchdog: %s timed out after %.1fs — recovering to %s",
                           navStateToString(current_state),
                           elapsed_sec,
                           navStateToString(recovery));
                transitionTo(recovery, "watchdog timeout");
            }
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
