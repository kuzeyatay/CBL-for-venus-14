#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

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

typedef struct {
    int x;
    int y;
} grid_cell_t;

static bool mission_running = false;
static bool map_initialized = false;
static cell_status_t map_grid[MAP_SIZE][MAP_SIZE];

static const char *colorToString(sample_color_t color);
static const char *eventToString(field_event_type_t event_type);
static const char *cellStatusToString(cell_status_t status);

static void markCellAhead(cell_status_t status);
static void markCurrentCell(cell_status_t status);
static void reportFieldEvent(field_event_t event);
static void handleBlockingFieldEvent(field_event_t event);
static bool scanAfterForwardMoveAndApproachObject(void);

static void setMapCellStatus(int grid_x, int grid_y, cell_status_t status);
static void markRectangularFootprint(float center_distance_cm,
                                     float footprint_width_cm,
                                     float footprint_depth_cm,
                                     cell_status_t status);
static void markDetectedObjectFootprint(field_event_t event, cell_status_t status);
static void markBlackTapeFootprint(void);

static bool pointToLocalGridCell(float x_cm, float y_cm, grid_cell_t *cell);
static bool scanDetectionProjectsToInvestigatableCell(int distance_mm, grid_cell_t *detected_cell);


static bool cellsEqual(grid_cell_t a, grid_cell_t b)
{
    return a.x == b.x && a.y == b.y;
}

static bool isInsideMap(int x, int y)
{
    return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE;
}

static bool isUsableDistance(int distance_mm)
{
    return distance_mm > 0 &&
           distance_mm != VL53L0X_INVALID_DISTANCE_MM &&
           distance_mm <= VL53L0X_MAX_REASONABLE_MM;
}

static void initMapOnce(void)
{
    if (map_initialized) {
        return;
    }

    for (int x = 0; x < MAP_SIZE; x++) {
        for (int y = 0; y < MAP_SIZE; y++) {
            map_grid[x][y] = CELL_UNKNOWN;
        }
    }

    map_grid[MAP_CENTER][MAP_CENTER] = CELL_EXPLORED;
    sendCellUpdate(MAP_CENTER, MAP_CENTER, CELL_EXPLORED);

    map_initialized = true;

    printf("Map initialized. Robot %s uses %s.\n",
           ROBOT_ID,
           missionFrameStartName());
}

static bool poseToLocalGridCell(float x_cm, float y_cm, int *grid_x, int *grid_y)
{
    int gx = MAP_CENTER + (int)roundf(x_cm / CELL_SIZE_CM);
    int gy = MAP_CENTER + (int)roundf(y_cm / CELL_SIZE_CM);

    if (!isInsideMap(gx, gy)) {
        return false;
    }

    if (grid_x != NULL) {
        *grid_x = gx;
    }

    if (grid_y != NULL) {
        *grid_y = gy;
    }

    return true;
}

static bool getCurrentLocalCell(grid_cell_t *cell)
{
    pose_t pose = getPose();
    int gx = 0;
    int gy = 0;

    if (!poseToLocalGridCell(pose.x, pose.y, &gx, &gy)) {
        return false;
    }

    if (cell != NULL) {
        cell->x = gx;
        cell->y = gy;
    }

    return true;
}
static bool pointToLocalGridCell(float x_cm, float y_cm, grid_cell_t *cell)
{
    int gx = 0;
    int gy = 0;

    if (!poseToLocalGridCell(x_cm, y_cm, &gx, &gy)) {
        return false;
    }

    if (cell != NULL) {
        cell->x = gx;
        cell->y = gy;
    }

    return true;
}

static bool scanDetectionProjectsToInvestigatableCell(int distance_mm, grid_cell_t *detected_cell)
{
    if (!isUsableDistance(distance_mm)) {
        return false;
    }

    pose_t pose = getPose();
    float yaw_rad = degToRad(pose.yaw);
    float distance_cm = (float)distance_mm / 10.0f;

    float detected_x_cm = pose.x + distance_cm * cosf(yaw_rad);
    float detected_y_cm = pose.y + distance_cm * sinf(yaw_rad);

    grid_cell_t cell;

    if (!pointToLocalGridCell(detected_x_cm, detected_y_cm, &cell)) {
        printf("Scan candidate ignored: projected point is outside map.\n");
        return false;
    }

    if (detected_cell != NULL) {
        *detected_cell = cell;
    }

    if (!missionFrameLocalGridIndexIsInAssignedHalf(cell.x, cell.y)) {
        printf("Scan candidate ignored: projected cell local=(%d,%d) is outside assigned half.\n",
               cell.x,
               cell.y);
        return false;
    }

    /*
     * Only investigate objects that project into unknown/reserved territory.
     * If the object projects into an explored cell, it has already been covered
     * and should not interrupt route-following through explored cells.
     */
if (map_grid[cell.x][cell.y] == CELL_RESERVED) {
    int global_x = 0;
    int global_y = 0;

    missionFrameLocalGridIndexToGlobalCell(cell.x, cell.y, &global_x, &global_y);

    printf("Scan candidate accepted: projected cell local=(%d,%d), global=(%d,%d), status=reserved. Object is inside the current target cell.\n",
           cell.x,
           cell.y,
           global_x,
           global_y);

    return true;
}

if (map_grid[cell.x][cell.y] != CELL_UNKNOWN) {
    int global_x = 0;
    int global_y = 0;

    missionFrameLocalGridIndexToGlobalCell(cell.x, cell.y, &global_x, &global_y);

    printf("Scan candidate ignored: projected cell local=(%d,%d), global=(%d,%d), status=%s\n",
           cell.x,
           cell.y,
           global_x,
           global_y,
           cellStatusToString(map_grid[cell.x][cell.y]));

    return false;
}

    return true;
}

static void markCurrentCell(cell_status_t status)
{
    grid_cell_t current;

    if (!getCurrentLocalCell(&current)) {
        return;
    }

    map_grid[current.x][current.y] = status;
    sendCellUpdate(current.x, current.y, status);
}

static void markCellAhead(cell_status_t status)
{
    pose_t pose = getPose();
    float yaw_rad = degToRad(pose.yaw);
    float ahead_x = pose.x + CELL_SIZE_CM * cosf(yaw_rad);
    float ahead_y = pose.y + CELL_SIZE_CM * sinf(yaw_rad);
    int gx = 0;
    int gy = 0;

    if (!poseToLocalGridCell(ahead_x, ahead_y, &gx, &gy)) {
        return;
    }

    map_grid[gx][gy] = status;
    sendCellUpdate(gx, gy, status);
}

static void setMapCellStatus(int grid_x, int grid_y, cell_status_t status)
{
    if (!isInsideMap(grid_x, grid_y)) {
        return;
    }

    /*
     * Do not mark the robot's current cell as an obstacle/hazard.
     * This prevents the robot from blocking itself in the planner.
     */
    grid_cell_t current;

    if (getCurrentLocalCell(&current)) {
        if (current.x == grid_x && current.y == grid_y &&
            status != CELL_EXPLORED &&
            status != CELL_RESERVED) {
            return;
        }
    }

    /*
     * Preserve stronger hazard information.
     * Unsafe cells should not be overwritten by hill/sample.
     */
    if (map_grid[grid_x][grid_y] == CELL_UNSAFE && status != CELL_UNSAFE) {
        return;
    }

    /*
     * Preserve hill cells against sample overwrite.
     * A large obstacle should stay blocked.
     */
    if (map_grid[grid_x][grid_y] == CELL_HILL && status == CELL_SAMPLE) {
        return;
    }

    map_grid[grid_x][grid_y] = status;
    sendCellUpdate(grid_x, grid_y, status);
}

static void markRectangularFootprint(float center_distance_cm,
                                     float footprint_width_cm,
                                     float footprint_depth_cm,
                                     cell_status_t status)
{
    pose_t pose = getPose();
    float yaw_rad = degToRad(pose.yaw);

    float forward_x = cosf(yaw_rad);
    float forward_y = sinf(yaw_rad);

    /*
     * Left/right direction perpendicular to robot heading.
     */
    float lateral_x = -sinf(yaw_rad);
    float lateral_y = cosf(yaw_rad);

    float center_x = pose.x + center_distance_cm * forward_x;
    float center_y = pose.y + center_distance_cm * forward_y;

    float half_width = footprint_width_cm / 2.0f;
    float half_depth = footprint_depth_cm / 2.0f;

    printf("Marking footprint: center_distance=%.1f cm, width=%.1f cm, depth=%.1f cm, status=%d\n",
           center_distance_cm,
           footprint_width_cm,
           footprint_depth_cm,
           status);

    /*
     * Check every grid-cell center and mark the cells whose centers fall inside
     * the estimated physical footprint rectangle.
     */
    for (int gx = 0; gx < MAP_SIZE; gx++) {
        for (int gy = 0; gy < MAP_SIZE; gy++) {
            float cell_center_x = (float)(gx - MAP_CENTER) * CELL_SIZE_CM;
            float cell_center_y = (float)(gy - MAP_CENTER) * CELL_SIZE_CM;

            float dx = cell_center_x - center_x;
            float dy = cell_center_y - center_y;

            float forward_projection = dx * forward_x + dy * forward_y;
            float lateral_projection = dx * lateral_x + dy * lateral_y;

            bool inside_depth = fabsf(forward_projection) <= half_depth;
            bool inside_width = fabsf(lateral_projection) <= half_width;

            if (inside_depth && inside_width) {
                setMapCellStatus(gx, gy, status);
            }
        }
    }
}

static void markDetectedObjectFootprint(field_event_t event, cell_status_t status)
{
    if (!isUsableDistance(event.distance_mm)) {
        /*
         * Fallback: if the measured distance is invalid, use the old one-cell-ahead
         * distance but still mark a footprint instead of one point.
         */
        if (status == CELL_HILL) {
            markRectangularFootprint(CELL_SIZE_CM,
                                     HILL_PHYSICAL_SIZE_CM + HILL_FOOTPRINT_MARGIN_CM,
                                     HILL_PHYSICAL_SIZE_CM + HILL_FOOTPRINT_MARGIN_CM,
                                     status);
        } else {
            markRectangularFootprint(CELL_SIZE_CM,
                                     CELL_SIZE_CM,
                                     CELL_SIZE_CM,
                                     status);
        }

        return;
    }

    float front_distance_cm = (float)event.distance_mm / 10.0f;

    if (status == CELL_HILL) {
        /*
         * A hill is at least 30 cm wide/deep according to the mission rules.
         * If the width scan gives a larger apparent width, use that instead.
         */
        float width_cm = HILL_PHYSICAL_SIZE_CM;

        if (event.width_cm > width_cm) {
            width_cm = event.width_cm;
        }

        width_cm += HILL_FOOTPRINT_MARGIN_CM;

        float depth_cm = HILL_PHYSICAL_SIZE_CM + HILL_FOOTPRINT_MARGIN_CM;

        /*
         * VL53L0X distance is approximately to the front face, so the center of
         * the hill footprint is estimated half a depth behind that front face.
         */
        float center_distance_cm = front_distance_cm + depth_cm / 2.0f;

        markRectangularFootprint(center_distance_cm,
                                 width_cm,
                                 depth_cm,
                                 CELL_HILL);

        return;
    }

    if (status == CELL_SAMPLE) {
        /*
         * Samples are 3 cm or 6 cm cubes. Use the classified size when available.
         * The margin makes sure a sample near a grid boundary still appears.
         */
        float sample_size_cm = 6.0f;

        if (event.sample_size_cm == 3 || event.sample_size_cm == 6) {
            sample_size_cm = (float)event.sample_size_cm;
        }

        float footprint_cm = sample_size_cm + SAMPLE_FOOTPRINT_MARGIN_CM;
        float center_distance_cm = front_distance_cm + sample_size_cm / 2.0f;

        /*
         * Guarantee at least one grid cell is marked.
         */
        if (footprint_cm < CELL_SIZE_CM) {
            footprint_cm = CELL_SIZE_CM;
        }

        markRectangularFootprint(center_distance_cm,
                                 footprint_cm,
                                 footprint_cm,
                                 CELL_SAMPLE);

        return;
    }

    /*
     * Generic fallback.
     */
    markRectangularFootprint(front_distance_cm,
                             CELL_SIZE_CM,
                             CELL_SIZE_CM,
                             status);
}

static void markBlackTapeFootprint(void)
{
    /*
     * The TCS3200 sees tape close to the robot/front-bottom area.
     * Model it as a short unsafe strip in front of the robot.
     */
    markRectangularFootprint(BLACK_TAPE_FOOTPRINT_DISTANCE_CM,
                             BLACK_TAPE_FOOTPRINT_WIDTH_CM,
                             BLACK_TAPE_FOOTPRINT_DEPTH_CM,
                             CELL_UNSAFE);
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

static const char *eventToString(field_event_type_t event_type)
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

static const char *cellStatusToString(cell_status_t status)
{
    switch (status) {
        case CELL_UNKNOWN:  return "unknown";
        case CELL_EXPLORED: return "explored";
        case CELL_RESERVED: return "reserved";
        case CELL_UNSAFE:   return "unsafe";
        case CELL_HILL:     return "hill";
        case CELL_SAMPLE:   return "sample";
        default:            return "invalid";
    }
}

static bool estimateSampleSize(float width_cm, int *sample_size_cm)
{
    /* Empirical thresholds for the VL53L0X width scan. Tune after tests. */
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

    if (data.black_tape_detected) {
        event.type = FIELD_BLACK_TAPE;
        event.color = COLOR_BLACK;
        return event;
    }

    if (data.front_object_detected && data.front_distance_mm <= FRONT_OBJECT_THRESHOLD_MM) {
        int sample_size = 0;

         printf("VL53L0X distance trigger: %d mm is smaller than FRONT_OBJECT_THRESHOLD_MM=%d mm. Starting width scan.\n",
           data.front_distance_mm,
           FRONT_OBJECT_THRESHOLD_MM);

        if (estimateSampleSize(data.estimated_width_cm, &sample_size)) {
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

static int headingToDirectionIndex(float yaw_deg)
{
    float yaw = normalizeAngle(yaw_deg);
    int dir = (int)floorf((yaw + 45.0f) / 90.0f);
    return dir % 4;
}

static float directionIndexToYaw(int direction_index)
{
    int dir = direction_index % 4;

    if (dir < 0) {
        dir += 4;
    }

    return (float)dir * 90.0f;
}

static float signedTurnErrorDeg(float target_yaw_deg, float current_yaw_deg)
{
    float diff = normalizeAngle(target_yaw_deg - current_yaw_deg);

    if (diff > 180.0f) {
        diff -= 360.0f;
    }

    return diff;
}

static int directionFromCellStep(grid_cell_t from, grid_cell_t to)
{
    int dx = to.x - from.x;
    int dy = to.y - from.y;

    if (dx == 1 && dy == 0) {
        return 0;
    }

    if (dx == 0 && dy == 1) {
        return 1;
    }

    if (dx == -1 && dy == 0) {
        return 2;
    }

    if (dx == 0 && dy == -1) {
        return 3;
    }

    return -1;
}

static bool isTraversableForPlanning(grid_cell_t cell)
{
    if (!isInsideMap(cell.x, cell.y)) {
        return false;
    }

    if (!missionFrameLocalGridIndexIsInAssignedHalf(cell.x, cell.y)) {
        return false;
    }

    return map_grid[cell.x][cell.y] == CELL_EXPLORED;
}

static bool isExplorationTarget(grid_cell_t cell)
{
    if (!isInsideMap(cell.x, cell.y)) {
        return false;
    }

    if (!missionFrameLocalGridIndexIsInAssignedHalf(cell.x, cell.y)) {
        return false;
    }

    return map_grid[cell.x][cell.y] == CELL_UNKNOWN;
}

static void getOrderedNeighborDirections(int base_direction, int ordered_dirs[4])
{
    ordered_dirs[0] = base_direction;
    ordered_dirs[1] = (base_direction + 1) % 4;
    ordered_dirs[2] = (base_direction + 3) % 4;
    ordered_dirs[3] = (base_direction + 2) % 4;
}

static grid_cell_t neighborInDirection(grid_cell_t cell, int direction)
{
    static const int dx[4] = {1, 0, -1, 0};
    static const int dy[4] = {0, 1, 0, -1};
    grid_cell_t neighbor;

    neighbor.x = cell.x + dx[direction];
    neighbor.y = cell.y + dy[direction];

    return neighbor;
}

static bool findNextCellTowardFrontier(grid_cell_t current, grid_cell_t *next_cell)
{
    bool visited[MAP_SIZE][MAP_SIZE] = {{false}};
    grid_cell_t parent[MAP_SIZE][MAP_SIZE];
    grid_cell_t queue[MAP_SIZE * MAP_SIZE];
    int head = 0;
    int tail = 0;
    int ordered_dirs[4];
    int base_dir = headingToDirectionIndex(getPose().yaw);

    for (int x = 0; x < MAP_SIZE; x++) {
        for (int y = 0; y < MAP_SIZE; y++) {
            parent[x][y].x = -1;
            parent[x][y].y = -1;
        }
    }

    getOrderedNeighborDirections(base_dir, ordered_dirs);

    visited[current.x][current.y] = true;
    queue[tail] = current;
    tail++;

    while (head < tail) {
        grid_cell_t cell = queue[head];
        head++;

        for (int i = 0; i < 4; i++) {
            grid_cell_t neighbor = neighborInDirection(cell, ordered_dirs[i]);

            if (!isInsideMap(neighbor.x, neighbor.y)) {
                continue;
            }

            if (visited[neighbor.x][neighbor.y]) {
                continue;
            }

            if (!missionFrameLocalGridIndexIsInAssignedHalf(neighbor.x, neighbor.y)) {
                continue;
            }

            parent[neighbor.x][neighbor.y] = cell;
            visited[neighbor.x][neighbor.y] = true;

            if (isExplorationTarget(neighbor)) {
    grid_cell_t frontier_target = neighbor;
    grid_cell_t step = neighbor;
    int path_length_cells = 1;

    while (!cellsEqual(parent[step.x][step.y], current)) {
        step = parent[step.x][step.y];
        path_length_cells++;

        if (step.x < 0 || step.y < 0) {
            return false;
        }
    }

    int current_global_x = 0;
    int current_global_y = 0;
    int step_global_x = 0;
    int step_global_y = 0;
    int target_global_x = 0;
    int target_global_y = 0;

    missionFrameLocalGridIndexToGlobalCell(current.x,
                                            current.y,
                                            &current_global_x,
                                            &current_global_y);

    missionFrameLocalGridIndexToGlobalCell(step.x,
                                            step.y,
                                            &step_global_x,
                                            &step_global_y);

    missionFrameLocalGridIndexToGlobalCell(frontier_target.x,
                                            frontier_target.y,
                                            &target_global_x,
                                            &target_global_y);

    printf("PLANNER DECISION:\n");
    printf("  current cell: local=(%d,%d), global=(%d,%d), status=%s\n",
           current.x,
           current.y,
           current_global_x,
           current_global_y,
           cellStatusToString(map_grid[current.x][current.y]));

    printf("  nearest frontier target: local=(%d,%d), global=(%d,%d), status=%s\n",
           frontier_target.x,
           frontier_target.y,
           target_global_x,
           target_global_y,
           cellStatusToString(map_grid[frontier_target.x][frontier_target.y]));

    printf("  nextly decided step: local=(%d,%d), global=(%d,%d), status=%s, path_length=%d cell(s)\n",
           step.x,
           step.y,
           step_global_x,
           step_global_y,
           cellStatusToString(map_grid[step.x][step.y]),
           path_length_cells);

    *next_cell = step;
    return true;
}

            if (isTraversableForPlanning(neighbor)) {
                if (tail < (MAP_SIZE * MAP_SIZE)) {
                    queue[tail] = neighbor;
                    tail++;
                }
            }
        }
    }

    return false;
}

static void moveToDistanceFromObject(int target_distance_mm)
{
    int distance_mm = readVL53L0XDistance();

    if (!isUsableDistance(distance_mm)) {
        printf("Sample approach failed: invalid distance reading = %d mm\n", distance_mm);
        return;
    }

    int error_mm = distance_mm - target_distance_mm;

    if (abs(error_mm) <= SAMPLE_APPROACH_TOLERANCE_MM) {
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

    if (!tape_scan_enabled) {
        printf("Post-move tape scan disabled: TCS3200 is not calibrated.\n");
    }

    /*
     * Scan symmetrically around the current heading.
     * At the end of the scan, current_angle_deg stores where the robot is
     * relative to the original heading.
     */
    turn(-half_scan_deg, POST_MOVE_SCAN_SPEED_CM_S);
    current_angle_deg = -half_scan_deg;

    while (current_angle_deg <= half_scan_deg + 0.01f) {
        int distance_mm = readVL53L0XDistance();
        bool black_tape_here = false;

        if (tape_scan_enabled) {
            black_tape_here = tcs3200DetectBlackTape();

            if (black_tape_here && !found_black_tape) {
                found_black_tape = true;
                black_tape_angle_deg = current_angle_deg;
            }
        }

        grid_cell_t projected_cell;
bool projected_cell_ok = false;

bool close_object_by_distance =
    isUsableDistance(distance_mm) &&
    distance_mm <= POST_MOVE_SCAN_MAX_DISTANCE_MM &&
    distance_mm <= FRONT_OBJECT_THRESHOLD_MM &&
    !black_tape_here;

bool object_candidate = false;

if (close_object_by_distance) {
    projected_cell_ok =
        scanDetectionProjectsToInvestigatableCell(distance_mm, &projected_cell);

    object_candidate = projected_cell_ok;
}

printf("Post-move scan: angle=%.1f deg, distance=%d mm, black_tape=%s, close_by_distance=%s, object_candidate=%s",
       current_angle_deg,
       distance_mm,
       black_tape_here ? "yes" : "no",
       close_object_by_distance ? "yes" : "no",
       object_candidate ? "yes" : "no");

if (close_object_by_distance && projected_cell_ok) {
    int global_x = 0;
    int global_y = 0;

    missionFrameLocalGridIndexToGlobalCell(projected_cell.x,
                                            projected_cell.y,
                                            &global_x,
                                            &global_y);

    printf(", projected_cell local=(%d,%d), global=(%d,%d), status=%s",
           projected_cell.x,
           projected_cell.y,
           global_x,
           global_y,
           cellStatusToString(map_grid[projected_cell.x][projected_cell.y]));
}

printf("\n");

if (object_candidate) {
    if (!found_object || distance_mm < best_distance_mm) {
        found_object = true;
        best_distance_mm = distance_mm;
        best_angle_deg = current_angle_deg;
    }
}
        if (current_angle_deg >= half_scan_deg) {
            break;
        }

        float step_deg = POST_MOVE_SCAN_STEP_DEG;

        if (current_angle_deg + step_deg > half_scan_deg) {
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
    if (found_black_tape) {
        printf("Post-move scan found black tape: angle=%.1f deg. Avoiding instead of approaching.\n",
               black_tape_angle_deg);

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

        /*
         * Return false because the robot did not continue safely toward the
         * reserved cell. The cell-exploration step should stop here and replan.
         */
        return false;
    }

if (found_object) {
    printf("Post-move scan found object: angle=%.1f deg, distance=%d mm\n",
           best_angle_deg,
           best_distance_mm);

    /*
     * Face the detected object, but do not classify it here.
     * The post-move scan already found the object. Stop this cell movement
     * and let the next navigation step decide what to do.
     */
    turn(best_angle_deg - current_angle_deg, POST_MOVE_SCAN_SPEED_CM_S);
    sendPoseUpdate();

    printf("Post-move scan stopped movement because an object is ahead. No width/color classification here.\n");

    return false;
}

    printf("Post-move scan found no object and no black tape. Returning to original heading.\n");
    turn(-current_angle_deg, POST_MOVE_SCAN_SPEED_CM_S);
    sendPoseUpdate();

    /*
     * Return true because nothing interrupted the cell-to-cell movement.
     */
    return true;
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

static void handleBlockingFieldEvent(field_event_t event)
{
    printf("Blocking event: %s\n", eventToString(event.type));

    switch (event.type) {
        case FIELD_BLACK_TAPE:
            markBlackTapeFootprint();
            reportFieldEvent(event);
            moveWithRamp(-REVERSE_DISTANCE_CM, DEFAULT_SPEED);
            turn(AVOID_TURN_DEG, DEFAULT_SPEED);
            sendPoseUpdate();
            break;

        case FIELD_HILL:
            markDetectedObjectFootprint(event, CELL_HILL);
            reportFieldEvent(event);
            moveWithRamp(-REVERSE_DISTANCE_CM, DEFAULT_SPEED);
            turn(AVOID_TURN_DEG, DEFAULT_SPEED);
            sendPoseUpdate();
            break;

        case FIELD_ROCK_SAMPLE:
            /*
             * First move to the sample and classify it.
             * Then use the final distance and sample size to mark the footprint.
             */
            finalizeRockSampleAtCloseRange(&event);
            markDetectedObjectFootprint(event, CELL_SAMPLE);
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

        case FIELD_CLEAR:
        default:
            break;
    }
}

static bool checkImmediateSafety(void)
{
    sensor_data_t sensor_data = readReliableSensorData();
    field_event_t event = interpretSensorData(sensor_data);

    if (event.type == FIELD_CLEAR) {
        return true;
    }

    handleBlockingFieldEvent(event);
    return false;
}

static bool moveForwardOneCellWithSafety(void)
{
    float remaining_cm = CELL_SIZE_CM;

    while (remaining_cm > 0.01f && mission_running) {
        float step_cm = FORWARD_INCREMENT_CM;

        if (step_cm > remaining_cm) {
            step_cm = remaining_cm;
        }

        if (!checkImmediateSafety()) {
            return false;
        }

        moveWithRamp(step_cm, DEFAULT_SPEED);
        sendPoseUpdate();

        /*
         * Restore the old behavior: after every normal forward increment,
         * scan around the robot. If the scan sees black tape, it avoids it.
         * If the scan sees an object, it turns/approaches it and the current
         * cell movement stops so the next navigation step can classify it.
         */
        if (!scanAfterForwardMoveAndApproachObject()) {
            return false;
        }

        remaining_cm -= step_cm;

        if (!checkImmediateSafety()) {
            return false;
        }
    }

    return true;
}

static bool moveToAdjacentCell(grid_cell_t current, grid_cell_t target)
{
    int direction = directionFromCellStep(current, target);

    if (direction < 0) {
        sendErrorMessage("non_adjacent_target");
        return false;
    }

    float target_yaw = directionIndexToYaw(direction);
    float turn_deg = signedTurnErrorDeg(target_yaw, getPose().yaw);

    if (fabsf(turn_deg) > 1.0f) {
        turn(turn_deg, DEFAULT_SPEED);
        sendPoseUpdate();
    }

    if (map_grid[target.x][target.y] == CELL_UNKNOWN) {
        map_grid[target.x][target.y] = CELL_RESERVED;
        sendCellUpdate(target.x, target.y, CELL_RESERVED);
    }

    if (!moveForwardOneCellWithSafety()) {
        return false;
    }

    markCurrentCell(CELL_EXPLORED);
    return true;
}

static void executeOneExplorationStep(void)
{
    grid_cell_t current;
    grid_cell_t next;
    int global_x = 0;
    int global_y = 0;

    if (!getCurrentLocalCell(&current)) {
        sendErrorMessage("pose_out_of_map");
        stopMission();
        return;
    }

    markCurrentCell(CELL_EXPLORED);

    missionFrameLocalGridIndexToGlobalCell(current.x, current.y, &global_x, &global_y);
    printf("Current local cell=(%d,%d), global cell=(%d,%d)\n",
           current.x,
           current.y,
           global_x,
           global_y);

    if (!findNextCellTowardFrontier(current, &next)) {
        printf("No reachable unexplored cell remains in assigned half.\n");
        sendStatusUpdate("side_complete", "none");
        stopMission();
        return;
    }

    missionFrameLocalGridIndexToGlobalCell(next.x, next.y, &global_x, &global_y);
printf("Next local cell=(%d,%d), global cell=(%d,%d), status=%d\n",
       next.x,
       next.y,
       global_x,
       global_y,
       map_grid[next.x][next.y]);

    (void)moveToAdjacentCell(current, next);
}

void startMission(void)
{
    initMapOnce();
    mission_running = true;
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
    initMapOnce();

    printf("Starting cell-by-cell autonomous navigation for %s.\n", ROBOT_ID);
    mission_running = true;

    for (int step = 0; step < MAX_NAVIGATION_STEPS && mission_running; step++) {
        pollESP32Messages();

        printf("\n--- Cell exploration step %d ---\n", step);
        printPose();
        sendPoseUpdate();

        executeOneExplorationStep();
    }

    printf("Navigation loop finished.\n");
}
