#include <math.h>
#include <stdlib.h>

#include "config.h"
#include "odometry.h"
#include "mission_frame.h"

int missionFrameSign(void)
{
#if ROBOT_START_ORIENTATION == ROBOT_START_BACK
    return -1;
#else
    return 1;
#endif
}

const char *missionFrameStartName(void)
{
#if ROBOT_START_ORIENTATION == ROBOT_START_BACK
    return "back_half_negative_x";
#else
    return "front_half_positive_x";
#endif
}

pose_t missionFrameLocalPoseToGlobal(pose_t local_pose)
{
    pose_t global_pose;
    int sign = missionFrameSign();
    float offset_cm = (float)ROBOT_START_OFFSET_CELLS * CELL_SIZE_CM;

    global_pose.x = (float)sign * offset_cm + (float)sign * local_pose.x;
    global_pose.y = (float)sign * local_pose.y;

    if (sign < 0) {
        global_pose.yaw = normalizeAngle(local_pose.yaw + 180.0f);
    } else {
        global_pose.yaw = normalizeAngle(local_pose.yaw);
    }

    return global_pose;
}

void missionFrameLocalPointToGlobal(float local_x_cm,
                                    float local_y_cm,
                                    float *global_x_cm,
                                    float *global_y_cm)
{
    int sign = missionFrameSign();
    float offset_cm = (float)ROBOT_START_OFFSET_CELLS * CELL_SIZE_CM;

    if (global_x_cm != NULL) {
        *global_x_cm = (float)sign * offset_cm + (float)sign * local_x_cm;
    }

    if (global_y_cm != NULL) {
        *global_y_cm = (float)sign * local_y_cm;
    }
}

void missionFrameLocalRelativeCellToGlobalCell(int local_rel_x,
                                                int local_rel_y,
                                                int *global_cell_x,
                                                int *global_cell_y)
{
    int sign = missionFrameSign();

    if (global_cell_x != NULL) {
        *global_cell_x = sign * ROBOT_START_OFFSET_CELLS + sign * local_rel_x;
    }

    if (global_cell_y != NULL) {
        *global_cell_y = sign * local_rel_y;
    }
}

void missionFrameLocalGridIndexToGlobalCell(int local_grid_x,
                                             int local_grid_y,
                                             int *global_cell_x,
                                             int *global_cell_y)
{
    int local_rel_x = local_grid_x - MAP_CENTER;
    int local_rel_y = local_grid_y - MAP_CENTER;

    missionFrameLocalRelativeCellToGlobalCell(local_rel_x,
                                               local_rel_y,
                                               global_cell_x,
                                               global_cell_y);
}

void missionFrameLocalPoseToGlobalCell(pose_t local_pose,
                                        int *global_cell_x,
                                        int *global_cell_y)
{
    int local_rel_x = (int)roundf(local_pose.x / CELL_SIZE_CM);
    int local_rel_y = (int)roundf(local_pose.y / CELL_SIZE_CM);

    missionFrameLocalRelativeCellToGlobalCell(local_rel_x,
                                               local_rel_y,
                                               global_cell_x,
                                               global_cell_y);
}

bool missionFrameGlobalCellIsInAssignedHalf(int global_cell_x)
{
#if EXPLORATION_USE_STATIC_PARTITION
    int sign = missionFrameSign();

    if (sign > 0) {
        return global_cell_x > PARTITION_BUFFER_CELLS;
    }

    return global_cell_x < -PARTITION_BUFFER_CELLS;
#else
    (void)global_cell_x;
    return true;
#endif
}

bool missionFrameLocalGridIndexIsInAssignedHalf(int local_grid_x,
                                                 int local_grid_y)
{
    int global_cell_x = 0;
    int global_cell_y = 0;

    missionFrameLocalGridIndexToGlobalCell(local_grid_x,
                                            local_grid_y,
                                            &global_cell_x,
                                            &global_cell_y);
    (void)global_cell_y;

    return missionFrameGlobalCellIsInAssignedHalf(global_cell_x);
}
