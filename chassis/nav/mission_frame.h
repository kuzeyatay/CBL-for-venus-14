#ifndef MISSION_FRAME_H
#define MISSION_FRAME_H

#include <stdbool.h>
#include "robot_types.h"

/*
 * Converts robot-local odometry/map coordinates into one shared mission frame.
 * This is what keeps a back-to-back two-robot start consistent in the GUI.
 */

int missionFrameSign(void);

pose_t missionFrameLocalPoseToGlobal(pose_t local_pose);
void missionFrameLocalPointToGlobal(float local_x_cm,
                                    float local_y_cm,
                                    float *global_x_cm,
                                    float *global_y_cm);

void missionFrameLocalGridIndexToGlobalCell(int local_grid_x,
                                             int local_grid_y,
                                             int *global_cell_x,
                                             int *global_cell_y);

void missionFrameLocalRelativeCellToGlobalCell(int local_rel_x,
                                                int local_rel_y,
                                                int *global_cell_x,
                                                int *global_cell_y);

void missionFrameLocalPoseToGlobalCell(pose_t local_pose,
                                        int *global_cell_x,
                                        int *global_cell_y);

bool missionFrameGlobalCellIsInAssignedHalf(int global_cell_x);
bool missionFrameLocalGridIndexIsInAssignedHalf(int local_grid_x,
                                                 int local_grid_y);

const char *missionFrameStartName(void);

#endif
