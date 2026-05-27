#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdbool.h>
#include "constants/robot_types.h"

void runNavigation(void);
void startMission(void);
void stopMission(void);
bool isMissionRunning(void);

#endif
