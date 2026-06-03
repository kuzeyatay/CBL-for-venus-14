#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdbool.h>
#include "constants/robot_types.h"

bool initESP32UART(void);
bool sendPayloadToESP32(const char *payload);
bool receivePayloadFromESP32(char *buffer, int buffer_size);
bool pollESP32Messages(void);
void handleBaseStationMessage(const char *message);

void sendPoseUpdate(void);
void sendFieldEventUpdate(field_event_t event);
void sendCellUpdate(int local_grid_x, int local_grid_y, cell_status_t status);
void sendScanRayUpdate(float yaw, int distance_mm);
void sendStatusUpdate(const char *state, const char *error_code);
void sendErrorMessage(const char *error_code);

#endif
