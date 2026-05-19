#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "robot_types.h"
#include "odometry.h"
#include "navigation.h"
#include "communication.h"

/*
 * ============================================================
 * COMMUNICATION MODULE
 * ============================================================
 *
 * This file is responsible for communication between:
 *
 *     PYNQ board  <->  ESP32 connectivity board  <->  Base station PC
 *
 * Important architecture:
 *
 * - The base station PC communicates using MQTT.
 * - The ESP32 handles the Wi-Fi/MQTT side.
 * - The PYNQ does not directly handle MQTT.
 * - The PYNQ only communicates with the ESP32 through UART.
 *
 * Therefore, from the PYNQ point of view:
 *
 *     outgoing robot data:
 *         PYNQ -> UART -> ESP32 -> MQTT -> base station
 *
 *     incoming base-station commands:
 *         base station -> MQTT -> ESP32 -> UART -> PYNQ
 *
 * This module hides that communication path from the rest of the robot code.
 * Other modules should only call functions such as:
 *
 *     sendPoseUpdate();
 *     sendFieldEventUpdate(event);
 *     sendStatusUpdate("ready", "none");
 *
 * They should not need to know the UART details.
 */


/*
 * Initializes the UART connection between the PYNQ and the ESP32.
 *
 * Current status:
 * - Not implemented yet.
 * - Currently returns false to show that UART is not ready.
 *
 * What this function should eventually do:
 * - Select the correct UART peripheral.
 * - Initialize UART using the libpynq UART functions.
 * - Clear/reset UART FIFOs before communication starts.
 * - Return true if initialization succeeds.
 * - Return false if initialization fails.
 *
 * Why this function exists:
 * - main.c should be able to initialize communication with one simple call.
 * - The rest of the program should not need to know low-level UART setup.
 */
bool initESP32UART(void) {
    /*
     * TODO:
     * Initialize UART between PYNQ and ESP32.
     */
    return false;
}


/*
 * Sends one text payload from the PYNQ to the ESP32.
 *
 * Current status:
 * - Not implemented as real UART yet.
 * - Currently only prints the payload to the terminal.
 *
 * Expected final UART frame format:
 *
 *     [4 bytes payload_length][payload bytes]
 *
 * Example payload:
 *
 *     "POSE,R1,40.00,20.00,90.00,12,11"
 *
 * Final behavior should be:
 * - Check that payload is not NULL.
 * - Compute payload length with strlen(payload).
 * - Send the length as 4 bytes.
 * - Send the actual payload characters.
 * - Return true if sending succeeded.
 * - Return false if sending failed.
 *
 * Why the payload length is needed:
 * - UART is a byte stream.
 * - It does not automatically know where one message ends.
 * - The length tells the receiver how many bytes belong to this payload.
 */
bool sendPayloadToESP32(const char *payload) {
    /*
     * TODO:
     * Send 4-byte payload length followed by payload bytes.
     *
     * Temporary behavior:
     * print the payload.
     */
    if (payload == NULL) {
        return false;
    }

    printf("UART OUT: %s\n", payload);
    return true;
}


/*
 * Receives one text payload from the ESP32 over UART.
 *
 * Current status:
 * - Not implemented yet.
 * - Currently always returns false.
 *
 * Expected final behavior:
 * - Check whether UART data is available.
 * - Read the first 4 bytes as payload length.
 * - Check whether the payload length fits inside buffer.
 * - Read exactly that many payload bytes.
 * - Add '\0' to make the received payload a valid C string.
 * - Return true if a full payload was received.
 * - Return false if no complete payload is available.
 *
 * Arguments:
 *
 * buffer:
 *     Destination array where the received text payload will be stored.
 *
 * buffer_size:
 *     Total size of the destination array.
 *     This must include space for the final '\0'.
 *
 * Example:
 *
 *     char payload[UART_PAYLOAD_MAX_SIZE];
 *     receivePayloadFromESP32(payload, sizeof(payload));
 *
 * Why this function does not directly handle the command:
 * - This function should only receive bytes and reconstruct a string.
 * - Command interpretation belongs to handleBaseStationMessage().
 */
bool receivePayloadFromESP32(char *buffer, int buffer_size) {
    /*
     * TODO:
     * Read UART frame from ESP32.
     */
    (void)buffer;
    (void)buffer_size;

    return false;
}


/*
 * Checks whether the ESP32 has forwarded a base-station command to the PYNQ.
 *
 * Remember the full path:
 *
 *     Base station PC
 *          -> MQTT
 *     ESP32
 *          -> UART
 *     PYNQ
 *
 * So although the base station sends the command using MQTT, this PYNQ-side
 * function reads the command from UART because the ESP32 acts as the bridge.
 *
 * Current behavior:
 * - Calls receivePayloadFromESP32().
 * - If no payload is available, returns false.
 * - If a payload is received, passes it to handleBaseStationMessage().
 *
 * Example incoming payloads:
 *
 *     "START_MISSION"
 *     "STOP_MISSION"
 *     "PING"
 *     "CAL_READ,red"
 *     "TCSCAL,..."
 *
 * Return:
 * - true  = a command was received and handled.
 * - false = no command was available.
 *
 * Where this should be called:
 * - In the main loop while waiting for START_MISSION.
 * - Inside the navigation loop so STOP_MISSION can be received.
 * - During calibration mode.
 */
bool pollESP32Messages(void) {
    char payload[UART_PAYLOAD_MAX_SIZE];

    if (!receivePayloadFromESP32(payload, sizeof(payload))) {
        return false;
    }

    handleBaseStationMessage(payload);
    return true;
}


/*
 * Handles one complete text command received from the base station.
 *
 * This function receives a normal C string, not raw UART bytes.
 * The UART byte handling happens earlier in receivePayloadFromESP32().
 *
 * Supported commands in this current version:
 *
 *     START_MISSION
 *     STOP_MISSION
 *     PING
 *
 * Planned future commands:
 *
 *     CAL_READ,<color>
 *     TCSCAL,<24 calibration values>
 *     SET_SPEED,<speed_cm_s>
 *     SET_TARGET_CELL,<grid_x>,<grid_y>
 *
 * Design rule:
 * - Unknown commands must not make the robot move.
 * - Unknown commands are reported as errors.
 */
void handleBaseStationMessage(const char *message) {
    if (message == NULL) {
        return;
    }

    /*
     * Command: START_MISSION
     *
     * Meaning:
     * - The base station tells the robot to begin autonomous exploration.
     *
     * Expected behavior:
     * - startMission() should set the mission-running flag.
     * - Navigation is allowed to run after this command.
     */
    if (strcmp(message, "START_MISSION") == 0) {
        startMission();
        return;
    }

    /*
     * Command: STOP_MISSION
     *
     * Meaning:
     * - The base station tells the robot to stop autonomous exploration.
     *
     * Expected behavior:
     * - stopMission() should clear the mission-running flag.
     * - The robot should stop sending new movement commands.
     */
    if (strcmp(message, "STOP_MISSION") == 0) {
        stopMission();
        return;
    }

    /*
     * Command: PING
     *
     * Meaning:
     * - The base station checks whether the robot is alive.
     *
     * Response:
     * - The robot sends "PONG,<robot_id>".
     *
     * Example:
     * - Incoming: "PING"
     * - Outgoing: "PONG,R1"
     */
    if (strcmp(message, "PING") == 0) {
        sendPayloadToESP32("PONG," ROBOT_ID);
        return;
    }

    /*
     * TODO:
     * Handle CAL_READ and TCSCAL here, or forward to color_sensor.c.
     *
     * CAL_READ example:
     *     "CAL_READ,red"
     *
     * Meaning:
     *     The base station asks the robot to measure raw TCS3200 values
     *     for the red calibration block.
     *
     * TCSCAL example:
     *     "TCSCAL,none_R,none_G,none_B,none_C,..."
     *
     * Meaning:
     *     The base station sends the full TCS3200 calibration profile.
     *
     * Suggested final behavior:
     * - If message starts with "CAL_READ,", call:
     *       tcs3200SendCalibrationReading(color_name);
     *
     * - If message starts with "TCSCAL,", call:
     *       tcs3200LoadCalibrationFromBaseStation(message);
     */

    /*
     * If the command reaches this point, it was not recognized.
     * The robot should not move or change state in response to an unknown
     * command.
     */
    printf("Unknown base station message: %s\n", message);
    sendErrorMessage("unknown_command");
}


/*
 * Sends the robot's current estimated pose to the base station.
 *
 * Payload structure:
 *
 *     POSE,<robot_id>,<x_cm>,<y_cm>,<yaw_deg>,<grid_x>,<grid_y>
 *
 * Example:
 *
 *     POSE,R1,40.00,20.00,90.00,12,11
 *
 * Current status:
 * - x, y, and yaw are taken from odometry.
 * - grid_x and grid_y are currently placeholders set to 0.
 *
 * Future improvement:
 * - navigation.c should expose a function that converts the current pose
 *   into the actual grid cell.
 * - Then this function should use that grid cell instead of 0,0.
 *
 * Why pose updates are needed:
 * - The base station GUI needs them to draw the robot position.
 * - The second robot can use them for collision avoidance/coordination.
 */
void sendPoseUpdate(void) {
    pose_t pose = getPose();

    int grid_x = 0;
    int grid_y = 0;

    /*
     * TODO:
     * Get actual grid cell from navigation/map module.
     */

    char payload[UART_PAYLOAD_MAX_SIZE];

    snprintf(payload, sizeof(payload),
             "POSE,%s,%.2f,%.2f,%.2f,%d,%d",
             ROBOT_ID,
             pose.x,
             pose.y,
             pose.yaw,
             grid_x,
             grid_y);

    sendPayloadToESP32(payload);
}


/*
 * Sends one detected field event to the base station.
 *
 * Payload structure should eventually be:
 *
 *     EVENT,
 *     <robot_id>,
 *     <event_type>,
 *     <x_cm>,
 *     <y_cm>,
 *     <yaw_deg>,
 *     <grid_x>,
 *     <grid_y>,
 *     <distance_mm>,
 *     <width_cm>,
 *     <color>,
 *     <sample_size_cm>,
 *     <temperature_c>
 *
 * Example:
 *
 *     EVENT,R1,rock_sample,60.00,20.00,90.00,3,1,120,3.20,red,3,24.50
 *
 * Current status:
 * - Not implemented yet.
 * - The event argument is currently unused to avoid compiler warnings.
 *
 * Why this function is needed:
 * - The base station needs event messages to update the map.
 * - Events include black tape, hills, rock samples, and sensor faults.
 */
void sendFieldEventUpdate(field_event_t event) {
    /*
     * TODO:
     * Build EVENT payload.
     */
    (void)event;
}


/*
 * Sends one grid-cell update to the base station.
 *
 * Payload structure should eventually be:
 *
 *     CELL_UPDATE,<robot_id>,<grid_x>,<grid_y>,<cell_status>
 *
 * Examples:
 *
 *     CELL_UPDATE,R1,12,11,explored
 *     CELL_UPDATE,R1,13,11,hill
 *     CELL_UPDATE,R1,14,11,sample
 *     CELL_UPDATE,R1,15,11,unsafe
 *
 * Current status:
 * - Not implemented yet.
 * - Arguments are currently unused to avoid compiler warnings.
 *
 * Why this function is needed:
 * - The robot has a local grid map.
 * - The base station GUI also needs to know which cells are explored,
 *   unsafe, hills, or samples.
 */
void sendCellUpdate(int grid_x, int grid_y, cell_status_t status) {
    /*
     * TODO:
     * Build CELL_UPDATE payload.
     */
    (void)grid_x;
    (void)grid_y;
    (void)status;
}


/*
 * Sends the robot's current status to the base station.
 *
 * Payload structure:
 *
 *     STATUS,<robot_id>,<state>,<battery_or_dummy>,<calibrated>,<error_code>
 *
 * Current payload:
 *
 *     STATUS,<robot_id>,<state>,-1,0,<error_code>
 *
 * Meaning of fields:
 *
 * robot_id:
 *     The robot identifier, for example R1 or R2.
 *
 * state:
 *     Robot state, for example:
 *     - "idle"
 *     - "ready"
 *     - "navigating"
 *     - "avoiding"
 *     - "stopped"
 *     - "fault"
 *
 * battery_or_dummy:
 *     Battery level if available.
 *     Currently -1 because battery measurement is not implemented.
 *
 * calibrated:
 *     1 if sensor calibration is loaded.
 *     0 otherwise.
 *     Currently hardcoded as 0.
 *
 * error_code:
 *     "none" if there is no error, otherwise an error label.
 *
 * Example:
 *
 *     STATUS,R1,navigating,-1,0,none
 */
void sendStatusUpdate(const char *state, const char *error_code) {
    char payload[UART_PAYLOAD_MAX_SIZE];

    snprintf(payload, sizeof(payload),
             "STATUS,%s,%s,-1,0,%s",
             ROBOT_ID,
             state,
             error_code);

    sendPayloadToESP32(payload);
}


/*
 * Sends an error message to the base station.
 *
 * Payload structure:
 *
 *     ERROR,<robot_id>,<error_code>
 *
 * Examples:
 *
 *     ERROR,R1,not_calibrated
 *     ERROR,R1,sensor_fault
 *     ERROR,R1,uart_error
 *     ERROR,R1,unknown_command
 *
 * This is used when something specific goes wrong and the base station should
 * display/log it.
 */
void sendErrorMessage(const char *error_code) {
    char payload[UART_PAYLOAD_MAX_SIZE];

    snprintf(payload, sizeof(payload),
             "ERROR,%s,%s",
             ROBOT_ID,
             error_code);

    sendPayloadToESP32(payload);
}
