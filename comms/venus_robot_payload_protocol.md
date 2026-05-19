# Venus Robot Payload Protocol Specification

**Project:** 5EID0 CBL Engineering Challenge for Venus  
**Purpose:** Shared payload protocol between the base station PC, ESP32 connectivity board, and PYNQ robot controller  
**Version:** Draft 1.0  

This document defines the text payloads used by the Venus exploration robot system. It is meant to be used by all software parts of the project:

- the **PYNQ embedded robot software**,
- the **ESP32 bridge software**,
- the **base station PC software**,
- the **GUI/mapping software**,
- and the **calibration/testing tools**.

The protocol is intentionally text-based and comma-separated so that it is easy to print, debug, parse, and log during development.

---

## 1. System Communication Overview

The robot system contains three communication levels:

1. **PYNQ internal data structures**  
   The robot software uses C structs such as `sensor_data_t`, `field_event_t`, pose variables, map cells, and motor commands.

2. **PYNQ ↔ ESP32 UART frame**  
   The PYNQ and ESP32 exchange byte streams over UART. Each UART message contains a length field and a text payload.

3. **ESP32 ↔ Base Station MQTT message**  
   The ESP32 forwards payloads between UART and MQTT. The base station receives these payloads, parses them, and updates the GUI/map.

The **payloads defined in this document are the text part** of the communication. They are independent of whether the payload is carried through UART or MQTT.

---

## 2. UART Frame Format

The PYNQ and ESP32 should use the following UART frame structure:

```text
[4 bytes payload_length][payload bytes]
```

### 2.1 Length Field

| Field | Type | Size | Meaning |
|---|---:|---:|---|
| `payload_length` | unsigned 32-bit integer | 4 bytes | Number of bytes in the payload |

Recommended interpretation:

```text
little-endian unsigned integer
```

Example: if the payload has length `12`, the length bytes are:

```text
0x0C 0x00 0x00 0x00
```

### 2.2 Payload Field

| Field | Type | Size | Meaning |
|---|---:|---:|---|
| `payload` | character array | `payload_length` bytes | Comma-separated text message |

The payload itself is **not required to include a null terminator**. After the PYNQ receives the bytes, it should add `\0` locally before parsing the payload as a C string.

Example UART payload:

```text
START_MISSION
```

Example UART frame:

```text
0D 00 00 00 53 54 41 52 54 5F 4D 49 53 53 49 4F 4E
```

The first four bytes say the payload has length 13. The remaining bytes are the ASCII characters of `START_MISSION`.

---

## 3. General Payload Format

All payloads use comma-separated text fields.

```text
MESSAGE_TYPE,arg1,arg2,arg3,...
```

The first field is always the **message type**. The remaining fields depend on that message type.

Examples:

```text
PING
CAL_READ,red
POSE,R1,40.00,20.00,90.00,12,11
EVENT,R1,rock_sample,60.00,20.00,90.00,120,3.2,red,3,24.5
```

### 3.1 Formatting Rules

- Do not include spaces inside payloads unless a parser explicitly supports them.
- Use a comma `,` as the field separator.
- Use a dot `.` as the decimal separator.
- Use fixed field order for every payload type.
- Unknown commands should be ignored safely or answered with an `ERROR` payload.
- The robot should not begin autonomous movement from an unknown or malformed command.

---

## 4. Common Data Types and Units

| Name | Example | Type | Unit / Allowed values |
|---|---:|---|---|
| `robot_id` | `R1` | string | Robot identifier, for example `R1`, `R2` |
| `x_cm` | `40.00` | float | centimeters |
| `y_cm` | `20.00` | float | centimeters |
| `yaw_deg` | `90.00` | float | degrees, normally 0 to 360 |
| `grid_x` | `12` | integer | map-grid x index |
| `grid_y` | `11` | integer | map-grid y index |
| `distance_mm` | `120` | integer | millimeters |
| `width_cm` | `3.2` | float | centimeters |
| `temperature_c` | `24.5` | float | degrees Celsius |
| `speed_cm_s` | `15.0` | float | centimeters per second |
| `step_count` | `1600` | integer | motor steps |
| `angle_deg` | `90.0` | float | degrees |
| `valid` | `1` | integer/bool | `1` true, `0` false |
| `calibrated` | `1` | integer/bool | `1` calibrated, `0` not calibrated |

---

## 5. Shared Enumerations

### 5.1 Color Names

These values are used for sample colors and TCS3200 classifications.

| Value | Meaning |
|---|---|
| `none` | calibrated background / no object |
| `white` | white sample |
| `black` | black sample or black surface |
| `red` | red sample |
| `green` | green sample |
| `blue` | blue sample |
| `unknown` | color could not be classified |

### 5.2 Field Event Types

| Value | Meaning |
|---|---|
| `clear` | no hazard or object detected |
| `black_tape` | black tape representing cliff or field boundary |
| `hill` | hill or large obstacle detected |
| `rock_sample` | valid sample detected and classified |
| `sensor_fault` | invalid or inconsistent sensor data |
| `robot_nearby` | another robot is close enough to require collision avoidance |
| `communication_fault` | communication problem detected |

### 5.3 Cell Status Values

| Value | Meaning |
|---|---|
| `unknown` | cell has not been explored |
| `explored` | cell has been visited or observed as safe |
| `unsafe` | cell contains cliff/boundary/black tape or should be avoided |
| `hill` | cell contains a hill or obstacle |
| `sample` | cell contains a rock sample |
| `reserved` | cell is reserved for another robot |

### 5.4 Robot State Values

| Value | Meaning |
|---|---|
| `booting` | robot software is starting |
| `idle` | robot is waiting for commands |
| `calibrating` | robot is performing calibration readings |
| `ready` | robot is ready to start mission |
| `navigating` | robot is autonomously exploring |
| `scanning` | robot is scanning object width or environment |
| `classifying` | robot is classifying an object/sample |
| `avoiding` | robot is avoiding tape, hill, sample, or robot |
| `stopped` | robot is stopped by command |
| `fault` | robot is in a fault state |

### 5.5 Error Codes

| Value | Meaning |
|---|---|
| `none` | no error |
| `not_calibrated` | required calibration has not been loaded |
| `bad_payload` | payload could not be parsed |
| `bad_calibration` | calibration message is invalid |
| `sensor_fault` | sensor reading failed or was invalid |
| `tcs_no_signal` | TCS3200 OUT pin did not toggle or all readings are zero |
| `vl53_timeout` | VL53L0X distance reading timed out |
| `ntc_invalid` | NTCC-10K reading was outside expected range |
| `uart_error` | UART receive/send error |
| `mqtt_error` | MQTT publish/subscribe error |
| `motor_error` | stepper motor command failed or was unsafe |
| `pose_out_of_map` | pose could not be converted to a valid map cell |
| `unknown_command` | command type was not recognized |

---

## 6. Sensor Data Model

This section defines every sensor-related data item that may appear in payloads.

---

### 6.1 TCS3200 Color Sensor Data

The TCS3200 is used for:

- sample color classification,
- black tape detection,
- distinguishing black samples from black tape using context.

#### 6.1.1 Raw TCS3200 Reading

| Field | Type | Unit | Meaning |
|---|---:|---:|---|
| `red_hz` | float | Hz | frequency measured with red filter selected |
| `green_hz` | float | Hz | frequency measured with green filter selected |
| `blue_hz` | float | Hz | frequency measured with blue filter selected |
| `clear_hz` | float | Hz | frequency measured with clear/no filter selected |

Payload field order:

```text
red_hz,green_hz,blue_hz,clear_hz
```

Example:

```text
1200.00,300.00,250.00,900.00
```

#### 6.1.2 TCS3200 Feature Values

These are normally computed locally and do not always need to be sent.

| Field | Type | Meaning |
|---|---:|---|
| `red_ratio` | float | `red_hz / (red_hz + green_hz + blue_hz)` |
| `green_ratio` | float | `green_hz / (red_hz + green_hz + blue_hz)` |
| `blue_ratio` | float | `blue_hz / (red_hz + green_hz + blue_hz)` |
| `brightness` | float | `clear_hz / white_clear_reference` |

Payload field order:

```text
red_ratio,green_ratio,blue_ratio,brightness
```

Example:

```text
0.686,0.171,0.143,0.818
```

#### 6.1.3 Calibrated TCS3200 Color Values

Allowed calibrated colors:

```text
none,white,black,red,green,blue,unknown
```

Important context rule:

```text
black + no close front object  -> black tape / cliff / boundary
black + close front object     -> possible black rock sample
```

The TCS3200 alone should not decide whether a black reading is tape or sample. The navigation layer must combine color data with distance/object context.

---

### 6.2 VL53L0X Distance Sensor Data

The VL53L0X is used for:

- detecting front objects,
- detecting hills/obstacles,
- helping classify object size,
- scanning object width by rotating left/right while measuring distance.

| Field | Type | Unit | Meaning |
|---|---:|---:|---|
| `front_distance_mm` | integer | mm | distance to object in front |
| `front_object_detected` | bool/int | none | `1` if object is within threshold, otherwise `0` |
| `distance_valid` | bool/int | none | `1` if distance reading is valid, otherwise `0` |
| `distance_status` | string | none | optional status such as `ok`, `timeout`, `out_of_range` |

Example:

```text
VL53_READING,R1,120,1,1,ok
```

---

### 6.3 Object Width / Size Estimation Data

Object width can be estimated by rotating slightly left and right while checking when the VL53L0X still sees the object.

| Field | Type | Unit | Meaning |
|---|---:|---:|---|
| `width_cm` | float | cm | estimated apparent object width |
| `left_angle_deg` | float | degrees | left edge angle of detected object |
| `right_angle_deg` | float | degrees | right edge angle of detected object |
| `size_cm` | integer | cm | classified sample side length, normally `3`, `6`, or `0` |

Size rules:

```text
size_cm = 3 -> small rock sample, 3x3x3 cm
size_cm = 6 -> large rock sample, 6x6x6 cm
size_cm = 0 -> not a valid sample size or not applicable
```

Example:

```text
WIDTH_SCAN,R1,3.2,-4.5,4.5,3
```

---

### 6.4 NTCC-10K Temperature Sensor Data

The NTCC-10K thermistor is used to measure the temperature near detected rock samples.

| Field | Type | Unit | Meaning |
|---|---:|---:|---|
| `adc_raw` | integer | ADC units | raw analog reading from PYNQ analog input |
| `voltage_v` | float | volts | converted voltage |
| `resistance_ohm` | float | ohms | calculated thermistor resistance |
| `temperature_c` | float | Celsius | calculated temperature |
| `temperature_valid` | bool/int | none | `1` if reading is valid, otherwise `0` |

Example:

```text
NTC_READING,R1,2134,1.72,9870.0,24.5,1
```

---

### 6.5 Combined Sensor Data Payload

The combined sensor data payload can be used for debugging or logging.

Structure:

```text
SENSOR_DATA,<robot_id>,<valid>,<black_detected>,<front_object_detected>,<front_distance_mm>,<width_cm>,<color>,<temperature_c>
```

Fields:

| Field | Type | Meaning |
|---|---:|---|
| `valid` | bool/int | whether the sensor package is valid |
| `black_detected` | bool/int | whether TCS3200 classified the current surface/object as black |
| `front_object_detected` | bool/int | whether VL53L0X sees a close object |
| `front_distance_mm` | integer | front distance in millimeters |
| `width_cm` | float | estimated object width |
| `color` | string | classified sample color |
| `temperature_c` | float | temperature reading |

Example:

```text
SENSOR_DATA,R1,1,0,1,120,3.2,red,24.5
```

---

## 7. Base Station to Robot Commands

These messages are sent from the base station PC to one robot through MQTT and then through the ESP32 UART bridge.

---

### 7.1 `PING`

Checks whether the robot is alive.

Structure:

```text
PING
```

Expected response:

```text
PONG,<robot_id>
```

Example:

```text
PING
PONG,R1
```

---

### 7.2 `CAL_READ`

Asks the robot to take one raw TCS3200 calibration reading.

Structure:

```text
CAL_READ,<color_name>
```

Allowed `color_name` values:

```text
none,white,black,red,green,blue
```

Meaning:

The base station tells the user/team to place the specified sample/background in front of the sensor. Then it sends this command. The robot measures all four TCS3200 channels and replies with `CALRAW`.

Example:

```text
CAL_READ,red
```

Expected response:

```text
CALRAW,red,<red_hz>,<green_hz>,<blue_hz>,<clear_hz>
```

Example response:

```text
CALRAW,red,1200.00,300.00,250.00,900.00
```

---

### 7.3 `TCSCAL`

Sends the complete TCS3200 calibration profile from the base station to the robot.

Structure:

```text
TCSCAL,<none_R>,<none_G>,<none_B>,<none_C>,<white_R>,<white_G>,<white_B>,<white_C>,<black_R>,<black_G>,<black_B>,<black_C>,<red_R>,<red_G>,<red_B>,<red_C>,<green_R>,<green_G>,<green_B>,<green_C>,<blue_R>,<blue_G>,<blue_B>,<blue_C>
```

There are 24 numeric values total:

| Calibration profile | Fields |
|---|---|
| `none` | `none_R, none_G, none_B, none_C` |
| `white` | `white_R, white_G, white_B, white_C` |
| `black` | `black_R, black_G, black_B, black_C` |
| `red` | `red_R, red_G, red_B, red_C` |
| `green` | `green_R, green_G, green_B, green_C` |
| `blue` | `blue_R, blue_G, blue_B, blue_C` |

Example:

```text
TCSCAL,120,130,125,180,900,870,850,1100,80,75,70,100,1200,300,250,900,300,1100,350,850,250,400,1200,870
```

Expected responses:

```text
TCS_CAL_OK
```

or:

```text
TCS_CAL_FAIL
```

---

### 7.4 `START_MISSION`

Starts autonomous navigation.

Structure:

```text
START_MISSION
```

Expected behavior:

- Check that required initialization has completed.
- Check that calibration has been loaded if real sensors are used.
- Start the autonomous navigation loop.

Expected response:

```text
MISSION_STARTED,<robot_id>
```

Possible error:

```text
ERROR,<robot_id>,not_calibrated
```

---

### 7.5 `STOP_MISSION`

Stops autonomous navigation.

Structure:

```text
STOP_MISSION
```

Expected behavior:

- Stop sending new movement commands.
- Stop motors or prevent new motor commands.
- Keep the robot in a safe state.

Expected response:

```text
MISSION_STOPPED,<robot_id>
```

---

### 7.6 `PAUSE_MISSION`

Temporarily pauses autonomous navigation.

Structure:

```text
PAUSE_MISSION
```

Expected behavior:

- Stop the navigation loop from advancing.
- Keep communication active.
- Preserve pose and map state.

Expected response:

```text
MISSION_PAUSED,<robot_id>
```

---

### 7.7 `RESUME_MISSION`

Resumes the mission after a pause.

Structure:

```text
RESUME_MISSION
```

Expected response:

```text
MISSION_RESUMED,<robot_id>
```

---

### 7.8 `SET_SPEED`

Sets the default navigation speed.

Structure:

```text
SET_SPEED,<speed_cm_s>
```

Example:

```text
SET_SPEED,12.5
```

Expected behavior:

- Parse speed in centimeters per second.
- Clamp it to the safe speed range.
- Store it as the default movement speed.

Expected response:

```text
SPEED_SET,<robot_id>,<speed_cm_s>
```

Example:

```text
SPEED_SET,R1,12.5
```

---

### 7.9 `MOVE_FORWARD`

Manual/debug command to move forward by a fixed distance. This should normally not be used during the autonomous mission.

Structure:

```text
MOVE_FORWARD,<distance_cm>,<speed_cm_s>
```

Example:

```text
MOVE_FORWARD,20.0,10.0
```

Expected response:

```text
MOTOR_DONE,<robot_id>,move_forward,<distance_cm>
```

Safety note:

This command is useful during testing but should not violate the autonomous-operation rule during official mission runs.

---

### 7.10 `MOVE_REVERSE`

Manual/debug command to reverse by a fixed distance.

Structure:

```text
MOVE_REVERSE,<distance_cm>,<speed_cm_s>
```

Example:

```text
MOVE_REVERSE,8.0,10.0
```

Expected response:

```text
MOTOR_DONE,<robot_id>,move_reverse,<distance_cm>
```

---

### 7.11 `TURN`

Manual/debug command to turn by a fixed angle.

Structure:

```text
TURN,<angle_deg>,<speed_cm_s>
```

Example:

```text
TURN,90.0,10.0
```

Expected response:

```text
MOTOR_DONE,<robot_id>,turn,<angle_deg>
```

---

### 7.12 `SET_TARGET_CELL`

Assigns a navigation target cell to a robot.

Structure:

```text
SET_TARGET_CELL,<grid_x>,<grid_y>
```

Example:

```text
SET_TARGET_CELL,12,8
```

Expected behavior:

- Store the target cell.
- Navigate toward that cell when autonomous navigation is running.

Expected response:

```text
TARGET_ACCEPTED,<robot_id>,<grid_x>,<grid_y>
```

---

### 7.13 `CELL_RESERVED`

Tells a robot that another robot has reserved a cell.

Structure:

```text
CELL_RESERVED,<other_robot_id>,<grid_x>,<grid_y>
```

Example:

```text
CELL_RESERVED,R2,11,10
```

Expected behavior:

- Mark the cell as reserved.
- Avoid selecting it as a target.
- Reduce duplicate exploration and collision risk.

---

### 7.14 `OTHER_ROBOT_POSE`

Forwards another robot's pose to this robot.

Structure:

```text
OTHER_ROBOT_POSE,<other_robot_id>,<x_cm>,<y_cm>,<yaw_deg>,<grid_x>,<grid_y>
```

Example:

```text
OTHER_ROBOT_POSE,R2,40.00,20.00,90.00,12,11
```

Expected behavior:

- Estimate whether the other robot is nearby.
- Avoid moving into the same or neighboring cell.
- Stop/wait/turn if collision risk is high.

---

### 7.15 `RESET_MAP`

Clears the robot's local map.

Structure:

```text
RESET_MAP
```

Expected response:

```text
MAP_RESET,<robot_id>
```

---

### 7.16 `RESET_POSE`

Resets the robot's relative pose estimate.

Structure:

```text
RESET_POSE,<x_cm>,<y_cm>,<yaw_deg>
```

Example:

```text
RESET_POSE,0.00,0.00,0.00
```

Expected response:

```text
POSE_RESET,<robot_id>,<x_cm>,<y_cm>,<yaw_deg>
```

---

### 7.17 `REQUEST_POSE`

Asks the robot to report its current pose.

Structure:

```text
REQUEST_POSE
```

Expected response:

```text
POSE,<robot_id>,<x_cm>,<y_cm>,<yaw_deg>,<grid_x>,<grid_y>
```

---

### 7.18 `REQUEST_STATUS`

Asks the robot to report its system status.

Structure:

```text
REQUEST_STATUS
```

Expected response:

```text
STATUS,<robot_id>,<state>,<battery_or_dummy>,<calibrated>,<error_code>
```

---

### 7.19 `REQUEST_SENSOR_DATA`

Asks the robot to report its latest sensor package.

Structure:

```text
REQUEST_SENSOR_DATA
```

Expected response:

```text
SENSOR_DATA,<robot_id>,<valid>,<black_detected>,<front_object_detected>,<front_distance_mm>,<width_cm>,<color>,<temperature_c>
```

---

### 7.20 `REQUEST_TCS_RAW`

Asks the robot to report a live raw TCS3200 reading.

Structure:

```text
REQUEST_TCS_RAW
```

Expected response:

```text
TCS_RAW,<robot_id>,<red_hz>,<green_hz>,<blue_hz>,<clear_hz>,<classified_color>
```

Example:

```text
TCS_RAW,R1,1200.00,300.00,250.00,900.00,red
```

---

### 7.21 `REQUEST_VL53`

Asks the robot to report a live distance reading.

Structure:

```text
REQUEST_VL53
```

Expected response:

```text
VL53_READING,<robot_id>,<front_distance_mm>,<front_object_detected>,<distance_valid>,<distance_status>
```

---

### 7.22 `REQUEST_NTC`

Asks the robot to report a live temperature reading.

Structure:

```text
REQUEST_NTC
```

Expected response:

```text
NTC_READING,<robot_id>,<adc_raw>,<voltage_v>,<resistance_ohm>,<temperature_c>,<temperature_valid>
```

---

## 8. Robot to Base Station Payloads

These messages are sent by the robot to the base station through the ESP32 bridge.

---

### 8.1 `READY`

Robot announces that it has booted and can receive commands.

Structure:

```text
READY,<robot_id>
```

Example:

```text
READY,R1
```

---

### 8.2 `PONG`

Response to `PING`.

Structure:

```text
PONG,<robot_id>
```

Example:

```text
PONG,R1
```

---

### 8.3 `CALRAW`

Raw TCS3200 calibration reading.

Structure:

```text
CALRAW,<color_name>,<red_hz>,<green_hz>,<blue_hz>,<clear_hz>
```

Example:

```text
CALRAW,red,1200.00,300.00,250.00,900.00
```

---

### 8.4 `TCS_CAL_OK`

Robot successfully loaded the TCS3200 calibration profile.

Structure:

```text
TCS_CAL_OK
```

---

### 8.5 `TCS_CAL_FAIL`

Robot failed to load the TCS3200 calibration profile.

Structure:

```text
TCS_CAL_FAIL
```

Possible reasons:

- wrong number of calibration values,
- parse error,
- profile with no signal,
- corrupted payload.

---

### 8.6 `MISSION_STARTED`

Robot has started autonomous navigation.

Structure:

```text
MISSION_STARTED,<robot_id>
```

Example:

```text
MISSION_STARTED,R1
```

---

### 8.7 `MISSION_STOPPED`

Robot has stopped autonomous navigation.

Structure:

```text
MISSION_STOPPED,<robot_id>
```

Example:

```text
MISSION_STOPPED,R1
```

---

### 8.8 `MISSION_PAUSED`

Robot has paused autonomous navigation.

Structure:

```text
MISSION_PAUSED,<robot_id>
```

---

### 8.9 `MISSION_RESUMED`

Robot has resumed autonomous navigation.

Structure:

```text
MISSION_RESUMED,<robot_id>
```

---

### 8.10 `POSE`

Reports the robot's current estimated pose.

Structure:

```text
POSE,<robot_id>,<x_cm>,<y_cm>,<yaw_deg>,<grid_x>,<grid_y>
```

Example:

```text
POSE,R1,40.00,20.00,90.00,12,11
```

Notes:

- `x_cm`, `y_cm`, and `yaw_deg` are relative dead-reckoning estimates.
- `grid_x` and `grid_y` are the corresponding map cell indices.
- The base station should not treat this as perfect absolute localization.

---

### 8.11 `EVENT`

Reports a detected field event.

Structure:

```text
EVENT,<robot_id>,<event_type>,<x_cm>,<y_cm>,<grid_x>,<grid_y>,<yaw_deg>,<distance_mm>,<width_cm>,<color>,<sample_size_cm>,<temperature_c>
```

Fields:

| Field | Meaning |
|---|---|
| `robot_id` | robot that detected the event |
| `event_type` | `clear`, `black_tape`, `hill`, `rock_sample`, `sensor_fault`, etc. |
| `x_cm`, `y_cm`, `yaw_deg` | robot pose when event was detected |
| `grid_x`, `grid_y` | corresponding map cell indices |
| `distance_mm` | measured front distance, if available |
| `width_cm` | estimated object width, if available |
| `color` | sample color or `unknown` |
| `sample_size_cm` | `3`, `6`, or `0` if not applicable |
| `temperature_c` | measured temperature or `0.0` if not applicable |

Example for black tape:

```text
EVENT,R1,black_tape,30.00,0.00,0.00,2,0,500,0.0,unknown,0,0.0
```

Example for hill:

```text
EVENT,R1,hill,40.00,20.00,90.00,2,1,100,30.0,unknown,0,0.0
```

Example for rock sample:

```text
EVENT,R1,rock_sample,60.00,20.00,90.00,3,1,120,3.2,red,3,24.5
```

Example for sensor fault:

```text
EVENT,R1,sensor_fault,60.00,20.00,90.00,3,1,0,0.0,unknown,0,0.0
```

---

### 8.12 `CELL_UPDATE`

Reports a local map-cell update.

Structure:

```text
CELL_UPDATE,<robot_id>,<grid_x>,<grid_y>,<cell_status>
```

Example:

```text
CELL_UPDATE,R1,12,11,explored
```

Possible `cell_status` values:

```text
unknown,explored,unsafe,hill,sample,reserved
```

---

### 8.13 `STATUS`

Reports the robot's current state.

Structure:

```text
STATUS,<robot_id>,<state>,<battery_or_dummy>,<calibrated>,<error_code>
```

Fields:

| Field | Meaning |
|---|---|
| `state` | current robot state |
| `battery_or_dummy` | battery level if available, otherwise `-1` |
| `calibrated` | `1` if calibration loaded, `0` otherwise |
| `error_code` | error code, or `none` |

Example:

```text
STATUS,R1,navigating,-1,1,none
```

---

### 8.14 `ERROR`

Reports an error.

Structure:

```text
ERROR,<robot_id>,<error_code>
```

Example:

```text
ERROR,R1,not_calibrated
```

Optional extended structure:

```text
ERROR,<robot_id>,<error_code>,<details>
```

Example:

```text
ERROR,R1,bad_payload,TCSCAL expected 24 values
```

---

### 8.15 `SENSOR_DATA`

Reports the latest combined sensor data.

Structure:

```text
SENSOR_DATA,<robot_id>,<valid>,<black_detected>,<front_object_detected>,<front_distance_mm>,<width_cm>,<color>,<temperature_c>
```

Example:

```text
SENSOR_DATA,R1,1,0,1,120,3.2,red,24.5
```

---

### 8.16 `TCS_RAW`

Reports a live raw TCS3200 reading.

Structure:

```text
TCS_RAW,<robot_id>,<red_hz>,<green_hz>,<blue_hz>,<clear_hz>,<classified_color>
```

Example:

```text
TCS_RAW,R1,1200.00,300.00,250.00,900.00,red
```

---

### 8.17 `VL53_READING`

Reports a live VL53L0X distance reading.

Structure:

```text
VL53_READING,<robot_id>,<front_distance_mm>,<front_object_detected>,<distance_valid>,<distance_status>
```

Example:

```text
VL53_READING,R1,120,1,1,ok
```

---

### 8.18 `WIDTH_SCAN`

Reports object-width scan result.

Structure:

```text
WIDTH_SCAN,<robot_id>,<width_cm>,<left_angle_deg>,<right_angle_deg>,<size_cm>
```

Example:

```text
WIDTH_SCAN,R1,3.2,-4.5,4.5,3
```

---

### 8.19 `NTC_READING`

Reports a live NTCC-10K temperature reading.

Structure:

```text
NTC_READING,<robot_id>,<adc_raw>,<voltage_v>,<resistance_ohm>,<temperature_c>,<temperature_valid>
```

Example:

```text
NTC_READING,R1,2134,1.72,9870.0,24.5,1
```

---

### 8.20 `MOTOR_DONE`

Reports that a movement command has completed.

Structure:

```text
MOTOR_DONE,<robot_id>,<motion_type>,<value>
```

Examples:

```text
MOTOR_DONE,R1,move_forward,20.0
MOTOR_DONE,R1,move_reverse,8.0
MOTOR_DONE,R1,turn,90.0
```

---

### 8.21 `MAP_RESET`

Reports that the local map was reset.

Structure:

```text
MAP_RESET,<robot_id>
```

---

### 8.22 `POSE_RESET`

Reports that the relative pose was reset.

Structure:

```text
POSE_RESET,<robot_id>,<x_cm>,<y_cm>,<yaw_deg>
```

Example:

```text
POSE_RESET,R1,0.00,0.00,0.00
```

---

### 8.23 `TARGET_ACCEPTED`

Reports that a target cell was accepted.

Structure:

```text
TARGET_ACCEPTED,<robot_id>,<grid_x>,<grid_y>
```

Example:

```text
TARGET_ACCEPTED,R1,12,8
```

---

## 9. Internal Robot Data to Payload Mapping

This section shows how the C structs should map to payload fields.

---

### 9.1 `sensor_data_t` Mapping

C structure:

```c
typedef struct {
    bool valid;
    bool black_tape_detected;
    bool front_object_detected;
    int front_distance_mm;
    float estimated_width_cm;
    sample_color_t object_color;
    float temperature_c;
} sensor_data_t;
```

Recommended payload:

```text
SENSOR_DATA,<robot_id>,<valid>,<black_detected>,<front_object_detected>,<front_distance_mm>,<width_cm>,<color>,<temperature_c>
```

Mapping:

| C field | Payload field |
|---|---|
| `valid` | `<valid>` |
| `black_tape_detected` | `<black_detected>` |
| `front_object_detected` | `<front_object_detected>` |
| `front_distance_mm` | `<front_distance_mm>` |
| `estimated_width_cm` | `<width_cm>` |
| `object_color` | `<color>` |
| `temperature_c` | `<temperature_c>` |

---

### 9.2 `field_event_t` Mapping

C structure:

```c
typedef struct {
    field_event_type_t type;
    int distance_mm;
    float width_cm;
    sample_color_t color;
    int sample_size_cm;
    float temperature_c;
} field_event_t;
```

Recommended payload:

```text
EVENT,<robot_id>,<event_type>,<x_cm>,<y_cm>,<yaw_deg>,<grid_x>,<grid_y>,<distance_mm>,<width_cm>,<color>,<sample_size_cm>,<temperature_c>
```

Mapping:

| C field | Payload field |
|---|---|
| `type` | `<event_type>` |
| `distance_mm` | `<distance_mm>` |
| `width_cm` | `<width_cm>` |
| `color` | `<color>` |
| `sample_size_cm` | `<sample_size_cm>` |
| `temperature_c` | `<temperature_c>` |
| `my_pose.x` | `<x_cm>` |
| `my_pose.y` | `<y_cm>` |
| `my_pose.yaw` | `<yaw_deg>` |

---

### 9.3 `pose` Mapping

C structure:

```c
struct pose {
    float x;
    float y;
    float yaw;
};
```

Recommended payload:

```text
POSE,<robot_id>,<x_cm>,<y_cm>,<yaw_deg>,<grid_x>,<grid_y>
```

Mapping:

| C field | Payload field |
|---|---|
| `my_pose.x` | `<x_cm>` |
| `my_pose.y` | `<y_cm>` |
| `my_pose.yaw` | `<yaw_deg>` |
| calculated grid x | `<grid_x>` |
| calculated grid y | `<grid_y>` |

---

## 10. Recommended First Implementation Subset

To avoid implementing too much at once, implement this minimum subset first.

### Base Station → Robot

```text
PING
CAL_READ,<color_name>
TCSCAL,<24 calibration values>
START_MISSION
STOP_MISSION
REQUEST_POSE
REQUEST_STATUS
REQUEST_SENSOR_DATA
```

### Robot → Base Station

```text
READY,<robot_id>
PONG,<robot_id>
CALRAW,<color_name>,R,G,B,C
TCS_CAL_OK
TCS_CAL_FAIL
MISSION_STARTED,<robot_id>
MISSION_STOPPED,<robot_id>
POSE,<robot_id>,x,y,yaw,grid_x,grid_y
EVENT,<robot_id>,event_type,x,y,yaw,grid_x,grid_y,distance,width,color,size,temp
SENSOR_DATA,<robot_id>,valid,black_detected,front_object_detected,distance,width,color,temp
STATUS,<robot_id>,state,battery,calibrated,error_code
ERROR,<robot_id>,error_code
```

This subset is enough for:

- TCS3200 calibration through the base station,
- starting/stopping navigation,
- reporting robot pose,
- reporting detected cliffs/boundaries/hills/samples,
- reporting sensor faults,
- and updating the GUI map.

---

## 11. Example Mission Message Flow

### 11.1 Startup

```text
Robot -> Base: READY,R1
Base  -> Robot: PING
Robot -> Base: PONG,R1
```

### 11.2 TCS3200 Calibration

```text
Base  -> Robot: CAL_READ,none
Robot -> Base: CALRAW,none,120.00,130.00,125.00,180.00

Base  -> Robot: CAL_READ,white
Robot -> Base: CALRAW,white,900.00,870.00,850.00,1100.00

Base  -> Robot: CAL_READ,black
Robot -> Base: CALRAW,black,80.00,75.00,70.00,100.00

Base  -> Robot: CAL_READ,red
Robot -> Base: CALRAW,red,1200.00,300.00,250.00,900.00

Base  -> Robot: CAL_READ,green
Robot -> Base: CALRAW,green,300.00,1100.00,350.00,850.00

Base  -> Robot: CAL_READ,blue
Robot -> Base: CALRAW,blue,250.00,400.00,1200.00,870.00

Base  -> Robot: TCSCAL,120,130,125,180,900,870,850,1100,80,75,70,100,1200,300,250,900,300,1100,350,850,250,400,1200,870
Robot -> Base: TCS_CAL_OK
```

### 11.3 Mission Start

```text
Base  -> Robot: START_MISSION
Robot -> Base: MISSION_STARTED,R1
Robot -> Base: STATUS,R1,navigating,-1,1,none
```

### 11.4 Black Tape Event

```text
Robot -> Base: POSE,R1,30.00,0.00,0.00,12,10
Robot -> Base: EVENT,R1,black_tape,30.00,0.00,0.00,500,0.0,unknown,0,0.0
Robot -> Base: CELL_UPDATE,R1,13,10,unsafe
```

### 11.5 Rock Sample Event

```text
Robot -> Base: POSE,R1,60.00,20.00,90.00,13,11
Robot -> Base: SENSOR_DATA,R1,1,0,1,120,3.2,red,24.5
Robot -> Base: EVENT,R1,rock_sample,60.00,20.00,90.00,120,3.2,red,3,24.5
Robot -> Base: CELL_UPDATE,R1,13,12,sample
```

### 11.6 Hill Event

```text
Robot -> Base: EVENT,R1,hill,40.00,20.00,90.00,100,30.0,unknown,0,0.0
Robot -> Base: CELL_UPDATE,R1,12,12,hill
```

### 11.7 Sensor Fault

```text
Robot -> Base: EVENT,R1,sensor_fault,60.00,20.00,90.00,0,0.0,unknown,0,0.0
Robot -> Base: ERROR,R1,sensor_fault
Robot -> Base: STATUS,R1,fault,-1,1,sensor_fault
```

---

## 12. Parser Recommendations

### 12.1 General Parser Strategy

On the PYNQ side:

1. Receive UART frame.
2. Extract payload length.
3. Read payload bytes.
4. Add `\0` terminator.
5. Check the message type using `strncmp()` or tokenization.
6. Parse fields with `strtok()`, `sscanf()`, or manual parsing.
7. Validate the number and type of fields.
8. Execute only safe recognized commands.

### 12.2 Safety Rules

- Reject payloads longer than the fixed buffer size.
- Reject empty payloads.
- Reject commands with missing fields.
- Clamp speeds and distances before sending motor commands.
- Do not start navigation if required calibration is missing.
- Use safe stop or avoidance behavior on malformed commands.
- Report parsing problems with `ERROR,<robot_id>,bad_payload`.

---

## 13. Notes for the GUI/Base Station

The GUI should use incoming payloads as follows:

| Payload | GUI action |
|---|---|
| `READY` | mark robot as online |
| `POSE` | update robot marker position |
| `EVENT black_tape` | draw cliff/boundary/unsafe cell |
| `EVENT hill` | draw hill/obstacle cell |
| `EVENT rock_sample` | draw sample with color, size, and temperature |
| `CELL_UPDATE` | update map grid cell state |
| `STATUS` | show robot state and errors |
| `ERROR` | log warning/error |
| `CALRAW` | store calibration reading |
| `TCS_CAL_OK` | mark calibration as loaded |
| `TCS_CAL_FAIL` | show calibration error |

The GUI should treat robot position as **relative**. Dead-reckoning may drift, so the map should tolerate errors of at least one grid cell during testing.

---

## 14. Protocol Extension Rules

When adding new commands:

1. Use uppercase message names for commands and reports.
2. Keep field order fixed.
3. Add the new message to this file.
4. Add an example payload.
5. Add expected response behavior.
6. Add error behavior.
7. Avoid changing existing message formats once the base station parser depends on them.

---

## 15. Summary of All Defined Payload Types

### Base Station → Robot

```text
PING
CAL_READ,<color_name>
TCSCAL,<24 calibration values>
START_MISSION
STOP_MISSION
PAUSE_MISSION
RESUME_MISSION
SET_SPEED,<speed_cm_s>
MOVE_FORWARD,<distance_cm>,<speed_cm_s>
MOVE_REVERSE,<distance_cm>,<speed_cm_s>
TURN,<angle_deg>,<speed_cm_s>
SET_TARGET_CELL,<grid_x>,<grid_y>
CELL_RESERVED,<other_robot_id>,<grid_x>,<grid_y>
OTHER_ROBOT_POSE,<other_robot_id>,<x_cm>,<y_cm>,<yaw_deg>,<grid_x>,<grid_y>
RESET_MAP
RESET_POSE,<x_cm>,<y_cm>,<yaw_deg>
REQUEST_POSE
REQUEST_STATUS
REQUEST_SENSOR_DATA
REQUEST_TCS_RAW
REQUEST_VL53
REQUEST_NTC
```

### Robot → Base Station

```text
READY,<robot_id>
PONG,<robot_id>
CALRAW,<color_name>,<red_hz>,<green_hz>,<blue_hz>,<clear_hz>
TCS_CAL_OK
TCS_CAL_FAIL
MISSION_STARTED,<robot_id>
MISSION_STOPPED,<robot_id>
MISSION_PAUSED,<robot_id>
MISSION_RESUMED,<robot_id>
POSE,<robot_id>,<x_cm>,<y_cm>,<yaw_deg>,<grid_x>,<grid_y>
EVENT,<robot_id>,<event_type>,<x_cm>,<y_cm>,<yaw_deg>,<grid_x>,<grid_y>,<distance_mm>,<width_cm>,<color>,<sample_size_cm>,<temperature_c>
CELL_UPDATE,<robot_id>,<grid_x>,<grid_y>,<cell_status>
STATUS,<robot_id>,<state>,<battery_or_dummy>,<calibrated>,<error_code>
ERROR,<robot_id>,<error_code>
ERROR,<robot_id>,<error_code>,<details>
SENSOR_DATA,<robot_id>,<valid>,<black_detected>,<front_object_detected>,<front_distance_mm>,<width_cm>,<color>,<temperature_c>
TCS_RAW,<robot_id>,<red_hz>,<green_hz>,<blue_hz>,<clear_hz>,<classified_color>
VL53_READING,<robot_id>,<front_distance_mm>,<front_object_detected>,<distance_valid>,<distance_status>
WIDTH_SCAN,<robot_id>,<width_cm>,<left_angle_deg>,<right_angle_deg>,<size_cm>
NTC_READING,<robot_id>,<adc_raw>,<voltage_v>,<resistance_ohm>,<temperature_c>,<temperature_valid>
MOTOR_DONE,<robot_id>,<motion_type>,<value>
MAP_RESET,<robot_id>
POSE_RESET,<robot_id>,<x_cm>,<y_cm>,<yaw_deg>
TARGET_ACCEPTED,<robot_id>,<grid_x>,<grid_y>
```
