#ifndef CONFIG_H
#define CONFIG_H

#define ROBOT_ID "R81"                  // Unique identifier for this robot. Use "R81" for robot 1 and "R83" for robot 2.

#define PI 3.14159265359               

#define WHEEL_DIAMETER_CM 8.0          // Diameter of the robot wheel in centimeters.
#define STEPS_PER_ROTATION 1600.0      // Number of stepper-motor steps for one full wheel rotation.
#define STEPS_PER_CM (STEPS_PER_ROTATION / (PI * WHEEL_DIAMETER_CM))
                                       // Number of motor steps needed to move the robot by 1 cm.
                                       // One wheel rotation moves the robot by pi * wheel diameter.
                                       // Therefore: steps per cm = 1600 / wheel circumference.

#define WHEEL_BASE_CM 12.5             // Distance between the left and right wheels in centimeters.
#define TURN_RADIUS (WHEEL_BASE_CM / 2.0)
                                       // Distance from the robot center to one wheel during an in-place turn.

#define STEPPER_SPEED_MIN_VALUE 3024   // Smallest allowed stepper speed value; physically the fastest speed.
#define STEPPER_SPEED_MAX_VALUE 65535  // Largest allowed stepper speed value; physically the slowest speed.

#define DEFAULT_SPEED 15               // Default robot movement speed in cm/s.

#define SECONDS_PER_SPEED_UNIT 0.00000009
                                       // Approximate conversion factor from raw stepper speed value to seconds per step.
                                       // This was tuned experimentally and is used in speedFromCmPerSec().

#define USE_MOCK_SENSORS 0             // 1 = use scripted fake sensor readings for navigation testing.
                                       // 0 = use real sensor-reading functions.

#define MAP_SIZE 21                    // Number of cells in one row/column of the square internal map.
                                       // Odd number gives a symmetric map around the center cell.

#define MAP_CENTER (MAP_SIZE / 2)      // Index of the map center cell.
                                       // The robot starts here because its absolute field position is unknown.

#define CELL_SIZE_CM 20.0              // Physical size of one grid cell in centimeters.
                                       // Continuous x/y odometry is converted into this grid.

#define FORWARD_INCREMENT_CM 10.0      // Distance the robot moves forward in one navigation step.
                                       // Short increments make obstacle and tape detection safer.

#define REVERSE_DISTANCE_CM 8.0        // Distance the robot reverses after detecting a hazard or sample.
                                       // Creates space before turning away.

#define AVOID_TURN_DEG 90.0            // Default turn angle after detecting a cliff, boundary, hill, or obstacle.
                                       // 90 degrees makes the robot choose a clearly different direction.

#define SAMPLE_AVOID_TURN_DEG 45.0     // Turn angle after detecting and reporting a rock sample.
                                       // Smaller than obstacle avoidance because the robot only needs to avoid pushing the sample.

#define FRONT_OBJECT_THRESHOLD_MM 180  // Distance threshold for deciding that an object is close enough to react to.
                                       // If VL53L0X distance is below this value, the robot treats it as a nearby object.

#define MAX_NAVIGATION_STEPS 80        // Maximum number of navigation-loop iterations during a test run.
                                       // Prevents the robot from running forever while debugging.

#define SENSOR_RETRY_COUNT 3           // Number of times to retry invalid sensor readings before declaring a sensor fault.

#define WIDTH_SCAN_STEP_DEG 3.0        // Angle turned at each step during object-width scanning.
                                       // Smaller values give better width resolution but make scanning slower.

#define WIDTH_SCAN_MAX_DEG 45.0        // Maximum scan angle to one side while searching for an object edge.
                                       // Prevents the robot from rotating too far during width estimation.

#define WIDTH_SCAN_TURN_SPEED 8.0      // Slow movement speed used during object-width scanning.
                                       // Slow turning improves scan stability.

#define WIDTH_SCAN_DISTANCE_MARGIN_MM 100
                                       // Allowed distance variation while still treating the reading as the same object.
                                       // If distance changes too much, the object edge is assumed to be reached.

#define WIDTH_SCAN_AVERAGE_COUNT 2     // Number of VL53L0X readings averaged at each scan angle.

#define VL53L0X_INVALID_DISTANCE_MM -1 // Invalid distance value returned when VL53L0X reading fails.

#define VL53L0X_MAX_REASONABLE_MM 2000 // Maximum distance considered reasonable for this project.
                                       // Larger readings are treated as invalid/out of useful range.

#define PIN_S0 IO_AR4                  // TCS3200 S0 pin: output-frequency scaling control.
#define PIN_S1 IO_AR5                  // TCS3200 S1 pin: output-frequency scaling control.
#define PIN_S2 IO_AR6                  // TCS3200 S2 pin: color-filter selection control.
#define PIN_S3 IO_AR7                  // TCS3200 S3 pin: color-filter selection control.
#define PIN_OUT IO_AR8                 // TCS3200 OUT pin: square-wave frequency output from the sensor.

#define SAMPLE_TIME_MS 150             // Time window used for one TCS3200 frequency measurement.
                                       // Longer time gives more stable readings but slower sensing.

#define AVERAGE_SAMPLE_COUNT 8         // Number of TCS3200 readings averaged during calibration.

#define SETTLE_TIME_MS 30              // Delay after switching TCS3200 filter before measuring.
                                       // Allows the sensor output to stabilize.

#define UART_PAYLOAD_MAX_SIZE 256      // Maximum number of characters allowed in one UART text payload.
                                       // Used for buffers when sending/receiving ESP32 messages.

#endif