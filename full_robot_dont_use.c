#include <libpynq.h>
#include <stepper.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/time.h>
#include <string.h>
#include <time.h>

#define ROBOT_ID "R1" // Unique identifier for this robot.

#define PI 3.14159265359
#define WHEEL_DIAMETER_CM 8.0
#define STEPS_PER_ROTATION 1600.0
#define STEPS_PER_CM (STEPS_PER_ROTATION / (PI * WHEEL_DIAMETER_CM))
// One step corresponds to x degress and 360 degrees is one full rotation of the motor and 1600 steps is one full rotation of the wheel
// and one full wheel rotation moves the robot forward by one wheel circumference where circumference = PI x WHEEL_DIAMETER_CM
// So 1600 steps = 1 wheel rotation and 1 wheel rotation = 25.13 cm of travel
// and therefore 1600 steps = 25.13 cm and 1 cm is 63.66 steps which is STEPS_PER_CM

#define WHEEL_BASE_CM 12.5
#define TURN_RADIUS (WHEEL_BASE_CM / 2.0) // distance from center of ratation to one end of the wheel

// Defined by the manual
#define STEPPER_SPEED_MIN_VALUE 3024  // actual fastest speed
#define STEPPER_SPEED_MAX_VALUE 65535 // actual slowest speed

#define DEFAULT_SPEED 15

/*
 * According to the documentation:
 *     speed value max 3024  ≈ 30 microseconds per step
 *     speed value min 65535 ≈ 655 microseconds per step
 * For the first one: 3024 / 30 ≈ 100.8 So: 3024 speed units ≈ 30 μs means: 1 μs ≈ 100 speed units Therefore: time in μs ≈ speed_value / 100
 * the second one: 65535 / 655 ≈ 100.05 approximately 100. So: 65535 speed units ≈ 655 μs
 * So approximately:  time_per_step_us = speed_value / 100 which  means one speed unit corresponds approximately to:
 * 0.01 microseconds = 10 nanoseconds = 0.00000001 seconds, however we multiply this by 9 so its actually true. Founded 9 by testing.
 */
#define SECONDS_PER_SPEED_UNIT 0.00000009

struct pose
{
    float x;
    float y;
    float yaw;
};

#define USE_MOCK_SENSORS 1 // 1 = use fake/predefined sensor readings for testing the navigation logic;
                           // 0 = use the real sensor-reading function when the hardware code is ready.

#define MAP_SIZE 21 // Number of cells in one row/column of the internal square map.
                    // A 21x21 map gives the robot room to map positions around its starting point, odd number because of symmetry.

#define MAP_CENTER (MAP_SIZE / 2) // Index of the center cell of the map.
                                  // The robot starts at this center cell because its real starting location is unknown.

#define CELL_SIZE_CM 20.0 // Physical size of one map cell in centimeters.
                          // The robot's continuous x/y position is converted into this grid.

#define FORWARD_INCREMENT_CM 10.0 // Distance the robot moves forward in one navigation step.
                                  // A short increment makes obstacle/tape detection safer.

#define REVERSE_DISTANCE_CM 8.0 // Distance the robot reverses after detecting a hazard or sample.
                                // This creates space before turning away.

#define AVOID_TURN_DEG 90.0 // Default turn angle after detecting a cliff, boundary, hill, or obstacle.
                            // 90 degrees makes the robot choose a clearly different direction.

#define SAMPLE_AVOID_TURN_DEG 45.0 // Turn angle after detecting and reporting a rock sample.
                                   // Smaller than obstacle avoidance because the robot only needs to avoid pushing the sample.

#define FRONT_OBJECT_THRESHOLD_MM 180 // Distance threshold for deciding that an object is close enough to react to.
                                      // If the VL53L0X reading is below this value, the robot treats it as a nearby object.

#define MAX_NAVIGATION_STEPS 80 // Maximum number of iterations in the navigation loop.
                                // Prevents the test program from running forever while debugging.

#define SENSOR_RETRY_COUNT 3 // Number of times the robot retries a sensor reading before treating it as a fault.
                             // This supports the planned fault handling for bad/inconsistent sensor values.

#define WIDTH_SCAN_STEP_DEG 3.0           // Robot turns this many degrees per scan step.
#define WIDTH_SCAN_MAX_DEG 45.0           // Maximum scan angle to one side.
#define WIDTH_SCAN_TURN_SPEED 8.0         // Slow speed used during object-width scanning.
#define WIDTH_SCAN_DISTANCE_MARGIN_MM 100 // Allowed distance variation while still treating the reading as the same object.
#define WIDTH_SCAN_AVERAGE_COUNT 2        // Number of VL53L0X readings averaged at each scan angle.
#define VL53L0X_INVALID_DISTANCE_MM -1    // Invalid distance value expected from readVL53L0XDistance().
#define VL53L0X_MAX_REASONABLE_MM 2000    // Distances above this are treated as invalid for this project.

/*
 * TCS3200 wiring:
 *
 * VCC    -> 3.3V
 * GND    -> GND
 * S0     -> AR4
 * S1     -> AR5
 * S2     -> AR6
 * S3     -> AR7
 * OUT    -> AR8
 * OE/EN  -> GND
 *
 * Important:
 * - Use 3.3V if OUT goes directly into the PYNQ.
 * - OE/EN must be connected to GND.
 */
#define PIN_S0 IO_AR4
#define PIN_S1 IO_AR5
#define PIN_S2 IO_AR6
#define PIN_S3 IO_AR7
#define PIN_OUT IO_AR8

#define SAMPLE_TIME_MS 150
#define AVERAGE_SAMPLE_COUNT 8
#define SETTLE_TIME_MS 30

// Add color here

typedef enum
{
    COLOR_UNKNOWN,
    COLOR_WHITE,
    COLOR_BLACK,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE
} sample_color_t;

typedef enum
{
    FIELD_CLEAR,
    FIELD_BLACK_TAPE,
    FIELD_HILL,
    FIELD_ROCK_SAMPLE,
    FIELD_SENSOR_FAULT
} field_event_type_t;

typedef enum
{
    CELL_UNKNOWN,
    CELL_EXPLORED,
    CELL_UNSAFE,
    CELL_HILL,
    CELL_SAMPLE
} cell_status_t;

typedef struct
{
    bool valid;

    bool black_tape_detected;   // from color sensor
    bool front_object_detected; // from VL53L0X

    int front_distance_mm;
    float estimated_width_cm;

    sample_color_t object_color;
    float temperature_c;
} sensor_data_t;

typedef struct
{
    field_event_type_t type;

    int distance_mm;
    float width_cm;

    sample_color_t color;
    int sample_size_cm;
    float temperature_c;
} field_event_t;

typedef enum
{
    FILTER_RED,
    FILTER_GREEN,
    FILTER_BLUE,
    FILTER_CLEAR
} tcs_filter_t;

typedef enum
{
    TCS_COLOR_NONE = 0,
    TCS_COLOR_WHITE = 1,
    TCS_COLOR_BLACK = 2,
    TCS_COLOR_RED = 3,
    TCS_COLOR_GREEN = 4,
    TCS_COLOR_BLUE = 5,
    TCS_COLOR_COUNT = 6
} tcs_color_t;

typedef struct
{
    double red;
    double green;
    double blue;
    double clear;
} color_reading_t;

typedef struct
{
    double red_ratio;
    double green_ratio;
    double blue_ratio;
    double brightness;
} color_feature_t;

typedef struct
{
    const char *name;
    color_reading_t raw;
    color_feature_t feature;
} color_profile_t;

static color_profile_t profiles[TCS_COLOR_COUNT] = {
    {"none", {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"white", {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"black", {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"red", {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"green", {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"blue", {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}}};

static bool calibrated = false;
static double white_clear_reference = 1.0;
static double none_radius = 0.05;

// Globals
struct pose my_pose;
cell_status_t map_grid[MAP_SIZE][MAP_SIZE];

// MUST IMPLEMENT:

/*
 * Reads one distance measurement from the VL53L0X time-of-flight sensor.
 *
 * What this function should do later:
 * - Request one distance measurement from the VL53L0X.
 * - Return the measured distance in millimeters.
 * - Return an invalid value, for example -1, if the sensor gives an error,
 *   times out, or reports an out-of-range value.
 *
 * Used by:
 * - readSensorDataReal()
 * - obstacle / hill detection
 * - sample-distance measurement
 */
int readVL53L0XDistance(void);

/*
 * Estimates the width of the object in front of the robot.
 *
 * What this function should do later:
 * - Use the VL53L0X sensor while the robot scans slightly left and right.
 * - Detect the left and right angles where the object disappears from view.
 * - Convert the angular width and object distance into an estimated width.
 * - Return the estimated object width in centimeters.
 * - Return 0.0 or a negative value if the width cannot be estimated.
 *
 * Used by:
 * - sample-size classification
 * - separating 3 cm samples, 6 cm samples, and large hills/obstacles
 */
float scanObjectWidth(void);

/*
 * Reads the temperature near a detected rock sample.
 *
 * What this function should do later:
 * - Read the analog voltage from the NTCC-10K voltage-divider circuit.
 * - Convert the voltage into thermistor resistance.
 * - Convert the resistance into temperature using the NTC equation.
 * - Return the temperature in degrees Celsius.
 * - Return an invalid value, for example -999.0, if the reading fails.
 *
 * Used by:
 * - rock-sample reporting
 */
float readNTCTemperature(void);

/*
 * Initializes the UART connection between the PYNQ board and the ESP32.
 *
 * What this function should do later:
 * - Configure the UART peripheral used for PYNQ <-> ESP32 communication.
 * - Reset/clear UART FIFOs before starting communication.
 * - Return true if UART initialization succeeds.
 * - Return false if UART initialization fails.
 *
 * Used by:
 * - main()
 * - communication startup
 */
bool initESP32UART(void);

/*
 * Sends one text payload from the PYNQ board to the ESP32 over UART.
 *
 * What this function should do later:
 * - Take a normal C string payload, for example:
 *       "EVENT,R1,rock_sample,60.00,20.00,90.00,120,3.2,red,3,24.5"
 * - Compute the payload length in bytes.
 * - Send the UART frame in this format:
 *       4 bytes payload length
 *       payload bytes
 * - Return true if the full payload was sent.
 * - Return false if the payload is NULL, too long, or sending fails.
 *
 * Used by:
 * - sendPoseUpdate()
 * - sendFieldEventUpdate()
 * - sendCellUpdate()
 * - sendStatusUpdate()
 * - sendErrorMessage()
 */
bool sendPayloadToESP32(const char *payload);

/*
 * Receives one text payload from the ESP32 over UART.
 *
 * What this function should do later:
 * - Check whether a complete UART frame is available.
 * - Read the first 4 bytes as the payload length.
 * - Check that the payload length fits inside buffer_size.
 * - Read the payload bytes into buffer.
 * - Add '\0' at the end so the payload becomes a valid C string.
 * - Return true if a complete payload was received.
 * - Return false if no payload is available, the payload is incomplete,
 *   or the payload is too large for the buffer.
 *
 * Arguments:
 * - buffer:
 *     Destination array where the received text payload will be stored.
 *
 * - buffer_size:
 *     Size of the destination buffer, including space for the final '\0'.
 *
 * Used by:
 * - pollESP32Messages()
 */
bool receivePayloadFromESP32(char *buffer, int buffer_size);

/*
 * Checks whether the ESP32 has forwarded a command to the PYNQ that it received from the base station.
 *
 * The base station sends commands using MQTT.
 * The ESP32 receives those MQTT messages and forwards the payload to the PYNQ
 * through UART.
 *
 * So the complete command path is:
 *
 *     Base station PC
 *          -> MQTT
 *     ESP32 connectivity board
 *          -> UART
 *     PYNQ board
 *
 * What this function should do later:
 * - Check the UART connection between ESP32 and PYNQ.
 * - Call receivePayloadFromESP32().
 * - If no UART payload is available, return false.
 * - If a payload is received, treat it as the base-station command.
 * - Pass that command string to handleBaseStationMessage().
 * - Return true if a command was received and handled.
 *
 * Example command payloads originally sent by the base station over MQTT:
 * - "CAL_READ,red"
 * - "TCSCAL,..."
 * - "START_MISSION"
 * - "STOP_MISSION"
 * - "PING"
 *
 * Used by:
 * - main loop
 * - navigation loop
 * - communication polling
 */
bool pollESP32Messages(void);

/*
 * Starts the autonomous mission.
 *
 * What this function should do later:
 * - Check that required subsystems are ready.
 * - Check that TCS3200 calibration is loaded if real sensors are used.
 * - Set a global mission-running flag to true.
 * - Send a status message such as:
 *       "MISSION_STARTED,R1"
 * - Allow runNavigation() to begin or continue.
 *
 * Used by:
 * - handleBaseStationMessage("START_MISSION")
 */
void startMission(void);

/*
 * Stops the autonomous mission.
 *
 * What this function should do later:
 * - Set a global mission-running flag to false.
 * - Stop or prevent new motor commands.
 * - Leave the robot in a safe state.
 * - Send a status message such as:
 *       "MISSION_STOPPED,R1"
 *
 * Used by:
 * - handleBaseStationMessage("STOP_MISSION")
 * - fault handling
 * - emergency stop behavior
 */
void stopMission(void);

/*
 * Sends the robot's current estimated pose to the base station.
 *
 * What this function should do later:
 * - Convert my_pose.x, my_pose.y, and my_pose.yaw into a text payload.
 * - Convert the current pose into grid_x and grid_y.
 * - Build a payload like:
 *       "POSE,R1,40.00,20.00,90.00,12,11"
 * - Send it using sendPayloadToESP32().
 *
 * Used by:
 * - navigation loop after movement
 * - periodic status updates
 * - two-robot coordination
 */
void sendPoseUpdate(void);

/*
 * Sends a detected field event to the base station.
 *
 * What this function should do later:
 * - Convert the field_event_t into a text payload.
 * - Include robot ID, event type, pose, distance, width, color, sample size,
 *   and temperature.
 * - Build a payload like:
 *       "EVENT,R1,rock_sample,60.00,20.00,90.00,120,3.20,red,3,24.50"
 * - Send it using sendPayloadToESP32().
 *
 * Used by:
 * - reportFieldEvent()
 * - black-tape reporting
 * - hill/obstacle reporting
 * - rock-sample reporting
 * - sensor-fault reporting
 */
void sendFieldEventUpdate(field_event_t event);

/*
 * Sends a grid-cell update to the base station.
 *
 * What this function should do later:
 * - Convert the cell status into text.
 * - Build a payload like:
 *       "CELL_UPDATE,R1,12,11,explored"
 *       "CELL_UPDATE,R1,13,11,hill"
 *       "CELL_UPDATE,R1,14,11,sample"
 * - Send it using sendPayloadToESP32().
 *
 * Used by:
 * - markCurrentCell()
 * - markCellAhead()
 * - map update reporting
 */
void sendCellUpdate(int grid_x, int grid_y, cell_status_t status);

/*
 * Sends the current robot status to the base station.
 *
 * What this function should do later:
 * - Build a status payload containing:
 *       robot ID
 *       robot state
 *       battery value if available, otherwise -1
 *       calibration status
 *       error code
 * - Example:
 *       "STATUS,R1,navigating,-1,1,none"
 *       "STATUS,R1,fault,-1,1,sensor_fault"
 * - Send it using sendPayloadToESP32().
 *
 * Arguments:
 * - state:
 *     Text description of the current robot state.
 *     Example values:
 *       "idle"
 *       "ready"
 *       "navigating"
 *       "avoiding"
 *       "stopped"
 *       "fault"
 *
 * - error_code:
 *     Text description of the current error.
 *     Use "none" if there is no error.
 *
 * Used by:
 * - startup
 * - mission start/stop
 * - fault handling
 * - periodic health reporting
 */
void sendStatusUpdate(const char *state, const char *error_code);

/*
 * Sends an error message to the base station.
 *
 * What this function should do later:
 * - Build an error payload like:
 *       "ERROR,R1,not_calibrated"
 *       "ERROR,R1,sensor_fault"
 *       "ERROR,R1,uart_error"
 * - Send it using sendPayloadToESP32().
 *
 * Arguments:
 * - error_code:
 *     Text name of the error.
 *
 * Example error codes:
 * - "not_calibrated"
 * - "bad_payload"
 * - "bad_calibration"
 * - "sensor_fault"
 * - "uart_error"
 * - "motor_error"
 * - "unknown_command"
 *
 * Used by:
 * - failed calibration loading
 * - invalid UART payloads
 * - sensor fault handling
 * - unknown base station commands
 */
void sendErrorMessage(const char *error_code);

// Helpers
float degToRad(float degrees)
{
    return degrees * PI / 180.0;
}

float normalizeAngle(float angle)
{
    while (angle >= 360.0)
    {
        angle -= 360.0;
    }

    while (angle < 0.0)
    {
        angle += 360.0;
    }
    return angle;
}

void printPose(void)
{
    printf("Pose: x = %.2f cm, y = %.2f cm, yaw = %.2f degrees\n",
           my_pose.x, my_pose.y, my_pose.yaw);
}

/*
 * Returns the current time in microseconds.
 * This is used to count how many rising edges happen in a fixed time window.
 */
static double now_msec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

/*
 * TCS3200 helper function.
 * Writes HIGH or LOW to one of the TCS3200 control pins.
 */
static void write_pin(io_t pin, bool high)
{
    gpio_set_level(pin, high ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
}

/*
 * Selects which photodiode filter the TCS3200 uses.
 *
 * Filter selection table:
 * S2 = LOW,  S3 = LOW  -> red
 * S2 = LOW,  S3 = HIGH -> blue
 * S2 = HIGH, S3 = LOW  -> clear
 * S2 = HIGH, S3 = HIGH -> green
 */
static void tcs_select_filter(tcs_filter_t filter)
{
    switch (filter)
    {
    case FILTER_RED:
        write_pin(PIN_S2, false);
        write_pin(PIN_S3, false);
        break;

    case FILTER_BLUE:
        write_pin(PIN_S2, false);
        write_pin(PIN_S3, true);
        break;

    case FILTER_CLEAR:
        write_pin(PIN_S2, true);
        write_pin(PIN_S3, false);
        break;

    case FILTER_GREEN:
        write_pin(PIN_S2, true);
        write_pin(PIN_S3, true);
        break;
    }

    sleep_msec(SETTLE_TIME_MS);
}

/*
 * Measures the frequency on the TCS3200 OUT pin.
 *
 * The sensor outputs a square wave.
 * The frequency of this square wave depends on the selected color filter
 * and the amount of reflected light.
 */
static double measure_frequency_hz(int sample_time_ms)
{
    double start = now_msec();

    gpio_level_t previous = gpio_get_level(PIN_OUT);
    uint32_t rising_edges = 0;

    while ((now_msec() - start) < sample_time_ms)
    {
        gpio_level_t current = gpio_get_level(PIN_OUT);

        if (previous == GPIO_LEVEL_LOW && current == GPIO_LEVEL_HIGH)
        {
            rising_edges++;
        }

        previous = current;
    }

    return (double)rising_edges * 1000.0 / (double)sample_time_ms;
}

/*
 * Initializes the TCS3200 pins.
 *
 * This should be called once after pynq_init() and gpio_init().
 */
static void tcs_init(void)
{
    switchbox_set_pin(PIN_S0, SWB_GPIO);
    switchbox_set_pin(PIN_S1, SWB_GPIO);
    switchbox_set_pin(PIN_S2, SWB_GPIO);
    switchbox_set_pin(PIN_S3, SWB_GPIO);
    switchbox_set_pin(PIN_OUT, SWB_GPIO);

    gpio_set_direction(PIN_S0, GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_S1, GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_S2, GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_S3, GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_OUT, GPIO_DIR_INPUT);

    /*
     * Frequency scaling.
     *
     * S0 = HIGH, S1 = LOW gives 20% scaling.
     * This usually works better than 2% for software frequency counting.
     */
    write_pin(PIN_S0, true);
    write_pin(PIN_S1, false);
}

/*
 * Reads one raw TCS3200 measurement.
 * The result contains red, green, blue, and clear-channel frequencies.
 */
static color_reading_t tcs_read_color_once(void)
{
    color_reading_t reading;

    tcs_select_filter(FILTER_RED);
    reading.red = measure_frequency_hz(SAMPLE_TIME_MS);

    tcs_select_filter(FILTER_GREEN);
    reading.green = measure_frequency_hz(SAMPLE_TIME_MS);

    tcs_select_filter(FILTER_BLUE);
    reading.blue = measure_frequency_hz(SAMPLE_TIME_MS);

    tcs_select_filter(FILTER_CLEAR);
    reading.clear = measure_frequency_hz(SAMPLE_TIME_MS);

    return reading;
}

/*
 * Reads the TCS3200 multiple times and returns the average.
 * This reduces random measurement noise.
 */
static color_reading_t tcs_read_color_average(int samples)
{
    color_reading_t sum = {0.0, 0.0, 0.0, 0.0};

    for (int i = 0; i < samples; i++)
    {
        color_reading_t r = tcs_read_color_once();

        sum.red += r.red;
        sum.green += r.green;
        sum.blue += r.blue;
        sum.clear += r.clear;

        sleep_msec(50);
    }

    sum.red /= samples;
    sum.green /= samples;
    sum.blue /= samples;
    sum.clear /= samples;

    return sum;
}

/*
 * Converts raw RGB/C frequency values into normalized features.
 *
 * The RGB ratios describe the hue.
 * The brightness value describes how bright the object is compared with
 * the calibrated white reference.
 */
static color_feature_t extract_feature(color_reading_t reading)
{
    color_feature_t f;

    double rgb_sum = reading.red + reading.green + reading.blue;

    if (rgb_sum <= 0.0)
    {
        f.red_ratio = 0.0;
        f.green_ratio = 0.0;
        f.blue_ratio = 0.0;
    }
    else
    {
        f.red_ratio = reading.red / rgb_sum;
        f.green_ratio = reading.green / rgb_sum;
        f.blue_ratio = reading.blue / rgb_sum;
    }

    if (white_clear_reference <= 0.0)
    {
        f.brightness = 0.0;
    }
    else
    {
        f.brightness = reading.clear / white_clear_reference;
    }

    return f;
}

/*
 * Prints only the raw TCS3200 values.
 */
static void print_raw_reading(const char *prefix, color_reading_t r)
{
    printf("%s RAW: R=%7.1f  G=%7.1f  B=%7.1f  C=%7.1f\n",
           prefix, r.red, r.green, r.blue, r.clear);
}

/*
 * Prints the raw TCS3200 values and the extracted normalized features.
 */
static void print_full_reading(const char *prefix, color_reading_t r)
{
    color_feature_t f = extract_feature(r);

    printf("%s RAW: R=%7.1f  G=%7.1f  B=%7.1f  C=%7.1f | ",
           prefix, r.red, r.green, r.blue, r.clear);

    printf("FEATURE: r=%.3f g=%.3f b=%.3f bright=%.3f\n",
           f.red_ratio, f.green_ratio, f.blue_ratio, f.brightness);
}

/*
 * Computes the distance between two color features.
 *
 * RGB ratios are weighted strongly because they represent hue.
 * Brightness is also included because it helps separate black, white, and none.
 */
static double feature_distance(color_feature_t a, color_feature_t b)
{
    double dr = a.red_ratio - b.red_ratio;
    double dg = a.green_ratio - b.green_ratio;
    double db = a.blue_ratio - b.blue_ratio;
    double dv = a.brightness - b.brightness;

    /*
     * Weighted squared distance.
     *
     * RGB ratios describe hue.
     * Brightness helps separate none/black/white.
     */
    return 3.0 * dr * dr +
           3.0 * dg * dg +
           3.0 * db * db +
           2.0 * dv * dv;
}

/*
 * Computes how close a live reading must be to the calibrated background
 * before it is classified as "none".
 */
static void compute_none_radius(void)
{
    /*
     * The NONE radius decides how close a live reading must be to the
     * calibrated background before it is called "none".
     */

    color_feature_t none_feature = profiles[TCS_COLOR_NONE].feature;

    double min_distance = feature_distance(none_feature, profiles[TCS_COLOR_WHITE].feature);

    for (int i = TCS_COLOR_BLACK; i < TCS_COLOR_COUNT; i++)
    {
        double d = feature_distance(none_feature, profiles[i].feature);

        if (d < min_distance)
        {
            min_distance = d;
        }
    }

    /*
     * If live reading is within this radius from NONE, classify as none.
     *
     * Bigger multiplier: more likely to say none.
     * Smaller multiplier: more likely to classify as a real color.
     */
    none_radius = min_distance * 0.45;

    if (none_radius < 0.02)
    {
        none_radius = 0.02;
    }
}

/*
 * Classifies a live TCS3200 reading using calibration profiles.
 *
 * The calibration profiles are expected to be loaded from the base station PC
 * before autonomous navigation starts.
 */
static tcs_color_t classify_color_enum(color_reading_t reading)
{
    /*
     * Classifies a live TCS3200 reading using the calibrated profiles.
     * This is based on your working calibration method:
     * - convert raw RGB/C values into ratios and brightness
     * - compare the current feature to the stored calibration profiles
     * - choose the closest calibrated color
     */

    if (!calibrated)
    {
        return TCS_COLOR_NONE;
    }

    if (reading.red <= 0.0 &&
        reading.green <= 0.0 &&
        reading.blue <= 0.0 &&
        reading.clear <= 0.0)
    {
        return TCS_COLOR_NONE;
    }

    color_feature_t current = extract_feature(reading);

    /*
     * First compare with the calibrated background/none profile.
     * This prevents empty floor/background from being forced into a color.
     */
    double d_none = feature_distance(current, profiles[TCS_COLOR_NONE].feature);

    if (d_none < none_radius)
    {
        return TCS_COLOR_NONE;
    }

    /*
     * Compare with the real color profiles and choose the closest one.
     */
    tcs_color_t best_color = TCS_COLOR_WHITE;
    double best_distance = feature_distance(current, profiles[TCS_COLOR_WHITE].feature);

    for (int i = TCS_COLOR_BLACK; i < TCS_COLOR_COUNT; i++)
    {
        double d = feature_distance(current, profiles[i].feature);

        if (d < best_distance)
        {
            best_distance = d;
            best_color = (tcs_color_t)i;
        }
    }

    return best_color;
}

/*
 * Converts the TCS3200-specific color enum into the color enum used by
 * the navigation/classification software.
 */
static sample_color_t tcsColorToSampleColor(tcs_color_t color)
{
    /*
     * Converts the TCS3200-specific color enum into the color enum used by
     * the navigation/classification software.
     */

    switch (color)
    {
    case TCS_COLOR_WHITE:
        return COLOR_WHITE;

    case TCS_COLOR_BLACK:
        return COLOR_BLACK;

    case TCS_COLOR_RED:
        return COLOR_RED;

    case TCS_COLOR_GREEN:
        return COLOR_GREEN;

    case TCS_COLOR_BLUE:
        return COLOR_BLUE;

    case TCS_COLOR_NONE:
    default:
        return COLOR_UNKNOWN;
    }
}

/*
 * Returns the calibrated sample color detected by the TCS3200.
 * Used when the navigation/detection code has already found an object
 * and wants to classify its color.
 */
sample_color_t classifyTCS3200Color(void)
{
    /*
     * Reads the TCS3200, averages a few measurements, and classifies the
     * object color using the calibration profiles loaded from the base station.
     */

    if (!calibrated)
    {
        printf("TCS3200 not calibrated. Cannot classify color.\n");
        return COLOR_UNKNOWN;
    }

    color_reading_t reading = tcs_read_color_average(3);

    print_full_reading("TCS SAMPLE", reading);

    tcs_color_t detected = classify_color_enum(reading);

    return tcsColorToSampleColor(detected);
}

/*
 * Returns true when the current reading matches the calibrated black profile.
 *
 * Important:
 * This does not prove that the black surface is tape.
 * It only means the TCS3200 currently sees black.
 *
 * The navigation layer should combine this with distance-sensor context:
 * - black + no close front object  -> black tape / cliff / boundary
 * - black + close front object     -> possible black rock sample
 */
bool tcs3200DetectBlackTape(void)
{
    /*
     * Returns true when the current reading matches the calibrated black
     * profile.
     *
     * Important:
     * This does not prove that the black surface is tape.
     * It only means the TCS3200 currently sees black.
     *
     * The navigation layer should combine this with distance-sensor context:
     * - black + no close front object  -> black tape / cliff / boundary
     * - black + close front object     -> possible black rock sample
     */

    if (!calibrated)
    {
        printf("TCS3200 not calibrated. Cannot detect black tape.\n");
        return false;
    }

    color_reading_t reading = tcs_read_color_average(3);

    print_full_reading("TCS TAPE", reading);

    tcs_color_t detected = classify_color_enum(reading);

    return detected == TCS_COLOR_BLACK;
}

/*
 * Loads TCS3200 calibration profiles received from the base station PC.
 *
 * Expected message format:
 *
 * TCSCAL,
 * none_R,none_G,none_B,none_C,
 * white_R,white_G,white_B,white_C,
 * black_R,black_G,black_B,black_C,
 * red_R,red_G,red_B,red_C,
 * green_R,green_G,green_B,green_C,
 * blue_R,blue_G,blue_B,blue_C
 *
 * All values are raw frequency values measured by the robot.
 */
bool tcs3200LoadCalibrationFromBaseStation(const char *message)
{
    /*
     * Loads TCS3200 calibration profiles received from the base station PC.
     *
     * Expected message format:
     *
     * TCSCAL,
     * none_R,none_G,none_B,none_C,
     * white_R,white_G,white_B,white_C,
     * black_R,black_G,black_B,black_C,
     * red_R,red_G,red_B,red_C,
     * green_R,green_G,green_B,green_C,
     * blue_R,blue_G,blue_B,blue_C
     *
     * All values are raw frequency values measured by the robot.
     */

    if (message == NULL)
    {
        return false;
    }

    if (strncmp(message, "TCSCAL,", 7) != 0)
    {
        return false;
    }

    const char *p = message + 7;
    double values[24];

    for (int i = 0; i < 24; i++)
    {
        char *endptr;

        values[i] = strtod(p, &endptr);

        if (endptr == p)
        {
            printf("TCS calibration parse error at value %d.\n", i);
            calibrated = false;
            return false;
        }

        p = endptr;

        if (*p == ',')
        {
            p++;
        }
    }

    /*
     * Store the 24 received values into 6 color profiles.
     * Each color has 4 channels: red, green, blue, clear.
     */
    for (int color = 0; color < TCS_COLOR_COUNT; color++)
    {
        int base = color * 4;

        profiles[color].raw.red = values[base + 0];
        profiles[color].raw.green = values[base + 1];
        profiles[color].raw.blue = values[base + 2];
        profiles[color].raw.clear = values[base + 3];
    }

    /*
     * Check that real colors produced some signal.
     * The background/none profile may be low, but the sample colors should
     * not all be zero.
     */
    for (int color = TCS_COLOR_WHITE; color < TCS_COLOR_COUNT; color++)
    {
        if (profiles[color].raw.red <= 0.0 &&
            profiles[color].raw.green <= 0.0 &&
            profiles[color].raw.blue <= 0.0 &&
            profiles[color].raw.clear <= 0.0)
        {

            printf("TCS calibration error: %s profile has no signal.\n",
                   profiles[color].name);

            calibrated = false;
            return false;
        }
    }

    /*
     * White is used as the brightness reference, same as in your working code.
     */
    white_clear_reference = profiles[TCS_COLOR_WHITE].raw.clear;

    if (white_clear_reference <= 0.0)
    {
        white_clear_reference = 1.0;
    }

    /*
     * Convert all raw calibration readings into features.
     */
    for (int color = 0; color < TCS_COLOR_COUNT; color++)
    {
        profiles[color].feature = extract_feature(profiles[color].raw);
    }

    /*
     * Recompute the allowed background/none radius.
     */
    compute_none_radius();

    calibrated = true;

    printf("TCS3200 calibration loaded from base station.\n");

    for (int color = 0; color < TCS_COLOR_COUNT; color++)
    {
        printf("%-5s ", profiles[color].name);
        print_full_reading("", profiles[color].raw);
    }

    printf("Computed none radius: %.5f\n", none_radius);

    return true;
}

/*
 * Called when the base station sends something like:
 *
 * CAL_READ,red
 *
 * The robot measures the current TCS3200 reading and returns:
 *
 * CALRAW,red,R,G,B,C
 *
 * For now this uses printf. Later replace printf with your UART send
 * function to send the payload to the ESP32/base station.
 */
void tcs3200SendCalibrationReading(const char *color_name)
{
    /*
     * Called when the base station sends something like:
     *
     * CAL_READ,red
     *
     * The robot measures the current TCS3200 reading and returns:
     *
     * CALRAW,red,R,G,B,C
     *
     * For now this uses printf. Later replace printf with your UART send
     * function to send the payload to the ESP32/base station.
     */

    color_reading_t reading = tcs_read_color_average(AVERAGE_SAMPLE_COUNT);

    printf("CALRAW,%s,%.2f,%.2f,%.2f,%.2f\n",
           color_name,
           reading.red,
           reading.green,
           reading.blue,
           reading.clear);
}

/*
 * Handles simple text commands from the base station.
 *
 * Later, this function should be called after extracting the UART payload
 * received from the ESP32.
 */
void handleBaseStationMessage(const char *message)
{
    /*
     * Handles simple text commands from the base station.
     *
     * Later, this function should be called after extracting the UART payload
     * received from the ESP32.
     */

    if (message == NULL)
    {
        return;
    }

    if (strncmp(message, "CAL_READ,", 9) == 0)
    {
        const char *color_name = message + 9;
        tcs3200SendCalibrationReading(color_name);
        return;
    }

    if (strncmp(message, "TCSCAL,", 7) == 0)
    {
        bool ok = tcs3200LoadCalibrationFromBaseStation(message);

        if (ok)
        {
            printf("TCS_CAL_OK\n");
        }
        else
        {
            printf("TCS_CAL_FAIL\n");
        }

        return;
    }

    if (strcmp(message, "START_MISSION") == 0)
    {
        if (!calibrated)
        {
            printf("Cannot start mission: TCS3200 is not calibrated.\n");
            return;
        }

        printf("Mission start command received.\n");

        /*
         * Later:
         * runNavigation();
         * must be called from here instead of main probably
         */
        return;
    }

    printf("Unknown base station message: %s\n", message);
}

/*
 * TCS3200 debug test.
 * This checks whether the OUT pin is toggling.
 */
static void debug_out_pin(void)
{
    printf("\nTesting OUT pin on AR8 for 3 seconds...\n");

    tcs_select_filter(FILTER_CLEAR);

    gpio_level_t previous = gpio_get_level(PIN_OUT);
    int changes = 0;

    for (int i = 0; i < 3000; i++)
    {
        gpio_level_t current = gpio_get_level(PIN_OUT);

        if (current != previous)
        {
            changes++;
            previous = current;
        }

        sleep_msec(1);
    }

    printf("OUT pin changes in 3 seconds: %d\n", changes);
    printf("Final OUT level: %d\n", gpio_get_level(PIN_OUT));

    if (changes == 0)
    {
        printf("WARNING: OUT did not toggle.\n");
        printf("Check OUT wire, OE/EN -> GND, VCC, GND, and PIN_OUT.\n");
    }
    else
    {
        printf("OUT pin is toggling. Sensor signal is present.\n");
    }
}

// Odometry implemented by deadreckoing(updating the pose after every move)
void updatePoseAfterMove(float distance)
{
    float yaw_rad = degToRad(my_pose.yaw);

    my_pose.x = my_pose.x + distance * cos(yaw_rad);
    my_pose.y = my_pose.y + distance * sin(yaw_rad);
}

void updatePoseAfterTurn(float angle)
{
    my_pose.yaw = my_pose.yaw + angle;
    my_pose.yaw = normalizeAngle(my_pose.yaw);
}

int speedFromCmPerSec(float cm_per_sec)
{
    // converts the desired speed in cm/s into the weird raw value that stepper_set_speed() expects.
    int speed_value = round(
        1.0 / (cm_per_sec * STEPS_PER_CM * SECONDS_PER_SPEED_UNIT));

    // Clamp the value so it stays inside the allowed range just in case.
    if (speed_value < STEPPER_SPEED_MIN_VALUE)
    {
        speed_value = STEPPER_SPEED_MIN_VALUE;
    }

    if (speed_value > STEPPER_SPEED_MAX_VALUE)
    {
        speed_value = STEPPER_SPEED_MAX_VALUE;
    }

    return speed_value;
}

const char *colorToString(sample_color_t color)
{
    switch (color)
    {
    case COLOR_WHITE:
        return "white";
    case COLOR_BLACK:
        return "black";
    case COLOR_RED:
        return "red";
    case COLOR_GREEN:
        return "green";
    case COLOR_BLUE:
        return "blue";
    default:
        return "unknown";
    }
}

const char *eventToString(field_event_type_t event)
{
    switch (event)
    {
    case FIELD_CLEAR:
        return "clear";
    case FIELD_BLACK_TAPE:
        return "black_tape";
    case FIELD_HILL:
        return "hill";
    case FIELD_ROCK_SAMPLE:
        return "rock_sample";
    case FIELD_SENSOR_FAULT:
        return "sensor_fault";
    default:
        return "unknown";
    }
}

bool isValidSampleColor(sample_color_t color)
{
    return color == COLOR_WHITE ||
           color == COLOR_BLACK ||
           color == COLOR_RED ||
           color == COLOR_GREEN ||
           color == COLOR_BLUE;
}

bool estimateSampleSize(float width_cm, int *sample_size_cm)
{
    if (fabs(width_cm - 3.0) <= 1.5)
    {
        *sample_size_cm = 3;
        return true;
    }

    if (fabs(width_cm - 6.0) <= 1.5)
    {
        *sample_size_cm = 6;
        return true;
    }

    return false;
}

void initMap(void)
{
    for (int x = 0; x < MAP_SIZE; x++)
    {
        for (int y = 0; y < MAP_SIZE; y++)
        {
            map_grid[x][y] = CELL_UNKNOWN;
        }
    }

    map_grid[MAP_CENTER][MAP_CENTER] = CELL_EXPLORED;
}

bool poseToGridCell(float x_cm, float y_cm, int *grid_x, int *grid_y)
{
    int gx = MAP_CENTER + (int)round(x_cm / CELL_SIZE_CM);
    int gy = MAP_CENTER + (int)round(y_cm / CELL_SIZE_CM);

    if (gx < 0 || gx >= MAP_SIZE || gy < 0 || gy >= MAP_SIZE)
    {
        return false;
    }

    *grid_x = gx;
    *grid_y = gy;
    return true;
}

void markCurrentCell(cell_status_t status)
{
    int gx, gy;

    if (poseToGridCell(my_pose.x, my_pose.y, &gx, &gy))
    {
        map_grid[gx][gy] = status;
    }
}

void markCellAhead(cell_status_t status)
{
    float yaw_rad = degToRad(my_pose.yaw);

    float ahead_x = my_pose.x + CELL_SIZE_CM * cos(yaw_rad);
    float ahead_y = my_pose.y + CELL_SIZE_CM * sin(yaw_rad);

    int gx, gy;

    if (poseToGridCell(ahead_x, ahead_y, &gx, &gy))
    {
        map_grid[gx][gy] = status;
    }
}

void moveWithRamp(float distance, float speed_cm_s)
{
    int total_steps = round(fabs(distance) * STEPS_PER_CM);

    if (total_steps <= 0)
    {
        return;
    }

    int direction = (distance >= 0) ? 1 : -1;

    int startspeed = STEPPER_SPEED_MAX_VALUE; // slowest allowed
    int targetspeed = speedFromCmPerSec(speed_cm_s);

    // Clamp target speed to safe stepper range
    if (targetspeed < STEPPER_SPEED_MIN_VALUE)
        targetspeed = STEPPER_SPEED_MIN_VALUE;
    if (targetspeed > STEPPER_SPEED_MAX_VALUE)
        targetspeed = STEPPER_SPEED_MAX_VALUE;

    int rampsteps = 0.2 * total_steps;

    if (rampsteps > 400)
    {
        rampsteps = 400;
    }

    int chunk = 5; // number of steps per command; can tune this

    for (int i = 0; i < total_steps; i += chunk)
    {
        int remaining = total_steps - i;
        int this_chunk = remaining < chunk ? remaining : chunk;

        int stepperspeed;

        // Acceleration
        if (rampsteps > 0 && i < rampsteps)
        {
            float progress = (float)i / rampsteps;
            stepperspeed = startspeed - progress * (startspeed - targetspeed);
        }

        // Deceleration
        else if (rampsteps > 0 && i >= total_steps - rampsteps)
        {
            float progress = (float)(total_steps - i) / rampsteps;
            stepperspeed = startspeed - progress * (startspeed - targetspeed);
        }

        // Constant speed
        else
        {
            stepperspeed = targetspeed;
        }

        stepper_set_speed(stepperspeed, stepperspeed);
        stepper_steps(direction * this_chunk, direction * this_chunk);

        while (!stepper_steps_done())
        {
            // wait until this chunk is completed
        }
    }

    sleep_msec(500);

    updatePoseAfterMove(distance);
}

void move(float distance, float speed)
{
    int steps = round(distance * STEPS_PER_CM);

    int stepper_speed = speedFromCmPerSec(speed);
    stepper_set_speed(stepper_speed, stepper_speed);
    stepper_steps(steps, steps);

    while (!stepper_steps_done())
        ; // while the motors are not done, keep waiting

    sleep_msec(500); // pause after movement so that command finishes properly and the next command isnt sent too fast

    updatePoseAfterMove(distance);
}

void turn(float angle, float speed)
{
    angle = fmodf(angle, 360);
    if (angle > 180)
        angle -= 360;

    float radians = angle * PI / 180.0;
    float distance = TURN_RADIUS * radians;
    int steps = round(distance * STEPS_PER_CM);

    int stepper_speed = speedFromCmPerSec(speed);
    stepper_set_speed(stepper_speed, stepper_speed);

    stepper_steps(-steps, steps);

    while (!stepper_steps_done())
        ; // while the motors are not done, keep waiting

    sleep_msec(500); // pause after movement so that command finishes properly and the next command isnt sent too fast

    updatePoseAfterTurn(angle);
}

void moveTo(struct pose target_pose, float speed)
{

    float delta_x = target_pose.x - my_pose.x;
    float delta_y = target_pose.y - my_pose.y;
    float angle_to_target = atan2(delta_y, delta_x);
    float angle_to_turn = angle_to_target - degToRad(my_pose.yaw);

    angle_to_turn = angle_to_turn * 180.0 / PI; // convert to degrees
    turn(angle_to_turn, speed);

    // Calculate the distance to the target
    float distance_to_target = sqrt(delta_x * delta_x + delta_y * delta_y);
    moveWithRamp(distance_to_target, speed);
}

void setSpeed(float speed)
{
    int stepper_speed = speedFromCmPerSec(speed);
    stepper_set_speed(stepper_speed, stepper_speed);
}

void dance()
{
    for (int i = 0; i < 5; i++)
    {
        turn(60, 25);
        turn(-60, 25);
    }
}

/*
 * Checks whether a VL53L0X distance reading is usable.
 *
 * A valid reading should be:
 * - positive
 * - not the invalid error value
 * - not unrealistically far for this project
 */
static bool isDistanceReadingValidForScan(int distance_mm) {
    if (distance_mm == VL53L0X_INVALID_DISTANCE_MM) {
        return false;
    }

    if (distance_mm <= 0) {
        return false;
    }

    if (distance_mm > VL53L0X_MAX_REASONABLE_MM) {
        return false;
    }

    return true;
}

/*
 * Reads the VL53L0X several times and averages the valid readings.
 *
 * Return:
 * - average distance in millimeters if at least one valid reading exists
 * - VL53L0X_INVALID_DISTANCE_MM if all readings are invalid
 */
static int readAverageDistanceForScan(int sample_count) {
    int valid_count = 0;
    int sum_mm = 0;

    for (int i = 0; i < sample_count; i++) {
        int distance_mm = readVL53L0XDistance();

        if (isDistanceReadingValidForScan(distance_mm)) {
            sum_mm += distance_mm;
            valid_count++;
        }

        sleep_msec(20);
    }

    if (valid_count == 0) {
        return VL53L0X_INVALID_DISTANCE_MM;
    }

    return sum_mm / valid_count;
}

/*
 * Checks whether the object is still visible from the current scan angle.
 *
 * The object is considered visible if:
 * - the distance reading is valid
 * - the measured distance is still close to the original center distance
 * - the measured distance is still within the front-object threshold region
 *
 * This prevents the robot from confusing a far wall/background with the object.
 */
static bool scanStillSeesObject(int distance_mm, int center_distance_mm) {
    if (!isDistanceReadingValidForScan(distance_mm)) {
        return false;
    }

    /*
     * If the distance suddenly becomes much larger than the center reading,
     * the sensor is probably no longer seeing the original object.
     */
    if (abs(distance_mm - center_distance_mm) > WIDTH_SCAN_DISTANCE_MARGIN_MM) {
        return false;
    }

    /*
     * Keep the scan tied to nearby objects.
     * The margin allows the measured distance to vary slightly at the edges.
     */
    if (distance_mm > FRONT_OBJECT_THRESHOLD_MM + WIDTH_SCAN_DISTANCE_MARGIN_MM) {
        return false;
    }

    return true;
}

/*
 * Scans in one direction until the object edge is found.
 *
 * Arguments:
 * - direction:
 *      -1 = scan left
 *      +1 = scan right
 *
 * - center_distance_mm:
 *      distance to the object when the robot is facing its center
 *
 * - total_turned_deg:
 *      output variable that stores how far the robot actually turned
 *
 * Return:
 * - estimated edge angle in degrees from the center direction
 *
 * Important:
 * This function leaves the robot turned away from the center.
 * The caller must turn the robot back afterwards.
 */
static float scanOneObjectEdge(int direction, int center_distance_mm, float *total_turned_deg) {
    float last_visible_angle = 0.0;
    float current_angle = 0.0;

    *total_turned_deg = 0.0;

    while (current_angle < WIDTH_SCAN_MAX_DEG) {
        /*
         * Turn a small amount toward the scan direction.
         *
         * direction = -1 gives left scan.
         * direction = +1 gives right scan.
         */
        turn(direction * WIDTH_SCAN_STEP_DEG, WIDTH_SCAN_TURN_SPEED);

        current_angle += WIDTH_SCAN_STEP_DEG;
        *total_turned_deg = current_angle;

        /*
         * Let the robot/sensor settle after the turn.
         */
        sleep_msec(100);

        int distance_mm = readAverageDistanceForScan(WIDTH_SCAN_AVERAGE_COUNT);

        printf("Width scan %s: angle=%.2f deg, distance=%d mm\n",
               direction < 0 ? "left" : "right",
               current_angle,
               distance_mm);

        /*
         * If the object is no longer visible, the edge is between the last
         * visible angle and the current angle.
         */
        if (!scanStillSeesObject(distance_mm, center_distance_mm)) {
            return (last_visible_angle + current_angle) / 2.0;
        }

        /*
         * Object is still visible at this angle.
         */
        last_visible_angle = current_angle;
    }

    /*
     * If the object is still visible at the maximum scan angle, we do not know
     * the true edge. Return the maximum angle as a lower-bound estimate.
     */
    return WIDTH_SCAN_MAX_DEG;
}

/*
 * Estimates the width of the object in front of the robot.
 *
 * Method:
 * 1. Measure the object distance while facing the object.
 * 2. Turn left in small steps until the object disappears.
 * 3. Return to the center direction.
 * 4. Turn right in small steps until the object disappears.
 * 5. Return to the center direction.
 * 6. Convert the total angular width into a physical width.
 *
 * Geometry:
 * If the object is approximately distance d away and has angular width theta:
 *
 *     width = 2 * d * tan(theta / 2)
 *
 * where:
 * - width is in centimeters
 * - d is in centimeters
 * - theta is in radians
 *
 * Return:
 * - estimated object width in centimeters
 * - 0.0 if the width cannot be estimated
 */
float scanObjectWidth(void) {
    /*
     * First measure the distance while facing the object directly.
     */
    int center_distance_mm = readAverageDistanceForScan(WIDTH_SCAN_AVERAGE_COUNT);

    if (!isDistanceReadingValidForScan(center_distance_mm)) {
        printf("Width scan failed: invalid center distance.\n");
        return 0.0;
    }

    if (center_distance_mm > FRONT_OBJECT_THRESHOLD_MM + WIDTH_SCAN_DISTANCE_MARGIN_MM) {
        printf("Width scan failed: object is too far away. Distance=%d mm\n",
               center_distance_mm);
        return 0.0;
    }

    printf("Width scan started. Center distance = %d mm\n", center_distance_mm);

    /*
     * Scan left.
     * The robot is left turned when scanOneObjectEdge() returns.
     */
    float left_total_turned = 0.0;
    float left_edge_deg = scanOneObjectEdge(-1, center_distance_mm, &left_total_turned);

    /*
     * Return from the left scan position back to the center direction.
     */
    turn(left_total_turned, WIDTH_SCAN_TURN_SPEED);
    sleep_msec(150);

    /*
     * Scan right.
     * The robot is right turned when scanOneObjectEdge() returns.
     */
    float right_total_turned = 0.0;
    float right_edge_deg = scanOneObjectEdge(+1, center_distance_mm, &right_total_turned);

    /*
     * Return from the right scan position back to the center direction.
     */
    turn(-right_total_turned, WIDTH_SCAN_TURN_SPEED);
    sleep_msec(150);

    /*
     * Total visible angular width of the object.
     */
    float angular_width_deg = left_edge_deg + right_edge_deg;

    if (angular_width_deg <= 0.0) {
        printf("Width scan failed: angular width is zero.\n");
        return 0.0;
    }

    /*
     * Convert distance from millimeters to centimeters.
     */
    float distance_cm = center_distance_mm / 10.0;

    /*
     * Convert angular width from degrees to radians.
     */
    float angular_width_rad = degToRad(angular_width_deg);

    /*
     * Estimate physical width using simple triangle geometry.
     */
    float width_cm = 2.0 * distance_cm * tan(angular_width_rad / 2.0);

    printf("Width scan result: left=%.2f deg, right=%.2f deg, total=%.2f deg, width=%.2f cm\n",
           left_edge_deg,
           right_edge_deg,
           angular_width_deg,
           width_cm);

    return width_cm;
}

// Sensors
// scripted sequence of certain event to see how the robot reacts
sensor_data_t readSensorDataMock(void)
{
    static int mock_index = 0;

    static sensor_data_t script[] = {
        //   valid,  tape, object, distance, width, color,        temp
        {true, false, false, 500, 0.0, COLOR_UNKNOWN, 0.0}, // clear
        {true, false, false, 500, 0.0, COLOR_UNKNOWN, 0.0}, // clear
        {true, true, false, 500, 0.0, COLOR_UNKNOWN, 0.0},  // black tape
        {true, false, false, 500, 0.0, COLOR_UNKNOWN, 0.0}, // clear
        {true, false, true, 110, 3.2, COLOR_RED, 24.5},     // small red sample
        {true, false, false, 500, 0.0, COLOR_UNKNOWN, 0.0}, // clear
        {true, false, true, 100, 30.0, COLOR_UNKNOWN, 0.0}, // hill
        {true, false, false, 500, 0.0, COLOR_UNKNOWN, 0.0}, // clear
        {false, false, false, 0, 0.0, COLOR_UNKNOWN, 0.0},  // sensor fault
        {true, false, true, 120, 6.1, COLOR_BLUE, 25.0}     // large blue sample
    };

    int script_length = sizeof(script) / sizeof(script[0]);

    sensor_data_t data = script[mock_index % script_length];
    mock_index++;

    return data;
}

sensor_data_t readSensorDataReal(void)
{
    sensor_data_t data;

    data.valid = true;

    /*
     * Replace these placeholder values with real sensor functions later:
     *
     * data.front_distance_mm     = readVL53L0XDistance();
     * data.black_tape_detected   = tcs3200DetectBlackTape();
     * data.front_object_detected = data.front_distance_mm < FRONT_OBJECT_THRESHOLD_MM;
     * data.object_color          = classifyTCS3200Color();
     * data.estimated_width_cm    = scanObjectWidth();
     * data.temperature_c         = readNTCTemperature();
     */

    /*
     * Distance sensor is still mocked/placeholder here.
     * Replace this line with:
     *
     * data.front_distance_mm = readVL53L0XDistance();
     */
    data.front_distance_mm = 500;
    data.front_object_detected = data.front_distance_mm < FRONT_OBJECT_THRESHOLD_MM;

    /*
     * Real TCS3200 color reading.
     *
     * The sensor is read once, then the same reading is used for both:
     * - black tape detection
     * - sample color classification
     *
     * This avoids reading the color sensor twice in one navigation step.
     */
    if (calibrated)
    {
        color_reading_t color_reading = tcs_read_color_average(3);
        tcs_color_t detected_color = classify_color_enum(color_reading);

        print_full_reading("TCS LIVE", color_reading);

        data.black_tape_detected = detected_color == TCS_COLOR_BLACK;
        data.object_color = tcsColorToSampleColor(detected_color);
    }
    else
    {
        printf("TCS3200 not calibrated. Marking sensor data as invalid.\n");

        data.valid = false;
        data.black_tape_detected = false;
        data.object_color = COLOR_UNKNOWN;
    }

    /*
     * Replace this placeholder with real scanObjectWidth() later.
     */
    data.estimated_width_cm = 0.0;

    /*
     * Replace this placeholder with real readNTCTemperature() later.
     */
    data.temperature_c = 0.0;

    return data;
}

sensor_data_t readSensorData(void)
{
    if (USE_MOCK_SENSORS)
    {
        return readSensorDataMock();
    }
    else
    {
        return readSensorDataReal();
    }
}

field_event_t interpretSensorData(sensor_data_t data)
{
    field_event_t event;

    event.type = FIELD_CLEAR;
    event.distance_mm = data.front_distance_mm;
    event.width_cm = data.estimated_width_cm;
    event.color = data.object_color;
    event.sample_size_cm = 0;
    event.temperature_c = data.temperature_c;

    if (!data.valid)
    {
        event.type = FIELD_SENSOR_FAULT;
        return event;
    }

    /*
     * Black tape vs black sample distinction:
     * - black detected below the robot, without front object: tape/cliff/boundary
     * - black object in front with valid width: possible sample
     */
    if (data.black_tape_detected && !data.front_object_detected)
    {
        event.type = FIELD_BLACK_TAPE;
        return event;
    }

    if (data.front_object_detected &&
        data.front_distance_mm <= FRONT_OBJECT_THRESHOLD_MM)
    {

        int sample_size = 0;

        if (isValidSampleColor(data.object_color) &&
            estimateSampleSize(data.estimated_width_cm, &sample_size))
        {

            event.type = FIELD_ROCK_SAMPLE;
            event.sample_size_cm = sample_size;
            return event;
        }

        event.type = FIELD_HILL;
        return event;
    }

    event.type = FIELD_CLEAR;
    return event;
}

sensor_data_t readReliableSensorData(void)
{
    sensor_data_t data;

    for (int attempt = 0; attempt < SENSOR_RETRY_COUNT; attempt++)
    {
        data = readSensorData();

        if (data.valid)
        {
            return data;
        }

        printf("Invalid sensor reading. Retry %d/%d\n",
               attempt + 1,
               SENSOR_RETRY_COUNT);

        sleep_msec(100);
    }

    data.valid = false;
    return data;
}

void reportFieldEvent(field_event_t event)
{
    printf(
        "REPORT: robot=%s, event=%s, x=%.2f, y=%.2f, yaw=%.2f, distance=%d mm, width=%.2f cm, color=%s, size=%d cm, temp=%.2f C\n",
        ROBOT_ID,
        eventToString(event.type),
        my_pose.x,
        my_pose.y,
        my_pose.yaw,
        event.distance_mm,
        event.width_cm,
        colorToString(event.color),
        event.sample_size_cm,
        event.temperature_c);

    /*
     * Later replace this with something like:
     *
     * sendUARTMessageToESP32(payload);
     *
     * Example payload:
     * EVENT,R1,rock_sample,x,y,yaw,distance,width,color,size,temp
     */
}

// Central navigation software
void handleFieldEvent(field_event_t event)
{
    printf("Navigation received event: %s\n", eventToString(event.type));

    switch (event.type)
    {
    case FIELD_CLEAR:
        markCurrentCell(CELL_EXPLORED);

        moveWithRamp(FORWARD_INCREMENT_CM, DEFAULT_SPEED);

        markCurrentCell(CELL_EXPLORED);
        break;

    case FIELD_BLACK_TAPE:
        printf("Black tape detected. Reversing and turning away.\n");

        markCellAhead(CELL_UNSAFE);
        reportFieldEvent(event);

        moveWithRamp(-REVERSE_DISTANCE_CM, DEFAULT_SPEED);
        turn(AVOID_TURN_DEG, DEFAULT_SPEED);
        break;

    case FIELD_HILL:
        printf("Hill/obstacle detected. Reversing and avoiding.\n");

        markCellAhead(CELL_HILL);
        reportFieldEvent(event);

        moveWithRamp(-REVERSE_DISTANCE_CM, DEFAULT_SPEED);
        turn(AVOID_TURN_DEG, DEFAULT_SPEED);
        break;

    case FIELD_ROCK_SAMPLE:
        printf("Rock sample detected. Reporting sample and avoiding contact.\n");

        markCellAhead(CELL_SAMPLE);
        reportFieldEvent(event);

        /*
         * After recognizing a sample, avoid pushing it.
         * Reverse slightly and turn away.
         */
        moveWithRamp(-REVERSE_DISTANCE_CM, DEFAULT_SPEED);
        turn(SAMPLE_AVOID_TURN_DEG, DEFAULT_SPEED);
        break;

    case FIELD_SENSOR_FAULT:
        printf("Sensor fault detected. Stopping and turning to safe direction.\n");

        reportFieldEvent(event);

        /*
         * Conservative behavior:
         * do not continue straight when sensor data is invalid.
         */
        turn(AVOID_TURN_DEG, DEFAULT_SPEED);
        break;

    default:
        printf("Unknown event. Stopping for safety.\n");
        break;
    }
}

// Navigation loop
void runNavigation(void)
{
    initMap();

    printf("Starting autonomous navigation...\n");

    for (int step = 0; step < MAX_NAVIGATION_STEPS; step++)
    {
        printf("\n--- Navigation step %d ---\n", step);
        printPose();

        sensor_data_t sensor_data = readReliableSensorData();
        field_event_t event = interpretSensorData(sensor_data);

        handleFieldEvent(event);
    }

    printf("Navigation finished.\n");
}

// Note that speed ranges from min 2.7 cm/s to max 58 cm/s, set accordingly
int main(void)
{
    pynq_init();

    if (!USE_MOCK_SENSORS)
    {
        gpio_init();
        tcs_init();

        /*
         * Optional debug test for the color sensor.
         * You can comment this out once the OUT pin is confirmed to work.
         */
        debug_out_pin();

        /*
         * In real operation, calibration should be loaded from the base station PC.
         * Example temporary test:
         *
         * handleBaseStationMessage(
         *     "TCSCAL,"
         *     "120,130,125,180,"
         *     "900,870,850,1100,"
         *     "80,75,70,100,"
         *     "1200,300,250,900,"
         *     "300,1100,350,850,"
         *     "250,400,1200,870"
         * );
         */
    }

    stepper_init();
    stepper_enable();

    // Set the robot starting point as 0, 0
    my_pose.x = 0;
    my_pose.y = 0;
    my_pose.yaw = 0; // Assume this?

    runNavigation();

    stepper_destroy();

    if (!USE_MOCK_SENSORS)
    {
        gpio_destroy();
    }

    pynq_destroy();
    return EXIT_SUCCESS;
}