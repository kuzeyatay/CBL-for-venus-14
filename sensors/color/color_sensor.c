#include <libpynq.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

#include "config.h"
#include "robot_types.h"
#include "color_sensor.h"

/*
 * ============================================================
 * TCS3200 COLOR SENSOR MODULE
 * ============================================================
 *
 * This file handles:
 * - TCS3200 GPIO initialization
 * - red/green/blue/clear frequency measurement
 * - calibrated color classification
 * - black-tape detection
 * - loading calibration profiles from the base station
 * - sending raw calibration readings
 *
 * Wiring:
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


/*
 * The TCS3200 lets the software choose which group of photodiodes is active.
 *
 * The selected filter determines what kind of light is measured:
 * - red-filtered photodiodes
 * - green-filtered photodiodes
 * - blue-filtered photodiodes
 * - clear/no-filter photodiodes
 */
typedef enum {
    FILTER_RED,
    FILTER_GREEN,
    FILTER_BLUE,
    FILTER_CLEAR
} tcs_filter_t;


/*
 * Internal TCS3200 color labels.
 *
 * These are separate from sample_color_t because sample_color_t belongs to the
 * higher-level robot/navigation system.
 */
typedef enum {
    TCS_COLOR_NONE  = 0,
    TCS_COLOR_WHITE = 1,
    TCS_COLOR_BLACK = 2,
    TCS_COLOR_RED   = 3,
    TCS_COLOR_GREEN = 4,
    TCS_COLOR_BLUE  = 5,
    TCS_COLOR_COUNT = 6
} tcs_color_t;


/*
 * Raw frequency reading from the TCS3200.
 *
 * Each field is a measured frequency in Hz.
 */
typedef struct {
    double red;
    double green;
    double blue;
    double clear;
} color_reading_t;


/*
 * Normalized color features used for classification.
 *
 * red_ratio, green_ratio, blue_ratio:
 *     Describe the relative color composition.
 *
 * brightness:
 *     Clear-channel brightness relative to calibrated white.
 */
typedef struct {
    double red_ratio;
    double green_ratio;
    double blue_ratio;
    double brightness;
} color_feature_t;


/*
 * A calibrated color profile.
 *
 * raw:
 *     The original measured R/G/B/C frequencies for that known color.
 *
 * feature:
 *     The normalized version used for comparison during live classification.
 */
typedef struct {
    const char *name;
    color_reading_t raw;
    color_feature_t feature;
} color_profile_t;


/*
 * Calibration profiles.
 *
 * These are filled when the base station sends a TCSCAL message.
 */
static color_profile_t profiles[TCS_COLOR_COUNT] = {
    {"none",  {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"white", {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"black", {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"red",   {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"green", {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"blue",  {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}}
};


/*
 * True after valid calibration data has been loaded.
 */
static bool calibrated = false;


/*
 * Clear-channel value of the calibrated white sample.
 *
 * Used as brightness reference.
 */
static double white_clear_reference = 1.0;


/*
 * Maximum feature-distance from the calibrated "none/background" profile
 * before a live reading is considered a real object color.
 */
static double none_radius = 0.05;


/*
 * Returns monotonic time in milliseconds.
 *
 * Monotonic time is used because it only moves forward and is therefore better
 * for elapsed-time measurements than wall-clock time.
 */
static double now_msec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}


/*
 * Writes HIGH or LOW to one TCS3200 control pin.
 */
static void write_pin(io_t pin, bool high)
{
    gpio_set_level(pin, high ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
}


/*
 * Selects which photodiode filter the TCS3200 uses.
 *
 * Filter selection table:
 *
 * S2 = LOW,  S3 = LOW  -> red
 * S2 = LOW,  S3 = HIGH -> blue
 * S2 = HIGH, S3 = LOW  -> clear
 * S2 = HIGH, S3 = HIGH -> green
 */
static void tcs_select_filter(tcs_filter_t filter)
{
    switch (filter) {
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

    /*
     * Give the sensor output time to settle after switching filters.
     */
    sleep_msec(SETTLE_TIME_MS);
}


/*
 * Measures the frequency on the TCS3200 OUT pin.
 *
 * The TCS3200 outputs a square wave.
 * Higher frequency means more reflected light for the selected filter.
 */
static double measure_frequency_hz(int sample_time_ms)
{
    double start = now_msec();

    gpio_level_t previous = gpio_get_level(PIN_OUT);
    uint32_t rising_edges = 0;

    while ((now_msec() - start) < sample_time_ms) {
        gpio_level_t current = gpio_get_level(PIN_OUT);

        if (previous == GPIO_LEVEL_LOW && current == GPIO_LEVEL_HIGH) {
            rising_edges++;
        }

        previous = current;
    }

    /*
     * Frequency = cycles / seconds.
     *
     * Since sample_time_ms is in milliseconds:
     *
     * frequency_hz = rising_edges * 1000 / sample_time_ms
     */
    return (double)rising_edges * 1000.0 / (double)sample_time_ms;
}


/*
 * Initializes the TCS3200 GPIO pins.
 *
 * Call once after:
 *
 *     pynq_init();
 *     gpio_init();
 */
void tcs3200Init(void)
{
    switchbox_set_pin(PIN_S0,  SWB_GPIO);
    switchbox_set_pin(PIN_S1,  SWB_GPIO);
    switchbox_set_pin(PIN_S2,  SWB_GPIO);
    switchbox_set_pin(PIN_S3,  SWB_GPIO);
    switchbox_set_pin(PIN_OUT, SWB_GPIO);

    gpio_set_direction(PIN_S0,  GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_S1,  GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_S2,  GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_S3,  GPIO_DIR_OUTPUT);
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
 * Reads one complete raw color measurement.
 *
 * The function measures:
 * - red-filter response
 * - green-filter response
 * - blue-filter response
 * - clear/no-filter response
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
 * Reads the color sensor multiple times and returns the average.
 *
 * Averaging reduces noise from small lighting changes and timing variation.
 */
static color_reading_t tcs_read_color_average(int samples)
{
    color_reading_t sum = {0.0, 0.0, 0.0, 0.0};

    if (samples <= 0) {
        samples = 1;
    }

    for (int i = 0; i < samples; i++) {
        color_reading_t r = tcs_read_color_once();

        sum.red   += r.red;
        sum.green += r.green;
        sum.blue  += r.blue;
        sum.clear += r.clear;

        sleep_msec(50);
    }

    sum.red   /= samples;
    sum.green /= samples;
    sum.blue  /= samples;
    sum.clear /= samples;

    return sum;
}


/*
 * Converts raw R/G/B/C frequency values into normalized features.
 */
static color_feature_t extract_feature(color_reading_t reading)
{
    color_feature_t f;

    double rgb_sum = reading.red + reading.green + reading.blue;

    if (rgb_sum <= 0.0) {
        f.red_ratio = 0.0;
        f.green_ratio = 0.0;
        f.blue_ratio = 0.0;
    } else {
        f.red_ratio   = reading.red   / rgb_sum;
        f.green_ratio = reading.green / rgb_sum;
        f.blue_ratio  = reading.blue  / rgb_sum;
    }

    if (white_clear_reference <= 0.0) {
        f.brightness = 0.0;
    } else {
        f.brightness = reading.clear / white_clear_reference;
    }

    return f;
}


/*
 * Prints only the raw TCS3200 reading.
 */
static void print_raw_reading(const char *prefix, color_reading_t r)
{
    printf("%s RAW: R=%7.1f  G=%7.1f  B=%7.1f  C=%7.1f\n",
           prefix,
           r.red,
           r.green,
           r.blue,
           r.clear);
}


/*
 * Prints raw values and normalized features.
 */
static void print_full_reading(const char *prefix, color_reading_t r)
{
    color_feature_t f = extract_feature(r);

    printf("%s RAW: R=%7.1f  G=%7.1f  B=%7.1f  C=%7.1f | ",
           prefix,
           r.red,
           r.green,
           r.blue,
           r.clear);

    printf("FEATURE: r=%.3f g=%.3f b=%.3f bright=%.3f\n",
           f.red_ratio,
           f.green_ratio,
           f.blue_ratio,
           f.brightness);
}


/*
 * Computes weighted squared distance between two normalized color features.
 *
 * RGB ratios mainly describe color/hue.
 * Brightness helps distinguish black, white, and background.
 */
static double feature_distance(color_feature_t a, color_feature_t b)
{
    double dr = a.red_ratio   - b.red_ratio;
    double dg = a.green_ratio - b.green_ratio;
    double db = a.blue_ratio  - b.blue_ratio;
    double dv = a.brightness  - b.brightness;

    return
        3.0 * dr * dr +
        3.0 * dg * dg +
        3.0 * db * db +
        2.0 * dv * dv;
}


/*
 * Computes the "none/background" radius.
 *
 * If a live reading is very close to the calibrated background reading, it is
 * classified as TCS_COLOR_NONE instead of being forced into a real color.
 */
static void compute_none_radius(void)
{
    color_feature_t none_feature = profiles[TCS_COLOR_NONE].feature;

    double min_distance =
        feature_distance(none_feature, profiles[TCS_COLOR_WHITE].feature);

    for (int i = TCS_COLOR_BLACK; i < TCS_COLOR_COUNT; i++) {
        double d = feature_distance(none_feature, profiles[i].feature);

        if (d < min_distance) {
            min_distance = d;
        }
    }

    /*
     * Bigger multiplier:
     *     More likely to classify live readings as "none".
     *
     * Smaller multiplier:
     *     More likely to classify live readings as a real color.
     */
    none_radius = min_distance * 0.45;

    if (none_radius < 0.02) {
        none_radius = 0.02;
    }
}


/*
 * Classifies a raw reading using the loaded calibration profiles.
 */
static tcs_color_t classify_color_enum(color_reading_t reading)
{
    if (!calibrated) {
        return TCS_COLOR_NONE;
    }

    if (reading.red <= 0.0 &&
        reading.green <= 0.0 &&
        reading.blue <= 0.0 &&
        reading.clear <= 0.0) {
        return TCS_COLOR_NONE;
    }

    color_feature_t current = extract_feature(reading);

    /*
     * First check whether the reading is close to background.
     */
    double d_none = feature_distance(current, profiles[TCS_COLOR_NONE].feature);

    if (d_none < none_radius) {
        return TCS_COLOR_NONE;
    }

    /*
     * Then choose the closest real calibrated color.
     */
    tcs_color_t best_color = TCS_COLOR_WHITE;
    double best_distance =
        feature_distance(current, profiles[TCS_COLOR_WHITE].feature);

    for (int i = TCS_COLOR_BLACK; i < TCS_COLOR_COUNT; i++) {
        double d = feature_distance(current, profiles[i].feature);

        if (d < best_distance) {
            best_distance = d;
            best_color = (tcs_color_t)i;
        }
    }

    return best_color;
}


/*
 * Converts internal TCS color enum to the navigation/sample color enum.
 */
static sample_color_t tcsColorToSampleColor(tcs_color_t color)
{
    switch (color) {
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
 * Returns whether TCS3200 calibration has been successfully loaded.
 */
bool isTCS3200Calibrated(void)
{
    return calibrated;
}


/*
 * Classifies the current object/sample color.
 *
 * This should be used when the navigation code has already detected a front
 * object and wants to know its color.
 */
sample_color_t classifyTCS3200Color(void)
{
    if (!calibrated) {
        printf("TCS3200 not calibrated. Cannot classify color.\n");
        return COLOR_UNKNOWN;
    }

    color_reading_t reading = tcs_read_color_average(3);

    print_full_reading("TCS SAMPLE", reading);

    tcs_color_t detected = classify_color_enum(reading);

    return tcsColorToSampleColor(detected);
}


/*
 * Detects whether the current TCS3200 reading matches the calibrated black
 * profile.
 *
 * Important:
 * This function only says "the sensor sees black".
 *
 * It does not prove that the black surface is tape.
 * The navigation layer must combine this with distance context:
 *
 *     black + no close front object -> black tape / cliff / boundary
 *     black + close front object    -> possible black rock sample
 */
bool tcs3200DetectBlackTape(void)
{
    if (!calibrated) {
        printf("TCS3200 not calibrated. Cannot detect black tape.\n");
        return false;
    }

    color_reading_t reading = tcs_read_color_average(3);

    print_full_reading("TCS TAPE", reading);

    tcs_color_t detected = classify_color_enum(reading);

    return detected == TCS_COLOR_BLACK;
}


/*
 * Loads TCS3200 calibration profiles received from the base station.
 *
 * Expected message format:
 *
 *     TCSCAL,
 *     none_R,none_G,none_B,none_C,
 *     white_R,white_G,white_B,white_C,
 *     black_R,black_G,black_B,black_C,
 *     red_R,red_G,red_B,red_C,
 *     green_R,green_G,green_B,green_C,
 *     blue_R,blue_G,blue_B,blue_C
 *
 * There are 24 numeric values:
 *
 *     6 profiles * 4 channels = 24 values
 */
bool tcs3200LoadCalibrationFromBaseStation(const char *message)
{
    if (message == NULL) {
        return false;
    }

    if (strncmp(message, "TCSCAL,", 7) != 0) {
        return false;
    }

    const char *p = message + 7;
    double values[24];

    for (int i = 0; i < 24; i++) {
        char *endptr;

        values[i] = strtod(p, &endptr);

        if (endptr == p) {
            printf("TCS calibration parse error at value %d.\n", i);
            calibrated = false;
            return false;
        }

        p = endptr;

        if (*p == ',') {
            p++;
        }
    }

    /*
     * Store values in profile table.
     *
     * Order:
     * none, white, black, red, green, blue
     *
     * Channels per color:
     * R, G, B, C
     */
    for (int color = 0; color < TCS_COLOR_COUNT; color++) {
        int base = color * 4;

        profiles[color].raw.red   = values[base + 0];
        profiles[color].raw.green = values[base + 1];
        profiles[color].raw.blue  = values[base + 2];
        profiles[color].raw.clear = values[base + 3];
    }

    /*
     * Check that real sample colors have some signal.
     *
     * The "none" background profile may be low, but the real color profiles
     * should not all be exactly zero.
     */
    for (int color = TCS_COLOR_WHITE; color < TCS_COLOR_COUNT; color++) {
        if (profiles[color].raw.red <= 0.0 &&
            profiles[color].raw.green <= 0.0 &&
            profiles[color].raw.blue <= 0.0 &&
            profiles[color].raw.clear <= 0.0) {

            printf("TCS calibration error: %s profile has no signal.\n",
                   profiles[color].name);

            calibrated = false;
            return false;
        }
    }

    /*
     * Use calibrated white as brightness reference.
     */
    white_clear_reference = profiles[TCS_COLOR_WHITE].raw.clear;

    if (white_clear_reference <= 0.0) {
        white_clear_reference = 1.0;
    }

    /*
     * Convert all raw calibration readings to normalized features.
     */
    for (int color = 0; color < TCS_COLOR_COUNT; color++) {
        profiles[color].feature = extract_feature(profiles[color].raw);
    }

    /*
     * Compute the background/none radius.
     */
    compute_none_radius();

    calibrated = true;

    printf("TCS3200 calibration loaded from base station.\n");

    for (int color = 0; color < TCS_COLOR_COUNT; color++) {
        printf("%-5s ", profiles[color].name);
        print_full_reading("", profiles[color].raw);
    }

    printf("Computed none radius: %.5f\n", none_radius);

    return true;
}


/*
 * Takes a raw calibration reading and prints/sends it in protocol format.
 *
 * Intended command from base station:
 *
 *     CAL_READ,red
 *
 * Response format:
 *
 *     CALRAW,red,R,G,B,C
 *
 * For now this function prints the payload.
 * Later, replace the printf with sendPayloadToESP32(payload), or call this
 * function from communication.c and send the returned/built payload there.
 */
void tcs3200SendCalibrationReading(const char *color_name)
{
    if (color_name == NULL) {
        color_name = "unknown";
    }

    color_reading_t reading = tcs_read_color_average(AVERAGE_SAMPLE_COUNT);

    printf("CALRAW,%s,%.2f,%.2f,%.2f,%.2f\n",
           color_name,
           reading.red,
           reading.green,
           reading.blue,
           reading.clear);
}


/*
 * Debug helper.
 *
 * Checks whether the TCS3200 OUT pin is toggling.
 * This helps detect wiring issues.
 */
void tcs3200DebugOutPin(void)
{
    printf("\nTesting TCS3200 OUT pin on AR8 for 3 seconds...\n");

    tcs_select_filter(FILTER_CLEAR);

    gpio_level_t previous = gpio_get_level(PIN_OUT);
    int changes = 0;

    for (int i = 0; i < 3000; i++) {
        gpio_level_t current = gpio_get_level(PIN_OUT);

        if (current != previous) {
            changes++;
            previous = current;
        }

        sleep_msec(1);
    }

    printf("OUT pin changes in 3 seconds: %d\n", changes);
    printf("Final OUT level: %d\n", gpio_get_level(PIN_OUT));

    if (changes == 0) {
        printf("WARNING: OUT did not toggle.\n");
        printf("Check OUT wire, OE/EN -> GND, VCC, GND, and PIN_OUT.\n");
    } else {
        printf("OUT pin is toggling. Sensor signal is present.\n");
    }
}