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
 * - manually loading calibration values
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

typedef enum {
    FILTER_RED,
    FILTER_GREEN,
    FILTER_BLUE,
    FILTER_CLEAR
} tcs_filter_t;

typedef enum {
    TCS_COLOR_WHITE = 0,
    TCS_COLOR_BLACK = 1,
    TCS_COLOR_RED   = 2,
    TCS_COLOR_GREEN = 3,
    TCS_COLOR_BLUE  = 4,
    TCS_COLOR_COUNT = 5
} tcs_color_t;

typedef struct {
    double red;
    double green;
    double blue;
    double clear;
} color_reading_t;

typedef struct {
    double red_ratio;
    double green_ratio;
    double blue_ratio;
    double brightness;
} color_feature_t;

typedef struct {
    const char *name;
    color_reading_t raw;
    color_feature_t feature;
} color_profile_t;

static color_profile_t profiles[TCS_COLOR_COUNT] = {
    {"white", {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"black", {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"red",   {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"green", {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"blue",  {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}}
};

static bool calibrated = false;

static double white_clear_reference = 1.0;
static double black_clear_threshold = 0.0;

static double now_msec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

static void write_pin(io_t pin, bool high)
{
    gpio_set_level(pin, high ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
}

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

    sleep_msec(SETTLE_TIME_MS);
}

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

    return (double)rising_edges * 1000.0 / (double)sample_time_ms;
}

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
     * Frequency scaling:
     * S0 = HIGH, S1 = LOW gives 20% scaling.
     */
    write_pin(PIN_S0, true);
    write_pin(PIN_S1, false);
}

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

static void compute_black_threshold(void)
{
    double black_clear = profiles[TCS_COLOR_BLACK].raw.clear;
    double min_nonblack_clear = profiles[TCS_COLOR_WHITE].raw.clear;

    for (int i = 0; i < TCS_COLOR_COUNT; i++) {
        if (i == TCS_COLOR_BLACK) {
            continue;
        }

        if (profiles[i].raw.clear < min_nonblack_clear) {
            min_nonblack_clear = profiles[i].raw.clear;
        }
    }

    /*
     * Anything darker than halfway between calibrated black and the darkest
     * non-black sample is classified as black.
     */
    black_clear_threshold = (black_clear + min_nonblack_clear) / 2.0;
}

static bool calibration_has_signal(void)
{
    for (int color = 0; color < TCS_COLOR_COUNT; color++) {
        if (profiles[color].raw.red <= 0.0 &&
            profiles[color].raw.green <= 0.0 &&
            profiles[color].raw.blue <= 0.0 &&
            profiles[color].raw.clear <= 0.0) {

            printf("TCS calibration error: %s profile has no signal.\n",
                   profiles[color].name);

            return false;
        }
    }

    return true;
}

static bool tcs3200ApplyCalibrationValues(const double values[20],
                                          const char *source_name)
{
    if (values == NULL) {
        calibrated = false;
        return false;
    }

    /*
     * Order:
     * white, black, red, green, blue
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

    if (!calibration_has_signal()) {
        calibrated = false;
        return false;
    }

    white_clear_reference = profiles[TCS_COLOR_WHITE].raw.clear;

    if (white_clear_reference <= 0.0) {
        white_clear_reference = 1.0;
    }

    for (int color = 0; color < TCS_COLOR_COUNT; color++) {
        profiles[color].feature = extract_feature(profiles[color].raw);
    }

    compute_black_threshold();

    calibrated = true;

    if (source_name == NULL) {
        source_name = "unknown source";
    }

    printf("TCS3200 calibration loaded from %s.\n", source_name);

    for (int color = 0; color < TCS_COLOR_COUNT; color++) {
        printf("%-5s ", profiles[color].name);
        print_full_reading("", profiles[color].raw);
    }

    printf("Computed black clear threshold: %.1f Hz\n", black_clear_threshold);

    return true;
}

static tcs_color_t classify_color_enum(color_reading_t reading)
{
    /*
     * Black is mainly a low-brightness case.
     * This pre-check makes black recognition more stable.
     */
    if (reading.clear < black_clear_threshold) {
        return TCS_COLOR_BLACK;
    }

    color_feature_t current = extract_feature(reading);

    tcs_color_t best_color = TCS_COLOR_WHITE;
    double best_distance =
        feature_distance(current, profiles[TCS_COLOR_WHITE].feature);

    /*
     * Skip black here because black is already handled by brightness.
     * This prevents noisy black hue ratios from interfering.
     */
    for (int i = 0; i < TCS_COLOR_COUNT; i++) {
        if (i == TCS_COLOR_BLACK) {
            continue;
        }

        double d = feature_distance(current, profiles[i].feature);

        if (d < best_distance) {
            best_distance = d;
            best_color = (tcs_color_t)i;
        }
    }

    return best_color;
}

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

        default:
            return COLOR_UNKNOWN;
    }
}

static const char *tcsColorToString(tcs_color_t color)
{
    switch (color) {
        case TCS_COLOR_WHITE: return "white";
        case TCS_COLOR_BLACK: return "black";
        case TCS_COLOR_RED:   return "red";
        case TCS_COLOR_GREEN: return "green";
        case TCS_COLOR_BLUE:  return "blue";
        default:              return "unknown";
    }
}

bool isTCS3200Calibrated(void)
{
    return calibrated;
}

sample_color_t classifyTCS3200Color(void)
{
    if (!calibrated) {
        printf("TCS3200 not calibrated. Cannot classify color.\n");
        return COLOR_UNKNOWN;
    }

    color_reading_t reading = tcs_read_color_average(3);

    if (reading.red <= 0.0 &&
        reading.green <= 0.0 &&
        reading.blue <= 0.0 &&
        reading.clear <= 0.0) {
        printf("TCS3200 no signal. Cannot classify color.\n");
        return COLOR_UNKNOWN;
    }

    tcs_color_t detected = classify_color_enum(reading);

    printf("TCS SAMPLE classified color: %s\n", tcsColorToString(detected));

    return tcsColorToSampleColor(detected);
}

bool tcs3200DetectBlackTape(void)
{
    if (!calibrated) {
        printf("TCS3200 not calibrated. Cannot detect black tape.\n");
        return false;
    }

    color_reading_t reading = tcs_read_color_average(3);

    if (reading.red <= 0.0 &&
        reading.green <= 0.0 &&
        reading.blue <= 0.0 &&
        reading.clear <= 0.0) {
        printf("TCS3200 no signal. Cannot detect black tape.\n");
        return false;
    }

    tcs_color_t detected = classify_color_enum(reading);
    bool black_tape_detected = reading.clear < black_clear_threshold;

    printf("TCS TAPE classified color: %s | black_tape=%s\n",
           tcsColorToString(detected),
           black_tape_detected ? "yes" : "no");

    return black_tape_detected;
}

bool tcs3200LoadManualCalibration(
    double white_r, double white_g, double white_b, double white_c,
    double black_r, double black_g, double black_b, double black_c,
    double red_r,   double red_g,   double red_b,   double red_c,
    double green_r, double green_g, double green_b, double green_c,
    double blue_r,  double blue_g,  double blue_b,  double blue_c)
{
    double values[20] = {
        white_r, white_g, white_b, white_c,
        black_r, black_g, black_b, black_c,
        red_r,   red_g,   red_b,   red_c,
        green_r, green_g, green_b, green_c,
        blue_r,  blue_g,  blue_b,  blue_c
    };

    return tcs3200ApplyCalibrationValues(values, "manual values");
}

/*
 * Loads TCS3200 calibration profiles received from the base station.
 *
 * Expected message format:
 *
 *     TCSCAL,
 *     white_R,white_G,white_B,white_C,
 *     black_R,black_G,black_B,black_C,
 *     red_R,red_G,red_B,red_C,
 *     green_R,green_G,green_B,green_C,
 *     blue_R,blue_G,blue_B,blue_C
 *
 * There are 20 numeric values:
 *
 *     5 profiles * 4 channels = 20 values
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
    double values[20];

    for (int i = 0; i < 20; i++) {
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

    return tcs3200ApplyCalibrationValues(values, "base station");
}

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