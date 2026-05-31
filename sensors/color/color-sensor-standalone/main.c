#include <libpynq.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/time.h>

/*
 * TCS3200 wiring for this file:
 *
 * TCS3200 VCC    -> 3.3V
 * TCS3200 GND    -> GND
 * TCS3200 S0     -> AR4
 * TCS3200 S1     -> AR5
 * TCS3200 S2     -> AR6
 * TCS3200 S3     -> AR7
 * TCS3200 OUT    -> AR8
 * TCS3200 OE/EN  -> GND
 *
 * Important:
 * - Use 3.3V if OUT is connected directly to PYNQ GPIO.
 * - OE/EN must be connected to GND on most TCS3200 modules.
 */

#define PIN_S0   IO_AR4
#define PIN_S1   IO_AR5
#define PIN_S2   IO_AR6
#define PIN_S3   IO_AR7
#define PIN_OUT  IO_AR8

#define SAMPLE_TIME_MS       150
#define AVERAGE_SAMPLE_COUNT 10
#define SETTLE_TIME_MS       30

/*
 * Set this to 1 if you want to use hard-coded calibration values.
 * Set this to 0 if you want to run the automatic calibration routine.
 */
#define USE_MANUAL_CALIBRATION 0

typedef enum {
    FILTER_RED,
    FILTER_GREEN,
    FILTER_BLUE,
    FILTER_CLEAR
} tcs_filter_t;

typedef enum {
    COLOR_WHITE = 0,
    COLOR_BLACK = 1,
    COLOR_RED   = 2,
    COLOR_GREEN = 3,
    COLOR_BLUE  = 4,
    COLOR_COUNT = 5
} known_color_t;

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

static color_profile_t profiles[COLOR_COUNT] = {
    {"white", {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"black", {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"red",   {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"green", {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}},
    {"blue",  {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}}
};

static bool calibrated = false;
static double white_clear_reference = 1.0;
static double black_clear_threshold = 0.0;

static uint64_t now_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);

    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

static void wait_for_enter(void) {
    int c;

    printf("Press ENTER when ready...");
    fflush(stdout);

    do {
        c = getchar();
    } while (c != '\n' && c != EOF);
}

static void write_pin(io_t pin, bool high) {
    gpio_set_level(pin, high ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
}

static void tcs_select_filter(tcs_filter_t filter) {
    switch (filter) {
        case FILTER_RED:
            /*
             * Red photodiodes:
             * S2 = LOW, S3 = LOW
             */
            write_pin(PIN_S2, false);
            write_pin(PIN_S3, false);
            break;

        case FILTER_BLUE:
            /*
             * Blue photodiodes:
             * S2 = LOW, S3 = HIGH
             */
            write_pin(PIN_S2, false);
            write_pin(PIN_S3, true);
            break;

        case FILTER_CLEAR:
            /*
             * Clear/no-filter photodiodes:
             * S2 = HIGH, S3 = LOW
             */
            write_pin(PIN_S2, true);
            write_pin(PIN_S3, false);
            break;

        case FILTER_GREEN:
            /*
             * Green photodiodes:
             * S2 = HIGH, S3 = HIGH
             */
            write_pin(PIN_S2, true);
            write_pin(PIN_S3, true);
            break;
    }

    sleep_msec(SETTLE_TIME_MS);
}

static double measure_frequency_hz(int sample_time_ms) {
    uint64_t start = now_us();
    uint64_t duration_us = (uint64_t)sample_time_ms * 1000ULL;

    gpio_level_t previous = gpio_get_level(PIN_OUT);
    uint32_t rising_edges = 0;

    while ((now_us() - start) < duration_us) {
        gpio_level_t current = gpio_get_level(PIN_OUT);

        if (previous == GPIO_LEVEL_LOW && current == GPIO_LEVEL_HIGH) {
            rising_edges++;
        }

        previous = current;
    }

    return (double)rising_edges * 1000.0 / (double)sample_time_ms;
}

static void tcs_init(void) {
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
     * Output frequency scaling.
     *
     * S0 = HIGH, S1 = LOW gives 20% scaling.
     * This is usually easier to measure than 2%, but less extreme than 100%.
     */
    write_pin(PIN_S0, true);
    write_pin(PIN_S1, false);
}

static color_reading_t tcs_read_color_once(void) {
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

static color_reading_t tcs_read_color_average(int samples) {
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

static color_feature_t extract_feature(color_reading_t reading) {
    color_feature_t f;

    double rgb_sum = reading.red + reading.green + reading.blue;

    if (rgb_sum <= 0.0) {
        f.red_ratio = 0.0;
        f.green_ratio = 0.0;
        f.blue_ratio = 0.0;
    } else {
        /*
         * These ratios describe hue.
         * They are less sensitive to distance and overall lighting than raw values.
         */
        f.red_ratio   = reading.red   / rgb_sum;
        f.green_ratio = reading.green / rgb_sum;
        f.blue_ratio  = reading.blue  / rgb_sum;
    }

    /*
     * Brightness is normalized to the calibrated white sample.
     * White should be near 1.0.
     * Black should be much lower.
     */
    if (white_clear_reference <= 0.0) {
        f.brightness = 0.0;
    } else {
        f.brightness = reading.clear / white_clear_reference;
    }

    return f;
}

static void print_reading(const char *prefix, color_reading_t r) {
    color_feature_t f = extract_feature(r);

    printf("%s RAW: R=%7.1f  G=%7.1f  B=%7.1f  C=%7.1f | ",
           prefix, r.red, r.green, r.blue, r.clear);

    printf("FEATURE: r=%.3f g=%.3f b=%.3f bright=%.3f\n",
           f.red_ratio, f.green_ratio, f.blue_ratio, f.brightness);
}

static double feature_distance(color_feature_t a, color_feature_t b) {
    double dr = a.red_ratio   - b.red_ratio;
    double dg = a.green_ratio - b.green_ratio;
    double db = a.blue_ratio  - b.blue_ratio;
    double dv = a.brightness  - b.brightness;

    /*
     * Weighted squared distance.
     *
     * Hue is very important for red/green/blue.
     * Brightness is important for separating black and white.
     */
    return
        3.0 * dr * dr +
        3.0 * dg * dg +
        3.0 * db * db +
        1.5 * dv * dv;
}

static void compute_black_threshold(void) {
    double black_clear = profiles[COLOR_BLACK].raw.clear;
    double min_nonblack_clear = profiles[COLOR_WHITE].raw.clear;

    for (int i = 0; i < COLOR_COUNT; i++) {
        if (i == COLOR_BLACK) {
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

static bool calibration_has_signal(void) {
    for (int i = 0; i < COLOR_COUNT; i++) {
        if (profiles[i].raw.red <= 0.0 &&
            profiles[i].raw.green <= 0.0 &&
            profiles[i].raw.blue <= 0.0 &&
            profiles[i].raw.clear <= 0.0) {
            printf("Calibration problem: %s produced no signal.\n", profiles[i].name);
            return false;
        }
    }

    return true;
}

static void print_copy_paste_manual_calibration(void) {
    printf("\n=====================================\n");
    printf("COPY-PASTE MANUAL CALIBRATION BLOCK\n");
    printf("=====================================\n");
    printf("Copy everything below into load_saved_manual_calibration():\n\n");

    printf("    load_manual_calibration(\n");

    printf("        %.1f, %.1f, %.1f, %.1f,     // white: R, G, B, C\n",
           profiles[COLOR_WHITE].raw.red,
           profiles[COLOR_WHITE].raw.green,
           profiles[COLOR_WHITE].raw.blue,
           profiles[COLOR_WHITE].raw.clear);

    printf("        %.1f, %.1f, %.1f, %.1f,     // black: R, G, B, C\n",
           profiles[COLOR_BLACK].raw.red,
           profiles[COLOR_BLACK].raw.green,
           profiles[COLOR_BLACK].raw.blue,
           profiles[COLOR_BLACK].raw.clear);

    printf("        %.1f, %.1f, %.1f, %.1f,     // red: R, G, B, C\n",
           profiles[COLOR_RED].raw.red,
           profiles[COLOR_RED].raw.green,
           profiles[COLOR_RED].raw.blue,
           profiles[COLOR_RED].raw.clear);

    printf("        %.1f, %.1f, %.1f, %.1f,     // green: R, G, B, C\n",
           profiles[COLOR_GREEN].raw.red,
           profiles[COLOR_GREEN].raw.green,
           profiles[COLOR_GREEN].raw.blue,
           profiles[COLOR_GREEN].raw.clear);

    printf("        %.1f, %.1f, %.1f, %.1f      // blue: R, G, B, C\n",
           profiles[COLOR_BLUE].raw.red,
           profiles[COLOR_BLUE].raw.green,
           profiles[COLOR_BLUE].raw.blue,
           profiles[COLOR_BLUE].raw.clear);

    printf("    );\n\n");

    printf("Then set:\n");
    printf("    #define USE_MANUAL_CALIBRATION 1\n");
    printf("and rerun the program.\n");
    printf("=====================================\n\n");
}

static void finalize_calibration(const char *source_name) {
    if (!calibration_has_signal()) {
        printf("\nCalibration failed because at least one sample had no signal.\n");
        printf("Check VCC, GND, OUT, and OE/EN wiring.\n");
        calibrated = false;
        return;
    }

    white_clear_reference = profiles[COLOR_WHITE].raw.clear;

    if (white_clear_reference <= 0.0) {
        white_clear_reference = 1.0;
    }

    for (int i = 0; i < COLOR_COUNT; i++) {
        profiles[i].feature = extract_feature(profiles[i].raw);
    }

    compute_black_threshold();

    calibrated = true;

    if (source_name == NULL) {
        source_name = "unknown source";
    }

    printf("\n=====================================\n");
    printf("CALIBRATION COMPLETE: %s\n", source_name);
    printf("=====================================\n");

    for (int i = 0; i < COLOR_COUNT; i++) {
        printf("%-5s raw: R=%7.1f G=%7.1f B=%7.1f C=%7.1f | ",
               profiles[i].name,
               profiles[i].raw.red,
               profiles[i].raw.green,
               profiles[i].raw.blue,
               profiles[i].raw.clear);

        printf("feature: r=%.3f g=%.3f b=%.3f bright=%.3f\n",
               profiles[i].feature.red_ratio,
               profiles[i].feature.green_ratio,
               profiles[i].feature.blue_ratio,
               profiles[i].feature.brightness);
    }

    printf("\nComputed black clear threshold: %.1f Hz\n", black_clear_threshold);

    print_copy_paste_manual_calibration();

    printf("Now recognition mode starts.\n\n");
}

static void load_manual_calibration(
    double white_r, double white_g, double white_b, double white_c,
    double black_r, double black_g, double black_b, double black_c,
    double red_r,   double red_g,   double red_b,   double red_c,
    double green_r, double green_g, double green_b, double green_c,
    double blue_r,  double blue_g,  double blue_b,  double blue_c)
{
    profiles[COLOR_WHITE].raw.red   = white_r;
    profiles[COLOR_WHITE].raw.green = white_g;
    profiles[COLOR_WHITE].raw.blue  = white_b;
    profiles[COLOR_WHITE].raw.clear = white_c;

    profiles[COLOR_BLACK].raw.red   = black_r;
    profiles[COLOR_BLACK].raw.green = black_g;
    profiles[COLOR_BLACK].raw.blue  = black_b;
    profiles[COLOR_BLACK].raw.clear = black_c;

    profiles[COLOR_RED].raw.red   = red_r;
    profiles[COLOR_RED].raw.green = red_g;
    profiles[COLOR_RED].raw.blue  = red_b;
    profiles[COLOR_RED].raw.clear = red_c;

    profiles[COLOR_GREEN].raw.red   = green_r;
    profiles[COLOR_GREEN].raw.green = green_g;
    profiles[COLOR_GREEN].raw.blue  = green_b;
    profiles[COLOR_GREEN].raw.clear = green_c;

    profiles[COLOR_BLUE].raw.red   = blue_r;
    profiles[COLOR_BLUE].raw.green = blue_g;
    profiles[COLOR_BLUE].raw.blue  = blue_b;
    profiles[COLOR_BLUE].raw.clear = blue_c;

    finalize_calibration("manual values");
}

static void load_saved_manual_calibration(void) {
    /*
     * Replace these values with the values printed after auto calibration.
     *
     * Order:
     *   white R, G, B, C
     *   black R, G, B, C
     *   red   R, G, B, C
     *   green R, G, B, C
     *   blue  R, G, B, C
     */
    load_manual_calibration(
        2110.7, 641.3, 4815.3, 1860.7,     // white: R, G, B, C
        96.0, 82.7, 256.7, 96.0,           // black: R, G, B, C
        1709.3, 578.0, 1976.7, 701.3,      // red: R, G, B, C
        676.7, 800.0, 1630.0, 532.0,       // green: R, G, B, C
        792.0, 542.7, 1403.3, 821.3        // blue: R, G, B, C
    );
}

static void calibrate_one_color(known_color_t color) {
    printf("\n=== Calibrating %s ===\n", profiles[color].name);
    printf("Place the %s block/sample at the normal measuring distance.\n",
           profiles[color].name);
    printf("Keep the sensor height, angle, and lighting the same as during the robot mission.\n");

    wait_for_enter();

    profiles[color].raw = tcs_read_color_average(AVERAGE_SAMPLE_COUNT);

    printf("Calibration result for %s:\n", profiles[color].name);
    print_reading("  ", profiles[color].raw);
}

static void run_calibration(void) {
    printf("\n=====================================\n");
    printf("TCS3200 AUTO CALIBRATION ROUTINE\n");
    printf("=====================================\n");
    printf("You will calibrate these samples:\n");
    printf("1. white\n");
    printf("2. black\n");
    printf("3. red\n");
    printf("4. green\n");
    printf("5. blue\n");
    printf("\nUse the same distance and lighting for every sample.\n");

    calibrate_one_color(COLOR_WHITE);
    calibrate_one_color(COLOR_BLACK);
    calibrate_one_color(COLOR_RED);
    calibrate_one_color(COLOR_GREEN);
    calibrate_one_color(COLOR_BLUE);

    finalize_calibration("auto calibration");
}

static const char *classify_color(color_reading_t reading) {
    if (!calibrated) {
        return "NOT_CALIBRATED";
    }

    if (reading.red <= 0.0 &&
        reading.green <= 0.0 &&
        reading.blue <= 0.0 &&
        reading.clear <= 0.0) {
        return "NO_SIGNAL";
    }

    /*
     * Black is mainly a low-brightness case.
     * This pre-check makes black recognition more stable.
     */
    if (reading.clear < black_clear_threshold) {
        return "black";
    }

    color_feature_t current = extract_feature(reading);

    int best_index = COLOR_WHITE;
    double best_distance = feature_distance(current, profiles[COLOR_WHITE].feature);

    /*
     * We skip black here because black is already handled by brightness.
     * This prevents noisy black hue ratios from interfering.
     */
    for (int i = 0; i < COLOR_COUNT; i++) {
        if (i == COLOR_BLACK) {
            continue;
        }

        double d = feature_distance(current, profiles[i].feature);

        if (d < best_distance) {
            best_distance = d;
            best_index = i;
        }
    }

    return profiles[best_index].name;
}

static void debug_out_pin(void) {
    printf("\nTesting OUT pin on AR8 for 3 seconds...\n");

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
        printf("Check OUT wire, OE/EN -> GND, VCC, GND, and PIN_OUT definition.\n");
    } else {
        printf("OUT pin is toggling. Sensor signal is present.\n");
    }
}

int main(void) {
    pynq_init();
    gpio_init();

    tcs_init();

    debug_out_pin();

#if USE_MANUAL_CALIBRATION
    printf("\nLoading manual calibration.\n");
    load_saved_manual_calibration();
#else
    printf("\nStarting auto calibration.\n");
    run_calibration();
#endif

    if (!calibrated) {
        gpio_destroy();
        pynq_destroy();
        return EXIT_FAILURE;
    }

    while (1) {
        color_reading_t reading = tcs_read_color_average(3);
        const char *color = classify_color(reading);

        print_reading("LIVE", reading);
        printf("CLASSIFIED COLOR: %s\n", color);
        printf("-------------------------------------\n");

        sleep_msec(500);
    }

    gpio_destroy();
    pynq_destroy();

    return EXIT_SUCCESS;
}