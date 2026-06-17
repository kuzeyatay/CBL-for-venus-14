#include <libpynq.h>

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/time.h>

#include "config.h"
#include "odometry.h"
#include "motion.h"
#include "distance_sensor.h"

/*
 * VL53L0X distance sensor driver for TU/e 5EID0 PYNQ / libpynq.
 *
 * Wiring:
 *   VL53L0X VIN -> 5V
 *   VL53L0X GND -> GND
 *   VL53L0X SCL -> AR_SCL
 *   VL53L0X SDA -> AR_SDA
 *
 * Important:
 *   libpynq uses the 7-bit I2C address.
 *   VL53L0X address = 0x29.
 *   Do NOT use 0x52 here.
 */

#define VL53L0X_ADDR 0x29

#define IDENTIFICATION_MODEL_ID                     0xC0
#define IDENTIFICATION_REVISION_ID                  0xC2

#define SYSRANGE_START                              0x00
#define SYSTEM_SEQUENCE_CONFIG                      0x01
#define SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A
#define SYSTEM_INTERRUPT_CLEAR                      0x0B
#define RESULT_INTERRUPT_STATUS                     0x13
#define RESULT_RANGE_STATUS                         0x14
#define MSRC_CONFIG_CONTROL                         0x60
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define GPIO_HV_MUX_ACTIVE_HIGH                     0x84

#define VL53L0X_TIMEOUT_MS 1000

static uint8_t stop_variable = 0;
static bool vl53l0x_initialized = false;
static uint8_t last_range_status = 0;

/* ---------------- Time helpers ---------------- */

static uint64_t now_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);

    return ((uint64_t)tv.tv_sec * 1000ULL) + ((uint64_t)tv.tv_usec / 1000ULL);
}

static bool timed_out(uint64_t start_ms) {
    return (now_ms() - start_ms) > VL53L0X_TIMEOUT_MS;
}

/* ---------------- I2C helpers ---------------- */

static bool vl_write_u8(uint8_t reg, uint8_t value) {
    return iic_write_register(IIC0, VL53L0X_ADDR, reg, &value, 1) == 0;
}

static bool vl_read_u8(uint8_t reg, uint8_t *value) {
    return iic_read_register(IIC0, VL53L0X_ADDR, reg, value, 1) == 0;
}

static bool vl_write_multi(uint8_t reg, const uint8_t *data, uint16_t len) {
    return iic_write_register(IIC0, VL53L0X_ADDR, reg, (uint8_t *)data, len) == 0;
}

static bool vl_read_multi(uint8_t reg, uint8_t *data, uint16_t len) {
    return iic_read_register(IIC0, VL53L0X_ADDR, reg, data, len) == 0;
}

static bool vl_write_u16_be(uint8_t reg, uint16_t value) {
    uint8_t data[2];

    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)(value & 0xFF);

    return vl_write_multi(reg, data, 2);
}

static bool vl_read_u16_be(uint8_t reg, uint16_t *value) {
    uint8_t data[2];

    if (!vl_read_multi(reg, data, 2)) {
        return false;
    }

    *value = ((uint16_t)data[0] << 8) | data[1];

    return true;
}

/* ---------------- Register table helpers ---------------- */

struct reg_value {
    uint8_t reg;
    uint8_t value;
};

static bool vl_write_reg_table(const struct reg_value *table, size_t n) {
    for (size_t i = 0; i < n; i++) {
        if (!vl_write_u8(table[i].reg, table[i].value)) {
            printf("VL53L0X: I2C write failed at register 0x%02X\n", table[i].reg);
            return false;
        }
    }

    return true;
}

static bool vl_set_signal_rate_limit_0p25_mcps(void) {
    /*
     * Register uses 9.7 fixed-point format.
     * 0.25 MCPS = 0.25 * 128 = 32.
     */
    return vl_write_u16_be(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32);
}

static bool vl_load_tuning_settings(void) {
    /*
     * Common VL53L0X default tuning settings used by lightweight drivers.
     */
    static const struct reg_value tuning[] = {
        {0xFF, 0x01}, {0x00, 0x00},

        {0xFF, 0x00}, {0x09, 0x00}, {0x10, 0x00}, {0x11, 0x00},
        {0x24, 0x01}, {0x25, 0xFF}, {0x75, 0x00},

        {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00}, {0x30, 0x20},

        {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00}, {0x31, 0x04},
        {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25}, {0x60, 0x00},
        {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00}, {0x52, 0x96},
        {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00}, {0x62, 0x00},
        {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},

        {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
        {0x4A, 0x00},

        {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00}, {0x78, 0x21},

        {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00}, {0x44, 0xFF},
        {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40}, {0x0E, 0x06},
        {0x20, 0x1A}, {0x43, 0x40},

        {0xFF, 0x00}, {0x34, 0x03}, {0x35, 0x44},

        {0xFF, 0x01}, {0x31, 0x04}, {0x4B, 0x09}, {0x4C, 0x05},
        {0x4D, 0x04},

        {0xFF, 0x00}, {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08},
        {0x48, 0x28}, {0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01},
        {0x72, 0xFE}, {0x76, 0x00}, {0x77, 0x00},

        {0xFF, 0x01}, {0x0D, 0x01},

        {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8},

        {0xFF, 0x01}, {0x8E, 0x01},

        {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
    };

    return vl_write_reg_table(tuning, sizeof(tuning) / sizeof(tuning[0]));
}

/* ---------------- Reference calibration ---------------- */

static bool vl_perform_single_ref_calibration(uint8_t vhv_init_byte) {
    uint8_t sysrange = 0;
    uint64_t start;

    if (!vl_write_u8(SYSRANGE_START, 0x01 | vhv_init_byte)) {
        return false;
    }

    start = now_ms();

    while (true) {
        if (!vl_read_u8(SYSRANGE_START, &sysrange)) {
            return false;
        }

        /*
         * Calibration is finished when bit 0 clears.
         */
        if ((sysrange & 0x01) == 0) {
            break;
        }

        if (timed_out(start)) {
            printf("VL53L0X: reference calibration timeout. SYSRANGE_START = 0x%02X\n",
                   sysrange);
            return false;
        }

        sleep_msec(10);
    }

    if (!vl_write_u8(SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        return false;
    }

    return true;
}

/* ---------------- Public init function ---------------- */

bool initVL53L0X(void) {
    uint8_t model_id = 0;
    uint8_t revision_id = 0;
    uint8_t tmp = 0;

    printf("VL53L0X: initializing distance sensor...\n");

    /*
     * Configure AR_SCL / AR_SDA for IIC0 inside the distance driver.
     * main.c does not need to do this separately.
     */
    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);

    iic_init(IIC0);
    sleep_msec(300);

    if (!vl_read_u8(IDENTIFICATION_MODEL_ID, &model_id)) {
        printf("VL53L0X: could not read model ID. Check wiring.\n");
        vl53l0x_initialized = false;
        return false;
    }

    if (!vl_read_u8(IDENTIFICATION_REVISION_ID, &revision_id)) {
        printf("VL53L0X: could not read revision ID.\n");
        vl53l0x_initialized = false;
        return false;
    }

    printf("VL53L0X: model ID 0xC0 = 0x%02X", model_id);

    if (model_id == 0xEE) {
        printf(" OK\n");
    } else {
        printf(" unexpected, continuing anyway\n");
    }

    printf("VL53L0X: revision ID 0xC2 = 0x%02X\n", revision_id);

    /*
     * Basic setup sequence.
     */
    if (!vl_write_u8(0x88, 0x00)) return false;

    /*
     * Read stop variable.
     */
    if (!vl_write_u8(0x80, 0x01)) return false;
    if (!vl_write_u8(0xFF, 0x01)) return false;
    if (!vl_write_u8(0x00, 0x00)) return false;

    if (!vl_read_u8(0x91, &stop_variable)) {
        printf("VL53L0X: failed to read stop_variable.\n");
        vl53l0x_initialized = false;
        return false;
    }

    if (!vl_write_u8(0x00, 0x01)) return false;
    if (!vl_write_u8(0xFF, 0x00)) return false;
    if (!vl_write_u8(0x80, 0x00)) return false;

    /*
     * Disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks.
     */
    if (!vl_read_u8(MSRC_CONFIG_CONTROL, &tmp)) return false;
    if (!vl_write_u8(MSRC_CONFIG_CONTROL, tmp | 0x12)) return false;

    if (!vl_set_signal_rate_limit_0p25_mcps()) {
        printf("VL53L0X: failed to set signal rate limit.\n");
        vl53l0x_initialized = false;
        return false;
    }

    /*
     * Enable all sequence steps while loading settings.
     */
    if (!vl_write_u8(SYSTEM_SEQUENCE_CONFIG, 0xFF)) return false;

    printf("VL53L0X: loading tuning settings...\n");

    if (!vl_load_tuning_settings()) {
        printf("VL53L0X: failed while loading tuning settings.\n");
        vl53l0x_initialized = false;
        return false;
    }

    /*
     * Configure interrupt behavior.
     */
    if (!vl_write_u8(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)) return false;

    if (!vl_read_u8(GPIO_HV_MUX_ACTIVE_HIGH, &tmp)) return false;
    if (!vl_write_u8(GPIO_HV_MUX_ACTIVE_HIGH, tmp & ~0x10)) return false;

    if (!vl_write_u8(SYSTEM_INTERRUPT_CLEAR, 0x01)) return false;

    /*
     * Reference calibrations.
     * These are needed to avoid constant invalid 8191 mm readings.
     * They are timeout-protected, so the robot should not hang forever.
     */
    printf("VL53L0X: running VHV reference calibration...\n");

    if (!vl_write_u8(SYSTEM_SEQUENCE_CONFIG, 0x01)) return false;

    if (!vl_perform_single_ref_calibration(0x40)) {
        printf("VL53L0X: VHV reference calibration failed.\n");
        vl53l0x_initialized = false;
        return false;
    }

    printf("VL53L0X: running phase reference calibration...\n");

    if (!vl_write_u8(SYSTEM_SEQUENCE_CONFIG, 0x02)) return false;

    if (!vl_perform_single_ref_calibration(0x00)) {
        printf("VL53L0X: phase reference calibration failed.\n");
        vl53l0x_initialized = false;
        return false;
    }

    /*
     * Restore final ranging sequence.
     */
    if (!vl_write_u8(SYSTEM_SEQUENCE_CONFIG, 0xE8)) return false;

    printf("VL53L0X: reference calibration finished.\n");
    printf("VL53L0X: init complete.\n");

    vl53l0x_initialized = true;
    return true;
}

/* ---------------- Single distance reading ---------------- */

static bool vl53l0x_read_single_mm(uint16_t *range_mm, uint8_t *range_status_out) {
    uint8_t status = 0;
    uint8_t range_status = 0;
    uint64_t start;

    *range_mm = 0;
    *range_status_out = 0;

    /*
     * Prepare for single-shot measurement.
     */
    if (!vl_write_u8(0x80, 0x01)) return false;
    if (!vl_write_u8(0xFF, 0x01)) return false;
    if (!vl_write_u8(0x00, 0x00)) return false;
    if (!vl_write_u8(0x91, stop_variable)) return false;
    if (!vl_write_u8(0x00, 0x01)) return false;
    if (!vl_write_u8(0xFF, 0x00)) return false;
    if (!vl_write_u8(0x80, 0x00)) return false;

    /*
     * Clear old interrupt and start measurement.
     */
    if (!vl_write_u8(SYSTEM_INTERRUPT_CLEAR, 0x01)) return false;
    if (!vl_write_u8(SYSRANGE_START, 0x01)) return false;

    /*
     * Give the sensor time before polling.
     */
    sleep_msec(60);

    start = now_ms();

    while (true) {
        if (!vl_read_u8(RESULT_INTERRUPT_STATUS, &status)) {
            return false;
        }

        if ((status & 0x07) != 0) {
            break;
        }

        if (timed_out(start)) {
            printf("VL53L0X: timeout waiting for result. RESULT_INTERRUPT_STATUS = 0x%02X\n",
                   status);
            return false;
        }

        sleep_msec(10);
    }

    if (!vl_read_u8(RESULT_RANGE_STATUS, &range_status)) {
        return false;
    }

    /*
     * Range is stored at RESULT_RANGE_STATUS + 10.
     */
    if (!vl_read_u16_be((uint8_t)(RESULT_RANGE_STATUS + 10), range_mm)) {
        return false;
    }

    *range_status_out = range_status;

    if (!vl_write_u8(SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        return false;
    }

    return true;
}

int readVL53L0XDistance(void) {
    uint16_t range_mm = 0;
    uint8_t range_status = 0;

    if (!vl53l0x_initialized) {
        printf("VL53L0X: read requested before successful init.\n");
        return VL53L0X_INVALID_DISTANCE_MM;
    }

    if (!vl53l0x_read_single_mm(&range_mm, &range_status)) {
        printf("VL53L0X: read failed.\n");
        return VL53L0X_INVALID_DISTANCE_MM;
    }

    last_range_status = range_status;

    /*
     * 8191 mm means invalid / saturated reading for this sensor.
     */
    if (range_mm == 0 || range_mm >= 8190) {
        printf("VL53L0X: invalid/out of range: %u mm | range_status = 0x%02X\n",
               range_mm,
               range_status);
        return VL53L0X_INVALID_DISTANCE_MM;
    }

    if (range_mm > VL53L0X_MAX_REASONABLE_MM) {
        printf("VL53L0X: reading too large for project use: %u mm | range_status = 0x%02X\n",
               range_mm,
               range_status);
        return VL53L0X_INVALID_DISTANCE_MM;
    }

    return (int)range_mm;
}

/* ---------------- Existing width-scan logic from repo ---------------- */

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

static bool scanStillSeesObject(int distance_mm, int center_distance_mm) {
    if (!isDistanceReadingValidForScan(distance_mm)) {
        return false;
    }

    if (distance_mm < FRONT_OBJECT_MIN_DISTANCE_MM) {
        return false;
    }

    if (abs(distance_mm - center_distance_mm) > WIDTH_SCAN_DISTANCE_MARGIN_MM) {
        return false;
    }

    if (distance_mm > FRONT_OBJECT_MAX_DISTANCE_MM + WIDTH_SCAN_DISTANCE_MARGIN_MM) {
        return false;
    }

    return true;
}

static float scanOneObjectEdge(int direction, int center_distance_mm,
                               float *total_turned_deg, bool *edge_is_sharp) {
    float last_visible_angle = 0.0;
    float current_angle = 0.0;

    *total_turned_deg = 0.0;
    *edge_is_sharp = true;

    while (current_angle < WIDTH_SCAN_MAX_DEG) {
        turn(direction * WIDTH_SCAN_STEP_DEG, WIDTH_SCAN_TURN_SPEED);

        current_angle += WIDTH_SCAN_STEP_DEG;
        *total_turned_deg = current_angle;

        sleep_msec(100);

        int distance_mm = readAverageDistanceForScan(WIDTH_SCAN_AVERAGE_COUNT);

        printf("Width scan %s: angle=%.2f deg, distance=%d mm\n",
               direction < 0 ? "left" : "right",
               current_angle,
               distance_mm);

        if (!scanStillSeesObject(distance_mm, center_distance_mm)) {
            /* Sharp edge: the beam fell off the object onto the background
             * (invalid reading or a large jump) — a real free-standing edge.
             * Soft edge: the reading is still near the center distance, so
             * the margin tripped on the gradual slope of a large oblique
             * face whose surface continues past this angle. */
            *edge_is_sharp =
                !isDistanceReadingValidForScan(distance_mm) ||
                distance_mm > center_distance_mm + WIDTH_SCAN_BACKGROUND_JUMP_MM;
            return (last_visible_angle + current_angle) / 2.0;
        }

        last_visible_angle = current_angle;
    }

    /* Object filled the whole sweep — certainly not a small sample. */
    *edge_is_sharp = false;
    return WIDTH_SCAN_MAX_DEG;
}

float scanObjectWidth(void) {
    int center_distance_mm = readAverageDistanceForScan(WIDTH_SCAN_AVERAGE_COUNT);

    if (!isDistanceReadingValidForScan(center_distance_mm)) {
        printf("Width scan failed: invalid center distance.\n");
        return 0.0;
    }

    if (center_distance_mm < FRONT_OBJECT_MIN_DISTANCE_MM ||
        center_distance_mm > FRONT_OBJECT_MAX_DISTANCE_MM) {
        printf("Width scan failed: center distance outside object window. Distance=%d mm, window=%d..%d mm\n",
               center_distance_mm,
               FRONT_OBJECT_MIN_DISTANCE_MM,
               FRONT_OBJECT_MAX_DISTANCE_MM);
        return 0.0;
    }

    printf("Width scan started. Center distance = %d mm\n", center_distance_mm);

    float left_total_turned = 0.0;
    bool left_edge_sharp = true;
    float left_edge_deg = scanOneObjectEdge(-1, center_distance_mm,
                                            &left_total_turned, &left_edge_sharp);

    turn(left_total_turned, WIDTH_SCAN_TURN_SPEED);

    sleep_msec(150);

    float right_total_turned = 0.0;
    bool right_edge_sharp = true;
    float right_edge_deg = scanOneObjectEdge(+1, center_distance_mm,
                                             &right_total_turned, &right_edge_sharp);

    turn(-right_total_turned, WIDTH_SCAN_TURN_SPEED);

    sleep_msec(150);

    float angular_width_deg = left_edge_deg + right_edge_deg;

    if (angular_width_deg <= 0.0) {
        printf("Width scan failed: angular width is zero.\n");
        return 0.0;
    }

    float distance_cm = center_distance_mm / 10.0;
    float angular_width_rad = degToRad(angular_width_deg);
    float width_cm = 2.0 * distance_cm * tan(angular_width_rad / 2.0);

    /*
     * A free-standing rock sample drops to the background past BOTH edges.
     * A soft edge means the surface continues beyond the detected angle —
     * the margin tripped on the slope of a large face seen obliquely, which
     * is how a 30 cm hill measures 5 cm and turns into a phantom sample.
     * Report hill-sized width so classification lands on hill.
     */
    if (!left_edge_sharp || !right_edge_sharp) {
        printf("Width scan soft edge (left=%s, right=%s): large oblique face, width %.2f -> %.2f cm\n",
               left_edge_sharp ? "sharp" : "soft",
               right_edge_sharp ? "sharp" : "soft",
               width_cm,
               (double)HILL_PHYSICAL_SIZE_CM);

        if (width_cm < HILL_PHYSICAL_SIZE_CM) {
            width_cm = HILL_PHYSICAL_SIZE_CM;
        }
    }

    printf("Width scan result: left=%.2f deg, right=%.2f deg, total=%.2f deg, width=%.2f cm\n",
           left_edge_deg,
           right_edge_deg,
           angular_width_deg,
           width_cm);

    return width_cm;
}
