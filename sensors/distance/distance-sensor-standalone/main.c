/*
 * Full standalone VL53L0X distance sensor test for TU/e 5EID0 PYNQ / libpynq.
 *
 * File name:
 *   main.c
 *
 * Wiring:
 *   VL53L0X VIN -> 5V
 *   VL53L0X GND -> GND
 *   VL53L0X SCL -> AR_SCL
 *   VL53L0X SDA -> AR_SDA
 *
 * Important:
 *   Use the 7-bit I2C address 0x29 with libpynq.
 *   Do NOT use 0x52 in the code.
 *
 * This version includes timeout-protected reference calibration.
 * It should not hang forever during VHV / phase calibration.
 */

#include <libpynq.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

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

#define TIMEOUT_MS 1000

static uint8_t stop_variable = 0;

/* ---------------- Time helpers ---------------- */

static uint64_t now_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);

    return ((uint64_t)tv.tv_sec * 1000ULL) + ((uint64_t)tv.tv_usec / 1000ULL);
}

static bool timed_out(uint64_t start_ms) {
    return (now_ms() - start_ms) > TIMEOUT_MS;
}

/* ---------------- I2C helper functions ---------------- */

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

/* ---------------- Debug helper ---------------- */

static void print_basic_registers(void) {
    uint8_t model = 0;
    uint8_t revision = 0;

    if (vl_read_u8(IDENTIFICATION_MODEL_ID, &model)) {
        printf("Model ID register 0xC0    = 0x%02X", model);

        if (model == 0xEE) {
            printf(" OK");
        } else {
            printf(" unexpected");
        }

        printf("\n");
    } else {
        printf("Could not read model ID register.\n");
    }

    if (vl_read_u8(IDENTIFICATION_REVISION_ID, &revision)) {
        printf("Revision ID register 0xC2 = 0x%02X\n", revision);
    } else {
        printf("Could not read revision ID register.\n");
    }
}

/* ---------------- VL53L0X tuning ---------------- */

struct reg_value {
    uint8_t reg;
    uint8_t value;
};

static bool vl_write_reg_table(const struct reg_value *table, size_t n) {
    for (size_t i = 0; i < n; i++) {
        if (!vl_write_u8(table[i].reg, table[i].value)) {
            printf("I2C write failed at register 0x%02X\n", table[i].reg);
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

/* ---------------- Timeout-protected reference calibration ---------------- */

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
            printf("Reference calibration timeout. SYSRANGE_START = 0x%02X\n", sysrange);
            return false;
        }

        sleep_msec(10);
    }

    if (!vl_write_u8(SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        return false;
    }

    return true;
}

/* ---------------- Full VL53L0X init ---------------- */

static bool vl53l0x_init(void) {
    uint8_t model_id = 0;
    uint8_t tmp = 0;

    printf("Reading sensor identity...\n");
    fflush(stdout);

    if (!vl_read_u8(IDENTIFICATION_MODEL_ID, &model_id)) {
        printf("Could not read VL53L0X model ID. Check wiring.\n");
        return false;
    }

    printf("VL53L0X model ID register 0xC0 = 0x%02X", model_id);

    if (model_id == 0xEE) {
        printf(" OK\n");
    } else {
        printf(" unexpected, continuing anyway\n");
    }

    printf("Running basic setup...\n");
    fflush(stdout);

    if (!vl_write_u8(0x88, 0x00)) return false;

    /*
     * Read stop variable.
     */
    if (!vl_write_u8(0x80, 0x01)) return false;
    if (!vl_write_u8(0xFF, 0x01)) return false;
    if (!vl_write_u8(0x00, 0x00)) return false;

    if (!vl_read_u8(0x91, &stop_variable)) {
        printf("Failed to read stop_variable.\n");
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
        printf("Failed to set signal rate limit.\n");
        return false;
    }

    /*
     * Enable all sequence steps while loading settings.
     */
    if (!vl_write_u8(SYSTEM_SEQUENCE_CONFIG, 0xFF)) return false;

    printf("Loading tuning settings...\n");
    fflush(stdout);

    if (!vl_load_tuning_settings()) {
        printf("Failed while loading tuning settings.\n");
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
     */
    printf("Running VHV reference calibration...\n");
    fflush(stdout);

    if (!vl_write_u8(SYSTEM_SEQUENCE_CONFIG, 0x01)) return false;

    if (!vl_perform_single_ref_calibration(0x40)) {
        printf("VHV reference calibration failed.\n");
        return false;
    }

    printf("Running phase reference calibration...\n");
    fflush(stdout);

    if (!vl_write_u8(SYSTEM_SEQUENCE_CONFIG, 0x02)) return false;

    if (!vl_perform_single_ref_calibration(0x00)) {
        printf("Phase reference calibration failed.\n");
        return false;
    }

    /*
     * Restore final ranging sequence.
     */
    if (!vl_write_u8(SYSTEM_SEQUENCE_CONFIG, 0xE8)) return false;

    printf("Reference calibration finished.\n");

    return true;
}

/* ---------------- Single distance read ---------------- */

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
            printf("Timeout waiting for result. RESULT_INTERRUPT_STATUS = 0x%02X\n", status);
            return false;
        }

        sleep_msec(10);
    }

    /*
     * Read status and range.
     */
    if (!vl_read_u8(RESULT_RANGE_STATUS, &range_status)) {
        return false;
    }

    if (!vl_read_u16_be((uint8_t)(RESULT_RANGE_STATUS + 10), range_mm)) {
        return false;
    }

    *range_status_out = range_status;

    if (!vl_write_u8(SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        return false;
    }

    return true;
}

/* ---------------- Main ---------------- */

int main(void) {
    pynq_init();

    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);

    iic_init(IIC0);
    sleep_msec(300);

    buttons_init();

    printf("\n--- VL53L0X full distance sensor test ---\n");
    printf("Wiring: VIN->5V, GND->GND, SCL->AR_SCL, SDA->AR_SDA\n");
    printf("I2C address used in code: 0x29\n");
    printf("Press BUTTON0 to stop.\n\n");

    print_basic_registers();
    printf("\n");

    if (!vl53l0x_init()) {
        printf("\nVL53L0X init failed.\n");
        printf("Try this:\n");
        printf("1. Fully power-cycle the PYNQ and sensor.\n");
        printf("2. Check VIN->5V, GND->GND, SCL->AR_SCL, SDA->AR_SDA.\n");
        printf("3. Test with a white paper target at 10-20 cm.\n");
        printf("4. If your breakout has XSHUT, make sure it is not pulled low.\n");

        buttons_destroy();
        iic_destroy(IIC0);
        pynq_destroy();

        return EXIT_FAILURE;
    }

    printf("\nInit finished. Starting readings...\n");
    printf("Test first with a white paper/cardboard target at 10-20 cm.\n\n");

    while (!get_button_state(BUTTON0)) {
        uint16_t mm = 0;
        uint8_t range_status = 0;

        bool ok = vl53l0x_read_single_mm(&mm, &range_status);

        if (!ok) {
            printf("Distance: read failed / timeout\n");
        } else if (mm == 0 || mm >= 8190) {
            printf("Distance: invalid/out of range: %u mm | range_status = 0x%02X\n",
                   mm,
                   range_status);
        } else {
            printf("Distance: %4u mm  |  %5.1f cm | range_status = 0x%02X",
                   mm,
                   mm / 10.0,
                   range_status);

            if (mm < 80) {
                printf("  VERY CLOSE");
            } else if (mm < 200) {
                printf("  NEAR");
            } else if (mm < 400) {
                printf("  MEDIUM");
            } else {
                printf("  FAR");
            }

            printf("\n");
        }

        sleep_msec(150);
    }

    printf("\nStopping VL53L0X test.\n");

    buttons_destroy();
    iic_destroy(IIC0);
    pynq_destroy();

    return EXIT_SUCCESS;
}