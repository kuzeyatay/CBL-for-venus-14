#include "gyro_sensor.h"

#include <libpynq.h>

#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/time.h>

/*
 * MPU6886 gyro / accelerometer driver for TU/e 5EID0 PYNQ / libpynq.
 *
 * This driver is intended to be used like the distance_sensor.c driver:
 *
 *   pynq_init();
 *
 *   initVL53L0X();
 *   initMPU6886();
 *
 *   sleep_msec(2000);
 *   calibrateMPU6886Gyro(1000);
 *   resetMPU6886Yaw();
 *
 *   while (true) {
 *       MPU6886Data imu;
 *
 *       if (readMPU6886Data(&imu)) {
 *           updateMPU6886YawFromGyroZ(imu.gz_dps);
 *           printf("yaw = %.2f deg\n", getMPU6886YawDeg());
 *       }
 *   }
 *
 * Wiring:
 *   MPU6886 GND -> PYNQ GND
 *   MPU6886 3V3 -> PYNQ 3V3
 *   MPU6886 SCL -> AR_SCL
 *   MPU6886 SDA -> AR_SDA
 *
 * Important:
 *   libpynq uses the 7-bit I2C address.
 *   MPU6886 address = 0x68.
 *   Do NOT use 0xD0 here.
 */

#define MPU6886_ADDR              0x68

#define MPU6886_SMPLRT_DIV        0x19
#define MPU6886_CONFIG            0x1A
#define MPU6886_GYRO_CONFIG       0x1B
#define MPU6886_ACCEL_CONFIG      0x1C
#define MPU6886_ACCEL_CONFIG2     0x1D
#define MPU6886_FIFO_EN           0x23
#define MPU6886_INT_PIN_CFG       0x37
#define MPU6886_INT_ENABLE        0x38
#define MPU6886_INT_STATUS        0x3A
#define MPU6886_ACCEL_XOUT_H      0x3B
#define MPU6886_USER_CTRL         0x6A
#define MPU6886_PWR_MGMT_1        0x6B
#define MPU6886_PWR_MGMT_2        0x6C
#define MPU6886_WHO_AM_I          0x75

#define MPU6886_EXPECTED_WHO_AM_I 0x19

#define DEG_PER_RAD               57.29577951308232f

/*
 * Default deadband.
 *
 * If |gz_dps| is below this value, it is treated as zero.
 * This reduces yaw drift while the robot is stationary.
 *
 * If the robot turns very slowly, reduce this to 0.2 or disable it during turns.
 */
#define MPU6886_DEFAULT_GYRO_DEADBAND_DPS 0.50f

/* ---------------- Internal state ---------------- */

static bool mpu6886_initialized = false;
static uint8_t last_who_am_i = 0;

static mpu6886_accel_range_t current_accel_range = MPU6886_ACCEL_RANGE_2G;
static mpu6886_gyro_range_t current_gyro_range = MPU6886_GYRO_RANGE_250DPS;

static float accel_lsb_to_g = 2.0f / 32768.0f;
static float gyro_lsb_to_dps = 250.0f / 32768.0f;

static float gyro_x_bias_dps = 0.0f;
static float gyro_y_bias_dps = 0.0f;
static float gyro_z_bias_dps = 0.0f;

static float yaw_deg = 0.0f;
static uint64_t last_yaw_update_us = 0;

static float gyro_deadband_dps = MPU6886_DEFAULT_GYRO_DEADBAND_DPS;
static float gyro_z_scale_correction = 1.0f;

/* ---------------- Time helpers ---------------- */

static uint64_t now_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);

    return ((uint64_t)tv.tv_sec * 1000000ULL) + (uint64_t)tv.tv_usec;
}

/* ---------------- I2C helpers ---------------- */

static bool mpu_write_u8(uint8_t reg, uint8_t value) {
    return iic_write_register(IIC0, MPU6886_ADDR, reg, &value, 1) == 0;
}

static bool mpu_read_u8(uint8_t reg, uint8_t *value) {
    return iic_read_register(IIC0, MPU6886_ADDR, reg, value, 1) == 0;
}

static bool mpu_read_multi(uint8_t reg, uint8_t *data, uint16_t len) {
    if (iic_read_register(IIC0, MPU6886_ADDR, reg, data, len) == 0) {
        return true;
    }
    return iic_read_register(IIC0, MPU6886_ADDR, reg, data, len) == 0;
}

static int16_t make_i16_be(uint8_t high, uint8_t low) {
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

/* ---------------- Scale helpers ---------------- */

static float accel_range_to_lsb_to_g(mpu6886_accel_range_t range) {
    switch (range) {
        case MPU6886_ACCEL_RANGE_2G:
            return 2.0f / 32768.0f;

        case MPU6886_ACCEL_RANGE_4G:
            return 4.0f / 32768.0f;

        case MPU6886_ACCEL_RANGE_8G:
            return 8.0f / 32768.0f;

        case MPU6886_ACCEL_RANGE_16G:
            return 16.0f / 32768.0f;

        default:
            return 2.0f / 32768.0f;
    }
}

static float gyro_range_to_lsb_to_dps(mpu6886_gyro_range_t range) {
    switch (range) {
        case MPU6886_GYRO_RANGE_250DPS:
            return 250.0f / 32768.0f;

        case MPU6886_GYRO_RANGE_500DPS:
            return 500.0f / 32768.0f;

        case MPU6886_GYRO_RANGE_1000DPS:
            return 1000.0f / 32768.0f;

        case MPU6886_GYRO_RANGE_2000DPS:
            return 2000.0f / 32768.0f;

        default:
            return 250.0f / 32768.0f;
    }
}

static void normalize_yaw_deg(void) {
    while (yaw_deg > 180.0f) {
        yaw_deg -= 360.0f;
    }

    while (yaw_deg < -180.0f) {
        yaw_deg += 360.0f;
    }
}

/* ---------------- Public init functions ---------------- */

bool initMPU6886(void) {
    /*
     * For robot odometry and pure degree turns, ±250 dps is much better
     * than ±2000 dps because it gives finer resolution.
     */
    return initMPU6886WithRanges(MPU6886_ACCEL_RANGE_2G,
                                 MPU6886_GYRO_RANGE_250DPS);
}

bool initMPU6886WithRanges(mpu6886_accel_range_t accel_range,
                           mpu6886_gyro_range_t gyro_range) {
    uint8_t who = 0;
    uint8_t dummy = 0;

    printf("MPU6886: initializing gyro/accelerometer...\n");

    /*
     * Same I2C pins as the VL53L0X distance sensor.
     * Both sensors can share IIC0 as long as the bus is electrically stable.
     */
    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);

    iic_init(IIC0);
    sleep_msec(100);

    if (!mpu_read_u8(MPU6886_WHO_AM_I, &who)) {
        printf("MPU6886: could not read WHO_AM_I. Check wiring and I2C address.\n");
        mpu6886_initialized = false;
        return false;
    }

    last_who_am_i = who;

    printf("MPU6886: WHO_AM_I 0x75 = 0x%02X", who);

    if (who == MPU6886_EXPECTED_WHO_AM_I) {
        printf(" OK\n");
    } else {
        printf(" unexpected, expected 0x%02X\n", MPU6886_EXPECTED_WHO_AM_I);
        mpu6886_initialized = false;
        return false;
    }

    /*
     * Reset device.
     * Bit 7 auto-clears after reset.
     */
    if (!mpu_write_u8(MPU6886_PWR_MGMT_1, 0x80)) {
        printf("MPU6886: failed to reset device.\n");
        mpu6886_initialized = false;
        return false;
    }

    sleep_msec(100);

    /*
     * Wake device and select clock source.
     * 0x01 selects PLL with gyro reference.
     */
    if (!mpu_write_u8(MPU6886_PWR_MGMT_1, 0x01)) {
        printf("MPU6886: failed to wake device.\n");
        mpu6886_initialized = false;
        return false;
    }

    sleep_msec(10);

    /*
     * Enable all accelerometer and gyroscope axes.
     */
    if (!mpu_write_u8(MPU6886_PWR_MGMT_2, 0x00)) {
        printf("MPU6886: failed to enable axes.\n");
        mpu6886_initialized = false;
        return false;
    }

    sleep_msec(10);

    /*
     * Disable FIFO and reset auxiliary user controls.
     */
    if (!mpu_write_u8(MPU6886_USER_CTRL, 0x00)) {
        printf("MPU6886: failed to configure USER_CTRL.\n");
        mpu6886_initialized = false;
        return false;
    }

    if (!mpu_write_u8(MPU6886_FIFO_EN, 0x00)) {
        printf("MPU6886: failed to disable FIFO.\n");
        mpu6886_initialized = false;
        return false;
    }

    /*
     * Gyro/temp DLPF:
     *   DLPF_CFG = 1 gives filtered gyro output.
     *
     * Sample-rate divider:
     *   With DLPF enabled, internal gyro rate is about 1 kHz.
     *   Output rate = 1 kHz / (1 + 1) = about 500 Hz.
     *   This matches the common MPU6886 setup and reduces turn-stop latency.
     */
    if (!mpu_write_u8(MPU6886_CONFIG, 0x01)) {
        printf("MPU6886: failed to configure DLPF.\n");
        mpu6886_initialized = false;
        return false;
    }

    if (!mpu_write_u8(MPU6886_SMPLRT_DIV, 0x01)) {
        printf("MPU6886: failed to configure sample-rate divider.\n");
        mpu6886_initialized = false;
        return false;
    }

    /*
     * Full-scale ranges.
     * Bits [4:3] store the selected range.
     */
    if (!mpu_write_u8(MPU6886_ACCEL_CONFIG, (uint8_t)(accel_range << 3))) {
        printf("MPU6886: failed to configure accelerometer range.\n");
        mpu6886_initialized = false;
        return false;
    }

    if (!mpu_write_u8(MPU6886_GYRO_CONFIG, (uint8_t)(gyro_range << 3))) {
        printf("MPU6886: failed to configure gyro range.\n");
        mpu6886_initialized = false;
        return false;
    }

    /*
     * Accelerometer DLPF.
     */
    if (!mpu_write_u8(MPU6886_ACCEL_CONFIG2, 0x01)) {
        printf("MPU6886: failed to configure accelerometer DLPF.\n");
        mpu6886_initialized = false;
        return false;
    }

    /*
     * Interrupts are not needed for polling mode.
     */
    if (!mpu_write_u8(MPU6886_INT_PIN_CFG, 0x00)) {
        printf("MPU6886: failed to configure interrupt pin.\n");
        mpu6886_initialized = false;
        return false;
    }

    if (!mpu_write_u8(MPU6886_INT_ENABLE, 0x00)) {
        printf("MPU6886: failed to disable interrupts.\n");
        mpu6886_initialized = false;
        return false;
    }

    current_accel_range = accel_range;
    current_gyro_range = gyro_range;

    accel_lsb_to_g = accel_range_to_lsb_to_g(current_accel_range);
    gyro_lsb_to_dps = gyro_range_to_lsb_to_dps(current_gyro_range);

    gyro_x_bias_dps = 0.0f;
    gyro_y_bias_dps = 0.0f;
    gyro_z_bias_dps = 0.0f;

    resetMPU6886Yaw();

    /*
     * Clear any old data-ready status by reading it once.
     */
    (void)mpu_read_u8(MPU6886_INT_STATUS, &dummy);

    mpu6886_initialized = true;

    printf("MPU6886: init complete. Accel range=±%dg, gyro range=±%d dps\n",
           (current_accel_range == MPU6886_ACCEL_RANGE_2G)  ? 2  :
           (current_accel_range == MPU6886_ACCEL_RANGE_4G)  ? 4  :
           (current_accel_range == MPU6886_ACCEL_RANGE_8G)  ? 8  : 16,
           (current_gyro_range == MPU6886_GYRO_RANGE_250DPS)  ? 250  :
           (current_gyro_range == MPU6886_GYRO_RANGE_500DPS)  ? 500  :
           (current_gyro_range == MPU6886_GYRO_RANGE_1000DPS) ? 1000 : 2000);

    return true;
}

bool isMPU6886Initialized(void) {
    return mpu6886_initialized;
}

uint8_t getMPU6886WhoAmI(void) {
    return last_who_am_i;
}

/* ---------------- Reading functions ---------------- */

bool readMPU6886Raw(MPU6886RawData *raw) {
    uint8_t buf[14];

    if (raw == NULL) {
        return false;
    }

    if (!mpu6886_initialized) {
        printf("MPU6886: read requested before successful init.\n");
        return false;
    }

    /*
     * Read accelerometer, temperature, and gyro in one burst:
     *   0x3B-0x40 accel
     *   0x41-0x42 temperature
     *   0x43-0x48 gyro
     */
    if (!mpu_read_multi(MPU6886_ACCEL_XOUT_H, buf, sizeof(buf))) {
        printf("MPU6886: failed to read sensor data block.\n");
        return false;
    }

    raw->ax   = make_i16_be(buf[0],  buf[1]);
    raw->ay   = make_i16_be(buf[2],  buf[3]);
    raw->az   = make_i16_be(buf[4],  buf[5]);
    raw->temp = make_i16_be(buf[6],  buf[7]);
    raw->gx   = make_i16_be(buf[8],  buf[9]);
    raw->gy   = make_i16_be(buf[10], buf[11]);
    raw->gz   = make_i16_be(buf[12], buf[13]);

    return true;
}

bool readMPU6886Data(MPU6886Data *data) {
    MPU6886RawData raw;

    if (data == NULL) {
        return false;
    }

    if (!readMPU6886Raw(&raw)) {
        return false;
    }

    data->ax_g = (float)raw.ax * accel_lsb_to_g;
    data->ay_g = (float)raw.ay * accel_lsb_to_g;
    data->az_g = (float)raw.az * accel_lsb_to_g;

    /*
     * Common MPU6886 temperature approximation.
     * This is mainly useful for diagnostics, not precise temperature sensing.
     */
    data->temp_c = ((float)raw.temp / 326.8f) + 25.0f;

    /*
     * Bias-corrected gyro values.
     */
    data->gx_dps = ((float)raw.gx * gyro_lsb_to_dps) - gyro_x_bias_dps;
    data->gy_dps = ((float)raw.gy * gyro_lsb_to_dps) - gyro_y_bias_dps;
    data->gz_dps = ((float)raw.gz * gyro_lsb_to_dps) - gyro_z_bias_dps;

    return true;
}

/* ---------------- Gyro calibration and yaw integration ---------------- */

bool updateMPU6886GyroBias(int sample_count) {
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    int valid_count = 0;

    if (!mpu6886_initialized) return false;
    if (sample_count <= 0) sample_count = 100;

    float prev_z = gyro_z_bias_dps;

    gyro_x_bias_dps = 0.0f;
    gyro_y_bias_dps = 0.0f;
    gyro_z_bias_dps = 0.0f;

    for (int i = 0; i < sample_count; i++) {
        MPU6886RawData raw;
        if (readMPU6886Raw(&raw)) {
            sum_x += (double)((float)raw.gx * gyro_lsb_to_dps);
            sum_y += (double)((float)raw.gy * gyro_lsb_to_dps);
            sum_z += (double)((float)raw.gz * gyro_lsb_to_dps);
            valid_count++;
        }
        sleep_msec(2);
    }

    if (valid_count <= 0) {
        gyro_x_bias_dps = 0.0f;
        gyro_y_bias_dps = 0.0f;
        gyro_z_bias_dps = prev_z;
        return false;
    }

    gyro_x_bias_dps = (float)(sum_x / (double)valid_count);
    gyro_y_bias_dps = (float)(sum_y / (double)valid_count);
    gyro_z_bias_dps = (float)(sum_z / (double)valid_count);

    printf("MPU6886: bias updated gz=%.5f dps (drift=%.5f dps)\n",
           gyro_z_bias_dps, gyro_z_bias_dps - prev_z);

    return true;
}

bool calibrateMPU6886Gyro(int sample_count) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;

    int valid_count = 0;

    if (!mpu6886_initialized) {
        printf("MPU6886: calibration requested before successful init.\n");
        return false;
    }

    if (sample_count <= 0) {
        sample_count = 1000;
    }

    printf("MPU6886: calibrating gyro bias with %d samples.\n", sample_count);
    printf("MPU6886: keep the robot COMPLETELY still...\n");

    /*
     * Temporarily remove old bias while measuring new bias.
     */
    gyro_x_bias_dps = 0.0f;
    gyro_y_bias_dps = 0.0f;
    gyro_z_bias_dps = 0.0f;

    /*
     * Throw away first samples after configuration.
     */
    for (int i = 0; i < 100; i++) {
        MPU6886RawData raw;
        (void)readMPU6886Raw(&raw);
        sleep_msec(2);
    }

    for (int i = 0; i < sample_count; i++) {
        MPU6886RawData raw;

        if (readMPU6886Raw(&raw)) {
            sum_x += (double)((float)raw.gx * gyro_lsb_to_dps);
            sum_y += (double)((float)raw.gy * gyro_lsb_to_dps);
            sum_z += (double)((float)raw.gz * gyro_lsb_to_dps);
            valid_count++;
        }

        sleep_msec(5);
    }

    if (valid_count <= 0) {
        printf("MPU6886: gyro calibration failed, no valid samples.\n");
        return false;
    }

    gyro_x_bias_dps = (float)(sum_x / (double)valid_count);
    gyro_y_bias_dps = (float)(sum_y / (double)valid_count);
    gyro_z_bias_dps = (float)(sum_z / (double)valid_count);

    resetMPU6886Yaw();

    printf("MPU6886: gyro bias gx=%.5f dps, gy=%.5f dps, gz=%.5f dps\n",
           gyro_x_bias_dps,
           gyro_y_bias_dps,
           gyro_z_bias_dps);

    return true;
}

void resetMPU6886Yaw(void) {
    yaw_deg = 0.0f;
    last_yaw_update_us = 0;
}

bool updateMPU6886Yaw(void) {
    MPU6886Data data;

    if (!readMPU6886Data(&data)) {
        return false;
    }

    return updateMPU6886YawFromGyroZ(data.gz_dps);
}

bool updateMPU6886YawFromGyroZ(float gz_dps) {
    uint64_t now = now_us();

    if (last_yaw_update_us == 0) {
        last_yaw_update_us = now;
        return true;
    }

    float dt_s = (float)(now - last_yaw_update_us) / 1000000.0f;
    last_yaw_update_us = now;

    /*
     * Avoid large jumps after pauses/debugging.
     */
    if (dt_s <= 0.0f || dt_s > 1.0f) {
        return true;
    }

    /*
     * Ignore tiny stationary gyro noise.
     */
    if (fabsf(gz_dps) < gyro_deadband_dps) {
        gz_dps = 0.0f;
    }

    yaw_deg += (gz_dps * gyro_z_scale_correction) * dt_s;
    normalize_yaw_deg();

    return true;
}

float getMPU6886YawDeg(void) {
    return yaw_deg;
}

float getMPU6886GyroZBiasDps(void) {
    return gyro_z_bias_dps;
}

void setMPU6886GyroDeadbandDps(float deadband_dps) {
    if (deadband_dps < 0.0f) {
        deadband_dps = 0.0f;
    }

    gyro_deadband_dps = deadband_dps;
}

void setMPU6886GyroZScaleCorrection(float scale_correction) {
    if (scale_correction <= 0.0f) {
        scale_correction = 1.0f;
    }

    gyro_z_scale_correction = scale_correction;
}

/* ---------------- Tilt helpers ---------------- */

float getMPU6886RollDeg(const MPU6886Data *data) {
    if (data == NULL) {
        return 0.0f;
    }

    return atan2f(data->ay_g, data->az_g) * DEG_PER_RAD;
}

float getMPU6886PitchDeg(const MPU6886Data *data) {
    if (data == NULL) {
        return 0.0f;
    }

    return atan2f(-data->ax_g,
                  sqrtf((data->ay_g * data->ay_g) + (data->az_g * data->az_g))) * DEG_PER_RAD;
}
