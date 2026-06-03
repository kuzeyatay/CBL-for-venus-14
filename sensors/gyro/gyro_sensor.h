#ifndef GYRO_SENSOR_H
#define GYRO_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

/*
 * MPU6886 gyro / accelerometer driver for TU/e 5EID0 PYNQ / libpynq.
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

typedef enum {
    MPU6886_ACCEL_RANGE_2G  = 0,
    MPU6886_ACCEL_RANGE_4G  = 1,
    MPU6886_ACCEL_RANGE_8G  = 2,
    MPU6886_ACCEL_RANGE_16G = 3
} mpu6886_accel_range_t;

typedef enum {
    MPU6886_GYRO_RANGE_250DPS  = 0,
    MPU6886_GYRO_RANGE_500DPS  = 1,
    MPU6886_GYRO_RANGE_1000DPS = 2,
    MPU6886_GYRO_RANGE_2000DPS = 3
} mpu6886_gyro_range_t;

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} MPU6886RawData;

typedef struct {
    float ax_g;
    float ay_g;
    float az_g;
    float temp_c;
    float gx_dps;
    float gy_dps;
    float gz_dps;
} MPU6886Data;

/* ---------------- Init and status ---------------- */

bool initMPU6886(void);

bool initMPU6886WithRanges(mpu6886_accel_range_t accel_range,
                           mpu6886_gyro_range_t gyro_range);

bool isMPU6886Initialized(void);

uint8_t getMPU6886WhoAmI(void);

/* ---------------- Reading ---------------- */

bool readMPU6886Raw(MPU6886RawData *raw);

bool readMPU6886Data(MPU6886Data *data);

/* ---------------- Gyro calibration and yaw ---------------- */

bool calibrateMPU6886Gyro(int sample_count);

/*
 * Fast in-mission bias update. Takes ~200 ms (100 samples × 2 ms).
 * Call while the robot is stationary to correct thermal drift.
 * Restores the old gz bias on failure.
 */
bool updateMPU6886GyroBias(int sample_count);

void resetMPU6886Yaw(void);

/*
 * Reads the IMU internally and updates yaw.
 * Useful if you only care about yaw.
 */
bool updateMPU6886Yaw(void);

/*
 * Updates yaw using a gyro-Z value that you already read with readMPU6886Data().
 * Prefer this in main loops where you already read and print/use the IMU data.
 */
bool updateMPU6886YawFromGyroZ(float gz_dps);

float getMPU6886YawDeg(void);

float getMPU6886GyroZBiasDps(void);

/*
 * Tune these if needed.
 *
 * Deadband:
 *   Small gyro values below this are treated as zero to reduce stationary drift.
 *
 * Scale correction:
 *   Use this if a 360-degree test gives a consistent scale error.
 *   Example:
 *     actual turn = 360 deg
 *     measured turn = 350 deg
 *     scale correction = 360.0 / 350.0 = 1.02857
 */
void setMPU6886GyroDeadbandDps(float deadband_dps);

void setMPU6886GyroZScaleCorrection(float scale_correction);

/* ---------------- Tilt helpers ---------------- */

float getMPU6886RollDeg(const MPU6886Data *data);

float getMPU6886PitchDeg(const MPU6886Data *data);

#endif
