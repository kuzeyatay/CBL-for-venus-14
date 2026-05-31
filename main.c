#include <libpynq.h>
#include <stepper.h>
#include <stdio.h>
#include <stdlib.h>

#include "constants/config.h"
#include "chassis/odom/odometry.h"
#include "chassis/drive/motion.h"
#include "sensors/color/color_sensor.h"
#include "sensors/distance/distance_sensor.h"
#include "sensors/gyro/gyro_sensor.h"
#include "sensors/temperature/temperature_sensor.h"
#include "comms/communication.h"
#include "chassis/nav/navigation.h"

int main(void)
{
    bool critical_sensors_ready = true;

    pynq_init();

    if (!USE_MOCK_SENSORS)
    {
        bool gyro_turn_ready = false;

        gpio_init();

        tcs3200Init();
        tcs3200DebugOutPin();
        tcs3200LoadManualCalibration(
            7146.0, 5950.0, 19400.0, 6909.3,     // white: R, G, B, C
        1153.3, 825.3, 2476.0, 945.3,     // black: R, G, B, C
        4248.7, 1644.7, 5848.0, 1970.7,     // red: R, G, B, C
        2910.0, 3265.3, 7721.3, 2176.7,     // green: R, G, B, C
        2774.0, 2084.0, 6832.0, 3512.0      // blue: R, G, B, C   // blue: R, G, B, C
        );

        if (!initVL53L0X()) {
            printf("VL53L0X: init failed; navigation will stay disabled.\n");
            critical_sensors_ready = false;
        }

        if (GYRO_TURN_ENABLED) {
            if (initMPU6886()) {
                setMPU6886GyroDeadbandDps(GYRO_DEADBAND_DPS);
                setMPU6886GyroZScaleCorrection(GYRO_Z_SCALE_CORRECTION);

                if (!calibrateMPU6886Gyro(GYRO_CALIBRATION_SAMPLES)) {
                    printf("MPU6886: gyro calibration failed; angle turns are disabled.\n");
                } else {
                    gyro_turn_ready = true;
                }
            } else {
                printf("MPU6886: init failed; angle turns are disabled.\n");
            }
        }
        setGyroTurnFeedbackEnabled(gyro_turn_ready);
        initNTCTemperatureSensor();
        initESP32UART();
    }

    stepper_init();
    stepper_enable();

    odometryInit(0.0, 0.0, 0.0);

    if (!critical_sensors_ready) {
        sendErrorMessage("critical_sensor_init_failed");
        sendStatusUpdate("fault", "distance_sensor_init_failed");

        while (1) {
            pollESP32Messages();
            sleep_msec(50);
        }
    }

    sendStatusUpdate("ready", "none");

    if (USE_MOCK_SENSORS)
    {
        runNavigation();
    }
    else
    {
        /*
         * Later:
         * Do not immediately start navigation.
         * Wait for START_MISSION from base station instead.
         */
        startMission();

        while (1)
        {
            pollESP32Messages();

            if (isMissionRunning())
            {
                runNavigation();
            }

            sleep_msec(100); // 100Hz main loop
        }
    }

    stepper_destroy();

    if (!USE_MOCK_SENSORS)
    {
        gpio_destroy();
    }

    pynq_destroy();

    return EXIT_SUCCESS;
}
