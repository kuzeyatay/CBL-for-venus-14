#include <libpynq.h>
#include <stepper.h>
#include <stdlib.h>

#include "constants/config.h"
#include "chassis/odom/odometry.h"
#include "chassis/drive/motion.h"
#include "sensors/color/color_sensor.h"
#include "sensors/distance/distance_sensor.h"
#include "sensors/temperature/temperature_sensor.h"
#include "comms/communication.h"
#include "chassis/nav/navigation.h"

int main(void)
{
    pynq_init();

    if (!USE_MOCK_SENSORS)
    {
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

        initVL53L0X();
        initNTCTemperatureSensor();
        initESP32UART();
    }

    stepper_init();
    stepper_enable();

    odometryInit(0.0, 0.0, 0.0);

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