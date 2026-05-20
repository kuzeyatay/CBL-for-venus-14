#include <libpynq.h>
#include <stepper.h>
#include <stdlib.h>

#include "config.h"
#include "odometry.h"
#include "motion.h"
#include "color_sensor.h"
#include "distance_sensor.h"
#include "temperature_sensor.h"
#include "communication.h"
#include "navigation.h"

int main(void)
{
    pynq_init();

    if (!USE_MOCK_SENSORS)
    {
        gpio_init();

        tcs3200Init();
        tcs3200DebugOutPin();
        tcs3200LoadManualCalibration(
            900.0, 920.0, 910.0, 950.0, // white
            80.0, 85.0, 82.0, 90.0,     // black
            700.0, 200.0, 180.0, 750.0, // red
            220.0, 700.0, 250.0, 760.0, // green
            180.0, 260.0, 720.0, 770.0  // blue
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
        startMission(void);

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