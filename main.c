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
            2110.7 ,641.3, 4815.3, 1860.7, // white
            96.0,82.7,256.7,96.0,     // black
            1709.3 ,578.0 ,1976.7 ,701.3 , // red
            676.7,800.0,1630.0,532.0, // green
            792.0,542.7,1403.3, 821.3 // blue
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