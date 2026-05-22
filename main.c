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
            11739.3,8008.7,23974.0,8879.3, // white
            4016.7,2398.7,7171.3,2604.0,     // black
            10152.7,4464.0,13643.3,5047.3 , // red
            7011.3,5428.7,13188.7 ,4685.3, // green
            7796.0,4816.7,12952.0, 6043.3 // blue
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