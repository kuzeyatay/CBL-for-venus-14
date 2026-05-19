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

int main(void) {
    pynq_init();

#if !USE_MOCK_SENSORS
    gpio_init();

    tcs3200Init();
    tcs3200DebugOutPin();

    initVL53L0X();
    initNTCTemperatureSensor();
    initESP32UART();
#endif

    stepper_init();
    stepper_enable();

    odometryInit(0.0, 0.0, 0.0);

    sendStatusUpdate("ready", "none");

#if USE_MOCK_SENSORS
    runNavigation();
#else
    /*
     * Later:
     * Do not immediately start navigation.
     * Wait for START_MISSION from base station instead.
     */
    while (1) {
        pollESP32Messages();

        if (isMissionRunning()) {
            runNavigation();
        }

        sleep_msec(100);
    }
#endif

    stepper_destroy();

#if !USE_MOCK_SENSORS
    gpio_destroy();
#endif

    pynq_destroy();

    return EXIT_SUCCESS;
}