#include <stdio.h>
#include <libpynq.h>

#include "config.h"
#include "robot_types.h"
#include "distance_sensor.h"
#include "temperature_sensor.h"
#include "color_sensor.h"
#include "sensor_manager.h"

// static sensor_data_t readSensorDataMock(void) {
//     static int mock_index = 0;

//     static sensor_data_t script[] = {
//         {true, false, false, 500, 0.0, COLOR_UNKNOWN, 0.0},
//         {true, false, false, 500, 0.0, COLOR_UNKNOWN, 0.0},
//         {true, true, false, 500, 0.0, COLOR_UNKNOWN, 0.0},
//         {true, false, false, 500, 0.0, COLOR_UNKNOWN, 0.0},
//         {true, false, true, 110, 3.2, COLOR_RED, 24.5},
//         {true, false, false, 500, 0.0, COLOR_UNKNOWN, 0.0},
//         {true, false, true, 100, 30.0, COLOR_UNKNOWN, 0.0},
//         {true, false, false, 500, 0.0, COLOR_UNKNOWN, 0.0},
//         {false, false, false, 0, 0.0, COLOR_UNKNOWN, 0.0},
//         {true, false, true, 120, 6.1, COLOR_BLUE, 25.0}
//     };

//     int script_length = sizeof(script) / sizeof(script[0]);

//     sensor_data_t data = script[mock_index % script_length];
//     mock_index++;

//     return data;
// }

static sensor_data_t readSensorDataReal(void)
{
    sensor_data_t data;

    data.valid = true;
    data.black_tape_detected = false;
    data.front_object_detected = false;
    data.front_distance_mm = VL53L0X_INVALID_DISTANCE_MM;
    data.estimated_width_cm = 0.0f;
    data.object_color = COLOR_UNKNOWN;
    data.temperature_c = 0.0f;

    data.front_distance_mm = readVL53L0XDistance();

    data.front_object_detected =
        data.front_distance_mm < FRONT_OBJECT_THRESHOLD_MM &&
        data.front_distance_mm > 0;

    /*
     * Only use the TCS3200 for black-tape detection when there is no close
     * front object. If there is a front object, black could also be a black
     * rock sample, so color is postponed until after width classification.
     */
    if (!isTCS3200Calibrated()) {
        data.valid = false;
        data.black_tape_detected = false;
        data.object_color = COLOR_UNKNOWN;
    } else if (!data.front_object_detected) {
        data.black_tape_detected = tcs3200DetectBlackTape();
    }

    if (data.front_object_detected) {
        data.estimated_width_cm = scanObjectWidth();
    }

    return data;
}

sensor_data_t readSensorData(void) {
#if USE_MOCK_SENSORS
    return readSensorDataMock();
#else
    return readSensorDataReal();
#endif
}

sensor_data_t readReliableSensorData(void) {
    sensor_data_t data;

    for (int attempt = 0; attempt < SENSOR_RETRY_COUNT; attempt++) {
        data = readSensorData();

        if (data.valid) {
            return data;
        }

        printf("Invalid sensor reading. Retry %d/%d\n",
               attempt + 1,
               SENSOR_RETRY_COUNT);

        sleep_msec(100);
    }

    data.valid = false;
    return data;
}