#include <libpynq.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "config.h"
#include "odometry.h"
#include "motion.h"
#include "distance_sensor.h"

bool initVL53L0X(void) {
    /*
     * TODO:
     * Initialize the VL53L0X sensor.
     */
    return false;
}

int readVL53L0XDistance(void) {
    /*
     * TODO:
     * Replace this with the real VL53L0X driver code.
     *
     * Return distance in millimeters.
     */
    return VL53L0X_INVALID_DISTANCE_MM;
}

static bool isDistanceReadingValidForScan(int distance_mm) {
    if (distance_mm == VL53L0X_INVALID_DISTANCE_MM) {
        return false;
    }

    if (distance_mm <= 0) {
        return false;
    }

    if (distance_mm > VL53L0X_MAX_REASONABLE_MM) {
        return false;
    }

    return true;
}

static int readAverageDistanceForScan(int sample_count) {
    int valid_count = 0;
    int sum_mm = 0;

    for (int i = 0; i < sample_count; i++) {
        int distance_mm = readVL53L0XDistance();

        if (isDistanceReadingValidForScan(distance_mm)) {
            sum_mm += distance_mm;
            valid_count++;
        }

        sleep_msec(20);
    }

    if (valid_count == 0) {
        return VL53L0X_INVALID_DISTANCE_MM;
    }

    return sum_mm / valid_count;
}

static bool scanStillSeesObject(int distance_mm, int center_distance_mm) {
    if (!isDistanceReadingValidForScan(distance_mm)) {
        return false;
    }

    if (abs(distance_mm - center_distance_mm) > WIDTH_SCAN_DISTANCE_MARGIN_MM) {
        return false;
    }

    if (distance_mm > FRONT_OBJECT_THRESHOLD_MM + WIDTH_SCAN_DISTANCE_MARGIN_MM) {
        return false;
    }

    return true;
}

static float scanOneObjectEdge(int direction, int center_distance_mm, float *total_turned_deg) {
    float last_visible_angle = 0.0;
    float current_angle = 0.0;

    *total_turned_deg = 0.0;

    while (current_angle < WIDTH_SCAN_MAX_DEG) {
        turn(direction * WIDTH_SCAN_STEP_DEG, WIDTH_SCAN_TURN_SPEED);

        current_angle += WIDTH_SCAN_STEP_DEG;
        *total_turned_deg = current_angle;

        sleep_msec(100);

        int distance_mm = readAverageDistanceForScan(WIDTH_SCAN_AVERAGE_COUNT);

        printf("Width scan %s: angle=%.2f deg, distance=%d mm\n",
               direction < 0 ? "left" : "right",
               current_angle,
               distance_mm);

        if (!scanStillSeesObject(distance_mm, center_distance_mm)) {
            return (last_visible_angle + current_angle) / 2.0;
        }

        last_visible_angle = current_angle;
    }

    return WIDTH_SCAN_MAX_DEG;
}

float scanObjectWidth(void) {
    int center_distance_mm = readAverageDistanceForScan(WIDTH_SCAN_AVERAGE_COUNT);

    if (!isDistanceReadingValidForScan(center_distance_mm)) {
        printf("Width scan failed: invalid center distance.\n");
        return 0.0;
    }

    if (center_distance_mm > FRONT_OBJECT_THRESHOLD_MM + WIDTH_SCAN_DISTANCE_MARGIN_MM) {
        printf("Width scan failed: object is too far away. Distance=%d mm\n",
               center_distance_mm);
        return 0.0;
    }

    printf("Width scan started. Center distance = %d mm\n", center_distance_mm);

    float left_total_turned = 0.0;
    float left_edge_deg = scanOneObjectEdge(-1, center_distance_mm, &left_total_turned);

    turn(left_total_turned, WIDTH_SCAN_TURN_SPEED);
    sleep_msec(150);

    float right_total_turned = 0.0;
    float right_edge_deg = scanOneObjectEdge(+1, center_distance_mm, &right_total_turned);

    turn(-right_total_turned, WIDTH_SCAN_TURN_SPEED);
    sleep_msec(150);

    float angular_width_deg = left_edge_deg + right_edge_deg;

    if (angular_width_deg <= 0.0) {
        printf("Width scan failed: angular width is zero.\n");
        return 0.0;
    }

    float distance_cm = center_distance_mm / 10.0;
    float angular_width_rad = degToRad(angular_width_deg);

    float width_cm = 2.0 * distance_cm * tan(angular_width_rad / 2.0);

    printf("Width scan result: left=%.2f deg, right=%.2f deg, total=%.2f deg, width=%.2f cm\n",
           left_edge_deg,
           right_edge_deg,
           angular_width_deg,
           width_cm);

    return width_cm;
}