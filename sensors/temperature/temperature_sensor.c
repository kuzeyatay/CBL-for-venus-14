#include <stdbool.h>
#include <math.h>
#include <libpynq.h>

#include "temperature_sensor.h"

/*
 * Wiring:
 *
 * PYNQ 3.3V -> 10k fixed resistor -> A3 -> NTCC-10K -> GND
 *
 * Because the PYNQ analog input behaves like it has about 3.4k to GND,
 * the ADC input is in parallel with the thermistor.
 */

#define TEMP_ADC_CHANNEL ADC3

#define R_FIXED_KOHM 10.0
#define R_ADC_KOHM   3.4

#define R0_KOHM      10.0
#define BETA         4050.0
#define T0_K         298.15

#define V_REF        3.226

#define INVALID_TEMPERATURE -999.0f

static bool temperature_sensor_initialized = false;

static double resistanceToTemperature(double r_ntc_kohm)
{
    double temperature_k;

    temperature_k = 1.0 / ((log(r_ntc_kohm / R0_KOHM) / BETA) + (1.0 / T0_K));

    return temperature_k - 273.15;
}

static double calculateNTCResistance(double v_out, double v_ref)
{
    double r_parallel;
    double r_ntc;

    /*
     * First calculate the resistance seen at the ADC node:
     *
     * 3.3V -> R_FIXED -> A3 -> lower resistance -> GND
     *
     * lower resistance = R_NTC || R_ADC
     */
    r_parallel = R_FIXED_KOHM * v_out / (v_ref - v_out);

    /*
     * r_parallel = (R_NTC * R_ADC) / (R_NTC + R_ADC)
     *
     * Solving for R_NTC:
     *
     * R_NTC = (r_parallel * R_ADC) / (R_ADC - r_parallel)
     */

    if (r_parallel <= 0.0 || r_parallel >= R_ADC_KOHM)
    {
        return -1.0;
    }

    r_ntc = (r_parallel * R_ADC_KOHM) / (R_ADC_KOHM - r_parallel);

    return r_ntc;
}

bool initNTCTemperatureSensor(void)
{
    /*
     * Initialize the ADC system used by A0-A5.
     * The temperature sensor itself is passive, so it needs no digital setup.
     */
    adc_init();

    temperature_sensor_initialized = true;

    return true;
}

float readNTCTemperature(void)
{
    if (!temperature_sensor_initialized)
    {
        return INVALID_TEMPERATURE;
    }

    double v_out;
    double r_ntc;
    double temperature;

    v_out = adc_read_channel(TEMP_ADC_CHANNEL);

    if (v_out <= 0.0 || v_out >= V_REF)
    {
        return INVALID_TEMPERATURE;
    }

    r_ntc = calculateNTCResistance(v_out, V_REF);

    if (r_ntc <= 0.0)
    {
        return INVALID_TEMPERATURE;
    }

    temperature = resistanceToTemperature(r_ntc);

    if (isnan(temperature) || isinf(temperature))
    {
        return INVALID_TEMPERATURE;
    }

    return (float)temperature;
}