#include <libpynq.h>
#include <stdio.h>
#include <math.h>

/*
 * Wiring:
 *
 * PYNQ 3.3V  -> 10k fixed resistor -> A3 -> NTCC-10K -> GND
 *
 * Because the PYNQ analog input behaves like it has about 3.3k to GND,
 * the ADC input is in parallel with the thermistor.
 *
 * So the circuit is actually:
 *
 * 3.3V -> R_FIXED -> A3 -> NTC -> GND
 *                    |
 *                    -> R_ADC -> GND
 */

#define TEMP_ADC_CHANNEL ADC3   // A3 pin

#define R_FIXED_KOHM 10.0       // Your real fixed resistor value
#define R_ADC_KOHM   3.4        // Measured resistance from ADC pin to GND

#define R0_KOHM      10.0       // NTC resistance at 25 C
#define BETA         4050.0     // Beta value from NTCC-10K datasheet
#define T0_K         298.15     // 25 C in Kelvin

#define V_REF        3.226      // Use your measured supply/reference voltage

double resistance_to_temperature(double r_ntc_kohm)
{
    double temperature_k;

    temperature_k = 1.0 / ((log(r_ntc_kohm / R0_KOHM) / BETA) + (1.0 / T0_K));

    return temperature_k - 273.15;
}

double calculate_ntc_resistance(double v_out, double v_ref)
{
    double r_parallel;
    double r_ntc;

    /*
     * First calculate the total lower resistance seen by the divider.
     *
     * Circuit:
     *
     * v_ref -> R_FIXED -> A3 -> lower resistance -> GND
     *
     * lower resistance = NTC || ADC input resistance
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
        return -1.0; // impossible / invalid value
    }

    r_ntc = (r_parallel * R_ADC_KOHM) / (R_ADC_KOHM - r_parallel);

    return r_ntc;
}

int main(void)
{
    pynq_init();
    adc_init();
    buttons_init();

    double v_out;
    double r_parallel;
    double r_ntc;
    double temperature;

    printf("NTCC-10K temperature sensor test\n");
    printf("Press BUTTON0 to stop.\n\n");

    while (!get_button_state(BUTTON0))
    {
        v_out = adc_read_channel(TEMP_ADC_CHANNEL);

        if (v_out <= 0.0 || v_out >= V_REF)
        {
            printf("Invalid ADC reading: V_out = %f V, V_ref = %f V\n",
                   v_out, V_REF);
        }
        else
        {
            r_parallel = R_FIXED_KOHM * v_out / (V_REF - v_out);
            r_ntc = calculate_ntc_resistance(v_out, V_REF);

            if (r_ntc < 0.0)
            {
                printf("Invalid resistance calculation: V_out = %f V, R_parallel = %f kOhm\n",
                       v_out, r_parallel);
            }
            else
            {
                temperature = resistance_to_temperature(r_ntc);

                printf("V_out: %f V, R_parallel: %f kOhm, R_NTC: %f kOhm, temperature: %f C\n",
                       v_out, r_parallel, r_ntc, temperature);
            }
        }

        sleep_msec(1000);
    }

    adc_destroy();
    buttons_destroy();
    pynq_destroy();

    return 0;
}