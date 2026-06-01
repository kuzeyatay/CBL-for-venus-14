#include <libpynq.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

int main(void) {
    pynq_init();

    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);

    iic_init(IIC0);
    sleep_msec(200);

    printf("\n--- I2C scanner on IIC0 / AR_SCL / AR_SDA ---\n");

    for (uint8_t addr = 1; addr < 127; addr++) {
        uint8_t dummy = 0;

        /*
         * Try reading register 0x00.
         * Some devices may not like this, but for basic detection it is useful.
         */
        int result = iic_read_register(IIC0, addr, 0x00, &dummy, 1);

        if (result == 0) {
            printf("Found device at 0x%02X\n", addr);
        }

        sleep_msec(5);
    }

    printf("Scan finished.\n");

    iic_destroy(IIC0);
    pynq_destroy();

    return 0;
}