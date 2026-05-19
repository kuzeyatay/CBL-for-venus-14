#include <libpynq.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>


#define ROBOT_ID "81"

// ---------------------------------------------------------
// PROTOCOL SEND FUNCTION
// ---------------------------------------------------------
void send_protocol_msg(const char *format, ...) {
    char payload[256];
    va_list args;
    va_start(args, format);
    vsnprintf(payload, sizeof(payload), format, args);
    va_end(args);

    uint32_t len = (uint32_t)strlen(payload);

    // Wait for ESP32 Ready Signal (AR3)
    if(gpio_get_level(IO_AR3) == GPIO_LEVEL_LOW) {
        while(gpio_get_level(IO_AR3) == GPIO_LEVEL_LOW) {
            sleep_msec(10); 
        }
    }

    // 1. Send 4-byte Length Header (Little Endian)
    uart_send(UART0, (uint8_t)(len & 0xFF));
    uart_send(UART0, (uint8_t)((len >> 8) & 0xFF));
    uart_send(UART0, (uint8_t)((len >> 16) & 0xFF));
    uart_send(UART0, (uint8_t)((len >> 24) & 0xFF));

    // 2. Send Payload Bytes (No Null Terminator over the wire)
    for(uint32_t i = 0; i < len; i++) {
        uart_send(UART0, payload[i]);
    }
}

// ---------------------------------------------------------
// PROTOCOL RECEIVE FUNCTION (Boolean-Safe)
// ---------------------------------------------------------
void handle_incoming_commands() {
    if (!uart_has_data(UART0)) return;

    // Read 4-byte header
    uint32_t len = 0;
    for (int i = 0; i < 4; i++) {
        while (!uart_has_data(UART0)); 
        len |= (uint32_t)uart_recv(UART0) << (i * 8);
    }

    if (len == 0 || len > 255) return;

    // Read payload
    char buffer[256];
    for (uint32_t i = 0; i < len; i++) {
        while (!uart_has_data(UART0));
        buffer[i] = (char)uart_recv(UART0);
    }
    buffer[len] = '\0'; // Add null terminator locally

    printf("Base Station says: %s\n", buffer);

    // SIMPLE PARSER EXAMPLE
    if (strncmp(buffer, "PING", 4) == 0) {
        send_protocol_msg("PONG,%s", ROBOT_ID);
    } else if (strncmp(buffer, "START_MISSION", 13) == 0) {
        send_protocol_msg("MISSION_STARTED,%s", ROBOT_ID);
    }
}

// ---------------------------------------------------------
// MAIN LOOP
// ---------------------------------------------------------
int main(void) {
    pynq_init();
    switchbox_init();
    buttons_init();
    adc_init();

    // Map UART0 to Pins AR0 (RX) and AR1 (TX)
    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);
    gpio_set_direction(IO_AR3, GPIO_DIR_INPUT);

    uart_init(UART0);
    uart_reset_fifos(UART0);

    // Protocol Section 8.1: Announce we are online
    send_protocol_msg("READY,%s", ROBOT_ID);

    while(!get_button_state(BUTTON0)) {
        
        // 1. Listen for commands (PING, START, etc.)
        handle_incoming_commands();

        // 2. Gather Real Sensor Data
        float temp = adc_get_temp();
        int dist = 120; // Placeholder for VL53L0X reading
        
        // 3. Send Periodic Pose/Sensor Update (Protocol Section 8.15)
        // Format: SENSOR_DATA,id,valid,black,obj,dist,width,color,temp
        send_protocol_msg("SENSOR_DATA,%s,1,0,1,%d,0.0,none,%.2f", 
                          ROBOT_ID, dist, temp);

        sleep_msec(1000);
    }

    pynq_destroy();
    return 0;
}
