#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "dynamixel.h"

// Custom

// const static gpio_num_t TXD = GPIO_NUM_5;
// const static gpio_num_t RXD = GPIO_NUM_8;
// const static gpio_num_t RTS = GPIO_NUM_6;
#define RX 4
#define TX 9
#define RTS 3
// #define RXE GPIO_NUM_44

#define BUF_SIZE 127
#define TX_DIRECTION 0
#define RX_DIRECTION 1

static const char *TAG = "main";

#define uart_num UART_NUM_1

void setup_uart()
{
    uart_config_t uart_config = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TX, RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, 0);
    uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX);

    // gpio_set_direction(RTS, GPIO_MODE_INPUT_OUTPUT);
}

void ping()
{
    uint8_t servo_id = 1;

    // Ping packet
    uint8_t ping_packet_length = 3;
    uint8_t *ping_packet = (uint8_t *)malloc(sizeof(uint8_t) * 7 + ping_packet_length);
    if (ping_packet == NULL)
    {
        ESP_LOGE(TAG, "Memory allocation failed");
        return;
    }
    generatePacket(ping_packet, servo_id, ping_packet_length, 0x01, NULL, 0, NULL, 0);
    for (size_t i = 0; i < 7 + ping_packet_length; ++i) // Header + ID + Length
    {
        printf("%02X ", ping_packet[i]);
    }
    printf("\n");

    uint8_t *buff = (uint8_t *)malloc(BUF_SIZE);
    if (buff == NULL)
    {
        ESP_LOGE(TAG, "Memory allocation failed");
        free(ping_packet);
        return;
    }
    memset(buff, 0, BUF_SIZE);
    size_t len = 0;

    for (int i = 0; i < 10; i++)
    {
        sendPacket(ping_packet, 7 + ping_packet_length, uart_num);
        ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 100));
        readPacket(buff, len, uart_num);
        sleep(2);
    }

    free(ping_packet);
    free(buff);
}
extern "C" void app_main(void)
{
    // ESP_LOGI(TAG, "Hello, world!");

    ESP_LOGI(TAG, "Starting test");
    setup_uart();

    uint8_t servo_id = 1;

    // LED Packet
    uint8_t cmd = 0x03;             // Write to control table
    uint8_t addr[2] = {0x41, 0x00}; // LED
    uint8_t value[1] = {0x00};      // Turn on LED
    size_t length = 3 + 3;
    size_t full_packet_size = 7 + length;
    uint8_t *packet = (uint8_t *)malloc(sizeof(uint8_t) * full_packet_size);
    memset(packet, 0, full_packet_size);
    generatePacket(
        packet,   // Packet buffer
        servo_id, // Servo id
        length,   // length
        cmd,      // instruction
        addr, 2,  // address and size
        value, 1  // Parameters and size
    );
    sendPacket(packet, full_packet_size, uart_num);

    // Ping packet
    cmd = 0x01; // Ping
    // addr = (void *)(0);
    // value = (void *)(0);
    length = 3;
    uint8_t *ping_packet = (uint8_t *)malloc(sizeof(uint8_t) * 4 + 2 + 1 + length);
    generatePacket(
        ping_packet, // Packet buffer
        servo_id,    // Servo id
        length,      // length
        cmd,         // instruction
        addr, 0,     // address and size
        value, 0     // Parameters and size
    );

    // return 0;
}
