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

// Custom
#include "dynamixel.h"

static const char *TAG = "main";

extern "C" void app_main(void)
{
    // ESP_LOGI(TAG, "Hello, world!");

    ESP_LOGI(TAG, "Starting test");
    setup_uart();

    uint8_t servo_id = 1;

    // LED Packet
    {
        uint8_t cmd = 0x03;             // Write to control table
        uint8_t addr[2] = {0x41, 0x00}; // LED
        uint8_t value[1] = {0x01};      // Turn on LED
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
    }
    sleep(1);

    // Torque enable packet
    {
        uint8_t cmd = 0x03;           // Write to control table
        uint8_t addr[2] = {64, 0x00}; // Torque enable
        uint8_t value[1] = {0x01};    // Enable
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
    }
    sleep(1);

    // Torque enable packet
    {

        uint8_t cmd = 0x03;              // Write to control table
        uint8_t addr[2] = {116, 0x00};   // Goal position
        uint8_t value[4] = {0, 2, 0, 0}; // Enable
        size_t length = 3 + 6;
        size_t full_packet_size = 7 + length;
        uint8_t *packet = (uint8_t *)malloc(sizeof(uint8_t) * full_packet_size);
        memset(packet, 0, full_packet_size);
        generatePacket(
            packet,   // Packet buffer
            servo_id, // Servo id
            length,   // length
            cmd,      // instruction
            addr, 2,  // address and size
            value, 4  // Parameters and size
        );
        sendPacket(packet, full_packet_size, uart_num);
    }

    sleep(1);

    // Torque enable packet
    {

        uint8_t cmd = 0x03;              // Write to control table
        uint8_t addr[2] = {116, 0x00};   // Goal position
        uint8_t value[4] = {0, 0, 0, 0}; // Enable
        size_t length = 3 + 6;
        size_t full_packet_size = 7 + length;
        uint8_t *packet = (uint8_t *)malloc(sizeof(uint8_t) * full_packet_size);
        memset(packet, 0, full_packet_size);
        generatePacket(
            packet,   // Packet buffer
            servo_id, // Servo id
            length,   // length
            cmd,      // instruction
            addr, 2,  // address and size
            value, 4  // Parameters and size
        );
        sendPacket(packet, full_packet_size, uart_num);
    }

    sleep(1);

    // Torque enable packet

    {
        uint8_t cmd = 0x03;           // Write to control table
        uint8_t addr[2] = {64, 0x00}; // Torque enable
        uint8_t value[1] = {0x00};    // Disable
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
    }
    sleep(1);

    // LED Packet
    {
        uint8_t cmd = 0x03;             // Write to control table
        uint8_t addr[2] = {0x41, 0x00}; // LED
        uint8_t value[1] = {0x00};      // Turn off LED
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
    }
    sleep(1);
}
