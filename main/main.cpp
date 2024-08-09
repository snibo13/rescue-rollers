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

// const static gpio_num_t TXD = GPIO_NUM_5;
// const static gpio_num_t RXD = GPIO_NUM_8;
// const static gpio_num_t RTS = GPIO_NUM_6;
#define TXD GPIO_NUM_5
#define RXD GPIO_NUM_8
#define RTS GPIO_NUM_4

#define BUF_SIZE 127
#define TX_DIRECTION 1
#define RX_DIRECTION 0

static const char *TAG = "main";

// Function to send packet to Dynamixel (implementation depends on your communication method, e.g., serial port)
void sendPacket(uint8_t *packet, size_t length, uart_port_t uart_num)
{
    gpio_set_level(RTS, TX_DIRECTION);
    while (gpio_get_level(RTS) != TX_DIRECTION)
    {
        ESP_LOGI(TAG, "Waiting for RTS (TX)");
        printf("RTS: %d\n", gpio_get_level(RTS));
        gpio_set_level(RTS, TX_DIRECTION);
        // uart_set_rts(uart_num, TX_DIRECTION);
        sleep(1);
    }
    printf("\n");
    int written = uart_write_bytes(uart_num, (const char *)packet, length);
    ESP_LOGI(TAG, "Bytes written: %d", written);
    assert(written == length);
    ESP_LOGI(TAG, "Packet sent");
    ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 100));
    ESP_ERROR_CHECK(uart_flush(uart_num));
    gpio_set_level(RTS, RX_DIRECTION);
    while (gpio_get_level(RTS) != RX_DIRECTION)
    {
        ESP_LOGI(TAG, "Waiting for RTS (RX)");
        printf("RTS: %d\n", gpio_get_level(RTS));
        // uart_set_rts(uart_num, RX_DIRECTION);
        gpio_set_level(RTS, RX_DIRECTION);
        sleep(1);
    }
}

// Function to read packet from Dynamixel (implementation depends on your communication method, e.g., serial port)
void readPacket(uint8_t *data, uart_port_t uart_num)
{
    ESP_LOGI(TAG, "Reading packet");

    int length = 0;
    // ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t *)&length));
    length = uart_read_bytes(uart_num, data, 127, 0);
    if (length == 0)
    {
        ESP_LOGI(TAG, "No data available");
        return;
    }
    ESP_LOGI(TAG, "Reading packet");
    uart_read_bytes(uart_num, data, length, 100);
    ESP_LOGI(TAG, "Packet read");
    length = strlen((char *)data);
    ESP_LOGI(TAG, "Length: %d", length);
    for (size_t i = 0; i < length; ++i)
    {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

// From Robotics
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
// Parameters:
// crc_accum: The initial CRC value (0)
// data_blk_ptr: The data block to calculate the CRC for
// data_blk_size: The size of the data block

{
    ESP_LOGI(TAG, "CRC");
    for (size_t i = 0; i < data_blk_size; ++i)
    {
        printf("%02X ", data_blk_ptr[i]);
    }
    printf("\n");
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

// Generates an instruction packet for Dynamixel Protocol 2.0
uint8_t *generatePacket(uint8_t *packet, uint8_t id, uint8_t length, uint8_t instruction, uint8_t *addr, size_t addr_size, uint8_t *data, size_t data_size)
{
    // Header (FF FF FD 00) ID (1 byte) Length (2 bytes) Instruction (1) Parameters (length) CRC (2)
    // assert(addr_size + data_size + 2 == length);
    // uint8_t packet[4 + 1 + 2 + 1 + length + 2];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;
    packet[5] = length & 0xFF;
    packet[6] = (length >> 8) & 0xFF;
    packet[7] = instruction;

    for (size_t i = 0; i < addr_size; ++i)
    {
        packet[8 + i] = addr[i];
    }

    for (size_t i = 0; i < data_size; ++i)
    {
        packet[8 + addr_size + i] = data[i];
    }

    uint16_t crc = update_crc(0, packet, 5 + length); // ID + Length + Instruction + Parameters (Length is same space as CRC)
    packet[8 + addr_size + data_size] = crc & 0xFF;
    packet[9 + addr_size + data_size] = (crc >> 8) & 0xFF;

    for (size_t i = 0; i < 4 + 1 + 2 + length; ++i) // Header + ID + Length
    {
        printf("%02X ", packet[i]);
    }
    printf("\n");

    return packet;
}

void valueToBytes(uint8_t *bytes, uint16_t value, size_t size)
{
    // Convert an integer into bytes
    for (size_t i = 0; i < size; ++i)
    {
        bytes[i] = (value >> (i * 8)) & 0xFF;
    }
}

// the entry point for the program
// it must be declared as "extern C" because the compiler assumes this will be a C function

#define gpio_put gpio_set_level
extern "C" void app_main(void)
{
    // ESP_LOGI(TAG, "Hello, world!");

    const uart_port_t uart_num = UART_NUM_0;
    // ESP_LOGI(TAG, "UART number: %d", uart_num);
    uart_config_t uart_config = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT};

    // ESP_LOGI(TAG, "Configuring UART");
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    // ESP_LOGI(TAG, "UART configured");

    // ESP_LOGI(TAG, "Setting UART pins");
    ESP_ERROR_CHECK(uart_set_pin(uart_num, TXD, RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // ESP_LOGI(TAG, "UART pins set");

    // ESP_LOGI(TAG, "Installing driver");
    // uart_port_t uart_num, int rx_buffer_size, int tx_buffer_size, int queue_size, QueueHandle_t *uart_queue, int intr_alloc_flags
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
    // ESP_LOGI(TAG, "UART driver installed");

    // ESP_LOGI(TAG, "Setting Mode");
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));
    // ESP_LOGI(TAG, "Mode set");

    // Since we're communicating half fuplex we also have to configure the direction pin
    // ESP_LOGI(TAG, "Configuring GPIO");

    ESP_ERROR_CHECK(gpio_set_direction(RTS, GPIO_MODE_INPUT_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(RTS, GPIO_PULLDOWN_ONLY)); // Default to read
    // ESP_LOGI(TAG, "GPIO configured");

    // ESP_LOGI(TAG, "Starting test");

    // uint8_t servo_id = 1; // Replace with your servo ID
    // // Ping packet
    // uint8_t ping_packet_length = 3;
    // uint8_t *ping_packet = (uint8_t *)malloc(sizeof(uint8_t) * 7 + ping_packet_length);
    // if (ping_packet == NULL)
    // {
    //     ESP_LOGE(TAG, "Memory allocation failed");
    //     return;
    // }
    // generatePacket(ping_packet, servo_id, ping_packet_length, 0x01, NULL, 0, NULL, 0);
    // for (size_t i = 0; i < 7 + ping_packet_length; ++i) // Header + ID + Length
    // {
    //     printf("%02X ", ping_packet[i]);
    // }
    // printf("\n");

    // uint8_t *buff = (uint8_t *)malloc(BUF_SIZE);
    // if (buff == NULL)
    // {
    //     ESP_LOGE(TAG, "Memory allocation failed");
    //     free(ping_packet);
    //     return;
    // }
    // memset(buff, 0, BUF_SIZE);

    // sendPacket(ping_packet, 7 + ping_packet_length, uart_num);
    uint8_t data[] = {};
    // uart_write_bytes(uart_num, data, 1);
    // Write data to UART, end with a break signal.
    uart_write_bytes_with_break(uart_num, "test break\n", strlen("test break\n"), 100);
    // sendPacket(ping_packet, 1, uart_num);
    // ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 100));
    // ESP_ERROR_CHECK(uart_set_rts(uart_num, RX_DIRECTION));
    // sleep(1);
    // for (int i = 0; i < 10; i++)
    // {
    //     readPacket(buff, uart_num);
    //     sleep(2);
    // }

    // free(ping_packet);
    // free(buff);
    // readPacket(buff, BUF_SIZE, uart_num);

    // return 0;
}
