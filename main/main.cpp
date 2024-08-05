#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"

// Custom
// #include "camera.hpp"
// #include "drivetrain.hpp"
#include "Esp32s3Hal.hpp"
#include <RadioLib.h>
// CLK (D8), MISO (D9), MOSI(D10), CS (D7)
Esp32S3Hal *hal = new Esp32S3Hal(7, 8, 9, 44);
// NSS (D0), DIO0 (D1), RST(D2), DIO1(D3)
SX1278 radio = new Module(hal, 1, 2, 43, 4);

static const char *TAG = "main";

// the entry point for the program
// it must be declared as "extern C" because the compiler assumes this will be a C function
extern "C" void app_main(void)
{
    // initialize just like with Arduino
    ESP_LOGI(TAG, "[SX1276] Initializing ... ");
    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE)
    {
        ESP_LOGI(TAG, "failed, code %d\n", state);
        while (true)
        {
            hal->delay(1000);
        }
    }
    ESP_LOGI(TAG, "success!\n");

    // loop forever
    for (;;)
    {
        // send a packet
        ESP_LOGI(TAG, "[SX1276] Transmitting packet ... ");
        state = radio.transmit("Hello World!");
        if (state == RADIOLIB_ERR_NONE)
        {
            // the packet was successfully transmitted
            ESP_LOGI(TAG, "success!");
        }
        else
        {
            ESP_LOGI(TAG, "failed, code %d\n", state);
        }

        // wait for a second before transmitting again
        hal->delay(1000);
    }
}