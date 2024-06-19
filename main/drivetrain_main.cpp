#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

// #include "drivetrain.hpp"

#define SPEED_MODE LEDC_LOW_SPEED_MODE
#define PWM_FREQ 5000
#define PWM_RES LEDC_TIMER_10_BIT

#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39

#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13

static ledc_timer_config_t ledc_timer = {
    .speed_mode = SPEED_MODE,
    .duty_resolution = PWM_RES,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK,
};
typedef struct motor
{
    gpio_num_t pwma;
    gpio_num_t pwmb;
    ledc_channel_t channela;
    ledc_channel_t channelb;
} motor_t;

motor_t left_motor = {
    .pwma = GPIO_NUM_1,
    .pwmb = GPIO_NUM_2,
    .channela = LEDC_CHANNEL_1,
    .channelb = LEDC_CHANNEL_0,
};

motor_t right_motor = {
    .pwma = GPIO_NUM_4,
    .pwmb = GPIO_NUM_3,
    .channela = LEDC_CHANNEL_2,
    .channelb = LEDC_CHANNEL_3,
};

void initialise_motor(motor_t m)
{
    ledc_channel_config_t ledc_channel_cf = {
        .gpio_num = m.pwma,
        .speed_mode = SPEED_MODE,
        .channel = m.channela,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = LEDC_FADE_NO_WAIT};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_cf));

    ledc_channel_cf = {
        .gpio_num = m.pwmb,
        .speed_mode = SPEED_MODE,
        .channel = m.channelb,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_cf));
}

void set_motor_speed(motor_t m, int speed, bool forward = true)
{
    if (forward)
    {
        ESP_ERROR_CHECK(ledc_set_duty(SPEED_MODE, m.channela, speed));
        ESP_ERROR_CHECK(ledc_set_duty(SPEED_MODE, m.channelb, 0));
    }
    else
    {
        ESP_ERROR_CHECK(ledc_set_duty(SPEED_MODE, m.channela, 0));
        ESP_ERROR_CHECK(ledc_set_duty(SPEED_MODE, m.channelb, speed));
    }
    ESP_ERROR_CHECK(ledc_update_duty(SPEED_MODE, m.channela));
    ESP_ERROR_CHECK(ledc_update_duty(SPEED_MODE, m.channelb));
}

extern "C"
{
    void app_main(void)
    {
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
        initialise_motor(left_motor);
        initialise_motor(right_motor);
        printf("Motors initialized\n");
        while (1)
        {
            printf("Forward\n");
            set_motor_speed(left_motor, 1024, true);
            set_motor_speed(right_motor, 1024, true);
            vTaskDelay(2000 / portTICK_PERIOD_MS);

            set_motor_speed(left_motor, 0, false);
            set_motor_speed(right_motor, 0, false);
            vTaskDelay(2000 / portTICK_PERIOD_MS);

            printf("Backward\n");
            set_motor_speed(left_motor, 1024, false);
            set_motor_speed(right_motor, 1024, false);
            vTaskDelay(2000 / portTICK_PERIOD_MS);

            set_motor_speed(left_motor, 0, false);
            set_motor_speed(right_motor, 0, false);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }
}