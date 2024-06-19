#pragma once

#include "esp_log.h"
#include "bdc_motor.h"
#include "driver/gpio.h"

#define LEFT_A GPIO_NUM_2
#define LEFT_B GPIO_NUM_3

#define RIGHT_A GPIO_NUM_5
#define RIGHT_B GPIO_NUM_4

#define PWM_TIMER_RES 10000000
#define PWM_FREQ_HZ 25000
#define DUTY_MAX (PWM_TIMER_RES / PWM_FREQ_HZ)

static bdc_motor_handle_t left_motor;
static bdc_motor_handle_t right_motor;

void initialise_motors()
{

    bdc_motor_config_t left_motor_config = {
        .pwma_gpio_num = LEFT_A,
        .pwmb_gpio_num = LEFT_B,
        .pwm_freq_hz = PWM_FREQ_HZ,
    };
    bdc_motor_config_t right_motor_config = {
        .pwma_gpio_num = RIGHT_A,
        .pwmb_gpio_num = RIGHT_B,
        .pwm_freq_hz = PWM_FREQ_HZ,

    };

    bdc_motor_mcpwm_config_t pwm_config_l = {
        .group_id = 0,
        .resolution_hz = PWM_TIMER_RES,
    };

    bdc_motor_mcpwm_config_t pwm_config_r = {
        .group_id = 1,
        .resolution_hz = PWM_TIMER_RES,
    };

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&left_motor_config, &pwm_config_l, &left_motor));
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&right_motor_config, &pwm_config_r, &right_motor));

    ESP_ERROR_CHECK(bdc_motor_enable(left_motor));
    ESP_ERROR_CHECK(bdc_motor_enable(right_motor));
}

void set_motor_speeds(int left_speed, int right_speed)
{
    ESP_ERROR_CHECK(bdc_motor_set_speed(right_motor, right_speed));
    ESP_ERROR_CHECK(bdc_motor_set_speed(left_motor, left_speed));
}

void set_motor_directions(bool left_forward, bool right_forward)
{
    if (left_forward)
    {
        ESP_ERROR_CHECK(bdc_motor_forward(left_motor));
    }
    else
    {
        ESP_ERROR_CHECK(bdc_motor_reverse(left_motor));
    }

    if (right_forward)
    {
        ESP_ERROR_CHECK(bdc_motor_forward(right_motor));
    }
    else
    {
        ESP_ERROR_CHECK(bdc_motor_reverse(right_motor));
    }
}
