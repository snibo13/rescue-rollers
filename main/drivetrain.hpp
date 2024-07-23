#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "std_msgs/msg/int32.h"

#include "rr_custom_messages/msg/filtered_teleop.h"

#define SPEED_MODE LEDC_LOW_SPEED_MODE
#define PWM_FREQ 5000
#define PWM_RES LEDC_TIMER_10_BIT

const static ledc_timer_config_t ledc_timer = {
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
    .pwma = GPIO_NUM_3,
    .pwmb = GPIO_NUM_4,
    .channela = LEDC_CHANNEL_2,
    .channelb = LEDC_CHANNEL_3,
};

rcl_subscription_t speed_sub;
std_msgs__msg__Int32 speed_msg;
rcl_subscription_t r_speed_sub;
std_msgs__msg__Int32 r_speed_msg;

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

void speed_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    int32_t left_velocity = msg->data;
    int32_t right_velocity = msg->data;
    int32_t speed_left = abs(left_velocity);
    int32_t speed_right = abs(right_velocity);
    if (left_velocity > 0)
    {
        set_motor_speed(left_motor, speed_left, true);
    }
    else
    {
        set_motor_speed(left_motor, speed_left, false);
    }
}

void r_speed_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    int32_t right_velocity = msg->data;
    int32_t speed_right = abs(right_velocity);

    if (right_velocity > 0)
    {
        set_motor_speed(right_motor, speed_right, true);
    }
    else
    {
        set_motor_speed(right_motor, speed_right, false);
    }
}