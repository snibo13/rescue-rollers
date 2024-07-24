#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "rescue_roller_custom_messages/msg/filtered_teleop.h"

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

rcl_subscription_t teleop_sub;
rescue_roller_custom_messages__msg__FilteredTeleop teleop_msg;

int max_wheel_speed = floor(100 * 1 / 60 * 2 * 3.1415); // 100 rpm = 100 * 1/60 * 2 * pi rad/s

void differential_drive_to_wheel_speed(float v, float w, float *left, float *right)
{

    *left = v - w * CONFIG_WHEEL_BASE / 2;
    *right = v + w * CONFIG_WHEEL_BASE / 2;
}

void wheel_speed_to_pwm(float speed, float *pwm)
// NOTE: Output is a signed pwm value
{
    if (speed > max_wheel_speed)
    {
        speed = max_wheel_speed;
    }
    else if (speed < -max_wheel_speed)
    {
        speed = -max_wheel_speed;
    }
    *pwm = speed / max_wheel_speed * 255;
}

void differential_drive_to_pwm(float v, float w, float *left_pwm, float *right_pwm)
{
    float left_speed, right_speed;
    differential_drive_to_wheel_speed(v, w, &left_speed, &right_speed);
    wheel_speed_to_pwm(left_speed, left_pwm);
    wheel_speed_to_pwm(right_speed, right_pwm);
}

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

void teleop_callback(const void *msgin)
{
    const rescue_roller_custom_messages__msg__FilteredTeleop *msg = (const rescue_roller_custom_messages__msg__FilteredTeleop *)msgin;
    uint8_t id = msg->id;
    if (id == CONFIG_ROBOT_ID)
    {

        float v = msg->v;
        float w = msg->w;
        float left_speed, right_speed;
        differential_drive_to_pwm(v, w, &left_speed, &right_speed);
        set_motor_speed(left_motor, left_speed, left_speed > 0);
        set_motor_speed(right_motor, right_speed, right_speed > 0);
    }
}