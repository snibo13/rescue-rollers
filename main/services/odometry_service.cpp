#include <math.h>
#include "Encoder.h"
#include "kalman.hpp"

#include <eigen3/Eigen/Core>

#include "Freertos/FreeRTOS.h"
#include "Freertos/task.h"

/* Constants */
#define LEFT_ENCODER_A 6
#define LEFT_ENCODER_B 7
#define RIGHT_ENCODER_A 8
#define RIGHT_ENCODER_B 9

const struct odometryState
{
    double x;
    double y;
    double theta;
} odometryState = {0, 0, 0};

/* Function Headers */
void read_encoders();
void read_imu();
void update_state();
void odometry_task(void *arg);
BaseType_t odometry_service(void);

/* Global Variables */
static int32_t oldPositionLeft = -999;
static int32_t oldPositionRight = -999;
static int32_t newPositionLeft = 0;
static int32_t newPositionRight = 0;
static bool positionChanged = false;
static Encoder leftEncoder(LEFT_ENCODER_A, LEFT_ENCODER_B);
static Encoder rightEncoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B);

Vector2f z_encoder;
Vector4f z_gyro;
Vector2f z_optical;

void odometry_task(void *arg)
{
    for (;;)
    {
        read_encoders();
        read_imu();
        update_state();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void read_encoders()
{
    newPositionLeft = leftEncoder.read();
    newPositionRight = rightEncoder.read();
}

void read_imu()
{
    continue;
}

void read_optical()
{
    continue;
}

void update_state()
{
    z_encoder = {newPositionLeft, newPositionRight};
    z_gyro = {0, 0, 0, 0};
    z_optical = {0, 0};
    ekf(z_encoder, z_gyro, z_optical, 1);
}

BaseType_t odometry_service(void)
{
    BaseType_t status;
    status = xTaskCreate(
        encoder_task,
        "encoder_task",
        4 * CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL);

    if (status == pdPASS)
    {
        ESP_LOGI(TAG, "service started");
    }
    else
    {
        ESP_LOGI(TAG, "Error starting the service");
    }
    return status;
}
