#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "mpu6050.h"

static mpu6050_handle_t mpu;

void initializeMPU6050()
{
    mpu = mpu6050_create(I2C_NUM_0, 0x68);
    ESP_ERROR_CHECK(mpu6050_wake_up(mpu));
}

void readMPU6050()
{
    mpu6050_data_t data;
    ESP_ERROR_CHECK(mpu6050_get_acce(mpu, &data));
    ESP_LOGI("MPU6050", "Accel: x=%d, y=%d, z=%d", data.accel_x, data.accel_y, data.accel_z);
    ESP_ERROR_CHECK(mpu6050_get_gyro(mpu, &data));
    ESP_LOGI("MPU6050", "Gyro: x=%d, y=%d, z=%d", data.gyro_x, data.gyro_y, data.gyro_z);
}