#pragma once

#include "esp_camera.h"
#include "driver/gpio.h"
#include <sensor_msgs/msg/compressed_image.h>
#include <rmw_microros/rmw_microros.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/compressed_image.h>
#include "esp_log.h"

#include "uros.h"

const char *TAG = "Camera";

#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 10
#define CAM_PIN_SIOD 40
#define CAM_PIN_SIOC 39

#define CAM_PIN_D7 48
#define CAM_PIN_D6 11
#define CAM_PIN_D5 12
#define CAM_PIN_D4 14
#define CAM_PIN_D3 16
#define CAM_PIN_D2 18
#define CAM_PIN_D1 17
#define CAM_PIN_D0 15
#define CAM_PIN_VSYNC 38
#define CAM_PIN_HREF 47
#define CAM_PIN_PCLK 13

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 20000000, // EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,   // QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 12,                 // 0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1,                      // When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_PSRAM,  // PSRAM or PSRAM
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY // CAMERA_GRAB_LATEST. Sets when buffers should be filled
};

static rcl_publisher_t cam_publisher;
static sensor_msgs__msg__CompressedImage image_msg;

esp_err_t camera_init()
{
    // initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

void initialise_image_msg(sensor_msgs__msg__CompressedImage *image_msg)
{
    image_msg->data.capacity = 10000;
    image_msg->data.data = (uint8_t *)malloc(image_msg->data.capacity * sizeof(uint8_t));
    image_msg->data.size = 0;

    image_msg->format.capacity = 10;
    image_msg->format.data = (char *)malloc(image_msg->format.capacity * sizeof(char));
    image_msg->format.size = 0;

    image_msg->header.frame_id.capacity = 10;
    image_msg->header.frame_id.data = (char *)malloc(image_msg->header.frame_id.capacity * sizeof(char));
    image_msg->header.frame_id.size = 0;
}

void publish_image_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer != NULL)
    {
        // ESP_LOGI(TAG, "Taking picture...");
        camera_fb_t *pic = esp_camera_fb_get();

        if (pic != NULL)
        {
            // ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", pic->len);
            if (pic->len <= image_msg.data.capacity)
            {
                image_msg.data.size = pic->len;
                memcpy(image_msg.data.data, pic->buf, pic->len);
                image_msg.header.frame_id = micro_ros_string_utilities_set(image_msg.header.frame_id, "RR Frame");
                image_msg.format = micro_ros_string_utilities_set(image_msg.format, "jpeg");
                RCSOFTCHECK(rcl_publish(&cam_publisher, &image_msg, NULL));
            }
            else
            {
                ESP_LOGE(TAG, "Image too large");
            }
        }
        esp_camera_fb_return(pic);
    }
}

void add_camera_publisher_to_ros(rcl_node_t *node, rclc_support_t *support, uint timer_timeout, rclc_executor_t *executor)
{
    // Create publisher.
    RCCHECK(rclc_publisher_init_default(
        &cam_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
        "image_publisher"));

    // Create timer.
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    RCCHECK(rclc_timer_init_default(
        &timer,
        support,
        RCL_MS_TO_NS(timer_timeout),
        publish_image_callback));

    RCCHECK(rclc_executor_add_timer(executor, &timer));
}