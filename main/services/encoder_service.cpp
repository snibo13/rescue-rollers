#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "uros.h"
#define TAG "encoder_service"

#include "Arduino.h"
#include "Encoder.h"

#pragma message "Building encoder service"


static Encoder encoder(ENCODER_A, ENCODER_B);
static int32_t newPosition = 0;
static int32_t oldPosition = -999;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 msgIn;

void subscription_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *temp = (const std_msgs__msg__Int32 *)msgin;
    ESP_LOGI(TAG, "Received %d", (int)temp->data);
}

void encoder_task(void *arg)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Static Agent IP and port can be used instead of autodisvery.
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    // RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "encoder", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "encoder_cnt_left"));

    // create sub
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "drive_left"));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msgIn, &subscription_callback, ON_NEW_DATA));

    msg.data = 0;

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        newPosition = encoder.read();
        if (newPosition != oldPosition)
        {
            oldPosition = newPosition;
            ESP_LOGI(TAG, "Encoder position: %ld", newPosition);
            msg.data = newPosition;
            ESP_LOGI(TAG, "Publishing!");
            RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
            ESP_LOGI(TAG, "Published");
        }
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

BaseType_t encoder_service(void)
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
