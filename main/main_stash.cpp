#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"

// uROS includes
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

// Custom
#include "drivetrain.hpp"


/* Constants */
#define TAG "micro_ros_app"
#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

/* Global Variables */

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 send_msg;
std_msgs__msg__Int32 recv_msg;

/* Function Headers*/

void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void subscription_callback(const void *msgin);
void micro_ros_task(void *arg);

extern "C" void app_main(void)
{
    // #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    //     ESP_ERROR_CHECK(uros_network_interface_initialize());
    // #endif
    // ESP_LOGD(TAG, "Starting micro_ros_app");
    // initialise_motors();
    // ESP_LOGD(TAG, "Motors initialized");
    // bool direction = true;
    // set_motor_directions(direction, direction);
    // ESP_LOGD(TAG, "Direction set");

    printf("Starting micro_ros_app\n");

    gpio_set_direction(LEFT_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(LEFT_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(RIGHT_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(RIGHT_B, GPIO_MODE_OUTPUT);

    gpio_set_level(LEFT_A, 0);
    gpio_set_level(LEFT_B, 1);
    gpio_set_level(RIGHT_A, 0);
    gpio_set_level(RIGHT_B, 1);

    while (true)
    {
        printf("Loop\n");
        // set_motor_directions(direction, direction);
        // ESP_LOGD(TAG, "GO");
        // set_motor_speeds(DUTY_MAX, DUTY_MAX);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // ESP_LOGD(TAG, "Stop");
        // set_motor_speeds(0, 0);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // direction = !direction;
    }

    // pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    // xTaskCreate(micro_ros_task,
    //             "uros_task",
    //             CONFIG_MICRO_ROS_APP_STACK,
    //             NULL,
    //             CONFIG_MICRO_ROS_APP_TASK_PRIO,
    //             NULL);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer != NULL)
    {
        RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
        printf("Sent: %d\n", (int)send_msg.data);
        send_msg.data++;
    }
}

void subscription_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    printf("Received: %d\n", (int)msg->data);
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Create init_options.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Static Agent IP and port can be used instead of autodisvery.
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    // RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif
    // Setup support structure.
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create node.
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "int32_publisher_subscriber_rclc", "", &support));

    // Create publisher.
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "int32_publisher"));

    // Create subscriber.
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "int32_subscriber"));

    // Create timer.
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // Create executor.
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    unsigned int rcl_wait_timeout = 1000; // in ms
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // Add timer and subscriber to executor.
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

    // Spin forever.
    send_msg.data = 0;
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }

    // Free resources.
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}