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
#include "camera.hpp"
#include "drivetrain.hpp"
#include "driver/gpio.h"

/* Constants */

/* Global Variables */

rcl_subscription_t subscriber;
std_msgs__msg__Int32 send_msg;
std_msgs__msg__Int32 recv_msg;

/* Function Headers*/
void micro_ros_task(void *arg);

extern "C" void app_main(void)
{

    gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_21, 1);

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
    ESP_ERROR_CHECK(camera_init());

    // LED on indicator

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    initialise_motor(left_motor);
    initialise_motor(right_motor);
    printf("Motors initialized\n");
    printf("Forward\n");
    set_motor_speed(left_motor, 1024, true);
    set_motor_speed(right_motor, 1024, true);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    set_motor_speed(left_motor, 0, false);
    set_motor_speed(right_motor, 0, false);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    printf("Backward\n");
    set_motor_speed(left_motor, 1024, false);
    set_motor_speed(right_motor, 1024, false);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    set_motor_speed(left_motor, 0, false);
    set_motor_speed(right_motor, 0, false);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    printf("Motor test complete\n");
    // pin micro - ros task in APP_CPU to make PRO_CPU to deal with wifi :
    xTaskCreate(micro_ros_task,
                "uros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);

    while (true)
    {
        usleep(100000);
    }
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
    ESP_LOGD("uros_task", "Support initialised");

    // Create node.
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "RR_Node", "", &support));
    ESP_LOGD("uros_task", "Node initialised");

    // Create executor.
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    ESP_LOGD("uros_task", "Executor initialised");
    unsigned int rcl_wait_timeout = 1000; // in ms
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // Create publisher.
    RCCHECK(rclc_publisher_init_default(
        &cam_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
        "image_publisher"));
    ESP_LOGD("uros_task", "Publisher initialised");

    // Create timer.
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(333),
        publish_image_callback));
    ESP_LOGD("uros_task", "Timer initialised");

    initialise_image_msg(&image_msg);
    ESP_LOGD("uros_task", "Image memory initialised");

    RCCHECK(rclc_subscription_init_default(
        &speed_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "wheel_speed"));
    ESP_LOGD("uros_task", "Sub initialised");
    RCCHECK(rclc_subscription_init_default(
        &r_speed_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "r_wheel_speed"));

    RCCHECK(rclc_executor_add_subscription(&executor, &speed_sub, &speed_msg, &speed_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &r_speed_sub, &r_speed_msg, &r_speed_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // Spin forever.
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }

    // Free resources.
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&cam_publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}