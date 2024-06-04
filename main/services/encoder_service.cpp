#include "services.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "Encoder.h"

#include "uros.h"
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define ENCODER_A 6
#define ENCODER_B 7

static Encoder encoder(ENCODER_A, ENCODER_B);
static int32_t newPosition = 0;
static int32_t oldPosition = -999;
static void encoder_task(void *pvParameters);
static const char *TAG = "encoder_service";

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
// Service
BaseType_t encoder_service(void)
{
    BaseType_t status;

    size_t stack_size = sizeof(Encoder) + 2 * sizeof(long) + 2 * sizeof(int);
    ESP_LOGI(TAG, "Stack size: %d", stack_size);

    status = xTaskCreate(
        encoder_task,
        "encoder_task",
        2 * stack_size + CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        1,
        NULL);

    if (status == pdPASS)
    {
        ESP_LOGI(TAG, "Encoder service started");
    }
    else
    {
        ESP_LOGI(TAG, "Error starting the encoder service");
    }
    return status;
}

void encoder_task(void *pvParameters)
{

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
    RCCHECK(rclc_node_init_default(&node, "esp32_int32_publisher", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "freertos_int32_publisher"));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    while (1)
    {
        rclc_executor_spin(&executor);
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
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}