/**
 * @file ros_comm_pub_sub_example.c
 * @brief ROS发布者和订阅者使用示例
 * @version 1.0
 * @date 2025-10-23
 * @author 嵌入式开发组
 * 
 * 本文件演示如何使用ROS通信模块的发布者和订阅者功能
 * 
 * @copyright Copyright (c) 2025 ROSC3 Project
 */

#include "ros_comm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 假设使用std_msgs/String消息类型
#include <std_msgs/msg/string.h>

static const char *TAG = "ROS_PUB_SUB_EXAMPLE";

// 发布者和订阅者句柄
static rcl_publisher_t g_publisher;
static rcl_subscription_t g_subscription;

// 订阅者消息缓冲区
static std_msgs__msg__String g_sub_msg;

/**
 * @brief 订阅者回调函数示例
 */
static void string_callback(const void *msg, void *user_data) {
    const std_msgs__msg__String *string_msg = (const std_msgs__msg__String *)msg;
    ESP_LOGI(TAG, "Received message: %s", string_msg->data.data);
    
    // user_data可以用来传递额外信息
    if (user_data != NULL) {
        int *counter = (int *)user_data;
        (*counter)++;
        ESP_LOGI(TAG, "Message count: %d", *counter);
    }
}

/**
 * @brief 发布者示例任务
 */
static void publisher_task(void *pvParameters) {
    std_msgs__msg__String pub_msg;
    std_msgs__msg__String__init(&pub_msg);
    
    // 分配字符串缓冲区
    char buffer[100];
    pub_msg.data.data = buffer;
    pub_msg.data.capacity = sizeof(buffer);
    
    int count = 0;
    
    while (1) {
        // 准备消息
        snprintf(buffer, sizeof(buffer), "Hello from ESP32-C3, count: %d", count++);
        pub_msg.data.size = strlen(buffer);
        
        // 发布消息
        esp_err_t ret = ros_comm_publish(&g_publisher, &pub_msg);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Published: %s", buffer);
        } else {
            ESP_LOGE(TAG, "Failed to publish message: %s", esp_err_to_name(ret));
        }
        
        // 每秒发布一次
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief 订阅者示例任务
 */
static void subscriber_task(void *pvParameters) {
    while (1) {
        // 处理订阅回调
        esp_err_t ret = ros_comm_spin_once(100);  // 100ms超时
        if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Failed to spin: %s", esp_err_to_name(ret));
        }
        
        // 10ms周期
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief 初始化发布者和订阅者示例
 */
esp_err_t ros_comm_pub_sub_example_init(void) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Initializing publisher and subscriber example...");
    
    // 创建发布者（使用控制指令QoS）
    ret = ros_comm_create_publisher(
        &g_publisher,
        "/example/chatter_pub",
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        &QOS_CONTROL_CMD
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create publisher: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Publisher created successfully");
    
    // 初始化订阅消息缓冲区
    std_msgs__msg__String__init(&g_sub_msg);
    
    // 创建订阅者（使用传感器数据QoS）
    static int msg_counter = 0;  // 用户数据示例
    ret = ros_comm_create_subscription(
        &g_subscription,
        "/example/chatter_sub",
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        &g_sub_msg,
        sizeof(g_sub_msg),
        &QOS_SENSOR_DATA,
        string_callback,
        &msg_counter  // 传递计数器作为用户数据
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create subscription: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Subscription created successfully");
    
    // 创建发布者任务
    xTaskCreate(
        publisher_task,
        "publisher_task",
        4096,
        NULL,
        5,
        NULL
    );
    
    // 创建订阅者任务
    xTaskCreate(
        subscriber_task,
        "subscriber_task",
        4096,
        NULL,
        5,
        NULL
    );
    
    ESP_LOGI(TAG, "Publisher and subscriber example initialized");
    
    return ESP_OK;
}