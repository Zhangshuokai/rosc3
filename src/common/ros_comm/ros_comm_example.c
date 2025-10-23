/**
 * @file ros_comm_example.c
 * @brief ROS通信模块使用示例
 * @version 1.0
 * @date 2025-10-23
 * 
 * 本文件展示如何使用ROS通信模块API
 * 
 * @note 此文件仅供参考，不会被编译到最终固件中
 */

#include "ros_comm.h"
#include "esp_log.h"

static const char *TAG = "ROS_EXAMPLE";

/**
 * @brief ROS通信模块使用示例
 * 
 * 演示如何初始化和连接ROS Agent
 */
void ros_comm_example(void) {
    esp_err_t ret;
    
    // 1. 配置ROS参数
    ros_config_t ros_config = {
        .agent_ip = "192.168.1.10",
        .agent_port = 8888,
        .node_name = "esp32_node",
        .node_namespace = "",
        .domain_id = 0,
        .ping_timeout_ms = 5000
    };
    
    // 2. 初始化ROS通信模块
    ESP_LOGI(TAG, "Initializing ROS communication...");
    ret = ros_comm_init(&ros_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ROS: %s", esp_err_to_name(ret));
        return;
    }
    
    // 3. 连接到ROS Agent
    ESP_LOGI(TAG, "Connecting to ROS Agent...");
    ret = ros_comm_connect(5000);  // 5秒超时
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to Agent: %s", esp_err_to_name(ret));
        return;
    }
    
    // 4. 检查连接状态
    if (ros_comm_is_connected()) {
        ESP_LOGI(TAG, "Successfully connected to ROS Agent!");
        ESP_LOGI(TAG, "Node is now visible in 'ros2 node list'");
    }
    
    // 5. 后续可以创建发布者、订阅者等（TASK-COMMON-006）
    // ...
    
    // 6. 断开连接（可选）
    // ros_comm_disconnect();
}