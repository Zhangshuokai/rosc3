/**
 * @file diagnostic_example.c
 * @brief 诊断服务模块使用示例
 * @version 1.0
 * @date 2025-10-23
 * @author 嵌入式开发组
 * 
 * 本文件演示如何使用诊断服务模块发布系统状态信息
 * 
 * @copyright Copyright (c) 2025 ROSC3 Project
 */

#include "diagnostic.h"
#include "common/ros_comm/ros_comm.h"
#include "common/wifi_manager/wifi_manager.h"

#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DIAG_EXAMPLE";

/**
 * @brief 诊断发布任务
 * 
 * 每秒发布一次系统状态诊断信息
 */
static void diagnostic_publish_task(void *arg) {
    ESP_LOGI(TAG, "Diagnostic publish task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(1000);  // 1Hz
    
    while (1) {
        // 等待下一个周期
        vTaskDelayUntil(&last_wake_time, frequency);
        
        // 清空之前的键值对
        diagnostic_clear_kv();
        
        // 添加系统状态键值对
        
        // 1. WiFi信号强度
        wifi_status_t wifi_status = {0};
        if (wifi_manager_get_status(&wifi_status) == ESP_OK) {
            char rssi_str[16];
            snprintf(rssi_str, sizeof(rssi_str), "%d dBm", wifi_status.rssi);
            diagnostic_add_kv("WiFi RSSI", rssi_str);
            
            char ip_str[16];
            snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&wifi_status.ip));
            diagnostic_add_kv("IP Address", ip_str);
        }
        
        // 2. 空闲堆内存
        uint32_t free_heap = esp_get_free_heap_size();
        char heap_str[16];
        snprintf(heap_str, sizeof(heap_str), "%lu bytes", free_heap);
        diagnostic_add_kv("Free Heap", heap_str);
        
        // 3. 最小堆内存（历史最低）
        uint32_t min_heap = esp_get_minimum_free_heap_size();
        char min_heap_str[16];
        snprintf(min_heap_str, sizeof(min_heap_str), "%lu bytes", min_heap);
        diagnostic_add_kv("Min Free Heap", min_heap_str);
        
        // 4. 运行时间
        uint32_t uptime_sec = esp_log_timestamp() / 1000;
        char uptime_str[16];
        snprintf(uptime_str, sizeof(uptime_str), "%lu sec", uptime_sec);
        diagnostic_add_kv("Uptime", uptime_str);
        
        // 5. ROS连接状态
        bool ros_connected = ros_comm_is_connected();
        diagnostic_add_kv("ROS Status", ros_connected ? "Connected" : "Disconnected");
        
        // 根据系统状态确定诊断级别
        uint8_t level = DIAGNOSTIC_OK;
        const char *message = "System running normally";
        
        // 检查告警条件
        if (!ros_connected) {
            level = DIAGNOSTIC_ERROR;
            message = "ROS disconnected";
        } else if (wifi_status.rssi < -75) {
            level = DIAGNOSTIC_WARN;
            message = "Weak WiFi signal";
        } else if (free_heap < 50000) {  // 低于50KB
            level = DIAGNOSTIC_WARN;
            message = "Low memory";
        }
        
        // 发布诊断消息
        esp_err_t ret = diagnostic_publish(level, message);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to publish diagnostic: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGD(TAG, "Diagnostic published: %s", message);
        }
    }
}

/**
 * @brief 诊断服务使用示例主函数
 */
void diagnostic_example_main(void) {
    ESP_LOGI(TAG, "Starting diagnostic service example");
    
    // 假设WiFi和ROS已经初始化和连接
    // wifi_manager_init(&wifi_config);
    // wifi_manager_connect();
    // ros_comm_init(&ros_config);
    // ros_comm_connect(5000);
    
    // 初始化诊断服务
    esp_err_t ret = diagnostic_init("esp32_node");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize diagnostic service: %s", 
                 esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Diagnostic service initialized");
    
    // 创建诊断发布任务
    xTaskCreate(
        diagnostic_publish_task,
        "diag_pub",
        4096,           // 堆栈大小
        NULL,
        5,              // 优先级
        NULL
    );
    
    ESP_LOGI(TAG, "Diagnostic publish task created");
}

/**
 * @brief 简单使用示例
 */
void diagnostic_simple_example(void) {
    // 初始化（假设ROS已连接）
    diagnostic_init("test_node");
    
    // 添加键值对
    diagnostic_add_kv("Temperature", "25.5 C");
    diagnostic_add_kv("Battery", "85%");
    diagnostic_add_kv("Motor Speed", "1500 RPM");
    
    // 发布正常状态
    diagnostic_publish(DIAGNOSTIC_OK, "All systems operational");
    
    // 清空键值对
    diagnostic_clear_kv();
    
    // 添加新的键值对
    diagnostic_add_kv("Error Code", "0x1234");
    diagnostic_add_kv("Timestamp", "2025-10-23 20:00:00");
    
    // 发布错误状态
    diagnostic_publish(DIAGNOSTIC_ERROR, "Motor overheating detected");
}

/**
 * @brief 高级使用示例 - 条件诊断
 */
void diagnostic_advanced_example(void) {
    // 检查多个系统状态
    float temperature = 65.5;  // 假设温度读数
    uint32_t free_heap = esp_get_free_heap_size();
    bool motor_enabled = true;
    
    // 清空旧数据
    diagnostic_clear_kv();
    
    // 添加当前状态
    char temp_str[16];
    snprintf(temp_str, sizeof(temp_str), "%.1f C", temperature);
    diagnostic_add_kv("Temperature", temp_str);
    
    char heap_str[16];
    snprintf(heap_str, sizeof(heap_str), "%lu B", free_heap);
    diagnostic_add_kv("Free Heap", heap_str);
    
    diagnostic_add_kv("Motor", motor_enabled ? "Enabled" : "Disabled");
    
    // 根据条件确定诊断级别
    uint8_t level;
    const char *message;
    
    if (temperature > 80.0) {
        level = DIAGNOSTIC_ERROR;
        message = "Critical: Temperature exceeds limit";
    } else if (temperature > 70.0) {
        level = DIAGNOSTIC_WARN;
        message = "Warning: Temperature high";
    } else if (free_heap < 10000) {
        level = DIAGNOSTIC_ERROR;
        message = "Critical: Memory low";
    } else if (free_heap < 50000) {
        level = DIAGNOSTIC_WARN;
        message = "Warning: Memory getting low";
    } else {
        level = DIAGNOSTIC_OK;
        message = "All systems normal";
    }
    
    // 发布诊断
    diagnostic_publish(level, message);
}