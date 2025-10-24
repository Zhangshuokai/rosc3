/**
 * @file diagnostic_example.c
 * @brief 诊断服务模块使用示例
 * @version 2.0
 * @date 2025-10-23
 * @author 嵌入式开发组
 *
 * 本文件演示如何使用诊断服务模块的所有功能：
 * - 发布诊断消息
 * - 记录事件日志
 * - 生成诊断报告
 * - 获取日志历史
 * - 注册异常处理器
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

/**
 * @brief 日志记录示例
 */
void diagnostic_log_example(void) {
    ESP_LOGI(TAG, "=== Diagnostic Log Example ===");
    
    // 初始化诊断服务（假设已初始化）
    // diagnostic_init("test_node");
    
    // 1. 记录不同级别的日志
    diagnostic_log(ESP_LOG_INFO, "CHASSIS", "Motor started");
    diagnostic_log(ESP_LOG_INFO, "CHASSIS", "Speed set to %d RPM", 1500);
    diagnostic_log(ESP_LOG_WARN, "CHASSIS", "Temperature high: %d°C", 75);
    diagnostic_log(ESP_LOG_ERROR, "CHASSIS", "Motor stalled!");
    
    // 2. 记录一些测试日志
    for (int i = 0; i < 5; i++) {
        diagnostic_log(ESP_LOG_INFO, "TEST", "Test log entry %d", i);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // 3. 获取日志历史
    diagnostic_log_entry_t logs[10];
    size_t count = diagnostic_get_log_history(logs, 10);
    
    ESP_LOGI(TAG, "Retrieved %zu log entries:", count);
    for (size_t i = 0; i < count; i++) {
        const char *level_str = "UNKNOWN";
        switch (logs[i].level) {
            case ESP_LOG_ERROR:   level_str = "ERROR"; break;
            case ESP_LOG_WARN:    level_str = "WARN"; break;
            case ESP_LOG_INFO:    level_str = "INFO"; break;
            case ESP_LOG_DEBUG:   level_str = "DEBUG"; break;
            case ESP_LOG_VERBOSE: level_str = "VERBOSE"; break;
        }
        
        ESP_LOGI(TAG, "[%s] %s: %s (ts: %lu)",
                 level_str,
                 logs[i].tag,
                 logs[i].message,
                 logs[i].timestamp);
    }
    
    // 4. 清空日志（可选）
    // diagnostic_clear_log_history();
}

/**
 * @brief 诊断报告生成示例
 */
void diagnostic_report_example(void) {
    ESP_LOGI(TAG, "=== Diagnostic Report Example ===");
    
    // 先记录一些测试日志
    diagnostic_log(ESP_LOG_INFO, "SYSTEM", "System initialized");
    diagnostic_log(ESP_LOG_WARN, "WIFI", "Signal weak: -78 dBm");
    diagnostic_log(ESP_LOG_ERROR, "ROS", "Connection lost");
    
    // 生成诊断报告
    char report[2048];
    size_t len = diagnostic_generate_report(report, sizeof(report));
    
    ESP_LOGI(TAG, "Generated report (%zu bytes):", len);
    printf("\n%s\n", report);
    
    // 报告可以通过多种方式使用：
    // 1. 打印到控制台（如上）
    // 2. 保存到文件
    // 3. 通过ROS发布
    // 4. 通过HTTP上传到服务器
}

/**
 * @brief 异常处理器示例
 */
void my_exception_handler(const char *exception_type,
                          const char *message,
                          void *context) {
    ESP_LOGE("EXCEPTION", "=== EXCEPTION DETECTED ===");
    ESP_LOGE("EXCEPTION", "Type: %s", exception_type);
    ESP_LOGE("EXCEPTION", "Message: %s", message);
    
    // 记录异常日志
    diagnostic_log(ESP_LOG_ERROR, "EXCEPTION",
                   "%s: %s", exception_type, message);
    
    // 生成崩溃报告
    char report[2048];
    diagnostic_generate_report(report, sizeof(report));
    
    ESP_LOGE("EXCEPTION", "Crash Report:\n%s", report);
    
    // 实际应用中可以：
    // - 保存报告到Flash
    // - 通过网络发送到服务器
    // - 触发系统重启
}

/**
 * @brief 异常处理器注册示例
 */
void diagnostic_exception_handler_example(void) {
    ESP_LOGI(TAG, "=== Exception Handler Example ===");
    
    // 注册异常处理器
    esp_err_t ret = diagnostic_register_exception_handler(my_exception_handler);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Exception handler registered successfully");
    } else {
        ESP_LOGE(TAG, "Failed to register exception handler: %s",
                 esp_err_to_name(ret));
    }
    
    // 模拟异常（实际使用中由系统自动触发）
    // my_exception_handler("TEST_EXCEPTION", "This is a test exception", NULL);
}

/**
 * @brief 综合示例 - 完整的诊断流程
 */
void diagnostic_comprehensive_example(void) {
    ESP_LOGI(TAG, "=== Comprehensive Diagnostic Example ===");
    
    // 1. 初始化（假设WiFi和ROS已连接）
    // diagnostic_init("chassis_node_01");
    
    // 2. 注册异常处理器
    diagnostic_register_exception_handler(my_exception_handler);
    
    // 3. 启动系统监控（每5秒）
    // diagnostic_start_monitor(5000);
    
    // 4. 在应用程序中使用日志记录
    diagnostic_log(ESP_LOG_INFO, "APP", "Application started");
    
    // 模拟一些操作
    for (int i = 0; i < 3; i++) {
        diagnostic_log(ESP_LOG_INFO, "APP", "Processing iteration %d", i);
        
        // 模拟警告
        if (i == 1) {
            diagnostic_log(ESP_LOG_WARN, "APP", "Resource usage high");
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // 5. 定期发布自定义诊断信息
    diagnostic_clear_kv();
    diagnostic_add_kv("Operation", "Normal");
    diagnostic_add_kv("Cycles", "1234");
    diagnostic_add_kv("Errors", "0");
    diagnostic_publish(DIAGNOSTIC_OK, "Application running normally");
    
    // 6. 需要时生成诊断报告
    char report[2048];
    diagnostic_generate_report(report, sizeof(report));
    ESP_LOGI(TAG, "Full diagnostic report:\n%s", report);
}

/**
 * @brief 所有示例的主入口
 */
void run_all_diagnostic_examples(void) {
    ESP_LOGI(TAG, "\n\n");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  DIAGNOSTIC MODULE EXAMPLES");
    ESP_LOGI(TAG, "========================================");
    
    // 等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 运行各个示例
    diagnostic_log_example();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    diagnostic_report_example();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    diagnostic_exception_handler_example();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    diagnostic_comprehensive_example();
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  ALL EXAMPLES COMPLETED");
    ESP_LOGI(TAG, "========================================");
}