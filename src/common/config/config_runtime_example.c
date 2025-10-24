/**
 * @file config_runtime_example.c
 * @brief 运行时配置更新功能示例
 * @version 1.0
 * @date 2025-10-23
 * 
 * 本文件演示TASK-COMMON-010运行时配置更新功能的使用方法：
 * 1. 注册配置变更回调
 * 2. 运行时更新配置（带通知）
 * 3. 配置备份和恢复
 * 4. 热更新支持
 * 
 * @copyright Copyright (c) 2025 Your Company
 */

#include "config_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CONFIG_EXAMPLE";

/*******************************************************************
 * 示例1: 配置变更回调函数
 *******************************************************************/

/**
 * @brief 配置变更回调示例
 * 
 * 当配置发生变化时被调用，可用于：
 * - 记录配置变更日志
 * - 触发相关模块重新加载配置
 * - 更新运行时参数
 */
static void config_change_handler(
    const char *key,
    config_change_type_t type,
    const void *value,
    void *user_data
) {
    const char *module_name = (const char *)user_data;
    
    ESP_LOGI(TAG, "[%s] Config changed: %s", 
             module_name ? module_name : "UNKNOWN", key);
    
    if (type == CONFIG_CHANGE_TYPE_STR) {
        ESP_LOGI(TAG, "  New value (str): %s", (const char *)value);
    } else if (type == CONFIG_CHANGE_TYPE_INT) {
        ESP_LOGI(TAG, "  New value (int): %ld", *(const int32_t *)value);
    }
    
    // 根据配置键执行特定操作
    if (strcmp(key, CFG_KEY_LOG_LEVEL) == 0) {
        ESP_LOGI(TAG, "  Log level updated (hot reload)");
    } else if (strcmp(key, CFG_KEY_WIFI_SSID) == 0) {
        ESP_LOGW(TAG, "  WiFi SSID updated (requires restart)");
    } else if (strcmp(key, CFG_KEY_ROS_AGENT_IP) == 0) {
        ESP_LOGW(TAG, "  ROS Agent IP updated (requires restart)");
    }
}

/*******************************************************************
 * 示例2: 基本使用流程
 *******************************************************************/

/**
 * @brief 演示基本的运行时配置更新流程
 */
void example_basic_runtime_update(void) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "=== Example: Basic Runtime Update ===");
    
    // 1. 注册配置变更回调
    ret = config_register_callback(config_change_handler, (void *)"WiFi_Module");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register callback");
        return;
    }
    ESP_LOGI(TAG, "Callback registered");
    
    // 2. 运行时更新日志级别（支持热更新）
    ESP_LOGI(TAG, "Updating log level to DEBUG (hot update)...");
    ret = config_update_int_and_notify(CFG_KEY_LOG_LEVEL, ESP_LOG_DEBUG, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update log level");
    }
    
    // 3. 运行时更新WiFi SSID（需要重启）
    ESP_LOGI(TAG, "Updating WiFi SSID (requires restart)...");
    ret = config_update_str_and_notify(CFG_KEY_WIFI_SSID, "NewNetwork", true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update WiFi SSID");
    }
    
    // 4. 延时观察回调执行
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 5. 注销回调（可选）
    ret = config_unregister_callback(config_change_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unregister callback");
    }
    ESP_LOGI(TAG, "Callback unregistered");
}

/*******************************************************************
 * 示例3: 配置备份和恢复
 *******************************************************************/

/**
 * @brief 演示配置备份和恢复功能
 */
void example_backup_restore(void) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "=== Example: Backup and Restore ===");
    
    // 1. 备份当前配置
    ESP_LOGI(TAG, "Backing up current configuration...");
    ret = config_backup("rosc3_backup");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to backup config");
        return;
    }
    ESP_LOGI(TAG, "Configuration backed up successfully");
    
    // 2. 修改一些配置
    ESP_LOGI(TAG, "Modifying configurations...");
    config_set_str(CFG_KEY_ROS_NODE_NAME, "test_node");
    config_set_int(CFG_KEY_ROS_AGENT_PORT, 9999);
    config_save();
    
    // 3. 从备份恢复
    ESP_LOGI(TAG, "Restoring from backup...");
    ret = config_restore("rosc3_backup");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restore config");
        return;
    }
    ESP_LOGI(TAG, "Configuration restored successfully");
}

/*******************************************************************
 * 示例4: 多模块回调管理
 *******************************************************************/

/**
 * @brief WiFi模块配置变更处理
 */
static void wifi_config_handler(
    const char *key,
    config_change_type_t type,
    const void *value,
    void *user_data
) {
    // 只处理WiFi相关配置
    if (strncmp(key, "wifi_", 5) != 0) {
        return;
    }
    
    ESP_LOGI(TAG, "[WiFi] Config changed: %s", key);
    // WiFi模块特定处理逻辑...
}

/**
 * @brief ROS模块配置变更处理
 */
static void ros_config_handler(
    const char *key,
    config_change_type_t type,
    const void *value,
    void *user_data
) {
    // 只处理ROS相关配置
    if (strncmp(key, "ros_", 4) != 0) {
        return;
    }
    
    ESP_LOGI(TAG, "[ROS] Config changed: %s", key);
    // ROS模块特定处理逻辑...
}

/**
 * @brief 演示多模块回调管理
 */
void example_multi_module_callbacks(void) {
    ESP_LOGI(TAG, "=== Example: Multi-Module Callbacks ===");
    
    // 注册多个模块的回调
    config_register_callback(wifi_config_handler, NULL);
    config_register_callback(ros_config_handler, NULL);
    config_register_callback(config_change_handler, (void *)"System");
    
    ESP_LOGI(TAG, "All callbacks registered");
    
    // 更新不同模块的配置
    config_update_str_and_notify(CFG_KEY_WIFI_SSID, "TestSSID", false);
    config_update_str_and_notify(CFG_KEY_ROS_AGENT_IP, "192.168.1.100", false);
    config_update_int_and_notify(CFG_KEY_LOG_LEVEL, ESP_LOG_INFO, false);
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 注销所有回调
    config_unregister_callback(wifi_config_handler);
    config_unregister_callback(ros_config_handler);
    config_unregister_callback(config_change_handler);
    
    ESP_LOGI(TAG, "All callbacks unregistered");
}

/*******************************************************************
 * 示例5: 热更新检测
 *******************************************************************/

/**
 * @brief 演示热更新检测功能
 */
void example_hot_update_detection(void) {
    ESP_LOGI(TAG, "=== Example: Hot Update Detection ===");
    
    // 检测各种配置是否支持热更新
    const char *test_keys[] = {
        CFG_KEY_LOG_LEVEL,
        CFG_KEY_WIFI_SSID,
        CFG_KEY_ROS_AGENT_IP,
        CFG_KEY_ROS_AGENT_PORT
    };
    
    for (int i = 0; i < sizeof(test_keys) / sizeof(test_keys[0]); i++) {
        bool hot_updatable = config_is_hot_updatable(test_keys[i]);
        ESP_LOGI(TAG, "%s: %s", 
                 test_keys[i],
                 hot_updatable ? "HOT UPDATE" : "REQUIRES RESTART");
    }
}

/*******************************************************************
 * 主示例任务
 *******************************************************************/

/**
 * @brief 运行时配置更新示例任务
 */
void config_runtime_example_task(void *pvParameters) {
    ESP_LOGI(TAG, "Starting config runtime update examples...");
    
    // 确保配置管理器已初始化
    esp_err_t ret = config_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize config manager");
        vTaskDelete(NULL);
        return;
    }
    
    // 运行各个示例
    vTaskDelay(pdMS_TO_TICKS(1000));
    example_basic_runtime_update();
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    example_backup_restore();
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    example_multi_module_callbacks();
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    example_hot_update_detection();
    
    ESP_LOGI(TAG, "All examples completed");
    vTaskDelete(NULL);
}

/*******************************************************************
 * 示例启动函数
 *******************************************************************/

/**
 * @brief 启动运行时配置更新示例
 * 
 * 在main.c中调用此函数以运行示例
 */
void start_config_runtime_example(void) {
    xTaskCreate(
        config_runtime_example_task,
        "config_runtime_example",
        4096,
        NULL,
        5,
        NULL
    );
}