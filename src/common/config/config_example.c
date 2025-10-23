/**
 * @file config_example.c
 * @brief 配置管理模块使用示例
 * @version 1.0
 * @date 2025-10-23
 * 
 * 演示如何使用config_load()和config_validate()函数
 */

#include "config_manager.h"
#include "esp_log.h"

static const char *TAG = "CONFIG_EXAMPLE";

/**
 * @brief 配置管理使用示例
 */
void config_usage_example(void) {
    esp_err_t ret;
    node_config_t config;
    
    // 步骤1: 初始化配置管理器
    ret = config_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize config manager: %s", 
                 esp_err_to_name(ret));
        return;
    }
    
    // 步骤2: 加载配置（NVS优先，否则使用sdkconfig）
    ret = config_load(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load configuration: %s", 
                 esp_err_to_name(ret));
        return;
    }
    
    // 步骤3: 验证配置有效性
    ret = config_validate(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Configuration validation failed: %s", 
                 esp_err_to_name(ret));
        return;
    }
    
    // 步骤4: 使用配置
    ESP_LOGI(TAG, "Configuration loaded and validated successfully!");
    ESP_LOGI(TAG, "WiFi will connect to: %s", config.wifi_ssid);
    ESP_LOGI(TAG, "ROS Agent at: %s:%d", 
             config.ros_agent_ip, config.ros_agent_port);
    ESP_LOGI(TAG, "ROS Node: %s (namespace: %s)", 
             config.ros_node_name, config.ros_node_namespace);
    
    // 步骤5: 运行时修改配置（可选）
    // 例如：更新WiFi SSID
    ret = config_set_str(CFG_KEY_WIFI_SSID, "NewSSID");
    if (ret == ESP_OK) {
        ret = config_save();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "WiFi SSID updated and saved to NVS");
        }
    }
}