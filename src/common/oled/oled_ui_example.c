/**
 * @file oled_ui_example.c
 * @brief OLED状态UI使用示例
 * @details 演示如何使用oled_ui模块显示系统状态信息
 * 
 * 使用方法：
 * 1. 在app_main()中调用oled_ui_example()
 * 2. 确保WiFi和ROS模块已初始化
 * 3. 示例会自动更新状态信息
 * 
 * @version 1.0
 * @date 2025-10-23
 */

#include "oled_ui.h"
#include "oled_display.h"
#include "wifi_manager.h"
#include "ros_comm.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "OLED_UI_EXAMPLE";

/**
 * @brief OLED UI示例任务
 * @details 定期更新显示内容，展示WiFi、ROS、IP和运行时间
 */
static void oled_ui_task(void *arg)
{
    ESP_LOGI(TAG, "OLED UI任务启动");

    // 等待其他模块初始化
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 创建状态显示UI
    esp_err_t ret = oled_ui_create_status_screen();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "创建UI失败: %d", ret);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UI创建成功，开始更新状态");

    while (1) {
        // 1. 更新WiFi状态
        wifi_status_t wifi_status;
        if (wifi_manager_get_status(&wifi_status) == ESP_OK) {
            bool wifi_connected = (wifi_status.state == WIFI_STATE_CONNECTED);
            oled_ui_update_wifi_status(wifi_connected, wifi_status.rssi);

            // 2. 更新IP地址
            if (wifi_connected) {
                char ip_str[16];
                snprintf(ip_str, sizeof(ip_str), IPSTR, 
                         IP2STR(&wifi_status.ip));
                oled_ui_update_ip_address(ip_str);
            } else {
                oled_ui_update_ip_address(NULL);
            }
        } else {
            // WiFi模块未初始化或获取状态失败
            oled_ui_update_wifi_status(false, 0);
            oled_ui_update_ip_address(NULL);
        }

        // 3. 更新ROS连接状态
        bool ros_connected = ros_comm_is_connected();
        oled_ui_update_ros_status(ros_connected);

        // 4. 更新系统运行时间
        uint32_t uptime_sec = esp_timer_get_time() / 1000000;
        oled_ui_update_uptime(uptime_sec);

        // 每秒更新一次（降低CPU占用）
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief OLED UI示例主函数
 * @details 初始化OLED显示并启动UI更新任务
 */
void oled_ui_example(void)
{
    ESP_LOGI(TAG, "初始化OLED UI示例");

    // 初始化OLED显示
    esp_err_t ret = oled_display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OLED初始化失败: %d", ret);
        return;
    }

    ESP_LOGI(TAG, "OLED初始化成功");

    // 创建UI更新任务
    BaseType_t task_ret = xTaskCreate(
        oled_ui_task,           // 任务函数
        "oled_ui",              // 任务名称
        4096,                   // 堆栈大小（字节）
        NULL,                   // 任务参数
        5,                      // 优先级
        NULL                    // 任务句柄
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "创建UI任务失败");
        return;
    }

    ESP_LOGI(TAG, "UI任务创建成功");
}

/**
 * @brief 基础OLED UI示例（无WiFi/ROS依赖）
 * @details 仅演示UI创建和基本更新，不依赖WiFi和ROS模块
 */
void oled_ui_basic_example(void)
{
    ESP_LOGI(TAG, "运行基础OLED UI示例");

    // 初始化OLED显示
    esp_err_t ret = oled_display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OLED初始化失败: %d", ret);
        return;
    }

    // 创建UI
    ret = oled_ui_create_status_screen();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "创建UI失败: %d", ret);
        return;
    }

    ESP_LOGI(TAG, "UI创建成功，开始演示状态更新");

    // 演示各种状态更新
    for (int i = 0; i < 60; i++) {
        // 模拟WiFi连接状态变化
        bool wifi_connected = (i % 10 < 7);  // 70%时间显示已连接
        int8_t rssi = wifi_connected ? (-50 - (i % 30)) : 0;
        oled_ui_update_wifi_status(wifi_connected, rssi);

        // 模拟ROS连接状态变化
        bool ros_connected = (i % 15 < 10);  // 66%时间显示已连接
        oled_ui_update_ros_status(ros_connected);

        // 模拟IP地址
        if (wifi_connected) {
            char ip_str[16];
            snprintf(ip_str, sizeof(ip_str), "192.168.1.%d", 100 + (i % 10));
            oled_ui_update_ip_address(ip_str);
        } else {
            oled_ui_update_ip_address(NULL);
        }

        // 更新运行时间
        oled_ui_update_uptime(i);

        // 每秒更新
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "基础示例完成");
}