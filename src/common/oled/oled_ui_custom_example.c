/**
 * @file oled_ui_custom_example.c
 * @brief OLED自定义信息显示功能示例 (TASK-COMMON-013)
 * @version 1.0
 * @date 2025-10-23
 */

#include "oled_ui.h"
#include "oled_display.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "OLED_UI_CUSTOM_EXAMPLE";

/**
 * @brief 示例1：自定义文本显示
 */
void example_custom_text(void)
{
    ESP_LOGI(TAG, "=== 示例1：自定义文本显示 ===");
    
    // 切换到自定义屏幕
    oled_ui_switch_screen(OLED_UI_SCREEN_CUSTOM);
    
    // 显示节点特定信息
    oled_ui_show_custom_text(0, "Motor Speed: 500rpm");
    oled_ui_show_custom_text(1, "Position: 45.2mm");
    oled_ui_show_custom_text(2, "Current: 1.2A");
    oled_ui_show_custom_text(3, "Temp: 28C");
    
    // 保持显示5秒
    vTaskDelay(pdMS_TO_TICKS(5000));
}

/**
 * @brief 示例2：进度条显示
 */
void example_progress_bar(void)
{
    ESP_LOGI(TAG, "=== 示例2：进度条显示 ===");
    
    // 切换到状态屏幕
    oled_ui_switch_screen(OLED_UI_SCREEN_STATUS);
    
    // 模拟固件升级过程
    for (int i = 0; i <= 100; i += 10) {
        oled_ui_show_progress(i, "Uploading...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // 显示完成
    oled_ui_show_progress(100, "Complete!");
    vTaskDelay(pdMS_TO_TICKS(2000));
}

/**
 * @brief 示例3：告警信息显示
 */
void example_alert_display(void)
{
    ESP_LOGI(TAG, "=== 示例3：告警信息显示 ===");
    
    // 显示告警（闪烁）
    oled_ui_show_alert("Temperature High!", true);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // 显示告警（不闪烁）
    oled_ui_show_alert("Motor Stopped", false);
    vTaskDelay(pdMS_TO_TICKS(3000));
}

/**
 * @brief 示例4：多屏切换
 */
void example_screen_switching(void)
{
    ESP_LOGI(TAG, "=== 示例4：多屏切换 ===");
    
    // 状态屏
    ESP_LOGI(TAG, "切换到状态屏");
    oled_ui_switch_screen(OLED_UI_SCREEN_STATUS);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 自定义屏
    ESP_LOGI(TAG, "切换到自定义屏");
    oled_ui_switch_screen(OLED_UI_SCREEN_CUSTOM);
    oled_ui_show_custom_text(0, "Line 0: Test");
    oled_ui_show_custom_text(1, "Line 1: Data");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 诊断屏
    ESP_LOGI(TAG, "切换到诊断屏");
    oled_ui_switch_screen(OLED_UI_SCREEN_DIAG);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 返回状态屏
    ESP_LOGI(TAG, "返回状态屏");
    oled_ui_switch_screen(OLED_UI_SCREEN_STATUS);
}

/**
 * @brief 示例5：组合使用场景
 */
void example_combined_scenario(void)
{
    ESP_LOGI(TAG, "=== 示例5：组合使用场景 ===");
    
    // 场景1：启动时显示自定义欢迎信息
    oled_ui_switch_screen(OLED_UI_SCREEN_CUSTOM);
    oled_ui_show_custom_text(0, "");
    oled_ui_show_custom_text(1, "System Starting...");
    oled_ui_show_custom_text(2, "");
    oled_ui_show_custom_text(3, "");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 场景2：显示初始化进度
    oled_ui_show_progress(30, "Init WiFi...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    oled_ui_show_progress(60, "Init ROS...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    oled_ui_show_progress(100, "Ready!");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 场景3：切换到状态屏显示正常运行
    oled_ui_switch_screen(OLED_UI_SCREEN_STATUS);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // 场景4：检测到异常，显示告警
    oled_ui_show_alert("Connection Lost!", true);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // 场景5：恢复正常，切换回状态屏
    oled_ui_switch_screen(OLED_UI_SCREEN_STATUS);
}

/**
 * @brief 主示例任务
 */
void oled_ui_custom_example_task(void *pvParameters)
{
    ESP_LOGI(TAG, "OLED自定义显示功能示例启动");
    
    // 等待OLED初始化完成
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 运行各个示例
    example_custom_text();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    example_progress_bar();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    example_alert_display();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    example_screen_switching();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    example_combined_scenario();
    
    ESP_LOGI(TAG, "所有示例运行完成");
    
    // 删除任务
    vTaskDelete(NULL);
}

/**
 * @brief 启动OLED自定义显示示例
 */
void oled_ui_custom_example_start(void)
{
    xTaskCreate(oled_ui_custom_example_task, 
                "oled_ui_custom_example", 
                4096, 
                NULL, 
                5, 
                NULL);
}