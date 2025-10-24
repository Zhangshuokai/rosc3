/**
 * @file main.c
 * @brief ESP32-C3 SSD1306 OLED主程序
 *
 * 功能：
 * - 初始化I2C管理器
 * - 初始化OLED显示模块
 * - 显示UI内容
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "common/i2c_manager/i2c_manager.h"
#include "common/oled/oled_display.h"
#include "common/oled/oled_ui.h"

static const char *TAG = "main";

/*******************************************************************************
 * 主程序入口
 ******************************************************************************/

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-C3 系统初始化开始 ===");
    
    /* -------------------------------------------------------------------------
     * 步骤1: 初始化I2C管理器
     * ------------------------------------------------------------------------- */
    ESP_LOGI(TAG, "[1/3] 初始化I2C总线管理器...");
    ESP_ERROR_CHECK(i2c_manager_init_default());
    ESP_LOGI(TAG, "  ✓ I2C管理器初始化成功");
    
    /* -------------------------------------------------------------------------
     * 步骤2: 初始化OLED显示模块
     * ------------------------------------------------------------------------- */
    ESP_LOGI(TAG, "[2/3] 初始化OLED显示模块...");
    ESP_ERROR_CHECK(oled_display_init());
    ESP_LOGI(TAG, "  ✓ OLED显示模块初始化成功");
    
    /* -------------------------------------------------------------------------
     * 步骤3: 创建并显示UI
     * ------------------------------------------------------------------------- */
    ESP_LOGI(TAG, "[3/3] 创建UI界面...");
    
    lv_disp_t *disp = oled_display_get_disp();
    if (disp == NULL) {
        ESP_LOGE(TAG, "  ✗ 无法获取显示对象");
        return;
    }
    
    // 创建UI界面
    ESP_ERROR_CHECK(oled_ui_create_status_screen());
    ESP_LOGI(TAG, "  ✓ UI界面创建成功");
    
    ESP_LOGI(TAG, "=== 系统初始化完成，正常运行中 ===");
    
    // 主循环
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}