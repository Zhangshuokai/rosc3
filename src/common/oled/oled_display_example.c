/**
 * @file oled_display_example.c
 * @brief OLED显示模块使用示例
 * @details 展示如何初始化OLED显示并创建简单UI
 * 
 * @version 1.0
 * @date 2025-10-23
 */

#include "oled_display.h"
#include "lvgl.h"
#include "esp_log.h"

static const char *TAG = "OLED_EXAMPLE";

/**
 * @brief OLED显示模块使用示例
 * @details 展示基本的初始化和UI创建流程
 */
void oled_display_example(void)
{
    ESP_LOGI(TAG, "开始OLED显示示例");

    // 1. 初始化OLED显示
    esp_err_t ret = oled_display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OLED初始化失败: %s", esp_err_to_name(ret));
        return;
    }

    // 2. 获取显示对象
    lv_disp_t *disp = oled_display_get_disp();
    if (!disp) {
        ESP_LOGE(TAG, "无法获取显示对象");
        return;
    }

    // 3. 获取当前活动屏幕
    lv_obj_t *scr = lv_disp_get_scr_act(disp);

    // 4. 创建标题标签
    lv_obj_t *label_title = lv_label_create(scr);
    lv_label_set_text(label_title, "ESP32-C3 OLED");
    lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 5);

    // 5. 创建状态标签
    lv_obj_t *label_status = lv_label_create(scr);
    lv_label_set_text(label_status, "LVGL Ready!");
    lv_obj_align(label_status, LV_ALIGN_CENTER, 0, 0);

    // 6. 创建滚动文本标签
    lv_obj_t *label_scroll = lv_label_create(scr);
    lv_label_set_long_mode(label_scroll, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_label_set_text(label_scroll, "SSD1306 OLED Display - 128x64 Monochrome");
    lv_obj_set_width(label_scroll, OLED_WIDTH - 10);
    lv_obj_align(label_scroll, LV_ALIGN_BOTTOM_MID, 0, -5);

    ESP_LOGI(TAG, "UI创建完成，显示分辨率: %dx%d", OLED_WIDTH, OLED_HEIGHT);

    // 注意：刷新由LVGL定时器任务自动处理，无需手动调用
}

/**
 * @brief 演示显示控制功能
 */
void oled_display_control_example(void)
{
    ESP_LOGI(TAG, "演示显示控制功能");

    // 清空显示
    oled_display_clear();
    ESP_LOGI(TAG, "显示已清空");

    // 等待1秒
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 重新获取显示对象并创建新内容
    lv_disp_t *disp = oled_display_get_disp();
    if (disp) {
        lv_obj_t *scr = lv_disp_get_scr_act(disp);
        lv_obj_t *label = lv_label_create(scr);
        lv_label_set_text(label, "Display\nControl\nExample");
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    }

    // 等待2秒
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 关闭显示
    oled_display_set_backlight(false);
    ESP_LOGI(TAG, "显示已关闭");

    // 等待1秒
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 开启显示
    oled_display_set_backlight(true);
    ESP_LOGI(TAG, "显示已开启");
}