/**
 * @file oled_ui.c
 * @brief OLED状态信息显示UI实现
 * @version 1.0
 * @date 2025-10-23
 */

#include "oled_ui.h"
#include "oled_display.h"
#include "config_manager.h"
#include "esp_log.h"
#include "lvgl.h"
#include <string.h>
#include <stdio.h>

/*====================
   日志标签
 *====================*/
static const char *TAG = "OLED_UI";

/*====================
   UI元素对象
 *====================*/
static lv_obj_t *label_title = NULL;       ///< 节点标题标签
static lv_obj_t *label_wifi = NULL;        ///< WiFi状态标签
static lv_obj_t *label_ros = NULL;         ///< ROS状态标签
static lv_obj_t *label_ip = NULL;          ///< IP地址标签
static lv_obj_t *label_uptime = NULL;      ///< 运行时间标签

/*====================
   静态标志
 *====================*/
static bool ui_created = false;            ///< UI创建标志

/*====================
   内部函数声明
 *====================*/

/**
 * @brief 检查UI是否已创建
 * @return true=已创建, false=未创建
 */
static inline bool is_ui_created(void)
{
    return ui_created && label_title != NULL;
}

/*====================
   公共函数实现
 *====================*/

esp_err_t oled_ui_create_status_screen(void)
{
    // 检查OLED是否已初始化
    lv_disp_t *disp = oled_display_get_disp();
    if (!disp) {
        ESP_LOGE(TAG, "OLED未初始化，无法创建UI");
        return ESP_ERR_INVALID_STATE;
    }

    // 检查是否已创建
    if (ui_created) {
        ESP_LOGW(TAG, "UI已创建，无需重复创建");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "开始创建状态显示UI");

    // 获取当前屏幕
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    if (!scr) {
        ESP_LOGE(TAG, "无法获取当前屏幕");
        return ESP_FAIL;
    }

    // 读取节点名称（从配置或使用默认值）
    char node_name[32] = OLED_UI_DEFAULT_NODE_NAME;
    node_config_t config;
    if (config_load(&config) == ESP_OK && strlen(config.ros_node_name) > 0) {
        strncpy(node_name, config.ros_node_name, sizeof(node_name) - 1);
        node_name[sizeof(node_name) - 1] = '\0';
    }

    // 创建节点标题标签（第1行，居中）
    label_title = lv_label_create(scr);
    if (!label_title) {
        ESP_LOGE(TAG, "创建标题标签失败");
        return ESP_ERR_NO_MEM;
    }
    lv_label_set_text(label_title, node_name);
    lv_obj_set_style_text_font(label_title, &lv_font_montserrat_14, 0);
    lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 0);

    // 创建WiFi状态标签（第2行，左对齐）
    label_wifi = lv_label_create(scr);
    if (!label_wifi) {
        ESP_LOGE(TAG, "创建WiFi标签失败");
        lv_obj_del(label_title);
        label_title = NULL;
        return ESP_ERR_NO_MEM;
    }
    lv_label_set_text(label_wifi, "WiFi: x Disconnected");
    lv_obj_set_style_text_font(label_wifi, &lv_font_montserrat_14, 0);
    lv_obj_align(label_wifi, LV_ALIGN_TOP_LEFT, 0, 14);

    // 创建ROS状态标签（第3行，左对齐）
    label_ros = lv_label_create(scr);
    if (!label_ros) {
        ESP_LOGE(TAG, "创建ROS标签失败");
        lv_obj_del(label_title);
        lv_obj_del(label_wifi);
        label_title = NULL;
        label_wifi = NULL;
        return ESP_ERR_NO_MEM;
    }
    lv_label_set_text(label_ros, "ROS:  x Disconnected");
    lv_obj_set_style_text_font(label_ros, &lv_font_montserrat_14, 0);
    lv_obj_align(label_ros, LV_ALIGN_TOP_LEFT, 0, 28);

    // 创建IP地址标签（第4行，左对齐）
    label_ip = lv_label_create(scr);
    if (!label_ip) {
        ESP_LOGE(TAG, "创建IP标签失败");
        lv_obj_del(label_title);
        lv_obj_del(label_wifi);
        lv_obj_del(label_ros);
        label_title = NULL;
        label_wifi = NULL;
        label_ros = NULL;
        return ESP_ERR_NO_MEM;
    }
    lv_label_set_text(label_ip, "IP: ---.---.---.---");
    lv_obj_set_style_text_font(label_ip, &lv_font_montserrat_14, 0);
    lv_obj_align(label_ip, LV_ALIGN_TOP_LEFT, 0, 42);

    // 创建运行时间标签（第5行，左对齐）
    label_uptime = lv_label_create(scr);
    if (!label_uptime) {
        ESP_LOGE(TAG, "创建运行时间标签失败");
        lv_obj_del(label_title);
        lv_obj_del(label_wifi);
        lv_obj_del(label_ros);
        lv_obj_del(label_ip);
        label_title = NULL;
        label_wifi = NULL;
        label_ros = NULL;
        label_ip = NULL;
        return ESP_ERR_NO_MEM;
    }
    lv_label_set_text(label_uptime, "Up: 00:00:00");
    lv_obj_set_style_text_font(label_uptime, &lv_font_montserrat_14, 0);
    lv_obj_align(label_uptime, LV_ALIGN_TOP_LEFT, 0, 56);

    // 标记UI已创建
    ui_created = true;

    ESP_LOGI(TAG, "状态显示UI创建成功");
    return ESP_OK;
}

esp_err_t oled_ui_update_wifi_status(bool connected, int8_t rssi)
{
    // 检查UI是否已创建
    if (!is_ui_created()) {
        ESP_LOGE(TAG, "UI未创建，无法更新WiFi状态");
        return ESP_ERR_INVALID_STATE;
    }

    // 使用LVGL锁保护
    lv_obj_t *obj = label_wifi;
    
    // 格式化状态字符串
    char status_str[32];
    if (connected) {
        snprintf(status_str, sizeof(status_str), "WiFi: %s %ddBm", 
                 OLED_UI_ICON_WIFI_OK, rssi);
    } else {
        snprintf(status_str, sizeof(status_str), "WiFi: %s Disconnected", 
                 OLED_UI_ICON_WIFI_FAIL);
    }

    // 更新标签文本
    lv_label_set_text(obj, status_str);

    ESP_LOGD(TAG, "WiFi状态已更新: %s", status_str);
    return ESP_OK;
}

esp_err_t oled_ui_update_ros_status(bool connected)
{
    // 检查UI是否已创建
    if (!is_ui_created()) {
        ESP_LOGE(TAG, "UI未创建，无法更新ROS状态");
        return ESP_ERR_INVALID_STATE;
    }

    // 使用LVGL锁保护
    lv_obj_t *obj = label_ros;
    
    // 格式化状态字符串
    char status_str[32];
    if (connected) {
        snprintf(status_str, sizeof(status_str), "ROS:  %s Connected", 
                 OLED_UI_ICON_ROS_OK);
    } else {
        snprintf(status_str, sizeof(status_str), "ROS:  %s Disconnected", 
                 OLED_UI_ICON_ROS_FAIL);
    }

    // 更新标签文本
    lv_label_set_text(obj, status_str);

    ESP_LOGD(TAG, "ROS状态已更新: %s", status_str);
    return ESP_OK;
}

esp_err_t oled_ui_update_ip_address(const char *ip_str)
{
    // 检查UI是否已创建
    if (!is_ui_created()) {
        ESP_LOGE(TAG, "UI未创建，无法更新IP地址");
        return ESP_ERR_INVALID_STATE;
    }

    // 使用LVGL锁保护
    lv_obj_t *obj = label_ip;
    
    // 格式化IP字符串
    char display_str[32];
    if (ip_str && strlen(ip_str) > 0) {
        snprintf(display_str, sizeof(display_str), "IP: %s", ip_str);
    } else {
        snprintf(display_str, sizeof(display_str), "IP: ---.---.---.---");
    }

    // 更新标签文本
    lv_label_set_text(obj, display_str);

    ESP_LOGD(TAG, "IP地址已更新: %s", display_str);
    return ESP_OK;
}

esp_err_t oled_ui_update_uptime(uint32_t uptime_sec)
{
    // 检查UI是否已创建
    if (!is_ui_created()) {
        ESP_LOGE(TAG, "UI未创建，无法更新运行时间");
        return ESP_ERR_INVALID_STATE;
    }

    // 使用LVGL锁保护
    lv_obj_t *obj = label_uptime;
    
    // 计算时、分、秒
    uint32_t hours = uptime_sec / 3600;
    uint32_t minutes = (uptime_sec % 3600) / 60;
    uint32_t seconds = uptime_sec % 60;

    // 格式化运行时间字符串
    char uptime_str[32];
    snprintf(uptime_str, sizeof(uptime_str), "Up: %02lu:%02lu:%02lu", 
             hours, minutes, seconds);

    // 更新标签文本
    lv_label_set_text(obj, uptime_str);

    ESP_LOGD(TAG, "运行时间已更新: %s", uptime_str);
    return ESP_OK;
}

esp_err_t oled_ui_destroy_status_screen(void)
{
    // 检查UI是否已创建
    if (!is_ui_created()) {
        ESP_LOGW(TAG, "UI未创建，无需销毁");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "开始销毁状态显示UI");

    // 删除所有标签对象
    if (label_uptime) {
        lv_obj_del(label_uptime);
        label_uptime = NULL;
    }
    if (label_ip) {
        lv_obj_del(label_ip);
        label_ip = NULL;
    }
    if (label_ros) {
        lv_obj_del(label_ros);
        label_ros = NULL;
    }
    if (label_wifi) {
        lv_obj_del(label_wifi);
        label_wifi = NULL;
    }
    if (label_title) {
        lv_obj_del(label_title);
        label_title = NULL;
    }

    // 清除UI创建标志
    ui_created = false;

    ESP_LOGI(TAG, "状态显示UI已销毁");
    return ESP_OK;
}