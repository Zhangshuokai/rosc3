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
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include <string.h>
#include <stdio.h>

/*====================
   日志标签
 *====================*/
static const char *TAG = "OLED_UI";

/*====================
   UI元素对象 - 状态屏
 *====================*/
static lv_obj_t *label_title = NULL;       ///< 节点标题标签
static lv_obj_t *label_wifi = NULL;        ///< WiFi状态标签
static lv_obj_t *label_ros = NULL;         ///< ROS状态标签
static lv_obj_t *label_ip = NULL;          ///< IP地址标签
static lv_obj_t *label_uptime = NULL;      ///< 运行时间标签

/*====================
   UI元素对象 - 自定义屏 (TASK-COMMON-013)
 *====================*/
static lv_obj_t *custom_labels[OLED_UI_CUSTOM_MAX_LINES] = {NULL}; ///< 自定义文本标签数组

/*====================
   UI元素对象 - 进度条/告警 (TASK-COMMON-013)
 *====================*/
static lv_obj_t *progress_bar = NULL;      ///< 进度条对象
static lv_obj_t *progress_label = NULL;    ///< 进度条标签
static lv_obj_t *progress_percent = NULL;  ///< 进度百分比标签
static lv_obj_t *alert_title = NULL;       ///< 告警标题
static lv_obj_t *alert_message = NULL;     ///< 告警消息
static lv_anim_t alert_anim;               ///< 告警闪烁动画

/*====================
   屏幕对象 (TASK-COMMON-013)
 *====================*/
static lv_obj_t *screen_status = NULL;     ///< 状态屏幕对象
static lv_obj_t *screen_custom = NULL;     ///< 自定义屏幕对象
static lv_obj_t *screen_diag = NULL;       ///< 诊断屏幕对象

/*====================
   静态标志
 *====================*/
static bool ui_created = false;            ///< UI创建标志
static uint8_t current_screen_id = OLED_UI_SCREEN_STATUS; ///< 当前屏幕ID
static bool custom_screen_created = false; ///< 自定义屏幕创建标志
static bool diag_screen_created = false;   ///< 诊断屏幕创建标志

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

/**
 * @brief 创建自定义屏幕
 * @return ESP_OK=成功, 其他=失败
 */
static esp_err_t create_custom_screen(void);

/**
 * @brief 创建诊断屏幕
 * @return ESP_OK=成功, 其他=失败
 */
static esp_err_t create_diag_screen(void);

/**
 * @brief 清除临时显示对象（进度条、告警）
 */
static void clear_temp_objects(void);

/**
 * @brief 告警闪烁动画回调
 * @param[in] a 动画对象
 * @param[in] v 当前值
 */
static void alert_anim_callback(void *a, int32_t v);

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

    // 创建状态屏幕对象
    screen_status = lv_obj_create(NULL);
    if (!screen_status) {
        ESP_LOGE(TAG, "创建状态屏幕对象失败");
        return ESP_ERR_NO_MEM;
    }
    lv_obj_clear_flag(screen_status, LV_OBJ_FLAG_SCROLLABLE);
    
    // 使用状态屏幕作为父对象
    lv_obj_t *scr = screen_status;

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
    current_screen_id = OLED_UI_SCREEN_STATUS;
    
    // 加载状态屏幕
    lv_disp_load_scr(screen_status);

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

/*====================
   内部函数实现 (TASK-COMMON-013)
 *====================*/

/**
 * @brief 创建自定义屏幕
 */
static esp_err_t create_custom_screen(void)
{
    if (custom_screen_created && screen_custom) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "创建自定义屏幕");

    // 创建自定义屏幕对象
    screen_custom = lv_obj_create(NULL);
    if (!screen_custom) {
        ESP_LOGE(TAG, "创建自定义屏幕对象失败");
        return ESP_ERR_NO_MEM;
    }
    lv_obj_clear_flag(screen_custom, LV_OBJ_FLAG_SCROLLABLE);

    // 创建标题
    lv_obj_t *title = lv_label_create(screen_custom);
    if (!title) {
        lv_obj_del(screen_custom);
        screen_custom = NULL;
        return ESP_ERR_NO_MEM;
    }
    lv_label_set_text(title, "Custom Display");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

    // 创建4行自定义文本标签
    for (int i = 0; i < OLED_UI_CUSTOM_MAX_LINES; i++) {
        custom_labels[i] = lv_label_create(screen_custom);
        if (!custom_labels[i]) {
            ESP_LOGE(TAG, "创建自定义标签%d失败", i);
            // 清理已创建的标签
            for (int j = 0; j < i; j++) {
                if (custom_labels[j]) {
                    lv_obj_del(custom_labels[j]);
                    custom_labels[j] = NULL;
                }
            }
            lv_obj_del(screen_custom);
            screen_custom = NULL;
            return ESP_ERR_NO_MEM;
        }
        lv_label_set_text(custom_labels[i], "");
        lv_obj_set_style_text_font(custom_labels[i], &lv_font_montserrat_14, 0);
        lv_obj_align(custom_labels[i], LV_ALIGN_TOP_LEFT, 0, 14 + i * 14);
    }

    custom_screen_created = true;
    ESP_LOGI(TAG, "自定义屏幕创建成功");
    return ESP_OK;
}

/**
 * @brief 创建诊断屏幕
 */
static esp_err_t create_diag_screen(void)
{
    if (diag_screen_created && screen_diag) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "创建诊断屏幕");

    // 创建诊断屏幕对象
    screen_diag = lv_obj_create(NULL);
    if (!screen_diag) {
        ESP_LOGE(TAG, "创建诊断屏幕对象失败");
        return ESP_ERR_NO_MEM;
    }
    lv_obj_clear_flag(screen_diag, LV_OBJ_FLAG_SCROLLABLE);

    // 创建标题
    lv_obj_t *title = lv_label_create(screen_diag);
    if (!title) {
        lv_obj_del(screen_diag);
        screen_diag = NULL;
        return ESP_ERR_NO_MEM;
    }
    lv_label_set_text(title, "Diagnostic");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

    diag_screen_created = true;
    ESP_LOGI(TAG, "诊断屏幕创建成功");
    return ESP_OK;
}

/**
 * @brief 清除临时显示对象（进度条、告警）
 */
static void clear_temp_objects(void)
{
    // 停止告警动画
    if (alert_title) {
        lv_anim_del(alert_title, NULL);
    }

    // 删除进度条相关对象
    if (progress_percent) {
        lv_obj_del(progress_percent);
        progress_percent = NULL;
    }
    if (progress_label) {
        lv_obj_del(progress_label);
        progress_label = NULL;
    }
    if (progress_bar) {
        lv_obj_del(progress_bar);
        progress_bar = NULL;
    }

    // 删除告警相关对象
    if (alert_message) {
        lv_obj_del(alert_message);
        alert_message = NULL;
    }
    if (alert_title) {
        lv_obj_del(alert_title);
        alert_title = NULL;
    }
}

/**
 * @brief 告警闪烁动画回调
 */
static void alert_anim_callback(void *a, int32_t v)
{
    lv_obj_t *obj = (lv_obj_t *)a;
    lv_obj_set_style_opa(obj, (lv_opa_t)v, 0);
}

/*====================
   新增公共函数实现 (TASK-COMMON-013)
 *====================*/

esp_err_t oled_ui_show_custom_text(uint8_t line, const char *text)
{
    // 参数检查
    if (line >= OLED_UI_CUSTOM_MAX_LINES) {
        ESP_LOGE(TAG, "行号无效: %d (最大: %d)", line, OLED_UI_CUSTOM_MAX_LINES - 1);
        return ESP_ERR_INVALID_ARG;
    }

    // 检查OLED是否已初始化
    if (!oled_display_get_disp()) {
        ESP_LOGE(TAG, "OLED未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    // 创建自定义屏幕（如果未创建）
    esp_err_t ret = create_custom_screen();
    if (ret != ESP_OK) {
        return ret;
    }

    // 使用LVGL锁保护
    if (lvgl_port_lock(0)) {
        // 更新文本
        if (text) {
            lv_label_set_text(custom_labels[line], text);
        } else {
            lv_label_set_text(custom_labels[line], "");
        }
        
        lvgl_port_unlock();
        ESP_LOGD(TAG, "自定义文本行%d已更新: %s", line, text ? text : "(清空)");
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "无法获取LVGL锁");
    return ESP_FAIL;
}

esp_err_t oled_ui_show_progress(uint8_t percentage, const char *label)
{
    // 参数检查
    if (percentage > 100) {
        ESP_LOGE(TAG, "进度百分比无效: %d (最大: 100)", percentage);
        return ESP_ERR_INVALID_ARG;
    }

    // 检查OLED是否已初始化
    lv_disp_t *disp = oled_display_get_disp();
    if (!disp) {
        ESP_LOGE(TAG, "OLED未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    // 使用LVGL锁保护
    if (lvgl_port_lock(0)) {
        // 获取当前活动屏幕
        lv_obj_t *scr = lv_disp_get_scr_act(disp);
        if (!scr) {
            lvgl_port_unlock();
            ESP_LOGE(TAG, "无法获取当前屏幕");
            return ESP_FAIL;
        }

        // 清除旧的临时对象
        clear_temp_objects();

        // 创建进度条标签（如果有）
        if (label) {
            progress_label = lv_label_create(scr);
            if (progress_label) {
                lv_label_set_text(progress_label, label);
                lv_obj_set_style_text_font(progress_label, &lv_font_montserrat_14, 0);
                lv_obj_align(progress_label, LV_ALIGN_CENTER, 0, -20);
            }
        }

        // 创建进度条
        progress_bar = lv_bar_create(scr);
        if (!progress_bar) {
            lvgl_port_unlock();
            ESP_LOGE(TAG, "创建进度条失败");
            return ESP_ERR_NO_MEM;
        }
        lv_obj_set_size(progress_bar, 100, 10);
        lv_obj_align(progress_bar, LV_ALIGN_CENTER, 0, 0);
        lv_bar_set_value(progress_bar, percentage, LV_ANIM_OFF);

        // 创建百分比标签
        progress_percent = lv_label_create(scr);
        if (progress_percent) {
            char percent_str[8];
            snprintf(percent_str, sizeof(percent_str), "%d%%", percentage);
            lv_label_set_text(progress_percent, percent_str);
            lv_obj_set_style_text_font(progress_percent, &lv_font_montserrat_14, 0);
            lv_obj_align(progress_percent, LV_ALIGN_CENTER, 0, 15);
        }

        lvgl_port_unlock();
        ESP_LOGI(TAG, "进度条已显示: %d%%", percentage);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "无法获取LVGL锁");
    return ESP_FAIL;
}

esp_err_t oled_ui_show_alert(const char *message, bool blink)
{
    // 参数检查
    if (!message || strlen(message) == 0) {
        ESP_LOGE(TAG, "告警消息为空");
        return ESP_ERR_INVALID_ARG;
    }

    // 检查OLED是否已初始化
    lv_disp_t *disp = oled_display_get_disp();
    if (!disp) {
        ESP_LOGE(TAG, "OLED未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    // 使用LVGL锁保护
    if (lvgl_port_lock(0)) {
        // 获取当前活动屏幕
        lv_obj_t *scr = lv_disp_get_scr_act(disp);
        if (!scr) {
            lvgl_port_unlock();
            ESP_LOGE(TAG, "无法获取当前屏幕");
            return ESP_FAIL;
        }

        // 清除旧的临时对象
        clear_temp_objects();

        // 创建告警标题
        alert_title = lv_label_create(scr);
        if (!alert_title) {
            lvgl_port_unlock();
            ESP_LOGE(TAG, "创建告警标题失败");
            return ESP_ERR_NO_MEM;
        }
        lv_label_set_text(alert_title, "!!! WARNING !!!");
        lv_obj_set_style_text_font(alert_title, &lv_font_montserrat_14, 0);
        lv_obj_align(alert_title, LV_ALIGN_TOP_MID, 0, 5);

        // 创建告警消息
        alert_message = lv_label_create(scr);
        if (!alert_message) {
            lv_obj_del(alert_title);
            alert_title = NULL;
            lvgl_port_unlock();
            ESP_LOGE(TAG, "创建告警消息失败");
            return ESP_ERR_NO_MEM;
        }
        lv_label_set_text(alert_message, message);
        lv_obj_set_style_text_font(alert_message, &lv_font_montserrat_14, 0);
        lv_obj_set_width(alert_message, 120);
        lv_label_set_long_mode(alert_message, LV_LABEL_LONG_WRAP);
        lv_obj_align(alert_message, LV_ALIGN_CENTER, 0, 0);

        // 如果需要闪烁，创建动画
        if (blink) {
            lv_anim_init(&alert_anim);
            lv_anim_set_var(&alert_anim, alert_title);
            lv_anim_set_exec_cb(&alert_anim, alert_anim_callback);
            lv_anim_set_values(&alert_anim, LV_OPA_COVER, LV_OPA_TRANSP);
            lv_anim_set_time(&alert_anim, 500);  // 500ms
            lv_anim_set_repeat_count(&alert_anim, LV_ANIM_REPEAT_INFINITE);
            lv_anim_set_playback_time(&alert_anim, 500);  // 500ms
            lv_anim_start(&alert_anim);
        }

        lvgl_port_unlock();
        ESP_LOGI(TAG, "告警已显示: %s (闪烁: %s)", message, blink ? "是" : "否");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "无法获取LVGL锁");
    return ESP_FAIL;
}

esp_err_t oled_ui_switch_screen(uint8_t screen_id)
{
    // 参数检查
    if (screen_id > OLED_UI_SCREEN_DIAG) {
        ESP_LOGE(TAG, "屏幕ID无效: %d", screen_id);
        return ESP_ERR_INVALID_ARG;
    }

    // 检查OLED是否已初始化
    if (!oled_display_get_disp()) {
        ESP_LOGE(TAG, "OLED未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    // 使用LVGL锁保护
    if (lvgl_port_lock(0)) {
        lv_obj_t *target_screen = NULL;
        esp_err_t ret = ESP_OK;

        // 清除临时对象
        clear_temp_objects();

        // 根据screen_id选择或创建目标屏幕
        switch (screen_id) {
            case OLED_UI_SCREEN_STATUS:
                if (!ui_created || !screen_status) {
                    lvgl_port_unlock();
                    ESP_LOGE(TAG, "状态屏幕未创建");
                    return ESP_ERR_INVALID_STATE;
                }
                target_screen = screen_status;
                break;

            case OLED_UI_SCREEN_CUSTOM:
                ret = create_custom_screen();
                if (ret != ESP_OK) {
                    lvgl_port_unlock();
                    return ret;
                }
                target_screen = screen_custom;
                break;

            case OLED_UI_SCREEN_DIAG:
                ret = create_diag_screen();
                if (ret != ESP_OK) {
                    lvgl_port_unlock();
                    return ret;
                }
                target_screen = screen_diag;
                break;

            default:
                lvgl_port_unlock();
                ESP_LOGE(TAG, "未知屏幕ID: %d", screen_id);
                return ESP_ERR_INVALID_ARG;
        }

        // 切换屏幕
        if (target_screen) {
            lv_disp_load_scr(target_screen);
            current_screen_id = screen_id;
            lvgl_port_unlock();
            ESP_LOGI(TAG, "已切换到屏幕: %d", screen_id);
            return ESP_OK;
        }

        lvgl_port_unlock();
        ESP_LOGE(TAG, "目标屏幕为空");
        return ESP_FAIL;
    }

    ESP_LOGE(TAG, "无法获取LVGL锁");
    return ESP_FAIL;
}