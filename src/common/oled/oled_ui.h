/**
 * @file oled_ui.h
 * @brief OLED状态信息显示UI接口定义
 * @details 提供系统状态信息的可视化显示，包括WiFi、ROS、IP地址和运行时间
 * 
 * UI布局（128x64 OLED）：
 * ┌────────────────────────┐
 * │ NODE-01 Chassis        │  ← 节点标题 (第1行)
 * ├────────────────────────┤
 * │ WiFi: ✓ -65dBm         │  ← WiFi状态 (第2行)
 * │ ROS:  ✓ Connected      │  ← ROS状态 (第3行)
 * │ IP: 192.168.1.101      │  ← IP地址 (第4行)
 * │ Up: 01:23:45           │  ← 运行时间 (第5行)
 * └────────────────────────┘
 * 
 * @version 1.0
 * @date 2025-10-23
 */

#ifndef OLED_UI_H
#define OLED_UI_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*====================
   配置参数
 *====================*/

/** @brief UI刷新频率（Hz），建议1Hz降低CPU占用 */
#define OLED_UI_REFRESH_RATE_HZ     1

/** @brief WiFi连接图标 */
#define OLED_UI_ICON_WIFI_OK        "+"  // 使用+代替✓，ASCII兼容

/** @brief WiFi断开图标 */
#define OLED_UI_ICON_WIFI_FAIL      "x"  // 使用x代替✗，ASCII兼容

/** @brief ROS连接图标 */
#define OLED_UI_ICON_ROS_OK         "+"  // 使用+代替✓，ASCII兼容

/** @brief ROS断开图标 */
#define OLED_UI_ICON_ROS_FAIL       "x"  // 使用x代替✗，ASCII兼容

/** @brief 默认节点名称 */
#define OLED_UI_DEFAULT_NODE_NAME   "ROSC3 Node"

/*====================
   函数声明
 *====================*/

/**
 * @brief 创建状态显示UI
 * @details 创建5个文本标签用于显示状态信息：
 *          - 节点标题（从配置读取或使用默认值）
 *          - WiFi状态（连接状态 + RSSI）
 *          - ROS状态（连接状态）
 *          - IP地址
 *          - 系统运行时间
 * 
 * @return 
 *     - ESP_OK: 创建成功
 *     - ESP_ERR_INVALID_STATE: OLED未初始化
 *     - ESP_ERR_NO_MEM: 内存不足
 *     - ESP_FAIL: 创建失败
 * 
 * @note 必须在oled_display_init()成功后调用
 * @note 该函数不是线程安全的，应在主任务中调用
 * @note 创建后UI会显示初始状态（未连接）
 */
esp_err_t oled_ui_create_status_screen(void);

/**
 * @brief 更新WiFi状态显示
 * @details 更新WiFi连接状态和信号强度（RSSI）
 *          格式："WiFi: ✓ -65dBm" 或 "WiFi: ✗ Disconnected"
 * 
 * @param[in] connected WiFi连接状态 (true=已连接, false=未连接)
 * @param[in] rssi 信号强度（dBm），仅在connected=true时有效，范围通常为-100至0
 * 
 * @return 
 *     - ESP_OK: 更新成功
 *     - ESP_ERR_INVALID_STATE: UI未创建
 *     - ESP_FAIL: 更新失败
 * 
 * @note 此函数是线程安全的（内部使用LVGL锁）
 * @note 当WiFi状态变化时应立即调用此函数
 */
esp_err_t oled_ui_update_wifi_status(bool connected, int8_t rssi);

/**
 * @brief 更新ROS连接状态显示
 * @details 更新ROS Agent连接状态
 *          格式："ROS:  ✓ Connected" 或 "ROS:  ✗ Disconnected"
 * 
 * @param[in] connected ROS连接状态 (true=已连接, false=未连接)
 * 
 * @return 
 *     - ESP_OK: 更新成功
 *     - ESP_ERR_INVALID_STATE: UI未创建
 *     - ESP_FAIL: 更新失败
 * 
 * @note 此函数是线程安全的（内部使用LVGL锁）
 * @note 当ROS状态变化时应立即调用此函数
 */
esp_err_t oled_ui_update_ros_status(bool connected);

/**
 * @brief 更新IP地址显示
 * @details 更新设备IP地址
 *          格式："IP: 192.168.1.101" 或 "IP: ---.---.---.---"（未分配时）
 * 
 * @param[in] ip_str IP地址字符串，格式为"xxx.xxx.xxx.xxx"，NULL表示清空显示
 * 
 * @return 
 *     - ESP_OK: 更新成功
 *     - ESP_ERR_INVALID_STATE: UI未创建
 *     - ESP_FAIL: 更新失败
 * 
 * @note 此函数是线程安全的（内部使用LVGL锁）
 * @note ip_str参数会被内部复制，调用者可以在函数返回后释放
 */
esp_err_t oled_ui_update_ip_address(const char *ip_str);

/**
 * @brief 更新系统运行时间显示
 * @details 更新系统运行时间，格式化为HH:MM:SS
 *          格式："Up: 01:23:45"
 * 
 * @param[in] uptime_sec 系统运行时间（秒）
 * 
 * @return 
 *     - ESP_OK: 更新成功
 *     - ESP_ERR_INVALID_STATE: UI未创建
 *     - ESP_FAIL: 更新失败
 * 
 * @note 此函数是线程安全的（内部使用LVGL锁）
 * @note 建议每秒调用一次以更新显示
 */
esp_err_t oled_ui_update_uptime(uint32_t uptime_sec);

/**
 * @brief 销毁状态显示UI
 * @details 清理所有UI元素，释放相关资源
 *
 * @return
 *     - ESP_OK: 销毁成功
 *     - ESP_ERR_INVALID_STATE: UI未创建
 *
 * @note 此函数不是线程安全的，应在主任务中调用
 */
esp_err_t oled_ui_destroy_status_screen(void);

/*====================
   自定义信息显示功能 (TASK-COMMON-013)
 *====================*/

/** @brief 屏幕ID定义 */
#define OLED_UI_SCREEN_STATUS    0  ///< 状态屏幕
#define OLED_UI_SCREEN_CUSTOM    1  ///< 自定义屏幕
#define OLED_UI_SCREEN_DIAG      2  ///< 诊断屏幕

/** @brief 自定义文本最大行数 */
#define OLED_UI_CUSTOM_MAX_LINES 4

/**
 * @brief 显示自定义文本
 * @details 在自定义屏幕上显示文本内容，支持4行独立更新
 *
 * @param[in] line 行号（0-3）
 * @param[in] text 文本内容，NULL表示清空该行
 *
 * @return
 *     - ESP_OK: 成功
 *     - ESP_ERR_INVALID_ARG: 参数无效（行号超出范围）
 *     - ESP_ERR_INVALID_STATE: 自定义屏幕未创建
 *     - ESP_FAIL: 显示失败
 *
 * @note 此函数是线程安全的（内部使用LVGL锁）
 * @note 文本内容会被内部复制，调用者可以在函数返回后释放
 * @note 如果当前不在自定义屏幕，文本会被缓存，切换到自定义屏幕后显示
 */
esp_err_t oled_ui_show_custom_text(uint8_t line, const char *text);

/**
 * @brief 显示进度条
 * @details 在当前屏幕上显示进度条和标签
 *
 * @param[in] percentage 进度百分比（0-100）
 * @param[in] label 标签文本，NULL表示不显示标签
 *
 * @return
 *     - ESP_OK: 成功
 *     - ESP_ERR_INVALID_ARG: 参数无效（百分比超出范围）
 *     - ESP_ERR_INVALID_STATE: UI未初始化
 *     - ESP_FAIL: 显示失败
 *
 * @note 此函数是线程安全的（内部使用LVGL锁）
 * @note 进度条会覆盖当前屏幕内容
 * @note 调用oled_ui_switch_screen()可清除进度条
 */
esp_err_t oled_ui_show_progress(uint8_t percentage, const char *label);

/**
 * @brief 显示告警信息
 * @details 在当前屏幕上显示告警信息，支持闪烁效果
 *
 * @param[in] message 告警内容
 * @param[in] blink true=闪烁，false=常亮
 *
 * @return
 *     - ESP_OK: 成功
 *     - ESP_ERR_INVALID_ARG: 参数无效（message为空）
 *     - ESP_ERR_INVALID_STATE: UI未初始化
 *     - ESP_FAIL: 显示失败
 *
 * @note 此函数是线程安全的（内部使用LVGL锁）
 * @note 告警会覆盖当前屏幕内容
 * @note 闪烁频率为1Hz（500ms亮/500ms暗）
 * @note 调用oled_ui_switch_screen()可清除告警
 */
esp_err_t oled_ui_show_alert(const char *message, bool blink);

/**
 * @brief 切换显示屏幕
 * @details 切换到指定的屏幕，清除临时显示内容（进度条、告警）
 *
 * @param[in] screen_id 屏幕ID（0=状态屏，1=自定义屏，2=诊断屏）
 *
 * @return
 *     - ESP_OK: 成功
 *     - ESP_ERR_INVALID_ARG: 参数无效（screen_id超出范围）
 *     - ESP_ERR_INVALID_STATE: UI未初始化
 *     - ESP_FAIL: 切换失败
 *
 * @note 此函数是线程安全的（内部使用LVGL锁）
 * @note 切换屏幕时会自动创建目标屏幕（如果未创建）
 * @note 状态屏需要先调用oled_ui_create_status_screen()创建
 */
esp_err_t oled_ui_switch_screen(uint8_t screen_id);

#ifdef __cplusplus
}
#endif

#endif /* OLED_UI_H */