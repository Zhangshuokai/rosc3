/**
 * @file wifi_manager.h
 * @brief WiFi管理模块接口定义
 * @version 1.0
 * @date 2025-10-23
 * @author 嵌入式开发组
 * 
 * 本文件定义WiFi连接管理的接口，包括初始化、连接、状态查询等功能
 * 支持WPA2/WPA3加密方式，提供事件驱动的连接管理
 * 
 * @copyright Copyright (c) 2025 ROSC3 Project
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_wifi_types.h"
#include "esp_netif_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WiFi状态枚举
 */
typedef enum {
    WIFI_STATE_DISCONNECTED,    ///< 未连接
    WIFI_STATE_CONNECTING,      ///< 连接中
    WIFI_STATE_CONNECTED,       ///< 已连接
    WIFI_STATE_ERROR            ///< 错误状态
} wifi_state_t;

/**
 * @brief WiFi配置结构体
 */
typedef struct {
    char ssid[32];              ///< WiFi SSID
    char password[64];          ///< WiFi密码
    wifi_auth_mode_t auth_mode; ///< 加密方式（WPA2/WPA3）
    uint8_t max_retry;          ///< 最大重试次数
    uint32_t timeout_ms;        ///< 连接超时（毫秒）
} wifi_config_t;

/**
 * @brief WiFi状态信息
 */
typedef struct {
    wifi_state_t state;         ///< 连接状态
    int8_t rssi;                ///< 信号强度（dBm）
    uint8_t channel;            ///< 信道
    esp_ip4_addr_t ip;          ///< IP地址
    esp_ip4_addr_t gateway;     ///< 网关
    esp_ip4_addr_t netmask;     ///< 子网掩码
    uint8_t mac[6];             ///< MAC地址
} wifi_status_t;

/**
 * @brief 初始化WiFi管理器
 * 
 * 初始化WiFi驱动、事件循环和网络接口。必须在其他WiFi操作之前调用。
 * 
 * @param[in] config WiFi配置参数
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效
 *   - ESP_ERR_NO_MEM: 内存不足
 *   - ESP_FAIL: 初始化失败
 * 
 * @note 此函数不是线程安全的，应在主任务中调用
 * @warning 重复调用会返回错误
 */
esp_err_t wifi_manager_init(const wifi_config_t *config);

/**
 * @brief 连接WiFi网络
 * 
 * 根据初始化时提供的配置连接到WiFi网络。此函数会阻塞直到连接成功或超时。
 * 
 * @return 
 *   - ESP_OK: 连接成功
 *   - ESP_ERR_WIFI_NOT_INIT: WiFi未初始化
 *   - ESP_ERR_WIFI_TIMEOUT: 连接超时
 *   - ESP_FAIL: 连接失败
 * 
 * @note 连接成功后会自动获取IP地址
 * @see wifi_manager_init()
 * @see wifi_manager_get_status()
 */
esp_err_t wifi_manager_connect(void);

/**
 * @brief 断开WiFi连接
 * 
 * 断开当前WiFi连接并停止WiFi驱动。
 * 
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_WIFI_NOT_INIT: WiFi未初始化
 * 
 * @note 断开后可以重新调用wifi_manager_connect()连接
 */
esp_err_t wifi_manager_disconnect(void);

/**
 * @brief 获取WiFi状态
 * 
 * 获取当前WiFi连接状态、信号强度、IP地址等信息。
 * 
 * @param[out] status 状态结构体指针
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: status为NULL
 *   - ESP_ERR_WIFI_NOT_INIT: WiFi未初始化
 * 
 * @note 此函数是线程安全的
 */
esp_err_t wifi_manager_get_status(wifi_status_t *status);

#ifdef __cplusplus
}
#endif

#endif // WIFI_MANAGER_H