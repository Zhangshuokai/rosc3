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
 * @brief WiFi重连参数结构体
 */
typedef struct {
    uint32_t base_delay_ms;     ///< 基础延迟（毫秒），默认：1000
    uint32_t max_delay_ms;      ///< 最大延迟（毫秒），默认：60000
    uint8_t max_attempts;       ///< 最大尝试次数，0=无限重连
    float backoff_factor;       ///< 退避因子，默认：2.0
} wifi_reconnect_params_t;

/**
 * @brief WiFi统计信息
 */
typedef struct {
    uint32_t connect_count;     ///< 总连接次数
    uint32_t disconnect_count;  ///< 总断开次数
    uint32_t reconnect_count;   ///< 重连次数
    int8_t rssi_avg;            ///< 平均RSSI
    int8_t rssi_min;            ///< 最小RSSI
    int8_t rssi_max;            ///< 最大RSSI
    uint32_t uptime_sec;        ///< 累计在线时间（秒）
} wifi_stats_t;

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

/**
 * @brief 启用/禁用自动重连
 *
 * 启用后，WiFi断线时会自动尝试重连，使用指数退避算法控制重连延迟。
 *
 * @param[in] enable true=启用自动重连, false=禁用自动重连
 * @return
 *   - ESP_OK: 成功
 *   - ESP_ERR_WIFI_NOT_INIT: WiFi未初始化
 *
 * @note 此功能默认禁用，需要显式启用
 * @note 线程安全
 */
esp_err_t wifi_manager_set_auto_reconnect(bool enable);

/**
 * @brief 设置自动重连参数
 *
 * 配置重连延迟算法的参数。重连延迟计算公式：
 * delay = min(base_delay * (backoff_factor ^ attempt), max_delay)
 *
 * @param[in] params 重连参数配置，NULL则使用默认参数
 * @return
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效
 *   - ESP_ERR_WIFI_NOT_INIT: WiFi未初始化
 *
 * @note 默认参数：base_delay=1000ms, max_delay=60000ms, max_attempts=0(无限), backoff_factor=2.0
 * @note 线程安全
 */
esp_err_t wifi_manager_set_reconnect_params(const wifi_reconnect_params_t *params);

/**
 * @brief 获取WiFi统计信息
 *
 * 获取WiFi连接统计数据，包括连接/断开次数、RSSI信息和在线时间
 *
 * @param[out] stats 统计信息结构体指针
 * @return
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: stats为NULL
 *   - ESP_ERR_WIFI_NOT_INIT: WiFi未初始化
 *
 * @note 此函数是线程安全的
 */
esp_err_t wifi_manager_get_stats(wifi_stats_t *stats);

/**
 * @brief 启动WiFi监控任务
 *
 * 创建独立任务定期监测WiFi信号强度（RSSI），更新统计信息。
 * 监控任务会定期采集RSSI、计算平均值、检测信号质量。
 *
 * @param[in] interval_ms 监控间隔（毫秒），建议范围：1000-10000
 * @return
 *   - ESP_OK: 成功启动
 *   - ESP_ERR_WIFI_NOT_INIT: WiFi未初始化
 *   - ESP_ERR_INVALID_STATE: 监控任务已在运行
 *   - ESP_ERR_NO_MEM: 内存不足
 *
 * @note 监控任务优先级为低优先级（tskIDLE_PRIORITY + 1）
 * @note 堆栈大小为2048字节
 * @note RSSI低于-75dBm时会输出警告日志
 */
esp_err_t wifi_manager_start_monitor(uint32_t interval_ms);

#ifdef __cplusplus
}
#endif

#endif // WIFI_MANAGER_H