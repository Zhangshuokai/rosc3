/**
 * @file config_manager.h
 * @brief 配置管理模块接口定义
 * @version 1.0
 * @date 2025-10-23
 * @author 嵌入式开发组
 * 
 * 本文件定义配置管理模块的接口，提供NVS持久化配置的读写功能
 * 
 * @copyright Copyright (c) 2025 Your Company
 */

#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 配置命名空间
 */
#define CONFIG_NAMESPACE "rosc3_cfg"

/**
 * @brief 标准配置键定义
 */
#define CFG_KEY_WIFI_SSID        "wifi_ssid"
#define CFG_KEY_WIFI_PASSWORD    "wifi_pass"
#define CFG_KEY_WIFI_AUTH        "wifi_auth"
#define CFG_KEY_ROS_AGENT_IP     "ros_ip"
#define CFG_KEY_ROS_AGENT_PORT   "ros_port"
#define CFG_KEY_ROS_NODE_NAME    "ros_node"
#define CFG_KEY_LOG_LEVEL        "log_level"
#define CFG_KEY_WIFI_MAX_RETRY   "wifi_retry"
#define CFG_KEY_WIFI_TIMEOUT     "wifi_timeout"
#define CFG_KEY_ROS_NAMESPACE    "ros_ns"
#define CFG_KEY_ROS_DOMAIN_ID    "ros_domain"

/**
 * @brief 节点配置结构体
 */
typedef struct {
    // WiFi配置
    char wifi_ssid[32];              ///< WiFi SSID
    char wifi_password[64];          ///< WiFi密码
    uint8_t wifi_auth_mode;          ///< 加密方式
    uint8_t wifi_max_retry;          ///< 最大重试次数
    uint32_t wifi_timeout_ms;        ///< 连接超时（毫秒）
    
    // ROS配置
    char ros_agent_ip[16];           ///< ROS Agent IP地址
    uint16_t ros_agent_port;         ///< ROS Agent端口
    char ros_node_name[32];          ///< ROS节点名称
    char ros_node_namespace[32];     ///< ROS节点命名空间
    uint8_t ros_domain_id;           ///< ROS Domain ID
    
    // 系统配置
    uint8_t log_level;               ///< 日志级别
} node_config_t;

/**
 * @brief 初始化配置管理器
 * 
 * 初始化NVS分区，如果NVS损坏则自动擦除并重建
 * 
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_NO_MEM: 内存不足
 *   - ESP_FAIL: 初始化失败
 * 
 * @note 此函数必须在使用其他配置函数前调用
 * @note 此函数是线程安全的
 */
esp_err_t config_manager_init(void);

/**
 * @brief 读取字符串配置
 * 
 * 从NVS读取指定键的字符串值
 * 
 * @param[in] key 配置键名
 * @param[out] value 配置值缓冲区
 * @param[in] max_len 缓冲区最大长度（包含终止符）
 * 
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效（key或value为NULL，max_len为0）
 *   - ESP_ERR_INVALID_STATE: 配置管理器未初始化
 *   - ESP_ERR_NVS_NOT_FOUND: 配置键不存在
 *   - ESP_ERR_NVS_INVALID_LENGTH: 缓冲区太小
 * 
 * @note 此函数是线程安全的
 */
esp_err_t config_get_str(const char *key, char *value, size_t max_len);

/**
 * @brief 写入字符串配置
 * 
 * 将字符串值写入NVS指定键
 * 
 * @param[in] key 配置键名
 * @param[in] value 配置值
 * 
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效（key或value为NULL）
 *   - ESP_ERR_INVALID_STATE: 配置管理器未初始化
 *   - ESP_ERR_NVS_INVALID_LENGTH: 字符串太长
 * 
 * @note 更改不会立即持久化到Flash，需要调用 config_save()
 * @note 此函数是线程安全的
 */
esp_err_t config_set_str(const char *key, const char *value);

/**
 * @brief 读取整数配置
 * 
 * 从NVS读取指定键的整数值
 * 
 * @param[in] key 配置键名
 * @param[out] value 配置值指针
 * 
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效（key或value为NULL）
 *   - ESP_ERR_INVALID_STATE: 配置管理器未初始化
 *   - ESP_ERR_NVS_NOT_FOUND: 配置键不存在
 * 
 * @note 此函数是线程安全的
 */
esp_err_t config_get_int(const char *key, int32_t *value);

/**
 * @brief 写入整数配置
 * 
 * 将整数值写入NVS指定键
 * 
 * @param[in] key 配置键名
 * @param[in] value 配置值
 * 
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效（key为NULL）
 *   - ESP_ERR_INVALID_STATE: 配置管理器未初始化
 * 
 * @note 更改不会立即持久化到Flash，需要调用 config_save()
 * @note 此函数是线程安全的
 */
esp_err_t config_set_int(const char *key, int32_t value);

/**
 * @brief 保存配置到Flash
 * 
 * 提交所有未保存的配置更改到Flash存储
 * 
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_STATE: 配置管理器未初始化
 *   - ESP_FAIL: 提交失败
 * 
 * @note 此函数是线程安全的
 * @warning 频繁调用此函数会缩短Flash寿命，建议批量更新后再保存
 */
esp_err_t config_save(void);

/**
 * @brief 恢复出厂设置
 * 
 * 擦除NVS中的所有配置数据，恢复到出厂状态
 * 
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_STATE: 配置管理器未初始化
 *   - ESP_FAIL: 擦除失败
 * 
 * @note 此函数是线程安全的
 * @warning 此操作不可逆，所有配置将丢失
 */
esp_err_t config_reset_to_defaults(void);

/**
 * @brief 加载配置（NVS优先，否则使用sdkconfig）
 *
 * 按照优先级从NVS和sdkconfig加载配置：
 * 1. 首先尝试从NVS读取用户配置
 * 2. 如果NVS中不存在，使用sdkconfig中的CONFIG_XXX宏作为默认值
 * 3. 首次运行时，将sdkconfig默认值写入NVS
 *
 * @param[out] config 配置结构体指针
 *
 * @return
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: config为NULL
 *   - ESP_ERR_INVALID_STATE: 配置管理器未初始化
 *
 * @note 此函数是线程安全的
 * @note 必须先调用 config_manager_init()
 */
esp_err_t config_load(node_config_t *config);

/**
 * @brief 验证配置有效性
 *
 * 验证规则：
 * - SSID非空（长度>0）
 * - WiFi密码长度符合要求（WPA2至少8字符）
 * - IP地址格式正确
 * - 端口范围1024-65535
 * - 节点名称非空且符合ROS命名规范
 *
 * @param[in] config 配置结构体指针
 *
 * @return
 *   - ESP_OK: 配置有效
 *   - ESP_ERR_INVALID_ARG: config为NULL或配置无效
 *
 * @note 配置无效时会输出详细的错误日志
 */
esp_err_t config_validate(const node_config_t *config);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_MANAGER_H