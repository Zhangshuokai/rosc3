/**
 * @file config_manager.c
 * @brief 配置管理模块实现
 * @version 1.0
 * @date 2025-10-23
 * @author 嵌入式开发组
 * 
 * 本文件实现基于NVS的配置持久化存储功能
 * 
 * @copyright Copyright (c) 2025 Your Company
 */

#include "config_manager.h"

// C标准库
#include <string.h>
#include <stdbool.h>
#include <ctype.h>

// ESP-IDF网络头文件
#include "esp_netif.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

// ESP-IDF系统头文件
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/*******************************************************************
 * 常量定义
 *******************************************************************/

static const char *TAG = "CONFIG_MGR";

/*******************************************************************
 * 回调链表节点结构
 *******************************************************************/

/**
 * @brief 配置变更回调链表节点
 */
typedef struct config_callback_node {
    config_change_callback_t callback;      ///< 回调函数
    void *user_data;                        ///< 用户数据
    struct config_callback_node *next;      ///< 下一个节点
} config_callback_node_t;

/*******************************************************************
 * 静态变量
 *******************************************************************/

/**
 * @brief NVS句柄
 */
static nvs_handle_t s_nvs_handle = 0;

/**
 * @brief 初始化标志
 */
static bool s_is_initialized = false;

/**
 * @brief 互斥锁保护NVS操作
 */
static SemaphoreHandle_t s_nvs_mutex = NULL;

/**
 * @brief 回调链表头
 */
static config_callback_node_t *s_callback_list = NULL;

/**
 * @brief 回调链表互斥锁
 */
static SemaphoreHandle_t s_callback_mutex = NULL;

/*******************************************************************
 * 内部函数声明
 *******************************************************************/

static esp_err_t nvs_init_and_open(void);

/*******************************************************************
 * 公共函数实现
 *******************************************************************/

esp_err_t config_manager_init(void) {
    esp_err_t ret = ESP_OK;
    
    // 检查是否已初始化
    if (s_is_initialized) {
        ESP_LOGW(TAG, "Config manager already initialized");
        return ESP_OK;
    }
    
    // 创建NVS互斥锁
    s_nvs_mutex = xSemaphoreCreateMutex();
    if (s_nvs_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create NVS mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // 创建回调互斥锁
    s_callback_mutex = xSemaphoreCreateMutex();
    if (s_callback_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create callback mutex");
        vSemaphoreDelete(s_nvs_mutex);
        s_nvs_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    // 初始化NVS并打开命名空间
    ret = nvs_init_and_open();
    if (ret != ESP_OK) {
        vSemaphoreDelete(s_nvs_mutex);
        vSemaphoreDelete(s_callback_mutex);
        s_nvs_mutex = NULL;
        s_callback_mutex = NULL;
        return ret;
    }
    
    // 初始化回调链表为空
    s_callback_list = NULL;
    
    s_is_initialized = true;
    ESP_LOGI(TAG, "Config manager initialized successfully");
    
    return ESP_OK;
}

esp_err_t config_get_str(const char *key, char *value, size_t max_len) {
    esp_err_t ret = ESP_OK;
    
    // 参数检查
    if (key == NULL || value == NULL || max_len == 0) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_is_initialized) {
        ESP_LOGE(TAG, "Config manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(s_nvs_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // 读取字符串
    size_t required_size = max_len;
    ret = nvs_get_str(s_nvs_handle, key, value, &required_size);
    
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Read string: %s = %s", key, value);
    } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGD(TAG, "Key not found: %s", key);
    } else {
        ESP_LOGE(TAG, "Failed to read string %s: %s", key, esp_err_to_name(ret));
    }
    
    // 释放互斥锁
    xSemaphoreGive(s_nvs_mutex);
    
    return ret;
}

esp_err_t config_set_str(const char *key, const char *value) {
    esp_err_t ret = ESP_OK;
    
    // 参数检查
    if (key == NULL || value == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_is_initialized) {
        ESP_LOGE(TAG, "Config manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(s_nvs_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // 写入字符串
    ret = nvs_set_str(s_nvs_handle, key, value);
    
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Write string: %s = %s", key, value);
    } else {
        ESP_LOGE(TAG, "Failed to write string %s: %s", key, esp_err_to_name(ret));
    }
    
    // 释放互斥锁
    xSemaphoreGive(s_nvs_mutex);
    
    return ret;
}

esp_err_t config_get_int(const char *key, int32_t *value) {
    esp_err_t ret = ESP_OK;
    
    // 参数检查
    if (key == NULL || value == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_is_initialized) {
        ESP_LOGE(TAG, "Config manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(s_nvs_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // 读取整数
    ret = nvs_get_i32(s_nvs_handle, key, value);
    
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Read integer: %s = %ld", key, (long)*value);
    } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGD(TAG, "Key not found: %s", key);
    } else {
        ESP_LOGE(TAG, "Failed to read integer %s: %s", key, esp_err_to_name(ret));
    }
    
    // 释放互斥锁
    xSemaphoreGive(s_nvs_mutex);
    
    return ret;
}

esp_err_t config_set_int(const char *key, int32_t value) {
    esp_err_t ret = ESP_OK;
    
    // 参数检查
    if (key == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_is_initialized) {
        ESP_LOGE(TAG, "Config manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(s_nvs_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // 写入整数
    ret = nvs_set_i32(s_nvs_handle, key, value);
    
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Write integer: %s = %ld", key, (long)value);
    } else {
        ESP_LOGE(TAG, "Failed to write integer %s: %s", key, esp_err_to_name(ret));
    }
    
    // 释放互斥锁
    xSemaphoreGive(s_nvs_mutex);
    
    return ret;
}

esp_err_t config_save(void) {
    esp_err_t ret = ESP_OK;
    
    if (!s_is_initialized) {
        ESP_LOGE(TAG, "Config manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(s_nvs_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // 提交更改到Flash
    ret = nvs_commit(s_nvs_handle);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Config saved to Flash successfully");
    } else {
        ESP_LOGE(TAG, "Failed to save config: %s", esp_err_to_name(ret));
    }
    
    // 释放互斥锁
    xSemaphoreGive(s_nvs_mutex);
    
    return ret;
}

esp_err_t config_reset_to_defaults(void) {
    esp_err_t ret = ESP_OK;
    
    if (!s_is_initialized) {
        ESP_LOGE(TAG, "Config manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(s_nvs_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // 擦除命名空间中的所有数据
    ret = nvs_erase_all(s_nvs_handle);
    
    if (ret == ESP_OK) {
        // 提交更改
        ret = nvs_commit(s_nvs_handle);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Config reset to defaults successfully");
        } else {
            ESP_LOGE(TAG, "Failed to commit after reset: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "Failed to erase config: %s", esp_err_to_name(ret));
    }
    
    // 释放互斥锁
    xSemaphoreGive(s_nvs_mutex);
    
    return ret;
}

/*******************************************************************
 * 内部函数实现
 *******************************************************************/

/**
 * @brief 初始化NVS并打开配置命名空间
 * 
 * 处理NVS损坏、版本不匹配等错误情况，必要时自动擦除并重建
 * 
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_FAIL: 失败
 */
static esp_err_t nvs_init_and_open(void) {
    esp_err_t ret;
    
    // 初始化NVS Flash
    ret = nvs_flash_init();
    
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS分区已满或版本不匹配，擦除并重新初始化
        ESP_LOGW(TAG, "NVS error (%s), erasing and reinitializing", esp_err_to_name(ret));
        
        ret = nvs_flash_erase();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase NVS: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = nvs_flash_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to reinitialize NVS: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ESP_LOGI(TAG, "NVS erased and reinitialized successfully");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 打开配置命名空间
    ret = nvs_open(CONFIG_NAMESPACE, NVS_READWRITE, &s_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s", 
                 CONFIG_NAMESPACE, esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "NVS namespace '%s' opened successfully", CONFIG_NAMESPACE);
    
    return ESP_OK;
}

/*******************************************************************
 * 辅助函数声明
 *******************************************************************/

static bool is_valid_ip_address(const char *ip);
static bool is_valid_ros_node_name(const char *name);

/*******************************************************************
 * 公共函数实现 - Menuconfig集成
 *******************************************************************/

esp_err_t config_load(node_config_t *config) {
    esp_err_t ret = ESP_OK;
    
    // 参数检查
    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid argument: config is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_is_initialized) {
        ESP_LOGE(TAG, "Config manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(s_nvs_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    ESP_LOGI(TAG, "Loading configuration (NVS > sdkconfig priority)");
    
    /*******************************************************************
     * WiFi配置加载
     *******************************************************************/
    
    // WiFi SSID
    size_t ssid_len = sizeof(config->wifi_ssid);
    ret = nvs_get_str(s_nvs_handle, CFG_KEY_WIFI_SSID,
                      config->wifi_ssid, &ssid_len);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        // 使用sdkconfig默认值
        #ifdef CONFIG_WIFI_SSID
        strlcpy(config->wifi_ssid, CONFIG_WIFI_SSID, sizeof(config->wifi_ssid));
        // 首次运行，写入NVS
        nvs_set_str(s_nvs_handle, CFG_KEY_WIFI_SSID, config->wifi_ssid);
        ESP_LOGI(TAG, "WiFi SSID: using sdkconfig default");
        #else
        config->wifi_ssid[0] = '\0';
        ESP_LOGW(TAG, "WiFi SSID: not configured");
        #endif
    } else if (ret == ESP_OK) {
        ESP_LOGI(TAG, "WiFi SSID: loaded from NVS");
    }
    
    // WiFi密码
    size_t pwd_len = sizeof(config->wifi_password);
    ret = nvs_get_str(s_nvs_handle, CFG_KEY_WIFI_PASSWORD,
                      config->wifi_password, &pwd_len);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        #ifdef CONFIG_WIFI_PASSWORD
        strlcpy(config->wifi_password, CONFIG_WIFI_PASSWORD, 
                sizeof(config->wifi_password));
        nvs_set_str(s_nvs_handle, CFG_KEY_WIFI_PASSWORD, config->wifi_password);
        ESP_LOGI(TAG, "WiFi Password: using sdkconfig default");
        #else
        config->wifi_password[0] = '\0';
        #endif
    } else if (ret == ESP_OK) {
        ESP_LOGI(TAG, "WiFi Password: loaded from NVS");
    }
    
    // WiFi认证模式
    int32_t auth_mode;
    ret = nvs_get_i32(s_nvs_handle, CFG_KEY_WIFI_AUTH, &auth_mode);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        #ifdef CONFIG_WIFI_AUTH_MODE_VALUE
        config->wifi_auth_mode = (uint8_t)CONFIG_WIFI_AUTH_MODE_VALUE;
        #else
        config->wifi_auth_mode = 3;  // 默认WPA2_PSK
        #endif
        nvs_set_i32(s_nvs_handle, CFG_KEY_WIFI_AUTH, config->wifi_auth_mode);
        ESP_LOGI(TAG, "WiFi Auth: using sdkconfig default (%d)", 
                 config->wifi_auth_mode);
    } else if (ret == ESP_OK) {
        config->wifi_auth_mode = (uint8_t)auth_mode;
        ESP_LOGI(TAG, "WiFi Auth: loaded from NVS (%d)", config->wifi_auth_mode);
    }
    
    // WiFi最大重试次数
    int32_t max_retry;
    ret = nvs_get_i32(s_nvs_handle, CFG_KEY_WIFI_MAX_RETRY, &max_retry);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        #ifdef CONFIG_WIFI_MAX_RETRY
        config->wifi_max_retry = (uint8_t)CONFIG_WIFI_MAX_RETRY;
        #else
        config->wifi_max_retry = 5;
        #endif
        nvs_set_i32(s_nvs_handle, CFG_KEY_WIFI_MAX_RETRY, config->wifi_max_retry);
    } else if (ret == ESP_OK) {
        config->wifi_max_retry = (uint8_t)max_retry;
    }
    
    // WiFi连接超时
    int32_t timeout;
    ret = nvs_get_i32(s_nvs_handle, CFG_KEY_WIFI_TIMEOUT, &timeout);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        #ifdef CONFIG_WIFI_CONNECT_TIMEOUT_MS
        config->wifi_timeout_ms = CONFIG_WIFI_CONNECT_TIMEOUT_MS;
        #else
        config->wifi_timeout_ms = 10000;
        #endif
        nvs_set_i32(s_nvs_handle, CFG_KEY_WIFI_TIMEOUT, config->wifi_timeout_ms);
    } else if (ret == ESP_OK) {
        config->wifi_timeout_ms = (uint32_t)timeout;
    }
    
    /*******************************************************************
     * ROS配置加载
     *******************************************************************/
    
    // ROS Agent IP
    size_t ip_len = sizeof(config->ros_agent_ip);
    ret = nvs_get_str(s_nvs_handle, CFG_KEY_ROS_AGENT_IP,
                      config->ros_agent_ip, &ip_len);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        #ifdef CONFIG_MICRO_ROS_AGENT_IP
        strlcpy(config->ros_agent_ip, CONFIG_MICRO_ROS_AGENT_IP,
                sizeof(config->ros_agent_ip));
        #else
        strlcpy(config->ros_agent_ip, "192.168.1.10", 
                sizeof(config->ros_agent_ip));
        #endif
        nvs_set_str(s_nvs_handle, CFG_KEY_ROS_AGENT_IP, config->ros_agent_ip);
        ESP_LOGI(TAG, "ROS Agent IP: using sdkconfig default");
    } else if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ROS Agent IP: loaded from NVS");
    }
    
    // ROS Agent端口
    int32_t port;
    ret = nvs_get_i32(s_nvs_handle, CFG_KEY_ROS_AGENT_PORT, &port);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        #ifdef CONFIG_MICRO_ROS_AGENT_PORT
        config->ros_agent_port = (uint16_t)CONFIG_MICRO_ROS_AGENT_PORT;
        #else
        config->ros_agent_port = 8888;
        #endif
        nvs_set_i32(s_nvs_handle, CFG_KEY_ROS_AGENT_PORT, config->ros_agent_port);
    } else if (ret == ESP_OK) {
        config->ros_agent_port = (uint16_t)port;
    }
    
    // ROS节点名称
    size_t node_name_len = sizeof(config->ros_node_name);
    ret = nvs_get_str(s_nvs_handle, CFG_KEY_ROS_NODE_NAME,
                      config->ros_node_name, &node_name_len);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        #ifdef CONFIG_ROS_NODE_NAME
        strlcpy(config->ros_node_name, CONFIG_ROS_NODE_NAME,
                sizeof(config->ros_node_name));
        #else
        strlcpy(config->ros_node_name, "esp32_node",
                sizeof(config->ros_node_name));
        #endif
        nvs_set_str(s_nvs_handle, CFG_KEY_ROS_NODE_NAME, config->ros_node_name);
        ESP_LOGI(TAG, "ROS Node Name: using sdkconfig default");
    } else if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ROS Node Name: loaded from NVS");
    }
    
    // ROS命名空间
    size_t ns_len = sizeof(config->ros_node_namespace);
    ret = nvs_get_str(s_nvs_handle, CFG_KEY_ROS_NAMESPACE,
                      config->ros_node_namespace, &ns_len);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        #ifdef CONFIG_ROS_NODE_NAMESPACE
        strlcpy(config->ros_node_namespace, CONFIG_ROS_NODE_NAMESPACE,
                sizeof(config->ros_node_namespace));
        #else
        strlcpy(config->ros_node_namespace, "/",
                sizeof(config->ros_node_namespace));
        #endif
        nvs_set_str(s_nvs_handle, CFG_KEY_ROS_NAMESPACE, 
                    config->ros_node_namespace);
    } else if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ROS Namespace: loaded from NVS");
    }
    
    // ROS Domain ID
    int32_t domain_id;
    ret = nvs_get_i32(s_nvs_handle, CFG_KEY_ROS_DOMAIN_ID, &domain_id);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        #ifdef CONFIG_ROS_DOMAIN_ID
        config->ros_domain_id = (uint8_t)CONFIG_ROS_DOMAIN_ID;
        #else
        config->ros_domain_id = 0;
        #endif
        nvs_set_i32(s_nvs_handle, CFG_KEY_ROS_DOMAIN_ID, config->ros_domain_id);
    } else if (ret == ESP_OK) {
        config->ros_domain_id = (uint8_t)domain_id;
    }
    
    /*******************************************************************
     * 系统配置加载
     *******************************************************************/
    
    // 日志级别
    int32_t log_level;
    ret = nvs_get_i32(s_nvs_handle, CFG_KEY_LOG_LEVEL, &log_level);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        #ifdef CONFIG_LOG_LEVEL_VALUE
        config->log_level = (uint8_t)CONFIG_LOG_LEVEL_VALUE;
        #else
        config->log_level = 3;  // INFO
        #endif
        nvs_set_i32(s_nvs_handle, CFG_KEY_LOG_LEVEL, config->log_level);
    } else if (ret == ESP_OK) {
        config->log_level = (uint8_t)log_level;
    }
    
    // 提交更改到Flash
    nvs_commit(s_nvs_handle);
    
    // 释放互斥锁
    xSemaphoreGive(s_nvs_mutex);
    
    ESP_LOGI(TAG, "Configuration loaded successfully");
    ESP_LOGI(TAG, "  WiFi: SSID=%s, Auth=%d, Retry=%d, Timeout=%lums",
             config->wifi_ssid, config->wifi_auth_mode, 
             config->wifi_max_retry, config->wifi_timeout_ms);
    ESP_LOGI(TAG, "  ROS: IP=%s:%d, Node=%s, NS=%s, Domain=%d",
             config->ros_agent_ip, config->ros_agent_port,
             config->ros_node_name, config->ros_node_namespace,
             config->ros_domain_id);
    ESP_LOGI(TAG, "  System: LogLevel=%d", config->log_level);
    
    return ESP_OK;
}

esp_err_t config_validate(const node_config_t *config) {
    // 参数检查
    if (config == NULL) {
        ESP_LOGE(TAG, "Validation failed: config is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Validating configuration...");
    
    /*******************************************************************
     * WiFi配置验证
     *******************************************************************/
    
    // SSID非空检查
    if (strlen(config->wifi_ssid) == 0) {
        ESP_LOGE(TAG, "Validation failed: WiFi SSID is empty");
        return ESP_ERR_INVALID_ARG;
    }
    
    // SSID长度检查
    if (strlen(config->wifi_ssid) > 32) {
        ESP_LOGE(TAG, "Validation failed: WiFi SSID too long (max 32 chars)");
        return ESP_ERR_INVALID_ARG;
    }
    
    // WiFi密码长度检查（WPA2/WPA3至少8字符）
    if (config->wifi_auth_mode == 3 || config->wifi_auth_mode == 7) {
        if (strlen(config->wifi_password) < 8) {
            ESP_LOGE(TAG, "Validation failed: WiFi password too short "
                     "(WPA2/WPA3 requires at least 8 characters)");
            return ESP_ERR_INVALID_ARG;
        }
        if (strlen(config->wifi_password) > 64) {
            ESP_LOGE(TAG, "Validation failed: WiFi password too long (max 64 chars)");
            return ESP_ERR_INVALID_ARG;
        }
    }
    
    // WiFi超时范围检查
    if (config->wifi_timeout_ms < 1000 || config->wifi_timeout_ms > 60000) {
        ESP_LOGE(TAG, "Validation failed: WiFi timeout out of range "
                 "(1000-60000ms)");
        return ESP_ERR_INVALID_ARG;
    }
    
    /*******************************************************************
     * ROS配置验证
     *******************************************************************/
    
    // IP地址格式验证
    if (!is_valid_ip_address(config->ros_agent_ip)) {
        ESP_LOGE(TAG, "Validation failed: Invalid ROS Agent IP address: %s",
                 config->ros_agent_ip);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 端口范围验证（uint16_t最大值为65535，无需检查上限）
    if (config->ros_agent_port < 1024) {
        ESP_LOGE(TAG, "Validation failed: ROS Agent port out of range "
                 "(1024-65535): %d", config->ros_agent_port);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 节点名称非空检查
    if (strlen(config->ros_node_name) == 0) {
        ESP_LOGE(TAG, "Validation failed: ROS node name is empty");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 节点名称格式验证（ROS命名规范）
    if (!is_valid_ros_node_name(config->ros_node_name)) {
        ESP_LOGE(TAG, "Validation failed: Invalid ROS node name: %s "
                 "(must contain only letters, numbers, underscores, "
                 "and cannot start with a number)", config->ros_node_name);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Domain ID范围检查
    if (config->ros_domain_id > 232) {
        ESP_LOGE(TAG, "Validation failed: ROS Domain ID out of range (0-232): %d",
                 config->ros_domain_id);
        return ESP_ERR_INVALID_ARG;
    }
    
    /*******************************************************************
     * 系统配置验证
     *******************************************************************/
    
    // 日志级别范围检查
    if (config->log_level > 5) {
        ESP_LOGE(TAG, "Validation failed: Log level out of range (0-5): %d",
                 config->log_level);
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Configuration validation passed");
    return ESP_OK;
}

/*******************************************************************
 * 辅助函数实现
 *******************************************************************/

/**
 * @brief 验证IP地址格式
 * 
 * 使用inet_pton()验证IPv4地址格式
 * 
 * @param[in] ip IP地址字符串
 * @return true=有效, false=无效
 */
static bool is_valid_ip_address(const char *ip) {
    if (ip == NULL || strlen(ip) == 0) {
        return false;
    }
    
    struct in_addr addr;
    // inet_pton返回1表示成功
    return inet_pton(AF_INET, ip, &addr) == 1;
}

/**
 * @brief 验证ROS节点名称格式
 * 
 * ROS命名规范：
 * - 仅包含字母、数字、下划线
 * - 不能以数字开头
 * - 长度1-255字符
 * 
 * @param[in] name 节点名称
 * @return true=有效, false=无效
 */
static bool is_valid_ros_node_name(const char *name) {
    if (name == NULL) {
        return false;
    }
    
    size_t len = strlen(name);
    if (len == 0 || len > 255) {
        return false;
    }
    
    // 不能以数字开头
    if (isdigit((unsigned char)name[0])) {
        return false;
    }
    
    // 检查每个字符
    for (size_t i = 0; i < len; i++) {
        char c = name[i];
        if (!isalnum((unsigned char)c) && c != '_') {
            return false;
        }
    }
    
    return true;
}

/*******************************************************************
 * 运行时配置更新功能实现 (TASK-COMMON-010)
 *******************************************************************/

/**
 * @brief 通知所有注册的回调函数
 * 
 * @param[in] key 配置键
 * @param[in] type 配置类型
 * @param[in] value 新值指针
 */
static void notify_callbacks(const char *key, config_change_type_t type, const void *value) {
    if (s_callback_mutex == NULL) {
        return;
    }
    
    // 获取回调互斥锁
    if (xSemaphoreTake(s_callback_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire callback mutex");
        return;
    }
    
    // 遍历回调链表
    config_callback_node_t *node = s_callback_list;
    while (node != NULL) {
        if (node->callback != NULL) {
            node->callback(key, type, value, node->user_data);
        }
        node = node->next;
    }
    
    // 释放互斥锁
    xSemaphoreGive(s_callback_mutex);
}

esp_err_t config_register_callback(
    config_change_callback_t callback,
    void *user_data
) {
    // 参数检查
    if (callback == NULL) {
        ESP_LOGE(TAG, "Invalid argument: callback is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_is_initialized) {
        ESP_LOGE(TAG, "Config manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 分配新节点
    config_callback_node_t *new_node = (config_callback_node_t *)malloc(sizeof(config_callback_node_t));
    if (new_node == NULL) {
        ESP_LOGE(TAG, "Failed to allocate callback node");
        return ESP_ERR_NO_MEM;
    }
    
    new_node->callback = callback;
    new_node->user_data = user_data;
    new_node->next = NULL;
    
    // 获取互斥锁
    if (xSemaphoreTake(s_callback_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire callback mutex");
        free(new_node);
        return ESP_ERR_TIMEOUT;
    }
    
    // 添加到链表头部
    new_node->next = s_callback_list;
    s_callback_list = new_node;
    
    // 释放互斥锁
    xSemaphoreGive(s_callback_mutex);
    
    ESP_LOGI(TAG, "Callback registered successfully");
    return ESP_OK;
}

esp_err_t config_unregister_callback(config_change_callback_t callback) {
    // 参数检查
    if (callback == NULL) {
        ESP_LOGE(TAG, "Invalid argument: callback is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_is_initialized) {
        ESP_LOGE(TAG, "Config manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(s_callback_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire callback mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // 在链表中查找并删除
    config_callback_node_t **pp = &s_callback_list;
    config_callback_node_t *node;
    bool found = false;
    
    while ((node = *pp) != NULL) {
        if (node->callback == callback) {
            *pp = node->next;
            free(node);
            found = true;
            break;
        }
        pp = &node->next;
    }
    
    // 释放互斥锁
    xSemaphoreGive(s_callback_mutex);
    
    if (!found) {
        ESP_LOGW(TAG, "Callback not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "Callback unregistered successfully");
    return ESP_OK;
}

esp_err_t config_update_str_and_notify(
    const char *key,
    const char *value,
    bool auto_save
) {
    esp_err_t ret;
    
    // 参数检查
    if (key == NULL || value == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_is_initialized) {
        ESP_LOGE(TAG, "Config manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 更新配置
    ret = config_set_str(key, value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update config: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 自动保存（如果需要）
    if (auto_save) {
        ret = config_save();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to auto-save config: %s", esp_err_to_name(ret));
        }
    }
    
    // 通知回调
    notify_callbacks(key, CONFIG_CHANGE_TYPE_STR, value);
    
    // 检查是否支持热更新
    if (config_is_hot_updatable(key)) {
        ESP_LOGI(TAG, "Config updated (hot): %s = %s", key, value);
    } else {
        ESP_LOGW(TAG, "Config updated (requires restart): %s = %s", key, value);
    }
    
    return ESP_OK;
}

esp_err_t config_update_int_and_notify(
    const char *key,
    int32_t value,
    bool auto_save
) {
    esp_err_t ret;
    
    // 参数检查
    if (key == NULL) {
        ESP_LOGE(TAG, "Invalid argument: key is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_is_initialized) {
        ESP_LOGE(TAG, "Config manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 更新配置
    ret = config_set_int(key, value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update config: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 自动保存（如果需要）
    if (auto_save) {
        ret = config_save();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to auto-save config: %s", esp_err_to_name(ret));
        }
    }
    
    // 通知回调
    notify_callbacks(key, CONFIG_CHANGE_TYPE_INT, &value);
    
    // 检查是否支持热更新
    if (config_is_hot_updatable(key)) {
        ESP_LOGI(TAG, "Config updated (hot): %s = %ld", key, (long)value);
        
        // 热更新：日志级别立即生效
        if (strcmp(key, CFG_KEY_LOG_LEVEL) == 0) {
            esp_log_level_set("*", (esp_log_level_t)value);
            ESP_LOGI(TAG, "Log level changed to %ld", (long)value);
        }
    } else {
        ESP_LOGW(TAG, "Config updated (requires restart): %s = %ld", key, (long)value);
    }
    
    return ESP_OK;
}

esp_err_t config_backup(const char *backup_namespace) {
    esp_err_t ret;
    nvs_handle_t backup_handle = 0;
    nvs_iterator_t it = NULL;
    
    // 参数检查
    if (backup_namespace == NULL) {
        ESP_LOGE(TAG, "Invalid argument: backup_namespace is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_is_initialized) {
        ESP_LOGE(TAG, "Config manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Backing up configuration to '%s'", backup_namespace);
    
    // 获取互斥锁
    if (xSemaphoreTake(s_nvs_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // 打开备份命名空间
    ret = nvs_open(backup_namespace, NVS_READWRITE, &backup_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open backup namespace: %s", esp_err_to_name(ret));
        xSemaphoreGive(s_nvs_mutex);
        return ret;
    }
    
    // 擦除备份命名空间（确保干净）
    ret = nvs_erase_all(backup_handle);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to erase backup namespace: %s", esp_err_to_name(ret));
        nvs_close(backup_handle);
        xSemaphoreGive(s_nvs_mutex);
        return ret;
    }
    
    // 遍历当前命名空间的所有键值对
    ret = nvs_entry_find(NVS_DEFAULT_PART_NAME, CONFIG_NAMESPACE, NVS_TYPE_ANY, &it);
    while (ret == ESP_OK && it != NULL) {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);
        
        // 根据类型复制键值对
        if (info.type == NVS_TYPE_STR) {
            size_t len = 0;
            ret = nvs_get_str(s_nvs_handle, info.key, NULL, &len);
            if (ret == ESP_OK && len > 0) {
                char *value = (char *)malloc(len);
                if (value != NULL) {
                    ret = nvs_get_str(s_nvs_handle, info.key, value, &len);
                    if (ret == ESP_OK) {
                        nvs_set_str(backup_handle, info.key, value);
                    }
                    free(value);
                }
            }
        } else if (info.type == NVS_TYPE_I32) {
            int32_t value;
            ret = nvs_get_i32(s_nvs_handle, info.key, &value);
            if (ret == ESP_OK) {
                nvs_set_i32(backup_handle, info.key, value);
            }
        }
        
        ret = nvs_entry_next(&it);
    }
    
    if (it != NULL) {
        nvs_release_iterator(it);
    }
    
    // 提交备份
    ret = nvs_commit(backup_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit backup: %s", esp_err_to_name(ret));
        nvs_close(backup_handle);
        xSemaphoreGive(s_nvs_mutex);
        return ret;
    }
    
    nvs_close(backup_handle);
    xSemaphoreGive(s_nvs_mutex);
    
    ESP_LOGI(TAG, "Configuration backed up successfully to '%s'", backup_namespace);
    return ESP_OK;
}

esp_err_t config_restore(const char *backup_namespace) {
    esp_err_t ret;
    nvs_handle_t backup_handle = 0;
    nvs_iterator_t it = NULL;
    
    // 参数检查
    if (backup_namespace == NULL) {
        ESP_LOGE(TAG, "Invalid argument: backup_namespace is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_is_initialized) {
        ESP_LOGE(TAG, "Config manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Restoring configuration from '%s'", backup_namespace);
    
    // 获取互斥锁
    if (xSemaphoreTake(s_nvs_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // 打开备份命名空间
    ret = nvs_open(backup_namespace, NVS_READONLY, &backup_handle);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGE(TAG, "Backup namespace '%s' not found", backup_namespace);
        } else {
            ESP_LOGE(TAG, "Failed to open backup namespace: %s", esp_err_to_name(ret));
        }
        xSemaphoreGive(s_nvs_mutex);
        return ret;
    }
    
    // 擦除当前配置
    ret = nvs_erase_all(s_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase current config: %s", esp_err_to_name(ret));
        nvs_close(backup_handle);
        xSemaphoreGive(s_nvs_mutex);
        return ret;
    }
    
    // 遍历备份命名空间的所有键值对
    ret = nvs_entry_find(NVS_DEFAULT_PART_NAME, backup_namespace, NVS_TYPE_ANY, &it);
    while (ret == ESP_OK && it != NULL) {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);
        
        // 根据类型复制键值对
        if (info.type == NVS_TYPE_STR) {
            size_t len = 0;
            ret = nvs_get_str(backup_handle, info.key, NULL, &len);
            if (ret == ESP_OK && len > 0) {
                char *value = (char *)malloc(len);
                if (value != NULL) {
                    ret = nvs_get_str(backup_handle, info.key, value, &len);
                    if (ret == ESP_OK) {
                        nvs_set_str(s_nvs_handle, info.key, value);
                    }
                    free(value);
                }
            }
        } else if (info.type == NVS_TYPE_I32) {
            int32_t value;
            ret = nvs_get_i32(backup_handle, info.key, &value);
            if (ret == ESP_OK) {
                nvs_set_i32(s_nvs_handle, info.key, value);
            }
        }
        
        ret = nvs_entry_next(&it);
    }
    
    if (it != NULL) {
        nvs_release_iterator(it);
    }
    
    // 提交恢复的配置
    ret = nvs_commit(s_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit restored config: %s", esp_err_to_name(ret));
        nvs_close(backup_handle);
        xSemaphoreGive(s_nvs_mutex);
        return ret;
    }
    
    nvs_close(backup_handle);
    xSemaphoreGive(s_nvs_mutex);
    
    ESP_LOGI(TAG, "Configuration restored successfully from '%s'", backup_namespace);
    ESP_LOGW(TAG, "Please restart the device for all changes to take effect");
    
    return ESP_OK;
}

bool config_is_hot_updatable(const char *key) {
    if (key == NULL) {
        return false;
    }
    
    // 支持热更新的配置项
    if (strcmp(key, CFG_KEY_LOG_LEVEL) == 0) {
        return true;
    }
    
    // 其他配置需要重启
    return false;
}