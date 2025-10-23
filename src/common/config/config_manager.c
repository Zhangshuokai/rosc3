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
    
    // 创建互斥锁
    s_nvs_mutex = xSemaphoreCreateMutex();
    if (s_nvs_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // 初始化NVS并打开命名空间
    ret = nvs_init_and_open();
    if (ret != ESP_OK) {
        vSemaphoreDelete(s_nvs_mutex);
        s_nvs_mutex = NULL;
        return ret;
    }
    
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