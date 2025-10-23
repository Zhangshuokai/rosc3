/**
 * @file wifi_manager.c
 * @brief WiFi管理模块实现
 * @version 1.0
 * @date 2025-10-23
 * @author 嵌入式开发组
 * 
 * 本文件实现WiFi连接管理功能，包括事件处理、状态同步和网络配置
 * 
 * @copyright Copyright (c) 2025 ROSC3 Project
 */

#include "wifi_manager.h"

#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

/*******************************************************************
 * 常量定义
 *******************************************************************/

static const char *TAG = "WIFI_MGR";

// 事件组位定义
#define WIFI_CONNECTED_BIT      BIT0
#define WIFI_FAIL_BIT           BIT1
#define WIFI_GOT_IP_BIT         BIT2

/*******************************************************************
 * 内部数据结构
 *******************************************************************/

/**
 * @brief WiFi管理器上下文
 */
typedef struct {
    wifi_config_t config;           ///< WiFi配置
    wifi_status_t status;           ///< 当前状态
    EventGroupHandle_t event_group; ///< 事件组
    SemaphoreHandle_t mutex;        ///< 互斥锁
    esp_netif_t *netif;             ///< 网络接口
    uint8_t retry_count;            ///< 当前重试次数
    bool initialized;               ///< 初始化标志
} wifi_manager_context_t;

/*******************************************************************
 * 静态变量
 *******************************************************************/

static wifi_manager_context_t g_wifi_ctx = {0};

/*******************************************************************
 * 内部函数声明
 *******************************************************************/

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);
static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data);

/*******************************************************************
 * 公共函数实现
 *******************************************************************/

esp_err_t wifi_manager_init(const wifi_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // 检查是否已初始化
    if (g_wifi_ctx.initialized) {
        ESP_LOGE(TAG, "WiFi manager already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 验证配置参数
    if (strlen(config->ssid) == 0) {
        ESP_LOGE(TAG, "SSID is empty");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Initializing WiFi manager");

    // 创建互斥锁
    g_wifi_ctx.mutex = xSemaphoreCreateMutex();
    if (g_wifi_ctx.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // 创建事件组
    g_wifi_ctx.event_group = xEventGroupCreate();
    if (g_wifi_ctx.event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        vSemaphoreDelete(g_wifi_ctx.mutex);
        return ESP_ERR_NO_MEM;
    }

    // 保存配置
    memcpy(&g_wifi_ctx.config, config, sizeof(wifi_config_t));

    // 初始化TCP/IP网络接口
    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize netif: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // 创建默认事件循环
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // 创建默认WiFi STA网络接口
    g_wifi_ctx.netif = esp_netif_create_default_wifi_sta();
    if (g_wifi_ctx.netif == NULL) {
        ESP_LOGE(TAG, "Failed to create WiFi STA netif");
        ret = ESP_FAIL;
        goto cleanup;
    }

    // 初始化WiFi驱动配置
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // 注册WiFi事件处理器
    ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                     &wifi_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register WiFi event handler: %s", 
                 esp_err_to_name(ret));
        goto cleanup;
    }

    // 注册IP事件处理器
    ret = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                     &ip_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IP event handler: %s", 
                 esp_err_to_name(ret));
        goto cleanup;
    }

    // 设置WiFi模式为STA
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // 初始化状态
    g_wifi_ctx.status.state = WIFI_STATE_DISCONNECTED;
    g_wifi_ctx.retry_count = 0;
    g_wifi_ctx.initialized = true;

    ESP_LOGI(TAG, "WiFi manager initialized successfully");
    ESP_LOGI(TAG, "SSID: %s, Auth: %d", config->ssid, config->auth_mode);

    return ESP_OK;

cleanup:
    if (g_wifi_ctx.event_group != NULL) {
        vEventGroupDelete(g_wifi_ctx.event_group);
        g_wifi_ctx.event_group = NULL;
    }
    if (g_wifi_ctx.mutex != NULL) {
        vSemaphoreDelete(g_wifi_ctx.mutex);
        g_wifi_ctx.mutex = NULL;
    }
    return ret;
}

esp_err_t wifi_manager_connect(void)
{
    if (!g_wifi_ctx.initialized) {
        ESP_LOGE(TAG, "WiFi manager not initialized");
        return ESP_ERR_WIFI_NOT_INIT;
    }

    esp_err_t ret;
    wifi_config_t wifi_config = {0};

    ESP_LOGI(TAG, "Connecting to WiFi SSID: %s", g_wifi_ctx.config.ssid);

    // 配置WiFi参数
    strncpy((char *)wifi_config.sta.ssid, g_wifi_ctx.config.ssid, 
            sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, g_wifi_ctx.config.password,
            sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = g_wifi_ctx.config.auth_mode;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    // 设置WiFi配置
    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }

    // 启动WiFi
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    // 更新状态
    if (xSemaphoreTake(g_wifi_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_wifi_ctx.status.state = WIFI_STATE_CONNECTING;
        xSemaphoreGive(g_wifi_ctx.mutex);
    }

    // 等待连接结果
    uint32_t timeout_ms = g_wifi_ctx.config.timeout_ms;
    if (timeout_ms == 0) {
        timeout_ms = 10000; // 默认10秒超时
    }

    EventBits_t bits = xEventGroupWaitBits(
        g_wifi_ctx.event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_GOT_IP_BIT,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(timeout_ms)
    );

    // 检查结果
    if (bits & WIFI_GOT_IP_BIT) {
        ESP_LOGI(TAG, "Connected to WiFi successfully");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "WiFi connection timeout");
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t wifi_manager_disconnect(void)
{
    if (!g_wifi_ctx.initialized) {
        ESP_LOGE(TAG, "WiFi manager not initialized");
        return ESP_ERR_WIFI_NOT_INIT;
    }

    ESP_LOGI(TAG, "Disconnecting WiFi");

    esp_err_t ret = esp_wifi_disconnect();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_STARTED) {
        ESP_LOGE(TAG, "Failed to disconnect: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_stop();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_INIT) {
        ESP_LOGE(TAG, "Failed to stop WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    // 更新状态
    if (xSemaphoreTake(g_wifi_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_wifi_ctx.status.state = WIFI_STATE_DISCONNECTED;
        xSemaphoreGive(g_wifi_ctx.mutex);
    }

    ESP_LOGI(TAG, "WiFi disconnected");
    return ESP_OK;
}

esp_err_t wifi_manager_get_status(wifi_status_t *status)
{
    if (status == NULL) {
        ESP_LOGE(TAG, "Status pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_wifi_ctx.initialized) {
        ESP_LOGE(TAG, "WiFi manager not initialized");
        return ESP_ERR_WIFI_NOT_INIT;
    }

    // 使用互斥锁保护状态读取
    if (xSemaphoreTake(g_wifi_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(status, &g_wifi_ctx.status, sizeof(wifi_status_t));
        xSemaphoreGive(g_wifi_ctx.mutex);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
}

/*******************************************************************
 * 内部函数实现
 *******************************************************************/

/**
 * @brief WiFi事件处理器
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base != WIFI_EVENT) {
        return;
    }

    switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi STA started, connecting...");
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_CONNECTED:
        {
            wifi_event_sta_connected_t *event = 
                (wifi_event_sta_connected_t *)event_data;
            ESP_LOGI(TAG, "Connected to AP, SSID: %s, Channel: %d",
                     event->ssid, event->channel);
            
            // 更新状态
            if (xSemaphoreTake(g_wifi_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                g_wifi_ctx.status.state = WIFI_STATE_CONNECTED;
                g_wifi_ctx.status.channel = event->channel;
                g_wifi_ctx.retry_count = 0;
                xSemaphoreGive(g_wifi_ctx.mutex);
            }
            
            xEventGroupSetBits(g_wifi_ctx.event_group, WIFI_CONNECTED_BIT);
            break;
        }

        case WIFI_EVENT_STA_DISCONNECTED:
        {
            wifi_event_sta_disconnected_t *event = 
                (wifi_event_sta_disconnected_t *)event_data;
            ESP_LOGW(TAG, "Disconnected from AP, reason: %d", event->reason);
            
            // 更新状态
            if (xSemaphoreTake(g_wifi_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                g_wifi_ctx.status.state = WIFI_STATE_DISCONNECTED;
                xSemaphoreGive(g_wifi_ctx.mutex);
            }

            // 检查是否需要重试
            if (g_wifi_ctx.retry_count < g_wifi_ctx.config.max_retry) {
                g_wifi_ctx.retry_count++;
                ESP_LOGI(TAG, "Retrying connection (%d/%d)...", 
                         g_wifi_ctx.retry_count, g_wifi_ctx.config.max_retry);
                esp_wifi_connect();
            } else {
                ESP_LOGE(TAG, "Max retry count reached");
                xEventGroupSetBits(g_wifi_ctx.event_group, WIFI_FAIL_BIT);
                
                if (xSemaphoreTake(g_wifi_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    g_wifi_ctx.status.state = WIFI_STATE_ERROR;
                    xSemaphoreGive(g_wifi_ctx.mutex);
                }
            }
            break;
        }

        default:
            break;
    }
}

/**
 * @brief IP事件处理器
 */
static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    if (event_base != IP_EVENT || event_id != IP_EVENT_STA_GOT_IP) {
        return;
    }

    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
    ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
    ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));

    // 更新状态
    if (xSemaphoreTake(g_wifi_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_wifi_ctx.status.ip = event->ip_info.ip;
        g_wifi_ctx.status.gateway = event->ip_info.gw;
        g_wifi_ctx.status.netmask = event->ip_info.netmask;

        // 获取MAC地址
        esp_wifi_get_mac(WIFI_IF_STA, g_wifi_ctx.status.mac);

        // 获取RSSI
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            g_wifi_ctx.status.rssi = ap_info.rssi;
            ESP_LOGI(TAG, "Signal strength (RSSI): %d dBm", ap_info.rssi);
        }

        xSemaphoreGive(g_wifi_ctx.mutex);
    }

    xEventGroupSetBits(g_wifi_ctx.event_group, WIFI_GOT_IP_BIT);
}