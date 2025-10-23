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
#include "freertos/timers.h"
#include <math.h>

/*******************************************************************
 * 常量定义
 *******************************************************************/

static const char *TAG = "WIFI_MGR";

// 重连历史记录最大条数
#define RECONNECT_HISTORY_SIZE  10

// RSSI滑动窗口大小
#define RSSI_WINDOW_SIZE        10

// RSSI警告阈值（dBm）
#define RSSI_WARN_THRESHOLD     -75

// 监控任务配置
#define MONITOR_TASK_STACK_SIZE 2048
#define MONITOR_TASK_PRIORITY   (tskIDLE_PRIORITY + 1)

// 事件组位定义
#define WIFI_CONNECTED_BIT      BIT0
#define WIFI_FAIL_BIT           BIT1
#define WIFI_GOT_IP_BIT         BIT2

/*******************************************************************
 * 内部数据结构
 *******************************************************************/

/**
 * @brief 重连历史记录条目
 */
typedef struct {
    uint32_t timestamp;             ///< 时间戳（系统运行时间，毫秒）
    bool success;                   ///< 重连是否成功
    uint8_t attempt;                ///< 尝试次数
} reconnect_history_entry_t;

/**
 * @brief WiFi管理器上下文
 */
typedef struct {
    wifi_manager_config_t config;   ///< WiFi配置
    wifi_status_t status;           ///< 当前状态
    EventGroupHandle_t event_group; ///< 事件组
    SemaphoreHandle_t mutex;        ///< 互斥锁
    esp_netif_t *netif;             ///< 网络接口
    uint8_t retry_count;            ///< 当前重试次数
    bool initialized;               ///< 初始化标志
    
    // 自动重连相关
    bool auto_reconnect_enabled;    ///< 自动重连启用标志
    wifi_reconnect_params_t reconnect_params;  ///< 重连参数
    TimerHandle_t reconnect_timer;  ///< 重连定时器
    uint8_t reconnect_attempt;      ///< 当前重连尝试次数
    
    // 重连历史记录
    reconnect_history_entry_t history[RECONNECT_HISTORY_SIZE];
    uint8_t history_index;          ///< 历史记录索引
    uint8_t history_count;          ///< 历史记录条数
    
    // WiFi统计信息
    wifi_stats_t stats;             ///< 统计数据
    int8_t rssi_window[RSSI_WINDOW_SIZE];  ///< RSSI滑动窗口
    uint8_t rssi_window_index;      ///< 窗口索引
    uint8_t rssi_window_count;      ///< 窗口有效数据数
    uint32_t connect_timestamp;     ///< 最后连接时间戳（毫秒）
    
    // 监控任务相关
    TaskHandle_t monitor_task;      ///< 监控任务句柄
    bool monitor_running;           ///< 监控任务运行标志
    uint32_t monitor_interval_ms;   ///< 监控间隔（毫秒）
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
static void reconnect_timer_callback(TimerHandle_t timer);
static void add_reconnect_history(bool success, uint8_t attempt);
static uint32_t calculate_reconnect_delay(uint8_t attempt);
static void wifi_monitor_task(void *pvParameters);
static void update_rssi_stats(int8_t rssi);
static int8_t calculate_rssi_average(void);

/*******************************************************************
 * 公共函数实现
 *******************************************************************/

esp_err_t wifi_manager_init(const wifi_manager_config_t *config)
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
    memcpy(&g_wifi_ctx.config, config, sizeof(wifi_manager_config_t));

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
    
    // 初始化自动重连参数（默认禁用）
    g_wifi_ctx.auto_reconnect_enabled = false;
    g_wifi_ctx.reconnect_params.base_delay_ms = 1000;
    g_wifi_ctx.reconnect_params.max_delay_ms = 60000;
    g_wifi_ctx.reconnect_params.max_attempts = 0;  // 0表示无限重连
    g_wifi_ctx.reconnect_params.backoff_factor = 2.0f;
    g_wifi_ctx.reconnect_attempt = 0;
    g_wifi_ctx.reconnect_timer = NULL;
    
    // 初始化重连历史记录
    memset(g_wifi_ctx.history, 0, sizeof(g_wifi_ctx.history));
    g_wifi_ctx.history_index = 0;
    g_wifi_ctx.history_count = 0;
    
    // 初始化WiFi统计信息
    memset(&g_wifi_ctx.stats, 0, sizeof(wifi_stats_t));
    g_wifi_ctx.stats.rssi_min = 0;
    g_wifi_ctx.stats.rssi_max = -128;
    memset(g_wifi_ctx.rssi_window, 0, sizeof(g_wifi_ctx.rssi_window));
    g_wifi_ctx.rssi_window_index = 0;
    g_wifi_ctx.rssi_window_count = 0;
    g_wifi_ctx.connect_timestamp = 0;
    
    // 初始化监控任务
    g_wifi_ctx.monitor_task = NULL;
    g_wifi_ctx.monitor_running = false;
    g_wifi_ctx.monitor_interval_ms = 0;

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
    strlcpy((char *)wifi_config.sta.ssid, g_wifi_ctx.config.ssid,
            sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, g_wifi_ctx.config.password,
            sizeof(wifi_config.sta.password));
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
                
                // 更新统计信息：连接次数
                g_wifi_ctx.stats.connect_count++;
                
                // 记录连接时间戳
                g_wifi_ctx.connect_timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
                
                xSemaphoreGive(g_wifi_ctx.mutex);
            }
            
            // 重连成功，记录历史并更新重连计数
            if (g_wifi_ctx.reconnect_attempt > 0) {
                add_reconnect_history(true, g_wifi_ctx.reconnect_attempt);
                ESP_LOGI(TAG, "Reconnect successful after %d attempts",
                         g_wifi_ctx.reconnect_attempt);
                
                // 更新统计信息：重连次数
                if (xSemaphoreTake(g_wifi_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    g_wifi_ctx.stats.reconnect_count++;
                    xSemaphoreGive(g_wifi_ctx.mutex);
                }
                
                g_wifi_ctx.reconnect_attempt = 0;
            }
            
            // 停止重连定时器（如果正在运行）
            if (g_wifi_ctx.reconnect_timer != NULL) {
                xTimerStop(g_wifi_ctx.reconnect_timer, 0);
            }
            
            xEventGroupSetBits(g_wifi_ctx.event_group, WIFI_CONNECTED_BIT);
            break;
        }

        case WIFI_EVENT_STA_DISCONNECTED:
        {
            wifi_event_sta_disconnected_t *event =
                (wifi_event_sta_disconnected_t *)event_data;
            ESP_LOGW(TAG, "Disconnected from AP, reason: %d", event->reason);
            
            // 更新状态和统计信息
            if (xSemaphoreTake(g_wifi_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                g_wifi_ctx.status.state = WIFI_STATE_DISCONNECTED;
                
                // 更新断开次数
                g_wifi_ctx.stats.disconnect_count++;
                
                // 累加在线时间（如果之前有连接时间戳）
                if (g_wifi_ctx.connect_timestamp > 0) {
                    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    uint32_t online_duration_ms = current_time - g_wifi_ctx.connect_timestamp;
                    g_wifi_ctx.stats.uptime_sec += (online_duration_ms / 1000);
                    g_wifi_ctx.connect_timestamp = 0;
                }
                
                xSemaphoreGive(g_wifi_ctx.mutex);
            }

            // 检查是否启用自动重连
            if (g_wifi_ctx.auto_reconnect_enabled) {
                // 检查是否达到最大重连次数
                if (g_wifi_ctx.reconnect_params.max_attempts == 0 ||
                    g_wifi_ctx.reconnect_attempt < g_wifi_ctx.reconnect_params.max_attempts) {
                    
                    // 计算重连延迟
                    uint32_t delay_ms = calculate_reconnect_delay(g_wifi_ctx.reconnect_attempt);
                    
                    ESP_LOGI(TAG, "Auto-reconnect attempt %d, delay: %lu ms",
                             g_wifi_ctx.reconnect_attempt + 1, delay_ms);
                    
                    // 创建或重启定时器
                    if (g_wifi_ctx.reconnect_timer == NULL) {
                        g_wifi_ctx.reconnect_timer = xTimerCreate(
                            "wifi_reconn",
                            pdMS_TO_TICKS(delay_ms),
                            pdFALSE,  // 单次触发
                            NULL,
                            reconnect_timer_callback
                        );
                    }
                    
                    if (g_wifi_ctx.reconnect_timer != NULL) {
                        xTimerChangePeriod(g_wifi_ctx.reconnect_timer,
                                          pdMS_TO_TICKS(delay_ms), 0);
                        xTimerStart(g_wifi_ctx.reconnect_timer, 0);
                    } else {
                        ESP_LOGE(TAG, "Failed to create reconnect timer");
                    }
                    
                    g_wifi_ctx.reconnect_attempt++;
                } else {
                    ESP_LOGE(TAG, "Max reconnect attempts (%d) reached",
                             g_wifi_ctx.reconnect_params.max_attempts);
                    add_reconnect_history(false, g_wifi_ctx.reconnect_attempt);
                    xEventGroupSetBits(g_wifi_ctx.event_group, WIFI_FAIL_BIT);
                    
                    if (xSemaphoreTake(g_wifi_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        g_wifi_ctx.status.state = WIFI_STATE_ERROR;
                        xSemaphoreGive(g_wifi_ctx.mutex);
                    }
                }
            } else {
                // 原有的重试逻辑（向后兼容）
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

/*******************************************************************
 * 内部辅助函数实现
 *******************************************************************/

/**
 * @brief 重连定时器回调函数
 */
static void reconnect_timer_callback(TimerHandle_t timer)
{
    ESP_LOGI(TAG, "Reconnect timer triggered, attempting to connect...");
    
    esp_err_t ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initiate WiFi connection: %s", 
                 esp_err_to_name(ret));
    }
}

/**
 * @brief 添加重连历史记录
 */
static void add_reconnect_history(bool success, uint8_t attempt)
{
    if (xSemaphoreTake(g_wifi_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        reconnect_history_entry_t *entry = 
            &g_wifi_ctx.history[g_wifi_ctx.history_index];
        
        entry->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        entry->success = success;
        entry->attempt = attempt;
        
        g_wifi_ctx.history_index = 
            (g_wifi_ctx.history_index + 1) % RECONNECT_HISTORY_SIZE;
        
        if (g_wifi_ctx.history_count < RECONNECT_HISTORY_SIZE) {
            g_wifi_ctx.history_count++;
        }
        
        ESP_LOGI(TAG, "Reconnect history added: %s after %d attempts",
                 success ? "SUCCESS" : "FAILED", attempt);
        
        xSemaphoreGive(g_wifi_ctx.mutex);
    }
}

/**
 * @brief 计算重连延迟（指数退避算法）
 */
static uint32_t calculate_reconnect_delay(uint8_t attempt)
{
    float delay = g_wifi_ctx.reconnect_params.base_delay_ms * 
                  powf(g_wifi_ctx.reconnect_params.backoff_factor, 
                       (float)attempt);
    
    // 限制在最大延迟范围内
    if (delay > g_wifi_ctx.reconnect_params.max_delay_ms) {
        delay = g_wifi_ctx.reconnect_params.max_delay_ms;
    }
    
    return (uint32_t)delay;
}

/*******************************************************************
 * 公共API实现
 *******************************************************************/

esp_err_t wifi_manager_set_auto_reconnect(bool enable)
{
    if (!g_wifi_ctx.initialized) {
        ESP_LOGE(TAG, "WiFi manager not initialized");
        return ESP_ERR_WIFI_NOT_INIT;
    }
    
    if (xSemaphoreTake(g_wifi_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_wifi_ctx.auto_reconnect_enabled = enable;
        
        if (enable) {
            ESP_LOGI(TAG, "Auto-reconnect enabled");
            ESP_LOGI(TAG, "  Base delay: %lu ms",
                     g_wifi_ctx.reconnect_params.base_delay_ms);
            ESP_LOGI(TAG, "  Max delay: %lu ms",
                     g_wifi_ctx.reconnect_params.max_delay_ms);
            ESP_LOGI(TAG, "  Max attempts: %d %s",
                     g_wifi_ctx.reconnect_params.max_attempts,
                     g_wifi_ctx.reconnect_params.max_attempts == 0 ?
                     "(unlimited)" : "");
            ESP_LOGI(TAG, "  Backoff factor: %.2f",
                     g_wifi_ctx.reconnect_params.backoff_factor);
        } else {
            ESP_LOGI(TAG, "Auto-reconnect disabled");
            
            // 停止重连定时器
            if (g_wifi_ctx.reconnect_timer != NULL) {
                xTimerStop(g_wifi_ctx.reconnect_timer, 0);
            }
            
            // 重置重连计数器
            g_wifi_ctx.reconnect_attempt = 0;
        }
        
        xSemaphoreGive(g_wifi_ctx.mutex);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
}

/*******************************************************************
 * WiFi监控相关函数实现
 *******************************************************************/

/**
 * @brief 更新RSSI统计信息
 */
static void update_rssi_stats(int8_t rssi)
{
    if (xSemaphoreTake(g_wifi_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // 添加到滑动窗口
        g_wifi_ctx.rssi_window[g_wifi_ctx.rssi_window_index] = rssi;
        g_wifi_ctx.rssi_window_index =
            (g_wifi_ctx.rssi_window_index + 1) % RSSI_WINDOW_SIZE;
        
        if (g_wifi_ctx.rssi_window_count < RSSI_WINDOW_SIZE) {
            g_wifi_ctx.rssi_window_count++;
        }
        
        // 更新最小值和最大值
        if (rssi < g_wifi_ctx.stats.rssi_min || g_wifi_ctx.stats.rssi_min == 0) {
            g_wifi_ctx.stats.rssi_min = rssi;
        }
        if (rssi > g_wifi_ctx.stats.rssi_max) {
            g_wifi_ctx.stats.rssi_max = rssi;
        }
        
        // 计算平均值
        g_wifi_ctx.stats.rssi_avg = calculate_rssi_average();
        
        xSemaphoreGive(g_wifi_ctx.mutex);
    }
}

/**
 * @brief 计算RSSI滑动窗口平均值
 */
static int8_t calculate_rssi_average(void)
{
    if (g_wifi_ctx.rssi_window_count == 0) {
        return 0;
    }
    
    int32_t sum = 0;
    for (uint8_t i = 0; i < g_wifi_ctx.rssi_window_count; i++) {
        sum += g_wifi_ctx.rssi_window[i];
    }
    
    return (int8_t)(sum / g_wifi_ctx.rssi_window_count);
}

/**
 * @brief WiFi监控任务
 */
static void wifi_monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "WiFi monitor task started");
    
    while (g_wifi_ctx.monitor_running) {
        // 检查WiFi是否已连接
        if (g_wifi_ctx.status.state == WIFI_STATE_CONNECTED) {
            wifi_ap_record_t ap_info;
            esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
            
            if (ret == ESP_OK) {
                int8_t rssi = ap_info.rssi;
                
                // 更新RSSI统计
                update_rssi_stats(rssi);
                
                // RSSI低于阈值时输出警告
                if (rssi < RSSI_WARN_THRESHOLD) {
                    ESP_LOGW(TAG, "Low WiFi signal: %d dBm (threshold: %d dBm)",
                             rssi, RSSI_WARN_THRESHOLD);
                }
                
                ESP_LOGD(TAG, "RSSI: %d dBm, Avg: %d dBm, Min: %d dBm, Max: %d dBm",
                         rssi, g_wifi_ctx.stats.rssi_avg,
                         g_wifi_ctx.stats.rssi_min, g_wifi_ctx.stats.rssi_max);
            } else {
                ESP_LOGD(TAG, "Failed to get AP info: %s", esp_err_to_name(ret));
            }
            
            // 更新累计在线时间
            if (xSemaphoreTake(g_wifi_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (g_wifi_ctx.connect_timestamp > 0) {
                    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    uint32_t online_duration_ms = current_time - g_wifi_ctx.connect_timestamp;
                    g_wifi_ctx.stats.uptime_sec = online_duration_ms / 1000;
                }
                xSemaphoreGive(g_wifi_ctx.mutex);
            }
        }
        
        // 延迟指定间隔
        vTaskDelay(pdMS_TO_TICKS(g_wifi_ctx.monitor_interval_ms));
    }
    
    ESP_LOGI(TAG, "WiFi monitor task stopped");
    g_wifi_ctx.monitor_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t wifi_manager_get_stats(wifi_stats_t *stats)
{
    if (stats == NULL) {
        ESP_LOGE(TAG, "Stats pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_wifi_ctx.initialized) {
        ESP_LOGE(TAG, "WiFi manager not initialized");
        return ESP_ERR_WIFI_NOT_INIT;
    }
    
    // 使用互斥锁保护统计数据读取
    if (xSemaphoreTake(g_wifi_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(stats, &g_wifi_ctx.stats, sizeof(wifi_stats_t));
        xSemaphoreGive(g_wifi_ctx.mutex);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t wifi_manager_start_monitor(uint32_t interval_ms)
{
    if (!g_wifi_ctx.initialized) {
        ESP_LOGE(TAG, "WiFi manager not initialized");
        return ESP_ERR_WIFI_NOT_INIT;
    }
    
    if (g_wifi_ctx.monitor_running) {
        ESP_LOGE(TAG, "Monitor task already running");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 验证间隔参数
    if (interval_ms < 100 || interval_ms > 60000) {
        ESP_LOGE(TAG, "Invalid monitor interval: %lu ms (valid range: 100-60000)",
                 interval_ms);
        return ESP_ERR_INVALID_ARG;
    }
    
    g_wifi_ctx.monitor_interval_ms = interval_ms;
    g_wifi_ctx.monitor_running = true;
    
    // 创建监控任务
    BaseType_t ret = xTaskCreate(
        wifi_monitor_task,
        "wifi_monitor",
        MONITOR_TASK_STACK_SIZE,
        NULL,
        MONITOR_TASK_PRIORITY,
        &g_wifi_ctx.monitor_task
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
        g_wifi_ctx.monitor_running = false;
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "WiFi monitor started, interval: %lu ms", interval_ms);
    return ESP_OK;
}

esp_err_t wifi_manager_set_reconnect_params(const wifi_reconnect_params_t *params)
{
    if (!g_wifi_ctx.initialized) {
        ESP_LOGE(TAG, "WiFi manager not initialized");
        return ESP_ERR_WIFI_NOT_INIT;
    }
    
    // 参数验证
    if (params != NULL) {
        if (params->base_delay_ms == 0 || params->max_delay_ms == 0) {
            ESP_LOGE(TAG, "Invalid delay parameters");
            return ESP_ERR_INVALID_ARG;
        }
        
        if (params->base_delay_ms > params->max_delay_ms) {
            ESP_LOGE(TAG, "Base delay cannot exceed max delay");
            return ESP_ERR_INVALID_ARG;
        }
        
        if (params->backoff_factor < 1.0f) {
            ESP_LOGE(TAG, "Backoff factor must be >= 1.0");
            return ESP_ERR_INVALID_ARG;
        }
    }
    
    if (xSemaphoreTake(g_wifi_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (params != NULL) {
            memcpy(&g_wifi_ctx.reconnect_params, params, 
                   sizeof(wifi_reconnect_params_t));
            
            ESP_LOGI(TAG, "Reconnect parameters updated:");
            ESP_LOGI(TAG, "  Base delay: %lu ms", params->base_delay_ms);
            ESP_LOGI(TAG, "  Max delay: %lu ms", params->max_delay_ms);
            ESP_LOGI(TAG, "  Max attempts: %d %s", params->max_attempts,
                     params->max_attempts == 0 ? "(unlimited)" : "");
            ESP_LOGI(TAG, "  Backoff factor: %.2f", params->backoff_factor);
        } else {
            // 恢复默认参数
            g_wifi_ctx.reconnect_params.base_delay_ms = 1000;
            g_wifi_ctx.reconnect_params.max_delay_ms = 60000;
            g_wifi_ctx.reconnect_params.max_attempts = 0;
            g_wifi_ctx.reconnect_params.backoff_factor = 2.0f;
            
            ESP_LOGI(TAG, "Reconnect parameters reset to defaults");
        }
        
        xSemaphoreGive(g_wifi_ctx.mutex);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }
}