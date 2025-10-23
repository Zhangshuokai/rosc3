/**
 * @file diagnostic.c
 * @brief 诊断服务模块实现
 * @version 1.0
 * @date 2025-10-23
 * @author 嵌入式开发组
 * 
 * 本文件实现诊断服务功能，发布系统状态信息到ROS 2
 * 
 * @copyright Copyright (c) 2025 ROSC3 Project
 */

#include "diagnostic.h"
#include "common/ros_comm/ros_comm.h"
#include "common/wifi_manager/wifi_manager.h"

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// Micro-ROS消息类型
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>

static const char *TAG = "DIAGNOSTIC";

// 全局诊断上下文
typedef struct {
    rcl_publisher_t publisher;              ///< 诊断消息发布者
    diagnostic_data_t data;                 ///< 诊断数据
    SemaphoreHandle_t mutex;                ///< 互斥锁
    bool is_initialized;                    ///< 初始化标志
    TaskHandle_t monitor_task;              ///< 监控任务句柄
    uint32_t monitor_interval_ms;           ///< 监控间隔
} diagnostic_context_t;

static diagnostic_context_t g_diag_ctx = {0};

// CPU使用率统计
static uint32_t g_last_total_runtime = 0;
static uint32_t g_last_idle_runtime = 0;

/**
 * @brief 获取MAC地址并格式化为字符串
 * 
 * @param[out] mac_str MAC地址字符串缓冲区，最小18字节
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_FAIL: 失败
 */
static esp_err_t get_mac_address_string(char *mac_str) {
    uint8_t mac[6] = {0};
    
    // 获取默认MAC地址
    esp_err_t ret = esp_efuse_mac_get_default(mac);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get MAC address: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 格式化为"XX:XX:XX:XX:XX:XX"
    snprintf(mac_str, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    return ESP_OK;
}

/**
 * @brief 初始化诊断服务
 */
esp_err_t diagnostic_init(const char *node_name) {
    esp_err_t ret = ESP_OK;
    
    // 参数检查
    if (node_name == NULL) {
        ESP_LOGE(TAG, "Invalid argument: node_name is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 防止重复初始化
    if (g_diag_ctx.is_initialized) {
        ESP_LOGE(TAG, "Diagnostic service already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Initializing diagnostic service for node: %s", node_name);
    
    // 检查ROS连接状态
    if (!ros_comm_is_connected()) {
        ESP_LOGE(TAG, "ROS not connected, cannot initialize diagnostic service");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 创建互斥锁
    g_diag_ctx.mutex = xSemaphoreCreateMutex();
    if (g_diag_ctx.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // 初始化诊断数据
    memset(&g_diag_ctx.data, 0, sizeof(diagnostic_data_t));
    
    // 设置节点名称
    strncpy(g_diag_ctx.data.name, node_name, sizeof(g_diag_ctx.data.name) - 1);
    g_diag_ctx.data.name[sizeof(g_diag_ctx.data.name) - 1] = '\0';
    
    // 获取MAC地址作为hardware_id
    ret = get_mac_address_string(g_diag_ctx.data.hardware_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get MAC address");
        vSemaphoreDelete(g_diag_ctx.mutex);
        g_diag_ctx.mutex = NULL;
        return ret;
    }
    
    ESP_LOGI(TAG, "Hardware ID (MAC): %s", g_diag_ctx.data.hardware_id);
    
    // 创建诊断消息发布者
    // 使用diagnostic_msgs/msg/DiagnosticStatus消息类型
    ret = ros_comm_create_publisher(
        &g_diag_ctx.publisher,
        "/diagnostics",
        ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus),
        &QOS_DIAGNOSTICS  // 使用RELIABLE + KEEP_LAST 10配置
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create diagnostic publisher: %s", esp_err_to_name(ret));
        vSemaphoreDelete(g_diag_ctx.mutex);
        g_diag_ctx.mutex = NULL;
        return ret;
    }
    
    g_diag_ctx.is_initialized = true;
    
    ESP_LOGI(TAG, "Diagnostic service initialized successfully");
    ESP_LOGI(TAG, "Publishing to topic: /diagnostics");
    
    return ESP_OK;
}

/**
 * @brief 发布诊断消息
 */
esp_err_t diagnostic_publish(uint8_t level, const char *message) {
    // 参数检查
    if (message == NULL) {
        ESP_LOGE(TAG, "Invalid argument: message is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 检查诊断级别有效性
    if (level > DIAGNOSTIC_STALE) {
        ESP_LOGE(TAG, "Invalid diagnostic level: %d", level);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 检查初始化状态
    if (!g_diag_ctx.is_initialized) {
        ESP_LOGE(TAG, "Diagnostic service not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 检查ROS连接状态
    if (!ros_comm_is_connected()) {
        ESP_LOGE(TAG, "ROS not connected");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(g_diag_ctx.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_FAIL;
    }
    
    // 创建诊断消息
    diagnostic_msgs__msg__DiagnosticStatus diag_msg;
    diagnostic_msgs__msg__DiagnosticStatus__init(&diag_msg);
    
    // 设置级别
    diag_msg.level = level;
    
    // 设置名称
    diag_msg.name.data = g_diag_ctx.data.name;
    diag_msg.name.size = strlen(g_diag_ctx.data.name);
    diag_msg.name.capacity = diag_msg.name.size + 1;
    
    // 设置消息
    diag_msg.message.data = (char *)message;
    diag_msg.message.size = strlen(message);
    diag_msg.message.capacity = diag_msg.message.size + 1;
    
    // 设置hardware_id
    diag_msg.hardware_id.data = g_diag_ctx.data.hardware_id;
    diag_msg.hardware_id.size = strlen(g_diag_ctx.data.hardware_id);
    diag_msg.hardware_id.capacity = diag_msg.hardware_id.size + 1;
    
    // 设置键值对
    if (g_diag_ctx.data.kv_count > 0) {
        // 分配键值对数组
        diag_msg.values.data = (diagnostic_msgs__msg__KeyValue *)malloc(
            g_diag_ctx.data.kv_count * sizeof(diagnostic_msgs__msg__KeyValue)
        );
        
        if (diag_msg.values.data == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for key-value pairs");
            xSemaphoreGive(g_diag_ctx.mutex);
            return ESP_ERR_NO_MEM;
        }
        
        diag_msg.values.size = g_diag_ctx.data.kv_count;
        diag_msg.values.capacity = g_diag_ctx.data.kv_count;
        
        // 复制键值对
        for (size_t i = 0; i < g_diag_ctx.data.kv_count; i++) {
            diagnostic_msgs__msg__KeyValue__init(&diag_msg.values.data[i]);
            
            // 设置键
            diag_msg.values.data[i].key.data = g_diag_ctx.data.kv[i].key;
            diag_msg.values.data[i].key.size = strlen(g_diag_ctx.data.kv[i].key);
            diag_msg.values.data[i].key.capacity = diag_msg.values.data[i].key.size + 1;
            
            // 设置值
            diag_msg.values.data[i].value.data = g_diag_ctx.data.kv[i].value;
            diag_msg.values.data[i].value.size = strlen(g_diag_ctx.data.kv[i].value);
            diag_msg.values.data[i].value.capacity = diag_msg.values.data[i].value.size + 1;
        }
    } else {
        diag_msg.values.data = NULL;
        diag_msg.values.size = 0;
        diag_msg.values.capacity = 0;
    }
    
    // 发布消息
    esp_err_t ret = ros_comm_publish(&g_diag_ctx.publisher, &diag_msg);
    
    // 清理内存
    if (diag_msg.values.data != NULL) {
        free(diag_msg.values.data);
    }
    
    xSemaphoreGive(g_diag_ctx.mutex);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to publish diagnostic message: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "Diagnostic published: level=%d, message='%s', kv_count=%d",
             level, message, g_diag_ctx.data.kv_count);
    
    return ESP_OK;
}

/**
 * @brief 添加诊断键值对
 */
esp_err_t diagnostic_add_kv(const char *key, const char *value) {
    // 参数检查
    if (key == NULL || value == NULL) {
        ESP_LOGE(TAG, "Invalid argument: key or value is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 检查长度
    if (strlen(key) >= 32) {
        ESP_LOGE(TAG, "Key too long (max 31 chars): %s", key);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strlen(value) >= 64) {
        ESP_LOGE(TAG, "Value too long (max 63 chars): %s", value);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 检查初始化状态
    if (!g_diag_ctx.is_initialized) {
        ESP_LOGE(TAG, "Diagnostic service not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(g_diag_ctx.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_FAIL;
    }
    
    // 检查是否已满
    if (g_diag_ctx.data.kv_count >= DIAGNOSTIC_MAX_KV_PAIRS) {
        xSemaphoreGive(g_diag_ctx.mutex);
        ESP_LOGE(TAG, "Key-value pairs list full (max %d)", DIAGNOSTIC_MAX_KV_PAIRS);
        return ESP_ERR_NO_MEM;
    }
    
    // 添加键值对
    size_t idx = g_diag_ctx.data.kv_count;
    strncpy(g_diag_ctx.data.kv[idx].key, key, sizeof(g_diag_ctx.data.kv[idx].key) - 1);
    g_diag_ctx.data.kv[idx].key[sizeof(g_diag_ctx.data.kv[idx].key) - 1] = '\0';
    
    strncpy(g_diag_ctx.data.kv[idx].value, value, sizeof(g_diag_ctx.data.kv[idx].value) - 1);
    g_diag_ctx.data.kv[idx].value[sizeof(g_diag_ctx.data.kv[idx].value) - 1] = '\0';
    
    g_diag_ctx.data.kv_count++;
    
    xSemaphoreGive(g_diag_ctx.mutex);
    
    ESP_LOGD(TAG, "Added KV pair: %s = %s", key, value);
    
    return ESP_OK;
}

/**
 * @brief 清空诊断键值对
 */
esp_err_t diagnostic_clear_kv(void) {
    // 检查初始化状态
    if (!g_diag_ctx.is_initialized) {
        ESP_LOGE(TAG, "Diagnostic service not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(g_diag_ctx.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_FAIL;
    }
    
    // 清空键值对
    memset(g_diag_ctx.data.kv, 0, sizeof(g_diag_ctx.data.kv));
    g_diag_ctx.data.kv_count = 0;
    
    xSemaphoreGive(g_diag_ctx.mutex);
    
    ESP_LOGD(TAG, "Cleared all key-value pairs");
    
    return ESP_OK;
}

/**
 * @brief 获取当前键值对数量
 */
size_t diagnostic_get_kv_count(void) {
    size_t count = 0;
    
    // 未初始化时返回0
    if (!g_diag_ctx.is_initialized) {
        return 0;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(g_diag_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        count = g_diag_ctx.data.kv_count;
        xSemaphoreGive(g_diag_ctx.mutex);
    }
    
    return count;
}

/**
 * @brief 获取CPU使用率
 */
uint8_t diagnostic_get_cpu_usage(void) {
    uint32_t total_runtime = 0;
    uint32_t idle_runtime = 0;
    
    // 获取任务数量
    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    if (task_count == 0) {
        return 0;
    }
    
    // 分配任务状态数组
    TaskStatus_t *task_status_array = (TaskStatus_t *)malloc(task_count * sizeof(TaskStatus_t));
    if (task_status_array == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for task status array");
        return 0;
    }
    
    // 获取任务状态
    task_count = uxTaskGetSystemState(task_status_array, task_count, &total_runtime);
    
    // 查找IDLE任务的运行时间
    for (UBaseType_t i = 0; i < task_count; i++) {
        if (strncmp(task_status_array[i].pcTaskName, "IDLE", 4) == 0) {
            idle_runtime = task_status_array[i].ulRunTimeCounter;
            break;
        }
    }
    
    free(task_status_array);
    
    // 首次调用时初始化
    if (g_last_total_runtime == 0) {
        g_last_total_runtime = total_runtime;
        g_last_idle_runtime = idle_runtime;
        return 0;
    }
    
    // 计算时间差
    uint32_t total_delta = total_runtime - g_last_total_runtime;
    uint32_t idle_delta = idle_runtime - g_last_idle_runtime;
    
    // 更新上次值
    g_last_total_runtime = total_runtime;
    g_last_idle_runtime = idle_runtime;
    
    // 计算CPU使用率
    if (total_delta == 0) {
        return 0;
    }
    
    uint32_t cpu_usage = 100 - (idle_delta * 100 / total_delta);
    
    // 限制在0-100范围内
    if (cpu_usage > 100) {
        cpu_usage = 100;
    }
    
    return (uint8_t)cpu_usage;
}

/**
 * @brief 获取空闲堆内存
 */
uint32_t diagnostic_get_free_heap(void) {
    return esp_get_free_heap_size();
}

/**
 * @brief 获取最小堆内存
 */
uint32_t diagnostic_get_minimum_free_heap(void) {
    return esp_get_minimum_free_heap_size();
}

/**
 * @brief 获取任务堆栈水位
 */
uint32_t diagnostic_get_task_high_water_mark(TaskHandle_t task_handle) {
    // uxTaskGetStackHighWaterMark返回的是以字为单位，需要转换为字节
    // ESP32的字大小为4字节
    UBaseType_t high_water_mark = uxTaskGetStackHighWaterMark(task_handle);
    return (uint32_t)(high_water_mark * sizeof(StackType_t));
}

/**
 * @brief 系统监控任务
 */
static void monitor_task(void *pvParameters) {
    char value_buf[64];
    
    ESP_LOGI(TAG, "System monitor task started, interval: %lu ms", 
             g_diag_ctx.monitor_interval_ms);
    
    while (1) {
        // 等待指定间隔
        vTaskDelay(pdMS_TO_TICKS(g_diag_ctx.monitor_interval_ms));
        
        // 检查诊断服务是否仍在运行
        if (!g_diag_ctx.is_initialized) {
            ESP_LOGW(TAG, "Diagnostic service deinitialized, stopping monitor");
            break;
        }
        
        // 清空键值对列表
        diagnostic_clear_kv();
        
        uint8_t diag_level = DIAGNOSTIC_OK;
        
        // 1. 采集CPU使用率
        uint8_t cpu_usage = diagnostic_get_cpu_usage();
        snprintf(value_buf, sizeof(value_buf), "%u%%", cpu_usage);
        diagnostic_add_kv("CPU Usage", value_buf);
        
        if (cpu_usage > CPU_USAGE_WARN_THRESHOLD) {
            diag_level = DIAGNOSTIC_WARN;
            ESP_LOGW(TAG, "CPU usage high: %u%%", cpu_usage);
        }
        
        // 2. 采集空闲堆内存
        uint32_t free_heap = diagnostic_get_free_heap();
        uint32_t free_heap_kb = free_heap / 1024;
        snprintf(value_buf, sizeof(value_buf), "%lu KB", free_heap_kb);
        diagnostic_add_kv("Free Heap", value_buf);
        
        if (free_heap_kb < FREE_HEAP_WARN_THRESHOLD) {
            diag_level = DIAGNOSTIC_WARN;
            ESP_LOGW(TAG, "Free heap low: %lu KB", free_heap_kb);
        }
        
        // 3. 采集最小堆内存
        uint32_t min_heap = diagnostic_get_minimum_free_heap();
        uint32_t min_heap_kb = min_heap / 1024;
        snprintf(value_buf, sizeof(value_buf), "%lu KB", min_heap_kb);
        diagnostic_add_kv("Min Heap", value_buf);
        
        // 4. 采集WiFi RSSI（如果WiFi已初始化）
        wifi_stats_t wifi_stats;
        if (wifi_manager_get_stats(&wifi_stats) == ESP_OK) {
            // 使用平均RSSI
            snprintf(value_buf, sizeof(value_buf), "%d dBm", wifi_stats.rssi_avg);
            diagnostic_add_kv("WiFi RSSI", value_buf);
            
            if (wifi_stats.rssi_avg < WIFI_RSSI_WARN_THRESHOLD) {
                diag_level = DIAGNOSTIC_WARN;
                ESP_LOGW(TAG, "WiFi signal weak: %d dBm", wifi_stats.rssi_avg);
            }
        }
        
        // 5. 发布诊断消息
        const char *message;
        if (diag_level == DIAGNOSTIC_OK) {
            message = "System status normal";
        } else {
            message = "System status warning - check details";
        }
        
        esp_err_t ret = diagnostic_publish(diag_level, message);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to publish monitor data: %s", esp_err_to_name(ret));
        }
    }
    
    // 清理任务句柄
    g_diag_ctx.monitor_task = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief 启动系统监控任务
 */
esp_err_t diagnostic_start_monitor(uint32_t interval_ms) {
    // 检查初始化状态
    if (!g_diag_ctx.is_initialized) {
        ESP_LOGE(TAG, "Diagnostic service not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 检查监控任务是否已运行
    if (g_diag_ctx.monitor_task != NULL) {
        ESP_LOGE(TAG, "Monitor task already running");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 保存监控间隔
    g_diag_ctx.monitor_interval_ms = interval_ms;
    
    // 创建监控任务
    BaseType_t ret = xTaskCreate(
        monitor_task,
        "diag_monitor",
        2048,
        NULL,
        tskIDLE_PRIORITY + 1,
        &g_diag_ctx.monitor_task
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
        g_diag_ctx.monitor_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "System monitor started with interval %lu ms", interval_ms);
    
    return ESP_OK;
}