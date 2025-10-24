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
#include <stdarg.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// Micro-ROS消息类型
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>
#include <std_msgs/msg/string.h>

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

// 日志环形缓冲区
typedef struct {
    diagnostic_log_entry_t buffer[DIAGNOSTIC_LOG_HISTORY_SIZE]; ///< 日志缓冲区
    size_t head;                            ///< 写入位置
    size_t count;                           ///< 当前日志数量
    SemaphoreHandle_t mutex;                ///< 互斥锁
    rcl_publisher_t log_publisher;          ///< 远程日志发布者
    bool log_publisher_initialized;         ///< 日志发布者初始化标志
} log_buffer_t;

static log_buffer_t g_log_buffer = {0};

// 异常处理器
#define MAX_EXCEPTION_HANDLERS  4
typedef struct {
    diagnostic_exception_handler_t handlers[MAX_EXCEPTION_HANDLERS];
    size_t count;
    SemaphoreHandle_t mutex;
} exception_handler_list_t;

static exception_handler_list_t g_exception_handlers = {0};

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
    
    // 初始化日志缓冲区
    memset(&g_log_buffer, 0, sizeof(log_buffer_t));
    g_log_buffer.mutex = xSemaphoreCreateMutex();
    if (g_log_buffer.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create log buffer mutex");
        vSemaphoreDelete(g_diag_ctx.mutex);
        g_diag_ctx.mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    // 初始化异常处理器列表
    memset(&g_exception_handlers, 0, sizeof(exception_handler_list_t));
    g_exception_handlers.mutex = xSemaphoreCreateMutex();
    if (g_exception_handlers.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create exception handler mutex");
        vSemaphoreDelete(g_diag_ctx.mutex);
        vSemaphoreDelete(g_log_buffer.mutex);
        g_diag_ctx.mutex = NULL;
        g_log_buffer.mutex = NULL;
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
    
    // 创建远程日志发布者（可选）
    ret = ros_comm_create_publisher(
        &g_log_buffer.log_publisher,
        "/diagnostics/logs",
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        &QOS_BEST_EFFORT
    );
    
    if (ret == ESP_OK) {
        g_log_buffer.log_publisher_initialized = true;
        ESP_LOGI(TAG, "Remote log publisher created");
    } else {
        ESP_LOGW(TAG, "Failed to create remote log publisher (non-critical): %s",
                 esp_err_to_name(ret));
        g_log_buffer.log_publisher_initialized = false;
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

/**
 * @brief 日志级别转字符串
 */
static const char *log_level_to_str(esp_log_level_t level) {
    switch (level) {
        case ESP_LOG_ERROR:   return "ERROR";
        case ESP_LOG_WARN:    return "WARN";
        case ESP_LOG_INFO:    return "INFO";
        case ESP_LOG_DEBUG:   return "DEBUG";
        case ESP_LOG_VERBOSE: return "VERBOSE";
        default:              return "UNKNOWN";
    }
}

/**
 * @brief 记录事件日志
 */
void diagnostic_log(esp_log_level_t level, const char *tag, 
                    const char *format, ...) {
    if (tag == NULL || format == NULL) {
        return;
    }
    
    // 如果日志缓冲区未初始化，直接使用ESP_LOG输出
    if (g_log_buffer.mutex == NULL) {
        va_list args;
        va_start(args, format);
        esp_log_writev(level, tag, format, args);
        va_end(args);
        return;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(g_log_buffer.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    // 创建日志条目
    diagnostic_log_entry_t *entry = &g_log_buffer.buffer[g_log_buffer.head];
    
    entry->level = level;
    entry->timestamp = esp_log_timestamp();
    
    // 复制标签
    strncpy(entry->tag, tag, sizeof(entry->tag) - 1);
    entry->tag[sizeof(entry->tag) - 1] = '\0';
    
    // 格式化消息
    va_list args;
    va_start(args, format);
    vsnprintf(entry->message, sizeof(entry->message), format, args);
    va_end(args);
    
    // 更新环形缓冲区索引
    g_log_buffer.head = (g_log_buffer.head + 1) % DIAGNOSTIC_LOG_HISTORY_SIZE;
    if (g_log_buffer.count < DIAGNOSTIC_LOG_HISTORY_SIZE) {
        g_log_buffer.count++;
    }
    
    xSemaphoreGive(g_log_buffer.mutex);
    
    // 同时输出到ESP_LOG
    va_start(args, format);
    esp_log_writev(level, tag, format, args);
    va_end(args);
    
    // 如果启用了远程日志发布，发布ERROR和WARN级别的日志
    if (g_log_buffer.log_publisher_initialized && 
        (level == ESP_LOG_ERROR || level == ESP_LOG_WARN)) {
        
        // 限流：每秒最多发布10条
        static uint32_t last_publish_ms = 0;
        static uint8_t publish_count = 0;
        uint32_t now_ms = esp_log_timestamp();
        
        if (now_ms - last_publish_ms >= 1000) {
            last_publish_ms = now_ms;
            publish_count = 0;
        }
        
        if (publish_count < 10) {
            char log_msg[256];
            snprintf(log_msg, sizeof(log_msg), "[%s] %s: %s",
                     log_level_to_str(level), tag, entry->message);
            
            std_msgs__msg__String log_ros_msg;
            log_ros_msg.data.data = log_msg;
            log_ros_msg.data.size = strlen(log_msg);
            log_ros_msg.data.capacity = log_ros_msg.data.size + 1;
            
            ros_comm_publish(&g_log_buffer.log_publisher, &log_ros_msg);
            publish_count++;
        }
    }
}

/**
 * @brief 生成诊断报告
 */
size_t diagnostic_generate_report(char *report, size_t max_len) {
    if (report == NULL || max_len == 0) {
        return 0;
    }
    
    size_t offset = 0;
    
    // 标题
    offset += snprintf(report + offset, max_len - offset,
        "==========================================\n"
        "ESP32-C3 DIAGNOSTIC REPORT\n"
        "==========================================\n\n");
    
    // 系统信息
    uint32_t uptime_sec = esp_log_timestamp() / 1000;
    uint32_t free_heap = diagnostic_get_free_heap();
    uint32_t min_heap = diagnostic_get_minimum_free_heap();
    uint8_t cpu_usage = diagnostic_get_cpu_usage();
    esp_reset_reason_t reset_reason = esp_reset_reason();
    
    offset += snprintf(report + offset, max_len - offset,
        "SYSTEM INFORMATION\n"
        "------------------------------------------\n"
        "Uptime:              %lu seconds (%.1f min)\n"
        "Free Heap:           %lu KB\n"
        "Minimum Free Heap:   %lu KB\n"
        "CPU Usage:           %u%%\n"
        "Reset Reason:        ",
        uptime_sec, uptime_sec / 60.0f,
        free_heap / 1024, min_heap / 1024, cpu_usage);
    
    // 重启原因
    const char *reset_reason_str = "Unknown";
    switch (reset_reason) {
        case ESP_RST_POWERON:   reset_reason_str = "Power-on Reset"; break;
        case ESP_RST_SW:        reset_reason_str = "Software Reset"; break;
        case ESP_RST_PANIC:     reset_reason_str = "Panic Reset"; break;
        case ESP_RST_INT_WDT:   reset_reason_str = "Interrupt Watchdog"; break;
        case ESP_RST_TASK_WDT:  reset_reason_str = "Task Watchdog"; break;
        case ESP_RST_WDT:       reset_reason_str = "Watchdog Reset"; break;
        case ESP_RST_DEEPSLEEP: reset_reason_str = "Deep Sleep Wake"; break;
        case ESP_RST_BROWNOUT:  reset_reason_str = "Brownout Reset"; break;
        default: break;
    }
    offset += snprintf(report + offset, max_len - offset, "%s\n\n", reset_reason_str);
    
    // 网络状态
    offset += snprintf(report + offset, max_len - offset,
        "NETWORK STATUS\n"
        "------------------------------------------\n");
    
    wifi_status_t wifi_status;
    if (wifi_manager_get_status(&wifi_status) == ESP_OK && 
        wifi_status.state == WIFI_STATE_CONNECTED) {
        offset += snprintf(report + offset, max_len - offset,
            "WiFi:                Connected\n"
            "RSSI:                %d dBm\n"
            "IP Address:          " IPSTR "\n\n",
            wifi_status.rssi, IP2STR(&wifi_status.ip));
    } else {
        offset += snprintf(report + offset, max_len - offset,
            "WiFi:                Disconnected\n\n");
    }
    
    // ROS状态
    offset += snprintf(report + offset, max_len - offset,
        "ROS STATUS\n"
        "------------------------------------------\n"
        "ROS Agent:           %s\n\n",
        ros_comm_is_connected() ? "Connected" : "Disconnected");
    
    // 最近错误（从日志缓冲区提取ERROR级别的日志）
    offset += snprintf(report + offset, max_len - offset,
        "RECENT ERRORS (Last 5)\n"
        "------------------------------------------\n");
    
    if (g_log_buffer.mutex != NULL && 
        xSemaphoreTake(g_log_buffer.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        
        size_t error_count = 0;
        uint32_t now = esp_log_timestamp();
        
        // 从最新到最旧遍历日志
        for (int i = 0; i < (int)g_log_buffer.count && error_count < 5; i++) {
            int idx = (g_log_buffer.head - 1 - i + DIAGNOSTIC_LOG_HISTORY_SIZE) % 
                      DIAGNOSTIC_LOG_HISTORY_SIZE;
            diagnostic_log_entry_t *entry = &g_log_buffer.buffer[idx];
            
            if (entry->level == ESP_LOG_ERROR || entry->level == ESP_LOG_WARN) {
                uint32_t age_sec = (now - entry->timestamp) / 1000;
                offset += snprintf(report + offset, max_len - offset,
                    "[%-5s] %-12s %s (%lus ago)\n",
                    log_level_to_str(entry->level),
                    entry->tag,
                    entry->message,
                    age_sec);
                error_count++;
            }
        }
        
        if (error_count == 0) {
            offset += snprintf(report + offset, max_len - offset,
                "No recent errors\n");
        }
        
        xSemaphoreGive(g_log_buffer.mutex);
    }
    
    offset += snprintf(report + offset, max_len - offset, "\n");
    
    // 日志历史（最近10条）
    offset += snprintf(report + offset, max_len - offset,
        "LOG HISTORY (Last 10 entries)\n"
        "------------------------------------------\n");
    
    if (g_log_buffer.mutex != NULL && 
        xSemaphoreTake(g_log_buffer.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        
        size_t log_count = g_log_buffer.count < 10 ? g_log_buffer.count : 10;
        uint32_t now = esp_log_timestamp();
        
        for (size_t i = 0; i < log_count; i++) {
            int idx = (g_log_buffer.head - log_count + i + DIAGNOSTIC_LOG_HISTORY_SIZE) % 
                      DIAGNOSTIC_LOG_HISTORY_SIZE;
            diagnostic_log_entry_t *entry = &g_log_buffer.buffer[idx];
            
            uint32_t age_sec = (now - entry->timestamp) / 1000;
            offset += snprintf(report + offset, max_len - offset,
                "[%-5s] %-12s %s (%lus ago)\n",
                log_level_to_str(entry->level),
                entry->tag,
                entry->message,
                age_sec);
        }
        
        xSemaphoreGive(g_log_buffer.mutex);
    }
    
    // 结尾
    offset += snprintf(report + offset, max_len - offset,
        "\n"
        "==========================================\n"
        "End of Report\n"
        "==========================================\n");
    
    return offset;
}

/**
 * @brief 注册异常处理器
 */
esp_err_t diagnostic_register_exception_handler(
    diagnostic_exception_handler_t handler) {
    
    if (handler == NULL) {
        ESP_LOGE(TAG, "Invalid argument: handler is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_exception_handlers.mutex == NULL) {
        ESP_LOGE(TAG, "Diagnostic service not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(g_exception_handlers.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_FAIL;
    }
    
    if (g_exception_handlers.count >= MAX_EXCEPTION_HANDLERS) {
        xSemaphoreGive(g_exception_handlers.mutex);
        ESP_LOGE(TAG, "Exception handler list full (max %d)", MAX_EXCEPTION_HANDLERS);
        return ESP_ERR_NO_MEM;
    }
    
    g_exception_handlers.handlers[g_exception_handlers.count] = handler;
    g_exception_handlers.count++;
    
    xSemaphoreGive(g_exception_handlers.mutex);
    
    ESP_LOGI(TAG, "Exception handler registered (total: %zu)", g_exception_handlers.count);
    
    return ESP_OK;
}

/**
 * @brief 获取日志历史
 */
size_t diagnostic_get_log_history(diagnostic_log_entry_t *logs, 
                                   size_t max_count) {
    if (logs == NULL || max_count == 0) {
        return 0;
    }
    
    if (g_log_buffer.mutex == NULL) {
        return 0;
    }
    
    if (xSemaphoreTake(g_log_buffer.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire log buffer mutex");
        return 0;
    }
    
    size_t count = g_log_buffer.count < max_count ? g_log_buffer.count : max_count;
    
    // 复制日志（从最旧到最新）
    for (size_t i = 0; i < count; i++) {
        size_t idx = (g_log_buffer.head - g_log_buffer.count + i + 
                     DIAGNOSTIC_LOG_HISTORY_SIZE) % DIAGNOSTIC_LOG_HISTORY_SIZE;
        memcpy(&logs[i], &g_log_buffer.buffer[idx], sizeof(diagnostic_log_entry_t));
    }
    
    xSemaphoreGive(g_log_buffer.mutex);
    
    return count;
}

/**
 * @brief 清空日志历史
 */
void diagnostic_clear_log_history(void) {
    if (g_log_buffer.mutex == NULL) {
        return;
    }
    
    if (xSemaphoreTake(g_log_buffer.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire log buffer mutex");
        return;
    }
    
    memset(&g_log_buffer.buffer, 0, sizeof(g_log_buffer.buffer));
    g_log_buffer.head = 0;
    g_log_buffer.count = 0;
    
    xSemaphoreGive(g_log_buffer.mutex);
    
    ESP_LOGI(TAG, "Log history cleared");
}