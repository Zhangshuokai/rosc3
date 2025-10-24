/**
 * @file diagnostic.h
 * @brief 诊断服务模块接口定义
 * @version 1.0
 * @date 2025-10-23
 * @author 嵌入式开发组
 * 
 * 本文件定义诊断服务接口，使用ROS 2的diagnostic_msgs定期发布系统状态信息
 * 
 * @copyright Copyright (c) 2025 ROSC3 Project
 */

#ifndef DIAGNOSTIC_H
#define DIAGNOSTIC_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 诊断级别定义
 * 
 * 对应 diagnostic_msgs/msg/DiagnosticStatus 的级别定义
 */
#define DIAGNOSTIC_OK     0  ///< 正常状态
#define DIAGNOSTIC_WARN   1  ///< 警告状态
#define DIAGNOSTIC_ERROR  2  ///< 错误状态
#define DIAGNOSTIC_STALE  3  ///< 数据过期

/**
 * @brief 系统监控阈值配置
 */
#define CPU_USAGE_WARN_THRESHOLD       70  ///< CPU使用率警告阈值（%）
#define FREE_HEAP_WARN_THRESHOLD       50  ///< 空闲堆内存警告阈值（KB）
#define STACK_WARN_THRESHOLD           512 ///< 堆栈水位警告阈值（字节）
#define WIFI_RSSI_WARN_THRESHOLD       -75 ///< WiFi信号警告阈值（dBm）

/**
 * @brief 最大键值对数量
 */
#define DIAGNOSTIC_MAX_KV_PAIRS  16

/**
 * @brief 日志历史缓冲区大小
 */
#define DIAGNOSTIC_LOG_HISTORY_SIZE  100

/**
 * @brief 日志消息最大长度
 */
#define DIAGNOSTIC_LOG_MESSAGE_MAX  128

/**
 * @brief 日志标签最大长度
 */
#define DIAGNOSTIC_LOG_TAG_MAX  16

/**
 * @brief 键值对结构体
 */
typedef struct {
    char key[32];    ///< 键名
    char value[64];  ///< 值
} diagnostic_kv_t;

/**
 * @brief 诊断数据结构体
 */
typedef struct {
    uint8_t level;                          ///< 诊断级别
    char name[32];                          ///< 诊断名称
    char message[128];                      ///< 状态描述
    char hardware_id[18];                   ///< 硬件ID（MAC地址）
    diagnostic_kv_t kv[DIAGNOSTIC_MAX_KV_PAIRS];  ///< 键值对列表
    size_t kv_count;                        ///< 键值对数量
} diagnostic_data_t;

/**
 * @brief 日志条目结构
 */
typedef struct {
    esp_log_level_t level;              ///< 日志级别
    char tag[DIAGNOSTIC_LOG_TAG_MAX];   ///< 日志标签
    char message[DIAGNOSTIC_LOG_MESSAGE_MAX]; ///< 日志消息
    uint32_t timestamp;                 ///< 时间戳（毫秒）
} diagnostic_log_entry_t;

/**
 * @brief 异常处理函数类型
 */
typedef void (*diagnostic_exception_handler_t)(
    const char *exception_type,
    const char *message,
    void *context
);

/**
 * @brief 诊断报告结构
 */
typedef struct {
    // 系统信息
    uint32_t uptime_sec;                ///< 运行时间（秒）
    uint32_t free_heap;                 ///< 空闲堆内存（字节）
    uint32_t minimum_free_heap;         ///< 最小堆内存（字节）
    uint8_t cpu_usage;                  ///< CPU使用率（%）
    
    // WiFi状态
    bool wifi_connected;                ///< WiFi连接状态
    int8_t wifi_rssi;                   ///< WiFi信号强度（dBm）
    char wifi_ip[16];                   ///< IP地址
    
    // ROS状态
    bool ros_connected;                 ///< ROS连接状态
    
    // 最近错误
    diagnostic_log_entry_t recent_errors[5]; ///< 最近5条错误日志
    size_t error_count;                 ///< 错误数量
    
    // 重启原因
    esp_reset_reason_t reset_reason;    ///< 重启原因
} diagnostic_report_t;

/**
 * @brief 初始化诊断服务
 * 
 * 创建诊断消息发布者，自动获取MAC地址作为hardware_id，
 * 并初始化键值对列表。
 * 
 * @param[in] node_name 节点名称，不能为NULL
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效
 *   - ESP_ERR_INVALID_STATE: ROS通信未初始化
 *   - ESP_ERR_NO_MEM: 内存不足
 *   - ESP_FAIL: 初始化失败
 * 
 * @note 此函数必须在ros_comm_init()和ros_comm_connect()成功后调用
 * @warning 重复调用会返回错误
 * 
 * @see diagnostic_publish()
 * @see diagnostic_add_kv()
 */
esp_err_t diagnostic_init(const char *node_name);

/**
 * @brief 发布诊断消息
 * 
 * 发布包含当前级别、状态描述和所有键值对的诊断消息到/diagnostics话题。
 * 消息包含自动添加的MAC地址作为hardware_id。
 * 
 * @param[in] level 诊断级别（DIAGNOSTIC_OK/WARN/ERROR/STALE）
 * @param[in] message 状态描述，不能为NULL
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效
 *   - ESP_ERR_INVALID_STATE: 诊断服务未初始化或ROS未连接
 *   - ESP_FAIL: 发布失败
 * 
 * @note 此函数是线程安全的
 * @note 发布后不会自动清空键值对，需手动调用diagnostic_clear_kv()
 * 
 * @see diagnostic_add_kv()
 * @see diagnostic_clear_kv()
 */
esp_err_t diagnostic_publish(uint8_t level, const char *message);

/**
 * @brief 添加诊断键值对
 * 
 * 向当前诊断数据中添加一个键值对。键值对会在下次发布诊断消息时包含。
 * 
 * @param[in] key 键名，不能为NULL，最大长度31字符
 * @param[in] value 值，不能为NULL，最大长度63字符
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效（NULL或超长）
 *   - ESP_ERR_INVALID_STATE: 诊断服务未初始化
 *   - ESP_ERR_NO_MEM: 键值对列表已满（最多16个）
 * 
 * @note 此函数是线程安全的
 * @note 如果键值对列表已满，返回ESP_ERR_NO_MEM
 * 
 * @see diagnostic_clear_kv()
 * @see diagnostic_publish()
 */
esp_err_t diagnostic_add_kv(const char *key, const char *value);

/**
 * @brief 清空诊断键值对
 * 
 * 清空当前所有键值对，为下一次诊断消息发布做准备。
 * 
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_STATE: 诊断服务未初始化
 * 
 * @note 此函数是线程安全的
 * @note 建议在每次发布后调用，避免旧数据累积
 * 
 * @see diagnostic_add_kv()
 * @see diagnostic_publish()
 */
esp_err_t diagnostic_clear_kv(void);

/**
 * @brief 获取当前键值对数量
 *
 * 返回当前已添加的键值对数量。
 *
 * @return 键值对数量（0-16）
 *
 * @note 此函数是线程安全的
 */
size_t diagnostic_get_kv_count(void);

/**
 * @brief 获取CPU使用率
 *
 * 计算系统CPU使用率，基于所有任务的运行时间统计。
 *
 * @return CPU使用率百分比（0-100）
 *
 * @note 首次调用可能返回0，需要等待统计数据积累
 * @note 此函数开销较大，不建议高频调用
 */
uint8_t diagnostic_get_cpu_usage(void);

/**
 * @brief 获取空闲堆内存
 *
 * 获取当前系统空闲堆内存大小。
 *
 * @return 空闲堆内存（字节）
 *
 * @note 返回值为当前可用的堆内存大小
 */
uint32_t diagnostic_get_free_heap(void);

/**
 * @brief 获取最小堆内存
 *
 * 获取系统运行以来的历史最低堆内存值。
 *
 * @return 历史最小堆内存（字节）
 *
 * @note 此值可用于评估系统内存压力
 */
uint32_t diagnostic_get_minimum_free_heap(void);

/**
 * @brief 获取任务堆栈水位
 *
 * 获取指定任务的剩余堆栈空间（高水位标记）。
 *
 * @param[in] task_handle 任务句柄，NULL表示当前任务
 * @return 剩余堆栈大小（字节）
 *
 * @note 返回值越小表示堆栈使用越接近上限
 * @warning 如果返回值小于STACK_WARN_THRESHOLD，应考虑增加堆栈大小
 */
uint32_t diagnostic_get_task_high_water_mark(TaskHandle_t task_handle);

/**
 * @brief 启动系统监控任务
 *
 * 创建独立的FreeRTOS任务定期采集系统状态，包括CPU使用率、
 * 内存使用情况、WiFi信号强度等，并自动发布诊断消息。
 *
 * @param[in] interval_ms 监控间隔（毫秒），建议5000-10000
 * @return
 *   - ESP_OK: 成功启动
 *   - ESP_ERR_INVALID_STATE: 诊断服务未初始化或监控任务已运行
 *   - ESP_ERR_NO_MEM: 内存不足
 *
 * @note 监控任务优先级为tskIDLE_PRIORITY + 1
 * @note 堆栈大小为2048字节
 * @note 超过阈值时自动触发DIAGNOSTIC_WARN级别告警
 * @note 监控任务会自动调用diagnostic_publish()发布数据
 *
 * @see diagnostic_init()
 */
esp_err_t diagnostic_start_monitor(uint32_t interval_ms);

/**
 * @brief 记录事件日志
 *
 * 将日志记录到环形缓冲区中，自动包含时间戳。
 * 可作为ESP_LOGx的替代或补充使用。
 *
 * @param[in] level 日志级别（ESP_LOG_ERROR/WARN/INFO/DEBUG/VERBOSE）
 * @param[in] tag 日志标签，不能为NULL，最大长度15字符
 * @param[in] format 格式化字符串
 * @param[in] ... 可变参数
 *
 * @note 此函数是线程安全的
 * @note 日志存储在环形缓冲区中（最多100条）
 * @note 超过缓冲区大小时，旧日志会被覆盖
 *
 * @see diagnostic_get_log_history()
 */
void diagnostic_log(esp_log_level_t level, const char *tag,
                    const char *format, ...);

/**
 * @brief 生成诊断报告
 *
 * 生成包含系统状态、WiFi状态、ROS状态和最近错误的完整诊断报告。
 * 报告格式为易读的文本格式。
 *
 * @param[out] report 报告缓冲区，不能为NULL
 * @param[in] max_len 缓冲区大小（字节）
 * @return 实际写入字节数（不包括结尾的\0）
 *
 * @note 建议缓冲区大小至少2048字节
 * @note 如果缓冲区太小，报告会被截断
 *
 * @see diagnostic_report_t
 */
size_t diagnostic_generate_report(char *report, size_t max_len);

/**
 * @brief 注册异常处理器
 *
 * 注册自定义的异常处理函数，当系统发生异常时会被调用。
 *
 * @param[in] handler 异常处理函数，不能为NULL
 * @return
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效
 *   - ESP_ERR_INVALID_STATE: 诊断服务未初始化
 *   - ESP_ERR_NO_MEM: 处理器列表已满
 *
 * @note 最多支持4个异常处理器
 * @note 异常处理器会在ISR上下文中被调用，需要保证线程安全
 */
esp_err_t diagnostic_register_exception_handler(
    diagnostic_exception_handler_t handler
);

/**
 * @brief 获取日志历史
 *
 * 获取环形缓冲区中存储的历史日志。
 * 日志按时间顺序排列（最早的在前）。
 *
 * @param[out] logs 日志数组，不能为NULL
 * @param[in] max_count 数组最大容量
 * @return 实际返回的日志数量
 *
 * @note 此函数是线程安全的
 * @note 返回的日志是副本，不会随缓冲区变化
 *
 * @see diagnostic_clear_log_history()
 */
size_t diagnostic_get_log_history(diagnostic_log_entry_t *logs,
                                   size_t max_count);

/**
 * @brief 清空日志历史
 *
 * 清空环形缓冲区中的所有日志。
 *
 * @note 此函数是线程安全的
 * @note 清空后无法恢复
 *
 * @see diagnostic_get_log_history()
 */
void diagnostic_clear_log_history(void);

#ifdef __cplusplus
}
#endif

#endif // DIAGNOSTIC_H