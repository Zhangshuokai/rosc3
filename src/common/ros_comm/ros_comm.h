/**
 * @file ros_comm.h
 * @brief ROS通信模块接口定义
 * @version 1.0
 * @date 2025-10-23
 * @author 嵌入式开发组
 * 
 * 本文件定义Micro-ROS通信管理接口，包括节点初始化、连接管理、
 * 发布者/订阅者创建等功能
 * 
 * @copyright Copyright (c) 2025 ROSC3 Project
 */

#ifndef ROS_COMM_H
#define ROS_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Micro-ROS头文件
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ROS回调函数类型
 *
 * 用于订阅者接收消息时的回调处理
 *
 * @param[in] msg 接收到的消息指针
 * @param[in] user_data 用户自定义数据指针
 */
typedef void (*ros_callback_t)(const void *msg, void *user_data);

/**
 * @brief 服务回调函数类型
 *
 * 服务请求处理回调函数，用于处理ROS服务请求并生成响应
 *
 * @param[in] request 请求消息指针
 * @param[out] response 响应消息指针，回调函数需要填充此结构
 * @param[in] user_data 用户自定义数据指针
 */
typedef void (*ros_service_callback_t)(
    const void *request,
    void *response,
    void *user_data
);

/**
 * @brief ROS配置结构体
 */
typedef struct {
    char agent_ip[16];          ///< Agent IP地址
    uint16_t agent_port;        ///< Agent端口（默认8888）
    char node_name[32];         ///< 节点名称
    char node_namespace[32];    ///< 节点命名空间
    uint8_t domain_id;          ///< Domain ID（默认0）
    uint32_t ping_timeout_ms;   ///< Ping超时（毫秒）
} ros_config_t;

/**
 * @brief ROS通信上下文（内部使用）
 */
typedef struct {
    rcl_node_t node;                ///< ROS节点
    rcl_executor_t executor;        ///< 执行器
    rclc_support_t support;         ///< 支持结构
    bool is_connected;              ///< 连接状态
    SemaphoreHandle_t mutex;        ///< 互斥锁
} ros_comm_context_t;

/**
 * @brief 初始化ROS通信模块
 * 
 * 初始化Micro-ROS支持结构、创建ROS节点、配置UDP传输层。
 * 必须在WiFi连接成功后调用。
 * 
 * @param[in] config ROS配置参数，不能为NULL
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效
 *   - ESP_ERR_INVALID_STATE: WiFi未连接
 *   - ESP_ERR_NO_MEM: 内存不足
 *   - ESP_FAIL: 初始化失败
 * 
 * @note 此函数不是线程安全的，应在主任务中调用
 * @warning 调用前必须确保WiFi已连接
 * @warning 重复调用会返回错误
 * 
 * @see ros_comm_connect()
 * @see ros_comm_disconnect()
 */
esp_err_t ros_comm_init(const ros_config_t *config);

/**
 * @brief 连接ROS Agent
 * 
 * 使用Ping机制检测与ROS Agent的连接，阻塞直到连接成功或超时。
 * 
 * @param[in] timeout_ms 超时时间（毫秒），0表示使用配置中的默认超时
 * @return 
 *   - ESP_OK: 连接成功
 *   - ESP_ERR_INVALID_STATE: ROS通信未初始化
 *   - ESP_ERR_TIMEOUT: 连接超时
 *   - ESP_FAIL: 连接失败
 * 
 * @note 此函数会阻塞直到连接建立或超时
 * @note 连接成功后节点将在`ros2 node list`中可见
 * 
 * @see ros_comm_init()
 * @see ros_comm_is_connected()
 */
esp_err_t ros_comm_connect(uint32_t timeout_ms);

/**
 * @brief 断开ROS Agent连接
 * 
 * 断开与ROS Agent的连接，释放相关资源。
 * 
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_STATE: ROS通信未初始化
 * 
 * @note 断开后可以重新调用ros_comm_connect()连接
 * @note 此函数是线程安全的
 */
esp_err_t ros_comm_disconnect(void);

/**
 * @brief 获取ROS连接状态
 * 
 * 查询当前是否与ROS Agent保持连接。
 * 
 * @return 
 *   - true: 已连接
 *   - false: 未连接或未初始化
 * 
 * @note 此函数是线程安全的
 * @note 返回false不一定表示错误，可能只是尚未连接
 */
bool ros_comm_is_connected(void);

/**
 * @brief 创建发布者
 *
 * 在已初始化的ROS节点上创建发布者，用于发布特定类型的消息。
 *
 * @param[out] publisher 发布者句柄，不能为NULL
 * @param[in] topic_name 话题名称，不能为NULL
 * @param[in] type_support 消息类型支持，不能为NULL
 * @param[in] qos QoS配置，可以为NULL（使用默认）
 * @return
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效
 *   - ESP_ERR_INVALID_STATE: ROS通信未初始化或未连接
 *   - ESP_FAIL: 创建失败
 *
 * @note 此函数是线程安全的
 * @note 发布者必须在ros_comm_init()成功后创建
 *
 * @see ros_comm_publish()
 */
esp_err_t ros_comm_create_publisher(
    rcl_publisher_t *publisher,
    const char *topic_name,
    const rosidl_message_type_support_t *type_support,
    const rmw_qos_profile_t *qos
);

/**
 * @brief 发布消息
 *
 * 使用已创建的发布者发布消息到指定话题。
 *
 * @param[in] publisher 发布者句柄，不能为NULL
 * @param[in] msg 消息指针，不能为NULL
 * @return
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效
 *   - ESP_ERR_INVALID_STATE: ROS通信未连接
 *   - ESP_FAIL: 发布失败
 *
 * @note 此函数是线程安全的
 * @note 消息必须是已初始化的有效消息结构
 *
 * @see ros_comm_create_publisher()
 */
esp_err_t ros_comm_publish(
    rcl_publisher_t *publisher,
    const void *msg
);

/**
 * @brief 创建订阅者
 *
 * 在已初始化的ROS节点上创建订阅者，用于接收特定类型的消息。
 * 订阅者接收到消息时会调用提供的回调函数。
 * 此函数会自动将订阅者添加到执行器。
 *
 * @param[out] subscription 订阅者句柄，不能为NULL
 * @param[in] topic_name 话题名称，不能为NULL
 * @param[in] type_support 消息类型支持，不能为NULL
 * @param[in] msg 消息缓冲区，用于接收消息，不能为NULL
 * @param[in] msg_size 消息缓冲区大小（字节）
 * @param[in] qos QoS配置，可以为NULL（使用默认）
 * @param[in] callback 消息回调函数，不能为NULL
 * @param[in] user_data 用户自定义数据，可以为NULL
 * @return
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效
 *   - ESP_ERR_INVALID_STATE: ROS通信未初始化或未连接
 *   - ESP_ERR_NO_MEM: 内存不足
 *   - ESP_FAIL: 创建失败
 *
 * @note 此函数是线程安全的
 * @note 订阅者必须在ros_comm_init()成功后创建
 * @note 回调函数在执行器上下文中执行，应尽快返回
 * @note 消息缓冲区必须在订阅者生命周期内保持有效
 *
 * @see ros_comm_spin_once()
 */
esp_err_t ros_comm_create_subscription(
    rcl_subscription_t *subscription,
    const char *topic_name,
    const rosidl_message_type_support_t *type_support,
    void *msg,
    size_t msg_size,
    const rmw_qos_profile_t *qos,
    ros_callback_t callback,
    void *user_data
);

/**
 * @brief 处理待处理的回调（单次）
 *
 * 非阻塞式处理执行器中待处理的订阅回调。应定期调用此函数
 * 以确保订阅者能够及时接收消息。
 *
 * @param[in] timeout_ms 超时时间（毫秒），0表示非阻塞立即返回
 * @return
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_STATE: ROS通信未连接
 *   - ESP_FAIL: 处理失败
 *
 * @note 此函数是线程安全的
 * @note 建议在独立任务中定期调用（如10-100ms间隔）
 * @note 超时参数控制等待新消息的最长时间
 *
 * @see ros_comm_create_subscription()
 */
esp_err_t ros_comm_spin_once(uint32_t timeout_ms);

/**
 * @brief 创建ROS服务
 *
 * 在已初始化的ROS节点上创建服务，用于响应特定类型的服务请求。
 * 服务接收到请求时会调用提供的回调函数处理。
 * 此函数会自动将服务添加到执行器。
 *
 * @param[out] service 服务句柄，不能为NULL
 * @param[in] service_name 服务名称，不能为NULL
 * @param[in] type_support 服务类型支持，不能为NULL
 * @param[in] request_msg 请求消息缓冲区，用于接收请求，不能为NULL
 * @param[in] response_msg 响应消息缓冲区，用于发送响应，不能为NULL
 * @param[in] callback 服务回调函数，不能为NULL
 * @param[in] user_data 用户自定义数据，可以为NULL
 * @return
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效
 *   - ESP_ERR_INVALID_STATE: ROS通信未初始化或未连接
 *   - ESP_ERR_NO_MEM: 内存不足
 *   - ESP_FAIL: 创建失败
 *
 * @note 此函数是线程安全的
 * @note 服务必须在ros_comm_init()成功后创建
 * @note 回调函数在执行器上下文中执行，应尽快返回
 * @note 请求和响应缓冲区必须在服务生命周期内保持有效
 *
 * @see ros_comm_call_service()
 * @see ros_comm_spin_once()
 */
esp_err_t ros_comm_create_service(
    rcl_service_t *service,
    const char *service_name,
    const rosidl_service_type_support_t *type_support,
    void *request_msg,
    void *response_msg,
    ros_service_callback_t callback,
    void *user_data
);

/**
 * @brief 同步调用ROS服务
 *
 * 创建临时服务客户端，发送请求并等待响应（阻塞）。
 * 支持超时机制，超时后返回错误。
 * 函数返回时会自动清理临时客户端资源。
 *
 * @param[in] service_name 服务名称，不能为NULL
 * @param[in] type_support 服务类型支持，不能为NULL
 * @param[in] request 请求消息指针，不能为NULL
 * @param[out] response 响应消息指针，用于接收响应，不能为NULL
 * @param[in] timeout_ms 超时时间（毫秒），0表示默认超时5000ms
 * @return
 *   - ESP_OK: 成功，响应已填充到response参数
 *   - ESP_ERR_INVALID_ARG: 参数无效
 *   - ESP_ERR_INVALID_STATE: ROS通信未连接
 *   - ESP_ERR_TIMEOUT: 服务调用超时
 *   - ESP_FAIL: 调用失败
 *
 * @note 此函数是线程安全的
 * @note 此函数会阻塞直到收到响应或超时
 * @note 建议设置合理的超时时间避免长时间阻塞
 * @note 临时客户端会在函数返回前自动清理
 *
 * @see ros_comm_create_service()
 */
esp_err_t ros_comm_call_service(
    const char *service_name,
    const rosidl_service_type_support_t *type_support,
    const void *request,
    void *response,
    uint32_t timeout_ms
);

/**
 * @brief 预定义QoS配置 - 控制指令
 *
 * 特性：
 * - 可靠传输（RELIABLE）
 * - 保持最后一条消息（KEEP_LAST, depth=1）
 *
 * 适用场景：控制指令、命令消息
 */
extern const rmw_qos_profile_t QOS_CONTROL_CMD;

/**
 * @brief 预定义QoS配置 - 传感器数据
 *
 * 特性：
 * - 尽力而为（BEST_EFFORT）
 * - 保持最后一条消息（KEEP_LAST, depth=1）
 *
 * 适用场景：高频传感器数据、实时数据流
 */
extern const rmw_qos_profile_t QOS_SENSOR_DATA;

/**
 * @brief 预定义QoS配置 - 尽力而为
 *
 * 特性：
 * - 尽力而为（BEST_EFFORT）
 * - 保持最后10条消息（KEEP_LAST, depth=10）
 *
 * 适用场景：非关键数据、允许丢失的消息
 */
extern const rmw_qos_profile_t QOS_BEST_EFFORT;

/**
 * @brief 预定义QoS配置 - 诊断信息
 *
 * 特性：
 * - 可靠传输（RELIABLE）
 * - 保持最后10条消息（KEEP_LAST, depth=10）
 *
 * 适用场景：诊断消息、日志消息、系统状态
 */
extern const rmw_qos_profile_t QOS_DIAGNOSTICS;

#ifdef __cplusplus
}
#endif

#endif // ROS_COMM_H