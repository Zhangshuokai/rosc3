/**
 * @file ros_comm.c
 * @brief ROS通信模块实现
 * @version 1.0
 * @date 2025-10-23
 * @author 嵌入式开发组
 * 
 * 本文件实现Micro-ROS通信管理功能，包括节点初始化、连接管理等
 * 
 * @copyright Copyright (c) 2025 ROSC3 Project
 */

#include "ros_comm.h"
#include "micro_ros_transport.h"
#include "common/wifi_manager/wifi_manager.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

static const char *TAG = "ROS_COMM";

// 全局ROS通信上下文
static ros_comm_context_t g_ros_ctx = {0};
static bool g_is_initialized = false;
static ros_config_t g_ros_config = {0};

// ============================================================================
// QoS配置定义
// ============================================================================

/**
 * @brief 预定义QoS配置 - 控制指令
 * 可靠传输 + 保持最后1条消息
 */
const rmw_qos_profile_t QOS_CONTROL_CMD = {
    .history = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    .depth = 1,
    .reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    .durability = RMW_QOS_POLICY_DURABILITY_VOLATILE,
    .deadline = RMW_QOS_DEADLINE_DEFAULT,
    .lifespan = RMW_QOS_LIFESPAN_DEFAULT,
    .liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    .liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    .avoid_ros_namespace_conventions = false
};

/**
 * @brief 预定义QoS配置 - 传感器数据
 * 尽力而为 + 保持最后1条消息
 */
const rmw_qos_profile_t QOS_SENSOR_DATA = {
    .history = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    .depth = 1,
    .reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    .durability = RMW_QOS_POLICY_DURABILITY_VOLATILE,
    .deadline = RMW_QOS_DEADLINE_DEFAULT,
    .lifespan = RMW_QOS_LIFESPAN_DEFAULT,
    .liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    .liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    .avoid_ros_namespace_conventions = false
};

/**
 * @brief 预定义QoS配置 - 尽力而为
 * 尽力而为 + 保持最后10条消息
 */
const rmw_qos_profile_t QOS_BEST_EFFORT = {
    .history = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    .depth = 10,
    .reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    .durability = RMW_QOS_POLICY_DURABILITY_VOLATILE,
    .deadline = RMW_QOS_DEADLINE_DEFAULT,
    .lifespan = RMW_QOS_LIFESPAN_DEFAULT,
    .liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    .liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    .avoid_ros_namespace_conventions = false
};

/**
 * @brief 预定义QoS配置 - 诊断信息
 * 可靠传输 + 保持最后10条消息
 */
const rmw_qos_profile_t QOS_DIAGNOSTICS = {
    .history = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    .depth = 10,
    .reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    .durability = RMW_QOS_POLICY_DURABILITY_VOLATILE,
    .deadline = RMW_QOS_DEADLINE_DEFAULT,
    .lifespan = RMW_QOS_LIFESPAN_DEFAULT,
    .liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    .liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    .avoid_ros_namespace_conventions = false
};

// ============================================================================
// ROS通信核心函数
// ============================================================================

/**
 * @brief 初始化ROS通信模块
 */
esp_err_t ros_comm_init(const ros_config_t *config) {
    esp_err_t ret = ESP_OK;
    
    // 参数检查
    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid argument: config is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 防止重复初始化
    if (g_is_initialized) {
        ESP_LOGE(TAG, "ROS communication already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Initializing ROS communication...");
    
    // 检查WiFi连接状态
    wifi_status_t wifi_status = {0};
    ret = wifi_manager_get_status(&wifi_status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get WiFi status: %s", esp_err_to_name(ret));
        return ESP_ERR_INVALID_STATE;
    }
    
    if (wifi_status.state != WIFI_STATE_CONNECTED) {
        ESP_LOGE(TAG, "WiFi not connected, cannot initialize ROS");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "WiFi connected, IP: " IPSTR, IP2STR(&wifi_status.ip));
    
    // 保存配置
    memcpy(&g_ros_config, config, sizeof(ros_config_t));
    
    // 创建互斥锁
    g_ros_ctx.mutex = xSemaphoreCreateMutex();
    if (g_ros_ctx.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    
    ESP_LOGI(TAG, "Registering custom UDP transport...");
    
    // 注册自定义UDP传输层
    // 注意：传输层参数（IP、端口）需要在transport结构中设置
    // 当前传输层为基础框架，将在TASK-COMMON-005中完整实现
    if (!rmw_uros_set_custom_transport(
            true,  // framing: 使用帧协议
            (void *)&g_ros_config,  // 传递配置作为用户数据
            custom_transport_open,
            custom_transport_close,
            custom_transport_write,
            custom_transport_read
        )) {
        ESP_LOGE(TAG, "Failed to set custom transport");
        ret = ESP_FAIL;
        goto cleanup;
    }
    
    ESP_LOGI(TAG, "Initializing ROS support structure...");
    
    // 初始化ROS支持结构
    rcl_ret_t rcl_ret = rclc_support_init(
        &g_ros_ctx.support,
        0,      // argc
        NULL,   // argv
        &g_ros_ctx.support.allocator
    );
    
    if (rcl_ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to initialize support: %d", rcl_ret);
        ret = ESP_FAIL;
        goto cleanup;
    }
    
    ESP_LOGI(TAG, "Creating ROS node: %s (namespace: %s)",
             config->node_name, config->node_namespace);
    
    // 创建ROS节点
    rcl_ret = rclc_node_init_default(
        &g_ros_ctx.node,
        config->node_name,
        config->node_namespace,
        &g_ros_ctx.support
    );
    
    if (rcl_ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to create node: %d", rcl_ret);
        ret = ESP_FAIL;
        goto cleanup_support;
    }
    
    ESP_LOGI(TAG, "Initializing executor...");
    
    // 初始化执行器
    rcl_ret = rclc_executor_init(
        &g_ros_ctx.executor,
        &g_ros_ctx.support.context,
        RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES,  // 初始句柄数
        &g_ros_ctx.support.allocator
    );
    
    if (rcl_ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to initialize executor: %d", rcl_ret);
        ret = ESP_FAIL;
        goto cleanup_node;
    }
    
    // 初始化连接状态
    g_ros_ctx.is_connected = false;
    g_is_initialized = true;
    
    ESP_LOGI(TAG, "ROS communication initialized successfully");
    ESP_LOGI(TAG, "Agent: %s:%d, Domain ID: %d",
             config->agent_ip, config->agent_port, config->domain_id);
    
    return ESP_OK;

cleanup_node:
    rcl_node_fini(&g_ros_ctx.node);
    
cleanup_support:
    rclc_support_fini(&g_ros_ctx.support);
    
cleanup:
    if (g_ros_ctx.mutex != NULL) {
        vSemaphoreDelete(g_ros_ctx.mutex);
        g_ros_ctx.mutex = NULL;
    }
    
    memset(&g_ros_ctx, 0, sizeof(ros_comm_context_t));
    
    ESP_LOGE(TAG, "ROS communication initialization failed");
    return ret;
}

/**
 * @brief 连接ROS Agent
 */
esp_err_t ros_comm_connect(uint32_t timeout_ms) {
    // 检查初始化状态
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "ROS communication not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 使用配置中的超时或传入的超时
    uint32_t ping_timeout = (timeout_ms > 0) ? timeout_ms : g_ros_config.ping_timeout_ms;
    
    ESP_LOGI(TAG, "Connecting to ROS Agent at %s:%d...",
             g_ros_config.agent_ip, g_ros_config.agent_port);
    ESP_LOGI(TAG, "Ping timeout: %lu ms", ping_timeout);
    
    // 使用Ping机制检测Agent连接
    // 注意：rmw_uros_ping_agent()会阻塞直到连接成功或超时
    const int PING_ATTEMPTS = 10;  // 尝试次数
    int attempt_timeout = ping_timeout / PING_ATTEMPTS;  // 每次尝试的超时
    
    if (attempt_timeout < 100) {
        attempt_timeout = 100;  // 最小100ms
    }
    
    bool connected = false;
    for (int i = 0; i < PING_ATTEMPTS && !connected; i++) {
        ESP_LOGI(TAG, "Ping attempt %d/%d...", i + 1, PING_ATTEMPTS);
        
        rmw_ret_t ping_ret = rmw_uros_ping_agent(attempt_timeout, 1);
        
        if (ping_ret == RMW_RET_OK) {
            connected = true;
            ESP_LOGI(TAG, "Successfully connected to ROS Agent");
            break;
        }
        
        ESP_LOGW(TAG, "Ping failed (attempt %d), retrying...", i + 1);
        vTaskDelay(pdMS_TO_TICKS(100));  // 短暂延迟后重试
    }
    
    if (!connected) {
        ESP_LOGE(TAG, "Failed to connect to ROS Agent after %d attempts", PING_ATTEMPTS);
        return ESP_ERR_TIMEOUT;
    }
    
    // 更新连接状态（线程安全）
    if (xSemaphoreTake(g_ros_ctx.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        g_ros_ctx.is_connected = true;
        xSemaphoreGive(g_ros_ctx.mutex);
    } else {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "ROS Agent connection established");
    ESP_LOGI(TAG, "Node '%s' is now visible in 'ros2 node list'",
             g_ros_config.node_name);
    
    return ESP_OK;
}

/**
 * @brief 断开ROS Agent连接
 */
esp_err_t ros_comm_disconnect(void) {
    // 检查初始化状态
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "ROS communication not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Disconnecting from ROS Agent...");
    
    // 更新连接状态（线程安全）
    if (xSemaphoreTake(g_ros_ctx.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        g_ros_ctx.is_connected = false;
        xSemaphoreGive(g_ros_ctx.mutex);
    } else {
        ESP_LOGW(TAG, "Failed to acquire mutex during disconnect");
    }
    
    // 注意：实际的传输层关闭将在TASK-COMMON-005中实现
    // 这里只是更新状态，传输层的custom_transport_close()会被调用
    
    ESP_LOGI(TAG, "Disconnected from ROS Agent");
    
    return ESP_OK;
}

/**
 * @brief 获取ROS连接状态
 */
bool ros_comm_is_connected(void) {
    bool connected = false;
    
    // 未初始化时返回false
    if (!g_is_initialized) {
        return false;
    }
    
    // 线程安全地读取连接状态
    if (xSemaphoreTake(g_ros_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        connected = g_ros_ctx.is_connected;
        xSemaphoreGive(g_ros_ctx.mutex);
    } else {
        ESP_LOGW(TAG, "Failed to acquire mutex in is_connected check");
    }
    
    return connected;
}

// ============================================================================
// 发布者和订阅者管理函数
// ============================================================================

/**
 * @brief 创建发布者
 */
esp_err_t ros_comm_create_publisher(
    rcl_publisher_t *publisher,
    const char *topic_name,
    const rosidl_message_type_support_t *type_support,
    const rmw_qos_profile_t *qos
) {
    // 参数检查
    if (publisher == NULL || topic_name == NULL || type_support == NULL) {
        ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 检查初始化状态
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "ROS communication not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 检查连接状态
    if (!ros_comm_is_connected()) {
        ESP_LOGE(TAG, "ROS not connected");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Creating publisher for topic: %s", topic_name);
    
    // 如果未提供QoS，使用默认配置
    const rmw_qos_profile_t *qos_profile = qos ? qos : &rmw_qos_profile_default;
    
    // 线程安全：获取互斥锁
    if (xSemaphoreTake(g_ros_ctx.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_FAIL;
    }
    
    // 初始化发布者
    rcl_ret_t rcl_ret = rcl_publisher_init(
        publisher,
        &g_ros_ctx.node,
        type_support,
        topic_name,
        qos_profile
    );
    
    xSemaphoreGive(g_ros_ctx.mutex);
    
    if (rcl_ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to create publisher: %d", rcl_ret);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Publisher created successfully for topic: %s", topic_name);
    return ESP_OK;
}

/**
 * @brief 发布消息
 */
esp_err_t ros_comm_publish(
    rcl_publisher_t *publisher,
    const void *msg
) {
    // 参数检查
    if (publisher == NULL || msg == NULL) {
        ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 检查连接状态
    if (!ros_comm_is_connected()) {
        ESP_LOGE(TAG, "ROS not connected");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 线程安全：获取互斥锁
    if (xSemaphoreTake(g_ros_ctx.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_FAIL;
    }
    
    // 发布消息
    rcl_ret_t rcl_ret = rcl_publish(publisher, msg, NULL);
    
    xSemaphoreGive(g_ros_ctx.mutex);
    
    if (rcl_ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to publish message: %d", rcl_ret);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

/**
 * @brief 订阅者回调包装器结构
 */
typedef struct {
    void *msg_buffer;              ///< 消息缓冲区地址（用作key）
    ros_callback_t user_callback;  ///< 用户回调函数
    void *user_data;               ///< 用户数据
} subscription_wrapper_t;

// 订阅者回调包装器数组（最多支持16个订阅者）
#define MAX_SUBSCRIPTIONS 16
static subscription_wrapper_t g_subscription_wrappers[MAX_SUBSCRIPTIONS] = {0};
static uint8_t g_subscription_count = 0;

/**
 * @brief 服务回调包装器结构
 */
typedef struct {
    void *request_buffer;                  ///< 请求消息缓冲区地址（用作key）
    void *response_buffer;                 ///< 响应消息缓冲区地址
    ros_service_callback_t user_callback;  ///< 用户回调函数
    void *user_data;                       ///< 用户数据
} service_wrapper_t;

// 服务回调包装器数组（最多支持8个服务）
#define MAX_SERVICES 8
static service_wrapper_t g_service_wrappers[MAX_SERVICES] = {0};
static uint8_t g_service_count = 0;

/**
 * @brief 服务客户端调用上下文
 */
typedef struct {
    rcl_client_t client;              ///< 服务客户端
    int64_t sequence_number;          ///< 请求序列号
    bool response_received;           ///< 响应接收标志
    EventGroupHandle_t event_group;   ///< 事件组（用于同步）
} service_client_context_t;

#define SERVICE_RESPONSE_BIT (1 << 0)  ///< 响应接收事件位

/**
 * @brief 内部回调包装函数
 * 将Micro-ROS回调转换为我们的回调格式
 */
static void subscription_callback_wrapper(const void *msg) {
    // 通过消息缓冲区地址查找对应的包装器
    for (uint8_t i = 0; i < g_subscription_count; i++) {
        if (g_subscription_wrappers[i].msg_buffer == msg &&
            g_subscription_wrappers[i].user_callback != NULL) {
            // 调用用户回调
            g_subscription_wrappers[i].user_callback(msg, g_subscription_wrappers[i].user_data);
            return;
        }
    }
    ESP_LOGW(TAG, "Subscription callback received but no matching wrapper found");
}

/**
 * @brief 创建订阅者
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
) {
    // 参数检查
    if (subscription == NULL || topic_name == NULL ||
        type_support == NULL || msg == NULL || callback == NULL) {
        ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (msg_size == 0) {
        ESP_LOGE(TAG, "Invalid argument: msg_size is zero");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 检查初始化状态
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "ROS communication not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 检查连接状态
    if (!ros_comm_is_connected()) {
        ESP_LOGE(TAG, "ROS not connected");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 检查订阅者数量限制
    if (g_subscription_count >= MAX_SUBSCRIPTIONS) {
        ESP_LOGE(TAG, "Maximum subscriptions (%d) reached", MAX_SUBSCRIPTIONS);
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Creating subscription for topic: %s", topic_name);
    
    // 如果未提供QoS，使用默认配置
    const rmw_qos_profile_t *qos_profile = qos ? qos : &rmw_qos_profile_default;
    
    // 线程安全：获取互斥锁
    if (xSemaphoreTake(g_ros_ctx.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_FAIL;
    }
    
    // 初始化订阅者
    rcl_ret_t rcl_ret = rclc_subscription_init_default(
        subscription,
        &g_ros_ctx.node,
        type_support,
        topic_name
    );
    
    if (rcl_ret != RCL_RET_OK) {
        xSemaphoreGive(g_ros_ctx.mutex);
        ESP_LOGE(TAG, "Failed to create subscription: %d", rcl_ret);
        return ESP_FAIL;
    }
    
    // 存储回调信息（使用消息缓冲区地址作为key）
    uint8_t wrapper_idx = g_subscription_count;
    g_subscription_wrappers[wrapper_idx].msg_buffer = msg;
    g_subscription_wrappers[wrapper_idx].user_callback = callback;
    g_subscription_wrappers[wrapper_idx].user_data = user_data;
    g_subscription_count++;
    
    // 将订阅者添加到执行器
    rcl_ret = rclc_executor_add_subscription(
        &g_ros_ctx.executor,
        subscription,
        msg,
        subscription_callback_wrapper,
        ON_NEW_DATA
    );
    
    if (rcl_ret != RCL_RET_OK) {
        // 回滚
        g_subscription_count--;
        g_subscription_wrappers[wrapper_idx].msg_buffer = NULL;
        g_subscription_wrappers[wrapper_idx].user_callback = NULL;
        g_subscription_wrappers[wrapper_idx].user_data = NULL;
        rcl_subscription_fini(subscription, &g_ros_ctx.node);
        
        xSemaphoreGive(g_ros_ctx.mutex);
        ESP_LOGE(TAG, "Failed to add subscription to executor: %d", rcl_ret);
        return ESP_FAIL;
    }
    
    xSemaphoreGive(g_ros_ctx.mutex);
    
    ESP_LOGI(TAG, "Subscription created and added to executor for topic: %s", topic_name);
    
    return ESP_OK;
}

// ============================================================================
// 服务管理函数
// ============================================================================

/**
 * @brief 内部服务回调包装函数
 * 将Micro-ROS服务回调转换为我们的回调格式
 */
static void service_callback_wrapper(const void *request, void *response) {
    // 通过请求缓冲区地址查找对应的包装器
    for (uint8_t i = 0; i < g_service_count; i++) {
        if (g_service_wrappers[i].request_buffer == request &&
            g_service_wrappers[i].user_callback != NULL) {
            // 调用用户回调
            g_service_wrappers[i].user_callback(
                request,
                response,
                g_service_wrappers[i].user_data
            );
            return;
        }
    }
    ESP_LOGW(TAG, "Service callback received but no matching wrapper found");
}

/**
 * @brief 创建ROS服务
 */
esp_err_t ros_comm_create_service(
    rcl_service_t *service,
    const char *service_name,
    const rosidl_service_type_support_t *type_support,
    void *request_msg,
    void *response_msg,
    ros_service_callback_t callback,
    void *user_data
) {
    // 参数检查
    if (service == NULL || service_name == NULL || type_support == NULL ||
        request_msg == NULL || response_msg == NULL || callback == NULL) {
        ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 检查初始化状态
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "ROS communication not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 检查连接状态
    if (!ros_comm_is_connected()) {
        ESP_LOGE(TAG, "ROS not connected");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 检查服务数量限制
    if (g_service_count >= MAX_SERVICES) {
        ESP_LOGE(TAG, "Maximum services (%d) reached", MAX_SERVICES);
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Creating service: %s", service_name);
    
    // 线程安全：获取互斥锁
    if (xSemaphoreTake(g_ros_ctx.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_FAIL;
    }
    
    // 初始化服务
    rcl_ret_t rcl_ret = rclc_service_init_default(
        service,
        &g_ros_ctx.node,
        type_support,
        service_name
    );
    
    if (rcl_ret != RCL_RET_OK) {
        xSemaphoreGive(g_ros_ctx.mutex);
        ESP_LOGE(TAG, "Failed to create service: %d", rcl_ret);
        return ESP_FAIL;
    }
    
    // 存储回调信息（使用请求缓冲区地址作为key）
    uint8_t wrapper_idx = g_service_count;
    g_service_wrappers[wrapper_idx].request_buffer = request_msg;
    g_service_wrappers[wrapper_idx].response_buffer = response_msg;
    g_service_wrappers[wrapper_idx].user_callback = callback;
    g_service_wrappers[wrapper_idx].user_data = user_data;
    g_service_count++;
    
    // 将服务添加到执行器
    rcl_ret = rclc_executor_add_service(
        &g_ros_ctx.executor,
        service,
        request_msg,
        response_msg,
        service_callback_wrapper
    );
    
    if (rcl_ret != RCL_RET_OK) {
        // 回滚
        g_service_count--;
        g_service_wrappers[wrapper_idx].request_buffer = NULL;
        g_service_wrappers[wrapper_idx].response_buffer = NULL;
        g_service_wrappers[wrapper_idx].user_callback = NULL;
        g_service_wrappers[wrapper_idx].user_data = NULL;
        rcl_service_fini(service, &g_ros_ctx.node);
        
        xSemaphoreGive(g_ros_ctx.mutex);
        ESP_LOGE(TAG, "Failed to add service to executor: %d", rcl_ret);
        return ESP_FAIL;
    }
    
    xSemaphoreGive(g_ros_ctx.mutex);
    
    ESP_LOGI(TAG, "Service created and added to executor: %s", service_name);
    
    return ESP_OK;
}

/**
 * @brief 同步调用ROS服务
 */
esp_err_t ros_comm_call_service(
    const char *service_name,
    const rosidl_service_type_support_t *type_support,
    const void *request,
    void *response,
    uint32_t timeout_ms
) {
    // 参数检查
    if (service_name == NULL || type_support == NULL ||
        request == NULL || response == NULL) {
        ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 检查连接状态
    if (!ros_comm_is_connected()) {
        ESP_LOGE(TAG, "ROS not connected");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 使用默认超时（5秒）如果未指定
    if (timeout_ms == 0) {
        timeout_ms = 5000;
    }
    
    ESP_LOGI(TAG, "Calling service: %s (timeout: %lu ms)", service_name, timeout_ms);
    
    esp_err_t ret = ESP_OK;
    service_client_context_t ctx = {0};
    
    // 创建事件组用于同步
    ctx.event_group = xEventGroupCreate();
    if (ctx.event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }
    
    // 线程安全：获取互斥锁
    if (xSemaphoreTake(g_ros_ctx.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        vEventGroupDelete(ctx.event_group);
        return ESP_FAIL;
    }
    
    // 初始化服务客户端
    rcl_ret_t rcl_ret = rcl_client_init(
        &ctx.client,
        &g_ros_ctx.node,
        type_support,
        service_name,
        &rmw_qos_profile_services_default
    );
    
    if (rcl_ret != RCL_RET_OK) {
        xSemaphoreGive(g_ros_ctx.mutex);
        vEventGroupDelete(ctx.event_group);
        ESP_LOGE(TAG, "Failed to initialize client: %d", rcl_ret);
        return ESP_FAIL;
    }
    
    // 发送服务请求
    rcl_ret = rcl_send_request(&ctx.client, request, &ctx.sequence_number);
    
    if (rcl_ret != RCL_RET_OK) {
        rcl_client_fini(&ctx.client, &g_ros_ctx.node);
        xSemaphoreGive(g_ros_ctx.mutex);
        vEventGroupDelete(ctx.event_group);
        ESP_LOGE(TAG, "Failed to send request: %d", rcl_ret);
        return ESP_FAIL;
    }
    
    ESP_LOGD(TAG, "Service request sent, sequence: %lld", ctx.sequence_number);
    
    xSemaphoreGive(g_ros_ctx.mutex);
    
    // 等待响应（使用轮询方式）
    TickType_t start_time = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    bool response_received = false;
    
    while (!response_received) {
        // 检查超时
        if ((xTaskGetTickCount() - start_time) >= timeout_ticks) {
            ESP_LOGE(TAG, "Service call timeout after %lu ms", timeout_ms);
            ret = ESP_ERR_TIMEOUT;
            break;
        }
        
        // 获取互斥锁
        if (xSemaphoreTake(g_ros_ctx.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;  // 稍后重试
        }
        
        // 尝试接收响应
        rmw_service_info_t request_header;
        rcl_ret = rcl_take_response(&ctx.client, &request_header, response);
        
        xSemaphoreGive(g_ros_ctx.mutex);
        
        if (rcl_ret == RCL_RET_OK) {
            // 验证序列号是否匹配
            if (request_header.request_id.sequence_number == ctx.sequence_number) {
                ESP_LOGI(TAG, "Service response received");
                response_received = true;
                ret = ESP_OK;
            } else {
                ESP_LOGW(TAG, "Received response with mismatched sequence: %lld (expected %lld)",
                         request_header.request_id.sequence_number, ctx.sequence_number);
            }
        } else if (rcl_ret != RCL_RET_SERVICE_TAKE_FAILED) {
            // RCL_RET_SERVICE_TAKE_FAILED 表示没有可用响应，这是正常的
            ESP_LOGW(TAG, "Failed to take response: %d", rcl_ret);
        }
        
        // 短暂延迟后重试
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // 清理资源
    if (xSemaphoreTake(g_ros_ctx.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        rcl_client_fini(&ctx.client, &g_ros_ctx.node);
        xSemaphoreGive(g_ros_ctx.mutex);
    } else {
        ESP_LOGW(TAG, "Failed to acquire mutex during cleanup");
    }
    
    vEventGroupDelete(ctx.event_group);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Service call completed successfully");
    }
    
    return ret;
}

/**
 * @brief 处理回调（单次）
 */
esp_err_t ros_comm_spin_once(uint32_t timeout_ms) {
    // 检查连接状态
    if (!ros_comm_is_connected()) {
        ESP_LOGE(TAG, "ROS not connected");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 线程安全：获取互斥锁
    if (xSemaphoreTake(g_ros_ctx.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_FAIL;
    }
    
    // 将毫秒转换为纳秒
    uint64_t timeout_ns = (uint64_t)timeout_ms * 1000000ULL;
    
    // 处理待处理的回调
    rcl_ret_t rcl_ret = rclc_executor_spin_some(&g_ros_ctx.executor, timeout_ns);
    
    xSemaphoreGive(g_ros_ctx.mutex);
    
    if (rcl_ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to spin executor: %d", rcl_ret);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}