/**
 * @file micro_ros_transport.c
 * @brief Micro-ROS UDP传输层完整实现
 * @version 1.0
 * @date 2025-10-23
 * @author 嵌入式开发组
 *
 * 本文件实现Micro-ROS自定义UDP传输层，包括UDP套接字创建、
 * 数据收发、超时处理和错误管理。
 *
 * @copyright Copyright (c) 2025 ROSC3 Project
 */

#include "micro_ros_transport.h"
#include "ros_comm.h"
#include "esp_log.h"

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

static const char *TAG = "MICRO_ROS_TRANSPORT";

/**
 * @brief UDP传输层上下文结构体
 */
typedef struct {
    int socket_fd;                  ///< UDP套接字文件描述符
    struct sockaddr_in agent_addr;  ///< Agent地址结构
    uint32_t tx_count;              ///< 发送字节数统计
    uint32_t rx_count;              ///< 接收字节数统计
} transport_context_t;

/**
 * @brief 打开传输层连接
 *
 * 创建UDP套接字并配置Agent地址和端口。
 *
 * @param[in] transport 传输层结构体指针
 * @return true=成功, false=失败
 */
bool custom_transport_open(struct uxrCustomTransport *transport) {
    if (transport == NULL) {
        ESP_LOGE(TAG, "Invalid argument: transport is NULL");
        return false;
    }
    
    ESP_LOGI(TAG, "Opening UDP transport...");
    
    // 分配传输层上下文
    transport_context_t *ctx = (transport_context_t *)malloc(sizeof(transport_context_t));
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Failed to allocate transport context");
        return false;
    }
    
    memset(ctx, 0, sizeof(transport_context_t));
    ctx->socket_fd = -1;
    
    // 获取ROS配置（从transport.args传递过来）
    ros_config_t *config = (ros_config_t *)transport->args;
    if (config == NULL) {
        ESP_LOGE(TAG, "ROS config not provided in transport args");
        free(ctx);
        return false;
    }
    
    // 创建UDP套接字
    ctx->socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (ctx->socket_fd < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno=%d (%s)",
                 errno, strerror(errno));
        free(ctx);
        return false;
    }
    
    ESP_LOGI(TAG, "Socket created: fd=%d", ctx->socket_fd);
    
    // 配置Agent地址
    memset(&ctx->agent_addr, 0, sizeof(struct sockaddr_in));
    ctx->agent_addr.sin_family = AF_INET;
    ctx->agent_addr.sin_port = htons(config->agent_port);
    
    // 将IP字符串转换为网络地址
    if (inet_pton(AF_INET, config->agent_ip, &ctx->agent_addr.sin_addr) <= 0) {
        ESP_LOGE(TAG, "Invalid Agent IP address: %s", config->agent_ip);
        close(ctx->socket_fd);
        free(ctx);
        return false;
    }
    
    // 保存上下文到transport
    transport->args = ctx;
    
    ESP_LOGI(TAG, "UDP transport opened successfully");
    ESP_LOGI(TAG, "Agent: %s:%d", config->agent_ip, config->agent_port);
    
    return true;
}

/**
 * @brief 关闭传输层连接
 *
 * 关闭UDP套接字并释放资源。
 *
 * @param[in] transport 传输层结构体指针
 * @return true=成功
 */
bool custom_transport_close(struct uxrCustomTransport *transport) {
    if (transport == NULL) {
        ESP_LOGW(TAG, "Transport is NULL in close");
        return true;
    }
    
    transport_context_t *ctx = (transport_context_t *)transport->args;
    if (ctx == NULL) {
        ESP_LOGW(TAG, "Transport context is NULL in close");
        return true;
    }
    
    ESP_LOGI(TAG, "Closing UDP transport...");
    ESP_LOGI(TAG, "Statistics - TX: %lu bytes, RX: %lu bytes",
             ctx->tx_count, ctx->rx_count);
    
    // 关闭套接字
    if (ctx->socket_fd >= 0) {
        close(ctx->socket_fd);
        ctx->socket_fd = -1;
        ESP_LOGD(TAG, "Socket closed");
    }
    
    // 释放上下文
    free(ctx);
    transport->args = NULL;
    
    ESP_LOGI(TAG, "UDP transport closed successfully");
    
    return true;
}

/**
 * @brief 通过传输层发送数据
 *
 * 使用UDP套接字发送数据到ROS Agent。
 *
 * @param[in] transport 传输层结构体指针
 * @param[in] buf 要发送的数据缓冲区
 * @param[in] len 数据长度
 * @param[out] err 错误码指针
 * @return 实际发送的字节数，失败返回0
 */
size_t custom_transport_write(
    struct uxrCustomTransport *transport,
    const uint8_t *buf,
    size_t len,
    uint8_t *err
) {
    if (transport == NULL || buf == NULL || len == 0) {
        ESP_LOGE(TAG, "Invalid arguments in write");
        if (err != NULL) {
            *err = 1;
        }
        return 0;
    }
    
    transport_context_t *ctx = (transport_context_t *)transport->args;
    if (ctx == NULL || ctx->socket_fd < 0) {
        ESP_LOGE(TAG, "Transport not initialized");
        if (err != NULL) {
            *err = 1;
        }
        return 0;
    }
    
    // 发送数据到Agent
    ssize_t bytes_sent = sendto(
        ctx->socket_fd,
        buf,
        len,
        0,
        (struct sockaddr *)&ctx->agent_addr,
        sizeof(ctx->agent_addr)
    );
    
    if (bytes_sent < 0) {
        ESP_LOGE(TAG, "sendto() failed: errno=%d (%s)", errno, strerror(errno));
        if (err != NULL) {
            *err = 1;
        }
        return 0;
    }
    
    // 累加发送统计
    ctx->tx_count += bytes_sent;
    
    ESP_LOGD(TAG, "Sent %d bytes (total: %lu)", bytes_sent, ctx->tx_count);
    
    if (err != NULL) {
        *err = 0;
    }
    
    return (size_t)bytes_sent;
}

/**
 * @brief 从传输层接收数据
 *
 * 使用UDP套接字接收来自ROS Agent的数据，支持超时设置。
 *
 * @param[in] transport 传输层结构体指针
 * @param[out] buf 接收数据缓冲区
 * @param[in] len 缓冲区大小
 * @param[in] timeout 超时时间（毫秒）
 * @param[out] err 错误码指针
 * @return 实际接收的字节数，超时或失败返回0
 */
size_t custom_transport_read(
    struct uxrCustomTransport *transport,
    uint8_t *buf,
    size_t len,
    int timeout,
    uint8_t *err
) {
    if (transport == NULL || buf == NULL || len == 0) {
        ESP_LOGE(TAG, "Invalid arguments in read");
        if (err != NULL) {
            *err = 1;
        }
        return 0;
    }
    
    transport_context_t *ctx = (transport_context_t *)transport->args;
    if (ctx == NULL || ctx->socket_fd < 0) {
        ESP_LOGE(TAG, "Transport not initialized");
        if (err != NULL) {
            *err = 1;
        }
        return 0;
    }
    
    // 设置接收超时
    struct timeval tv;
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;
    
    if (setsockopt(ctx->socket_fd, SOL_SOCKET, SO_RCVTIMEO,
                   &tv, sizeof(tv)) < 0) {
        ESP_LOGE(TAG, "setsockopt(SO_RCVTIMEO) failed: errno=%d (%s)",
                 errno, strerror(errno));
        if (err != NULL) {
            *err = 1;
        }
        return 0;
    }
    
    // 接收数据
    struct sockaddr_in src_addr;
    socklen_t src_addr_len = sizeof(src_addr);
    
    ssize_t bytes_received = recvfrom(
        ctx->socket_fd,
        buf,
        len,
        0,
        (struct sockaddr *)&src_addr,
        &src_addr_len
    );
    
    if (bytes_received < 0) {
        // 超时不是错误，只是没有数据
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            ESP_LOGD(TAG, "Receive timeout (%d ms)", timeout);
        } else {
            ESP_LOGE(TAG, "recvfrom() failed: errno=%d (%s)",
                     errno, strerror(errno));
        }
        
        if (err != NULL) {
            *err = 0;  // 超时不算错误
        }
        return 0;
    }
    
    // 累加接收统计
    ctx->rx_count += bytes_received;
    
    ESP_LOGD(TAG, "Received %d bytes (total: %lu)", bytes_received, ctx->rx_count);
    
    if (err != NULL) {
        *err = 0;
    }
    
    return (size_t)bytes_received;
}