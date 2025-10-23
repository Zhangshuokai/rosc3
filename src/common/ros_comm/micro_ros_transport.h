/**
 * @file micro_ros_transport.h
 * @brief Micro-ROS UDP传输层接口声明
 * @version 1.0
 * @date 2025-10-23
 * @author 嵌入式开发组
 * 
 * 本文件声明Micro-ROS自定义UDP传输层接口函数。
 * 完整实现将在TASK-COMMON-005中完成。
 * 
 * @copyright Copyright (c) 2025 ROSC3 Project
 */

#ifndef MICRO_ROS_TRANSPORT_H
#define MICRO_ROS_TRANSPORT_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 打开传输层连接
 * 
 * 创建UDP套接字并配置连接参数。
 * 
 * @param[in] transport 传输层结构体指针
 * @return 
 *   - true: 成功打开
 *   - false: 打开失败
 * 
 * @note 此函数在ros_comm_init()中通过rmw_uros_set_custom_transport()注册
 */
bool custom_transport_open(struct uxrCustomTransport *transport);

/**
 * @brief 关闭传输层连接
 * 
 * 关闭UDP套接字并释放资源。
 * 
 * @param[in] transport 传输层结构体指针
 * @return 
 *   - true: 成功关闭
 *   - false: 关闭失败
 * 
 * @note 此函数在ros_comm_disconnect()中被调用
 */
bool custom_transport_close(struct uxrCustomTransport *transport);

/**
 * @brief 通过传输层发送数据
 * 
 * 使用UDP套接字发送数据到ROS Agent。
 * 
 * @param[in] transport 传输层结构体指针
 * @param[in] buf 要发送的数据缓冲区
 * @param[in] len 数据长度（字节）
 * @param[out] err 错误码指针（可选）
 * @return 实际发送的字节数，失败返回0
 * 
 * @note 非阻塞发送
 */
size_t custom_transport_write(
    struct uxrCustomTransport *transport,
    const uint8_t *buf,
    size_t len,
    uint8_t *err
);

/**
 * @brief 从传输层接收数据
 * 
 * 使用UDP套接字接收来自ROS Agent的数据。
 * 
 * @param[in] transport 传输层结构体指针
 * @param[out] buf 接收数据缓冲区
 * @param[in] len 缓冲区大小（字节）
 * @param[in] timeout 超时时间（毫秒）
 * @param[out] err 错误码指针（可选）
 * @return 实际接收的字节数，超时或失败返回0
 * 
 * @note 阻塞接收，直到收到数据或超时
 */
size_t custom_transport_read(
    struct uxrCustomTransport *transport,
    uint8_t *buf,
    size_t len,
    int timeout,
    uint8_t *err
);

#ifdef __cplusplus
}
#endif

#endif // MICRO_ROS_TRANSPORT_H