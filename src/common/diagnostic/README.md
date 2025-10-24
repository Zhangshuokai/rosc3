# 诊断服务模块

## 概述

诊断服务模块使用ROS 2的`diagnostic_msgs`包定期发布系统状态信息，为系统监控和故障诊断提供数据支持。

## 功能特性

- ✅ 发布诊断消息到`/diagnostics`话题
- ✅ 支持4种诊断级别（OK/WARN/ERROR/STALE）
- ✅ 自动获取MAC地址作为hardware_id
- ✅ 支持最多16个自定义键值对
- ✅ 线程安全设计
- ✅ 符合ROS 2 `diagnostic_msgs`规范
- ✅ 环形缓冲区日志记录（最多100条）
- ✅ 生成完整诊断报告
- ✅ 远程日志发布（`/diagnostics/logs`话题）
- ✅ 异常处理器注册
- ✅ CPU和内存监控

## 文件说明

| 文件 | 说明 |
|------|------|
| [`diagnostic.h`](diagnostic.h) | 诊断服务接口定义 |
| [`diagnostic.c`](diagnostic.c) | 诊断服务实现 |
| [`diagnostic_example.c`](diagnostic_example.c) | 使用示例 |
| [`README.md`](README.md) | 本文档 |

## 依赖关系

本模块依赖以下组件：
- **TASK-COMMON-006**: ROS通信模块（发布者管理）
- **diagnostic_msgs**: ROS 2诊断消息包

## API参考

### 初始化函数

```c
esp_err_t diagnostic_init(const char *node_name);
```

初始化诊断服务，创建发布者并获取MAC地址。

**参数**：
- `node_name`: 节点名称

**返回值**：
- `ESP_OK`: 成功
- `ESP_ERR_INVALID_ARG`: 参数无效
- `ESP_ERR_INVALID_STATE`: ROS未连接
- `ESP_FAIL`: 初始化失败

**注意**：
- 必须在`ros_comm_connect()`成功后调用
- 重复调用会返回错误

### 发布函数

```c
esp_err_t diagnostic_publish(uint8_t level, const char *message);
```

发布诊断消息到`/diagnostics`话题。

**参数**：
- `level`: 诊断级别（`DIAGNOSTIC_OK`/`WARN`/`ERROR`/`STALE`）
- `message`: 状态描述

**返回值**：
- `ESP_OK`: 成功
- `ESP_ERR_INVALID_ARG`: 参数无效
- `ESP_ERR_INVALID_STATE`: 未初始化或ROS未连接
- `ESP_FAIL`: 发布失败

### 键值对管理

```c
esp_err_t diagnostic_add_kv(const char *key, const char *value);
esp_err_t diagnostic_clear_kv(void);
size_t diagnostic_get_kv_count(void);
```

管理诊断消息中的键值对。

## 使用示例

### 基本使用

```c
#include "common/diagnostic/diagnostic.h"

void app_main(void) {
    // 1. 初始化WiFi和ROS（省略）
    // ...
    
    // 2. 初始化诊断服务
    esp_err_t ret = diagnostic_init("esp32_node");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init diagnostic");
        return;
    }
    
    // 3. 添加键值对
    diagnostic_add_kv("WiFi RSSI", "-65 dBm");
    diagnostic_add_kv("Free Heap", "128000 bytes");
    diagnostic_add_kv("CPU Usage", "25%");
    
    // 4. 发布诊断消息
    diagnostic_publish(DIAGNOSTIC_OK, "System running normally");
    
    // 5. 清空键值对（为下次发布准备）
    diagnostic_clear_kv();
}
```

### 周期发布

```c
static void diagnostic_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    
    while (1) {
        // 每秒发布一次
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
        
        // 清空旧数据
        diagnostic_clear_kv();
        
        // 收集系统状态
        uint32_t free_heap = esp_get_free_heap_size();
        char heap_str[16];
        snprintf(heap_str, sizeof(heap_str), "%lu B", free_heap);
        diagnostic_add_kv("Free Heap", heap_str);
        
        // 根据状态确定级别
        uint8_t level = (free_heap < 50000) ? 
            DIAGNOSTIC_WARN : DIAGNOSTIC_OK;
        const char *msg = (free_heap < 50000) ? 
            "Low memory" : "System OK";
        
        // 发布
        diagnostic_publish(level, msg);
    }
}

// 创建任务
xTaskCreate(diagnostic_task, "diag", 4096, NULL, 5, NULL);
```

### 条件诊断

```c
void check_system_health(void) {
    diagnostic_clear_kv();
    
    // 检查WiFi
    wifi_status_t wifi;
    wifi_manager_get_status(&wifi);
    char rssi_str[16];
    snprintf(rssi_str, sizeof(rssi_str), "%d dBm", wifi.rssi);
    diagnostic_add_kv("WiFi RSSI", rssi_str);
    
    // 检查内存
    uint32_t heap = esp_get_free_heap_size();
    char heap_str[16];
    snprintf(heap_str, sizeof(heap_str), "%lu B", heap);
    diagnostic_add_kv("Free Heap", heap_str);
    
    // 确定诊断级别
    uint8_t level;
    const char *message;
    
    if (!ros_comm_is_connected()) {
        level = DIAGNOSTIC_ERROR;
        message = "ROS disconnected";
    } else if (wifi.rssi < -75) {
        level = DIAGNOSTIC_WARN;
        message = "Weak WiFi signal";
    } else if (heap < 50000) {
        level = DIAGNOSTIC_WARN;
        message = "Low memory";
    } else {
        level = DIAGNOSTIC_OK;
        message = "All systems normal";
    }
    
    diagnostic_publish(level, message);
}
```

## 诊断级别说明

| 级别 | 宏定义 | 值 | 说明 |
|------|--------|-------|------|
| OK | `DIAGNOSTIC_OK` | 0 | 系统正常运行 |
| WARN | `DIAGNOSTIC_WARN` | 1 | 警告，需要注意 |
| ERROR | `DIAGNOSTIC_ERROR` | 2 | 错误，需要处理 |
| STALE | `DIAGNOSTIC_STALE` | 3 | 数据过期 |

## 消息格式

诊断消息使用`diagnostic_msgs/msg/DiagnosticStatus`格式：

```
byte level              # 诊断级别
string name             # 节点名称
string message          # 状态描述
string hardware_id      # MAC地址
KeyValue[] values       # 键值对列表
```

### 键值对格式

```
string key              # 键名（最大31字符）
string value            # 值（最大63字符）
```

## 配置说明

### QoS配置

诊断消息使用`QOS_DIAGNOSTICS`配置：
- **可靠性**: RELIABLE（可靠传输）
- **历史**: KEEP_LAST 10（保留最后10条）
- **耐久性**: VOLATILE（易失）

### 话题名称

- **发布话题**: `/diagnostics`
- **消息类型**: `diagnostic_msgs/msg/DiagnosticStatus`

## 限制说明

- 最多支持16个键值对（`DIAGNOSTIC_MAX_KV_PAIRS`）
- 键名最大长度：31字符
- 键值最大长度：63字符
- 节点名称最大长度：31字符
- 状态描述最大长度：127字符

## 线程安全

所有公共API函数都是线程安全的，使用互斥锁保护共享资源。可以从多个任务中安全调用。

## 性能考虑

- **CPU占用**: 发布操作约占用<1% CPU（1Hz频率）
- **内存占用**: 约12KB（包括消息缓冲区和日志缓冲区）
- **建议频率**: 1Hz（每秒1次）
- **日志缓冲区**: 100条 × 176字节 = 17.6KB

## 调试建议

### 查看诊断消息

```bash
# 订阅诊断话题
ros2 topic echo /diagnostics

# 查看话题信息
ros2 topic info /diagnostics

# 查看消息类型
ros2 interface show diagnostic_msgs/msg/DiagnosticStatus
```

### 常见问题

**Q: 诊断消息没有发布？**
- 检查ROS是否已连接：`ros_comm_is_connected()`
- 检查是否已初始化：`diagnostic_init()`
- 查看ESP32日志输出

**Q: 键值对丢失？**
- 检查是否超过16个限制
- 检查字符串长度是否超限
- 确认在发布前添加

**Q: MAC地址不正确？**
- 检查ESP32硬件
- 确认`esp_efuse_mac_get_default()`返回成功

## 测试验证

### 单元测试

```c
void test_diagnostic(void) {
    // 测试初始化
    assert(diagnostic_init("test") == ESP_OK);
    
    // 测试添加键值对
    assert(diagnostic_add_kv("key1", "value1") == ESP_OK);
    assert(diagnostic_get_kv_count() == 1);
    
    // 测试发布
    assert(diagnostic_publish(DIAGNOSTIC_OK, "test") == ESP_OK);
    
    // 测试清空
    assert(diagnostic_clear_kv() == ESP_OK);
    assert(diagnostic_get_kv_count() == 0);
}
```

### 集成测试

1. 启动ROS 2 Agent
2. 运行ESP32节点
3. 订阅`/diagnostics`话题
4. 验证消息接收和内容正确性

## 日志记录示例

### 基本日志记录

```c
// 记录不同级别的日志
diagnostic_log(ESP_LOG_INFO, "CHASSIS", "Motor started");
diagnostic_log(ESP_LOG_WARN, "CHASSIS", "Temperature high: %d°C", temp);
diagnostic_log(ESP_LOG_ERROR, "CHASSIS", "Motor stalled!");
```

### 获取日志历史

```c
diagnostic_log_entry_t logs[10];
size_t count = diagnostic_get_log_history(logs, 10);

for (size_t i = 0; i < count; i++) {
    printf("[%s] %s: %s (timestamp: %lu)\n",
           log_level_str(logs[i].level),
           logs[i].tag,
           logs[i].message,
           logs[i].timestamp);
}
```

### 生成诊断报告

```c
char report[2048];
size_t len = diagnostic_generate_report(report, sizeof(report));
printf("%s\n", report);

// 或者通过ROS发布报告
std_msgs__msg__String report_msg;
report_msg.data.data = report;
report_msg.data.size = len;
ros_comm_publish(&report_publisher, &report_msg);
```

### 注册异常处理器

```c
void my_exception_handler(const char *exception_type,
                          const char *message,
                          void *context) {
    ESP_LOGE("EXCEPTION", "Type: %s, Message: %s",
             exception_type, message);
    
    // 生成诊断报告
    char report[2048];
    diagnostic_generate_report(report, sizeof(report));
    
    // 保存到Flash或发送到服务器
    save_crash_report(report);
}

// 注册处理器
diagnostic_register_exception_handler(my_exception_handler);
```

## 诊断报告格式

完整的诊断报告包含以下部分：

1. **系统信息**
   - 运行时间
   - 空闲堆内存和最小堆内存
   - CPU使用率
   - 重启原因

2. **网络状态**
   - WiFi连接状态和信号强度
   - IP地址

3. **ROS状态**
   - ROS Agent连接状态

4. **最近错误**
   - 最近5条ERROR/WARN级别日志
   - 包含时间戳和内容

5. **日志历史**
   - 最近10条所有级别日志

## 远程日志功能

系统会自动将ERROR和WARN级别的日志发布到`/diagnostics/logs`话题：

```bash
# 订阅远程日志
ros2 topic echo /diagnostics/logs

# 查看话题信息
ros2 topic info /diagnostics/logs
```

**限流机制**：
- 每秒最多发布10条日志
- 避免网络拥塞
- 只发布ERROR和WARN级别

## 参考文档

- [ROS 2 diagnostic_msgs文档](http://docs.ros.org/en/humble/p/diagnostic_msgs/)
- [任务分解文档](../../../任务分解-v2/任务分解-通用基础模块.md) - TASK-COMMON-014/015/016
- [开发规范](../../../开发架构文档/开发规范.md)
- [ROS接口定义](../../../需求分析/ROS接口定义文档.md)

## 许可证

Copyright (c) 2025 ROSC3 Project