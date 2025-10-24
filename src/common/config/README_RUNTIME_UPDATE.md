# 运行时配置更新功能 (TASK-COMMON-010)

## 概述

本模块提供运行时配置更新功能，支持：
- ✅ 配置变更通知机制（回调函数）
- ✅ 运行时修改配置（无需重启）
- ✅ 配置备份和恢复
- ✅ 热更新（部分配置立即生效）

## 功能特性

### 1. 配置变更回调

支持注册多个回调函数，当配置发生变更时自动通知：

```c
// 定义回调函数
void my_config_handler(
    const char *key,
    config_change_type_t type,
    const void *value,
    void *user_data
) {
    ESP_LOGI("MY_MODULE", "Config changed: %s", key);
}

// 注册回调
config_register_callback(my_config_handler, NULL);

// 注销回调
config_unregister_callback(my_config_handler);
```

### 2. 运行时配置更新

支持运行时更新配置并通知所有监听者：

```c
// 更新字符串配置（带自动保存）
config_update_str_and_notify(CFG_KEY_WIFI_SSID, "NewSSID", true);

// 更新整数配置（不自动保存）
config_update_int_and_notify(CFG_KEY_LOG_LEVEL, ESP_LOG_DEBUG, false);
```

### 3. 配置备份和恢复

支持创建配置备份点和从备份恢复：

```c
// 备份当前配置
config_backup("rosc3_backup");

// 修改配置...
config_set_str(CFG_KEY_WIFI_SSID, "TestNetwork");
config_save();

// 从备份恢复
config_restore("rosc3_backup");
```

### 4. 热更新支持

部分配置支持立即生效，无需重启：

```c
// 检查是否支持热更新
bool hot = config_is_hot_updatable(CFG_KEY_LOG_LEVEL);  // true

// 日志级别支持热更新
config_update_int_and_notify(CFG_KEY_LOG_LEVEL, ESP_LOG_DEBUG, true);
// 立即生效！
```

## API参考

### 回调管理

#### config_register_callback()
```c
esp_err_t config_register_callback(
    config_change_callback_t callback,
    void *user_data
);
```
注册配置变更回调函数。

**参数：**
- `callback`: 回调函数指针
- `user_data`: 用户自定义数据

**返回：**
- `ESP_OK`: 成功
- `ESP_ERR_INVALID_ARG`: 参数无效
- `ESP_ERR_NO_MEM`: 内存不足

#### config_unregister_callback()
```c
esp_err_t config_unregister_callback(config_change_callback_t callback);
```
注销配置变更回调函数。

### 配置更新

#### config_update_str_and_notify()
```c
esp_err_t config_update_str_and_notify(
    const char *key,
    const char *value,
    bool auto_save
);
```
更新字符串配置并通知。

**参数：**
- `key`: 配置键
- `value`: 新值
- `auto_save`: 是否自动保存到Flash

#### config_update_int_and_notify()
```c
esp_err_t config_update_int_and_notify(
    const char *key,
    int32_t value,
    bool auto_save
);
```
更新整数配置并通知。

### 备份恢复

#### config_backup()
```c
esp_err_t config_backup(const char *backup_namespace);
```
备份当前配置到指定命名空间。

#### config_restore()
```c
esp_err_t config_restore(const char *backup_namespace);
```
从备份命名空间恢复配置。

### 热更新检测

#### config_is_hot_updatable()
```c
bool config_is_hot_updatable(const char *key);
```
检查配置键是否支持热更新。

**当前支持热更新的配置：**
- `CFG_KEY_LOG_LEVEL`: 日志级别

**需要重启的配置：**
- `CFG_KEY_WIFI_SSID`: WiFi SSID
- `CFG_KEY_WIFI_PASSWORD`: WiFi密码
- `CFG_KEY_ROS_AGENT_IP`: ROS Agent IP
- `CFG_KEY_ROS_AGENT_PORT`: ROS Agent端口
- 其他网络和ROS相关配置

## 使用场景

### 场景1: 动态日志级别调整

```c
// 在调试时动态提升日志级别
config_update_int_and_notify(CFG_KEY_LOG_LEVEL, ESP_LOG_DEBUG, false);

// 调试完成后恢复
config_update_int_and_notify(CFG_KEY_LOG_LEVEL, ESP_LOG_INFO, true);
```

### 场景2: 配置更新前备份

```c
// 更新配置前先备份
config_backup("before_update");

// 尝试新配置
config_update_str_and_notify(CFG_KEY_ROS_AGENT_IP, "192.168.1.200", true);

// 如果出问题，恢复备份
if (connection_failed) {
    config_restore("before_update");
    esp_restart();
}
```

### 场景3: 多模块配置监听

```c
// WiFi模块监听WiFi配置变更
void wifi_handler(const char *key, ...) {
    if (strncmp(key, "wifi_", 5) == 0) {
        // 重新连接WiFi
    }
}

// ROS模块监听ROS配置变更
void ros_handler(const char *key, ...) {
    if (strncmp(key, "ros_", 4) == 0) {
        // 重新连接ROS Agent
    }
}

config_register_callback(wifi_handler, NULL);
config_register_callback(ros_handler, NULL);
```

## 注意事项

1. **线程安全**: 所有API都是线程安全的，使用互斥锁保护
2. **回调执行**: 回调函数在配置更新的上下文中执行，避免耗时操作
3. **Flash寿命**: 频繁保存会缩短Flash寿命，建议批量更新后再保存
4. **重启需求**: 大部分配置需要重启才能完全生效
5. **内存管理**: 回调节点动态分配，记得注销不再使用的回调

## 示例代码

完整示例请参考 [`config_runtime_example.c`](config_runtime_example.c)

## 编译选项

无需额外配置，功能已集成到 `config_manager` 模块中。

## 版本历史

- v1.0 (2025-10-23): 初始版本，实现TASK-COMMON-010
  - 配置变更回调机制
  - 运行时配置更新
  - 配置备份和恢复
  - 热更新支持