# OLED状态UI模块使用指南

## 概述

OLED状态UI模块提供了一个简洁的界面来显示系统运行状态信息，包括WiFi连接状态、ROS连接状态、IP地址和系统运行时间。

## UI布局

```
┌────────────────────────┐
│ NODE-01 Chassis        │  ← 节点标题 (从配置读取)
├────────────────────────┤
│ WiFi: + -65dBm         │  ← WiFi状态 (+ = 已连接)
│ ROS:  + Connected      │  ← ROS状态 (+ = 已连接)
│ IP: 192.168.1.101      │  ← IP地址
│ Up: 01:23:45           │  ← 系统运行时间
└────────────────────────┘

屏幕分辨率: 128x64
字体: Montserrat 12pt
刷新频率: 1Hz (推荐)
```

## 快速开始

### 1. 基本使用

```c
#include "oled_ui.h"
#include "oled_display.h"

void app_main(void)
{
    // 1. 初始化OLED显示
    esp_err_t ret = oled_display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OLED初始化失败");
        return;
    }

    // 2. 创建状态显示UI
    ret = oled_ui_create_status_screen();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UI创建失败");
        return;
    }

    // 3. UI创建成功，可以开始更新状态
}
```

### 2. 更新WiFi状态

```c
#include "wifi_manager.h"

// 获取WiFi状态并更新显示
wifi_status_t wifi_status;
if (wifi_manager_get_status(&wifi_status) == ESP_OK) {
    bool connected = (wifi_status.state == WIFI_STATE_CONNECTED);
    oled_ui_update_wifi_status(connected, wifi_status.rssi);
}
```

### 3. 更新IP地址

```c
// 方式1: 从WiFi状态获取
wifi_status_t wifi_status;
if (wifi_manager_get_status(&wifi_status) == ESP_OK) {
    char ip_str[16];
    snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&wifi_status.ip));
    oled_ui_update_ip_address(ip_str);
}

// 方式2: 直接传入IP字符串
oled_ui_update_ip_address("192.168.1.100");

// 方式3: 清空IP显示
oled_ui_update_ip_address(NULL);
```

### 4. 更新ROS状态

```c
#include "ros_comm.h"

// 更新ROS连接状态
bool ros_connected = ros_comm_is_connected();
oled_ui_update_ros_status(ros_connected);
```

### 5. 更新运行时间

```c
#include "esp_timer.h"

// 获取系统运行时间（秒）
uint32_t uptime_sec = esp_timer_get_time() / 1000000;
oled_ui_update_uptime(uptime_sec);
```

## 完整示例

### 方式1: 使用独立任务自动更新

```c
#include "oled_ui.h"
#include "oled_display.h"
#include "wifi_manager.h"
#include "ros_comm.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void oled_ui_task(void *arg)
{
    // 创建UI
    oled_ui_create_status_screen();

    while (1) {
        // 更新WiFi状态
        wifi_status_t wifi_status;
        if (wifi_manager_get_status(&wifi_status) == ESP_OK) {
            bool wifi_connected = (wifi_status.state == WIFI_STATE_CONNECTED);
            oled_ui_update_wifi_status(wifi_connected, wifi_status.rssi);

            if (wifi_connected) {
                char ip_str[16];
                snprintf(ip_str, sizeof(ip_str), IPSTR, 
                         IP2STR(&wifi_status.ip));
                oled_ui_update_ip_address(ip_str);
            }
        }

        // 更新ROS状态
        bool ros_connected = ros_comm_is_connected();
        oled_ui_update_ros_status(ros_connected);

        // 更新运行时间
        uint32_t uptime_sec = esp_timer_get_time() / 1000000;
        oled_ui_update_uptime(uptime_sec);

        // 每秒更新一次
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    // 初始化OLED
    oled_display_init();

    // 创建UI更新任务
    xTaskCreate(oled_ui_task, "oled_ui", 4096, NULL, 5, NULL);
}
```

### 方式2: 在主循环中手动更新

```c
void app_main(void)
{
    // 初始化OLED和UI
    oled_display_init();
    oled_ui_create_status_screen();

    while (1) {
        // 定期更新状态
        update_wifi_status();
        update_ros_status();
        update_uptime();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

## API参考

### 创建和销毁

- `oled_ui_create_status_screen()` - 创建状态显示UI
- `oled_ui_destroy_status_screen()` - 销毁UI并释放资源

### 状态更新

- `oled_ui_update_wifi_status(bool connected, int8_t rssi)` - 更新WiFi状态
- `oled_ui_update_ros_status(bool connected)` - 更新ROS状态  
- `oled_ui_update_ip_address(const char *ip_str)` - 更新IP地址
- `oled_ui_update_uptime(uint32_t uptime_sec)` - 更新运行时间

## 注意事项

### 线程安全

所有更新函数都是线程安全的，可以从不同任务中调用。但创建和销毁函数不是线程安全的，应在主任务中调用。

### 性能优化

1. **刷新频率**: 建议使用1Hz刷新频率，过高的刷新频率会增加CPU占用
2. **LVGL缓冲**: 模块内部使用LVGL的单缓冲模式，内存占用约1KB
3. **任务优先级**: UI更新任务建议使用较低优先级（5或更低）

### 节点名称配置

节点标题默认显示"ROSC3 Node"，可以通过配置管理器设置自定义名称：

```c
#include "config_manager.h"

node_config_t config;
config_load(&config);
strcpy(config.ros_node_name, "NODE-01 Chassis");
config_save();
```

### 错误处理

所有函数都返回`esp_err_t`类型的错误码：

- `ESP_OK` - 成功
- `ESP_ERR_INVALID_STATE` - UI未创建或OLED未初始化
- `ESP_ERR_NO_MEM` - 内存不足
- `ESP_FAIL` - 其他错误

## 示例代码

参考 [`oled_ui_example.c`](oled_ui_example.c) 获取完整的使用示例。

## 依赖模块

- **OLED显示模块** ([`oled_display.h`](oled_display.h)) - 必需
- **WiFi管理模块** ([`wifi_manager.h`](../wifi_manager/wifi_manager.h)) - 可选
- **ROS通信模块** ([`ros_comm.h`](../ros_comm/ros_comm.h)) - 可选
- **配置管理模块** ([`config_manager.h`](../config/config_manager.h)) - 可选

## 编译配置

确保在 `src/CMakeLists.txt` 中包含了以下源文件：

```cmake
idf_component_register(
    SRCS
        "common/oled/oled_display.c"
        "common/oled/oled_ui.c"
        # ... 其他源文件
    INCLUDE_DIRS
        "common/oled"
        # ... 其他包含目录
)
```

## 故障排除

### UI不显示

1. 检查OLED是否正确初始化：`oled_display_init()` 返回 `ESP_OK`
2. 检查I2C连接是否正常
3. 检查LVGL配置：`lv_conf.h` 中的字体是否启用

### 显示内容不更新

1. 检查LVGL定时器任务是否运行
2. 检查更新函数的返回值
3. 确保在创建UI后才调用更新函数

### 内存不足

1. 增加 `lv_conf.h` 中的 `LV_MEM_SIZE`
2. 减少其他模块的内存使用
3. 检查是否有内存泄漏

## 版本历史

- v1.0 (2025-10-23) - 初始版本
  - 支持WiFi状态显示
  - 支持ROS状态显示
  - 支持IP地址显示
  - 支持运行时间显示

## 许可证

Copyright (c) 2025 ROSC3 Project