# I2C Manager 模块

## 模块概述

I2C Manager 是一个统一的 I2C 总线管理模块，负责管理 ESP32-C3 的 I2C 总线，提供标准化的初始化和设备访问接口。

## 主要特性

- ✅ 支持 ESP-IDF 5.x 新驱动 API (`driver/i2c_master.h`)
- ✅ 单例模式管理 I2C 总线，避免重复初始化
- ✅ 自动设备注册与查询功能
- ✅ 线程安全的总线访问（互斥锁保护）
- ✅ 灵活的配置选项（默认配置或自定义配置）

## 硬件配置

### 默认引脚配置

| 信号 | GPIO | 说明 |
|------|------|------|
| SCL  | GPIO6 | I2C 时钟线 |
| SDA  | GPIO5 | I2C 数据线 |

### 默认参数

- **I2C 端口**: I2C_NUM_0
- **时钟频率**: 400kHz
- **内部上拉**: 启用
- **毛刺过滤**: 7 个时钟周期

## 使用方法

### 1. 基础初始化（使用默认配置）

```c
#include "common/i2c_manager/i2c_manager.h"

void app_main(void) {
    // 初始化 I2C 管理器（默认配置）
    ESP_ERROR_CHECK(i2c_manager_init_default());
    
    // ... 其他代码
}
```

### 2. 自定义配置初始化

```c
#include "common/i2c_manager/i2c_manager.h"

void app_main(void) {
    // 自定义配置
    i2c_manager_config_t config = {
        .scl_pin = GPIO_NUM_8,
        .sda_pin = GPIO_NUM_9,
        .freq_hz = 100000,  // 100kHz
        .enable_pullup = false,
        .glitch_ignore_cnt = 7
    };
    
    // 初始化
    ESP_ERROR_CHECK(i2c_manager_init(&config));
}
```

### 3. 在其他模块中使用 I2C 总线

```c
#include "common/i2c_manager/i2c_manager.h"
#include "esp_lcd_panel_io.h"

esp_err_t my_device_init(void) {
    // 1. 获取 I2C 总线句柄
    i2c_master_bus_handle_t i2c_bus = i2c_manager_get_bus_handle();
    if (i2c_bus == NULL) {
        ESP_LOGE(TAG, "I2C 管理器未初始化");
        return ESP_FAIL;
    }
    
    // 2. 注册设备
    ESP_ERROR_CHECK(i2c_manager_register_device(0x3C, "SSD1306"));
    
    // 3. 使用总线句柄创建 I2C 设备
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = 0x3C,
        .control_phase_bytes = 1,
        // ... 其他配置
    };
    
    esp_lcd_panel_io_handle_t io_handle;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));
    
    return ESP_OK;
}
```

### 4. 设备管理

```c
// 检查设备是否已注册
if (i2c_manager_is_device_registered(0x3C)) {
    ESP_LOGI(TAG, "SSD1306 已注册");
}

// 获取所有已注册设备
i2c_device_info_t devices[I2C_MAX_DEVICES];
size_t count = i2c_manager_get_devices(devices, I2C_MAX_DEVICES);

for (size_t i = 0; i < count; i++) {
    ESP_LOGI(TAG, "设备 %zu: 0x%02X (%s)", 
             i, devices[i].addr, devices[i].name);
}
```

### 5. 反初始化

```c
// 程序退出时反初始化
ESP_ERROR_CHECK(i2c_manager_deinit());
```

## API 参考

### 初始化函数

#### `i2c_manager_init_default()`

使用默认配置初始化 I2C 管理器。

**返回值**:
- `ESP_OK`: 成功
- `ESP_ERR_INVALID_STATE`: 已初始化
- `ESP_FAIL`: 初始化失败

#### `i2c_manager_init(const i2c_manager_config_t *config)`

使用自定义配置初始化 I2C 管理器。

**参数**:
- `config`: I2C 总线配置

**返回值**:
- `ESP_OK`: 成功
- `ESP_ERR_INVALID_ARG`: 参数无效
- `ESP_ERR_INVALID_STATE`: 已初始化
- `ESP_FAIL`: 初始化失败

### 访问函数

#### `i2c_manager_get_bus_handle()`

获取 I2C 总线句柄。

**返回值**:
- I2C 总线句柄，如果未初始化则返回 `NULL`

### 设备管理函数

#### `i2c_manager_register_device(uint8_t addr, const char *name)`

注册 I2C 设备。

**参数**:
- `addr`: 设备 I2C 地址 (0x00-0x7F)
- `name`: 设备名称（用于调试）

**返回值**:
- `ESP_OK`: 成功
- `ESP_ERR_INVALID_ARG`: 参数无效
- `ESP_ERR_NO_MEM`: 设备槽已满

#### `i2c_manager_is_device_registered(uint8_t addr)`

查询设备是否已注册。

**参数**:
- `addr`: 设备 I2C 地址

**返回值**:
- `true`: 已注册
- `false`: 未注册

#### `i2c_manager_get_devices(i2c_device_info_t *devices, size_t max_count)`

获取已注册的设备列表。

**参数**:
- `devices`: 输出设备信息数组
- `max_count`: 数组最大容量

**返回值**:
- 实际设备数量

### 清理函数

#### `i2c_manager_deinit()`

反初始化 I2C 管理器。

**返回值**:
- `ESP_OK`: 成功

## 设计说明

### 单例模式

I2C Manager 使用单例模式，确保系统中只有一个 I2C 总线实例，避免资源冲突。

### 线程安全

所有涉及共享状态的操作都使用 FreeRTOS 互斥锁保护，确保多任务环境下的安全性。

### 设备注册机制

设备注册功能主要用于：
- 调试和诊断（记录总线上的设备）
- 避免地址冲突检测
- 系统状态监控

## 注意事项

1. **初始化顺序**: 必须在使用 I2C 总线之前调用初始化函数
2. **单次初始化**: 不能重复调用初始化函数，除非先调用 `deinit()`
3. **引脚配置**: 确保引脚配置与实际硬件连接一致
4. **设备地址**: I2C 设备地址范围为 0x00-0x7F (7位地址)
5. **最大设备数**: 默认最多支持 8 个设备，可通过 `I2C_MAX_DEVICES` 修改

## 依赖项

- ESP-IDF >= 5.0
- FreeRTOS (互斥锁)
- ESP-IDF I2C Master 驱动 (`driver/i2c_master.h`)

## 版本历史

- **v1.0** (2025-10-24): 初始版本
  - 基础 I2C 总线管理功能
  - 设备注册机制
  - 线程安全保护

## 相关文档

- [ESP-IDF I2C Master Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/i2c.html)
- [项目架构设计文档](../../../开发架构文档/软件架构设计.md)
- [代码结构规范](../../../开发架构文档/代码结构规范.md)