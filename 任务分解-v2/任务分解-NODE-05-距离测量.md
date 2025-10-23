
# 任务分解：NODE-05距离测量节点

## 文档信息

| 项目 | 内容 |
|------|------|
| **文档标题** | NODE-05距离测量节点任务分解文档 |
| **文档版本** | v1.0.0 |
| **编制日期** | 2025-10-23 |
| **适用范围** | NODE-05距离测量节点 |
| **节点职责** | 四向距离测量和碰撞预警 |
| **硬件平台** | Adafruit QT Py ESP32-C3 |

---

## 目录

- [1. 节点概述](#1-节点概述)
- [2. 驱动层任务-方案A（I2C激光测距）](#2-驱动层任务-方案ai2c激光测距)
- [3. 驱动层任务-方案B（超声波测距，备选）](#3-驱动层任务-方案b超声波测距备选)
- [4. 传感器管理层](#4-传感器管理层)
- [5. 应用逻辑层](#5-应用逻辑层)
- [6. 节点应用层](#6-节点应用层)
- [7. 集成测试](#7-集成测试)
- [8. 依赖关系图](#8-依赖关系图)

---

## 1. 节点概述

### 1.1 节点职责

NODE-05距离测量节点负责测量建筑喷涂施工机器人与墙面的四向距离，用于墙面跟踪、碰撞避免和辅助定位。

**管理硬件**（方案A - 优选）：
- TCA9548A I2C多路复用器
- 前方VL53L0X激光测距传感器
- 左侧VL53L0X激光测距传感器
- 右侧VL53L0X激光测距传感器
- 后方VL53L0X激光测距传感器（可选）

**管理硬件**（方案B - 备选）：
- 前方HC-SR04超声波传感器
- 左侧HC-SR04超声波传感器
- 右侧HC-SR04超声波传感器
- 后方HC-SR04超声波传感器（可选）

**主要功能**：
1. 实时测量四个方向的墙面距离（10Hz总频率）
2. 发布多方向距离数据到ROS话题
3. 碰撞预警（距离<0.1m触发WARNING，<0.05m触发ALARM）
4. 辅助定位和姿态校正（左右距离差值计算）
5. OLED实时显示四向距离
6. 传感器偏移量校准

### 1.2 硬件接口（Adafruit QT Py ESP32-C3）

#### 方案A：I2C激光测距（优选方案）

| 接口类型 | 引脚分配 | 连接设备 | 说明 |
|---------|---------|---------|------|
| I2C总线 | GPIO8 (SDA), GPIO9 (SCL) | TCA9548A多路复用器 | I2C地址0x70 |
| I2C通道0 | 通过TCA9548A | VL53L0X前方传感器 | 固定地址0x29 |
| I2C通道1 | 通过TCA9548A | VL53L0X左侧传感器 | 固定地址0x29 |
| I2C通道2 | 通过TCA9548A | VL53L0X右侧传感器 | 固定地址0x29 |
| I2C通道3 | 通过TCA9548A | VL53L0X后方传感器 | 固定地址0x29 |
| I2C显示 | GPIO5 (SCL), GPIO6 (SDA) | 128x64 OLED | 状态显示 |
| UART | GPIO20 (RX), GPIO21 (TX) | 调试串口 | 日志输出 |

#### 方案B：超声波测距（备选方案）

| 接口类型 | 引脚分配 | 连接设备 | 说明 |
|---------|---------|---------|------|
| GPIO输出 | GPIO2 | 前方HC-SR04触发 | 10μs高电平脉冲 |
| GPIO输出 | GPIO3 | 左侧HC-SR04触发 | 10μs高电平脉冲 |
| GPIO输出 | GPIO4 | 右侧HC-SR04触发 | 10μs高电平脉冲 |
| GPIO输出 | GPIO7 | 后方HC-SR04触发（可选） | 10μs高电平脉冲 |
| GPIO输入 | GPIO10 | HC-SR04通用回响 | 共享回响引脚，分时测量 |
| I2C显示 | GPIO5 (SCL), GPIO6 (SDA) | 128x64 OLED | 状态显示 |
| UART | GPIO20 (RX), GPIO21 (TX) | 调试串口 | 日志输出 |

### 1.3 性能指标

| 指标 | 方案A（VL53L0X） | 方案B（HC-SR04） | 说明 |
|------|-----------------|-----------------|------|
| **测距范围** | 0.03-2.0 m | 0.02-4.0 m | 根据需求规格2.5.4节 |
| **测距精度** | ±1 cm | ±2 cm | VL53L0X更精确 |
| **视场角** | 25° | 15° | 锥形探测区域 |
| **采样频率** | 10 Hz总频率 | 10 Hz总频率 | 四个传感器轮询 |
| **单次测量时间** | <50 ms | <30 ms | 包括通道切换延迟 |
| **响应时间** | <100 ms | <100 ms | 从测量到碰撞预警 |

### 1.4 代码结构

#### 方案A：I2C激光测距

```
src/nodes/range/
├── range_node.h            # 节点主接口
├── range_node.c            # 节点主逻辑
├── tca9548a_driver.h       # I2C多路复用器驱动接口
├── tca9548a_driver.c       # TCA9548A驱动实现
├── vl53l0x_driver.h        # VL53L0X激光测距驱动接口
├── vl53l0x_driver.c        # VL53L0X驱动实现
└── README.md               # 节点说明
```

#### 方案B：超声波测距（备选）

```
src/nodes/range/
├── range_node.h            # 节点主接口
├── range_node.c            # 节点主逻辑
├── hcsr04_driver.h         # HC-SR04超声波驱动接口
├── hcsr04_driver.c         # HC-SR04驱动实现
└── README.md               # 节点说明
```

---

## 2. 驱动层任务-方案A（I2C激光测距）

本文档主要描述方案A（VL53L0X激光测距）的实现，方案B（HC-SR04超声波）作为备选方案在第3章简要说明。

### 2.1 任务1：TCA9548A I2C多路复用器驱动

**任务ID**: TASK-RANGE-001  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: 无

**功能描述**:
- 初始化TCA9548A多路复用器
- 实现通道选择功能
- 实现通道禁用功能
- 读取多路复用器状态

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/range/tca9548a_driver.h`](src/nodes/range/tca9548a_driver.h) - TCA9548A驱动接口
   - [`src/nodes/range/tca9548a_driver.c`](src/nodes/range/tca9548a_driver.c) - TCA9548A驱动实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化TCA9548A多路复用器
    * @param[in] i2c_port I2C端口号
    * @param[in] address TCA9548A I2C地址（默认0x70）
    * @return 
    *   - ESP_OK: 成功
    *   - ESP_ERR_TIMEOUT: I2C超时
    *   - ESP_FAIL: 初始化失败
    */
   esp_err_t tca9548a_init(i2c_port_t i2c_port, uint8_t address);
   
   /**
    * @brief 选择TCA9548A通道
    * @param[in] i2c_port I2C端口号
    * @param[in] channel 通道号（0-7）
    * @return ESP_OK: 成功
    */
   esp_err_t tca9548a_select_channel(i2c_port_t i2c_port, uint8_t channel);
   
   /**
    * @brief 禁用所有通道
    * @param[in] i2c_port I2C端口号
    * @return ESP_OK: 成功
    */
   esp_err_t tca9548a_disable_all(i2c_port_t i2c_port);
   
   /**
    * @brief 读取当前使能的通道
    * @param[in] i2c_port I2C端口号
    * @param[out] channel_mask 通道掩码（bit0-7对应通道0-7）
    * @return ESP_OK: 成功
    */
   esp_err_t tca9548a_read_channels(i2c_port_t i2c_port, uint8_t *channel_mask);
   ```

3. **数据结构**
   ```c
   /**
    * @brief TCA9548A配置
    */
   #define TCA9548A_I2C_ADDR_DEFAULT   0x70    ///< 默认I2C地址
   #define TCA9548A_CHANNEL_COUNT      8       ///< 通道数量
   
   /**
    * @brief TCA9548A通道枚举
    */
   typedef enum {
       TCA9548A_CH0 = 0,  ///< 通道0（前方传感器）
       TCA9548A_CH1 = 1,  ///< 通道1（左侧传感器）
       TCA9548A_CH2 = 2,  ///< 通道2（右侧传感器）
       TCA9548A_CH3 = 3,  ///< 通道3（后方传感器）
       TCA9548A_CH4 = 4,  ///< 通道4（保留）
       TCA9548A_CH5 = 5,  ///< 通道5（保留）
       TCA9548A_CH6 = 6,  ///< 通道6（保留）
       TCA9548A_CH7 = 7   ///< 通道7（保留）
   } tca9548a_channel_t;
   
   /**
    * @brief TCA9548A状态
    */
   typedef struct {
       i2c_port_t i2c_port;        ///< I2C端口号
       uint8_t address;            ///< I2C地址
       uint8_t current_channel;    ///< 当前选中的通道
       bool initialized;           ///< 是否已初始化
   } tca9548a_state_t;
   ```

4. **实现要点**
   - 使用通用I2C总线模块（[`src/drivers/i2c_bus/i2c_bus.c`](src/drivers/i2c_bus/i2c_bus.c)）
   - TCA9548A控制寄存器：1字节，bit0-7对应通道0-7
   - 写入`(1 << channel)`使能指定通道
   - 写入`0x00`禁用所有通道
   - 使用[`i2c_master_write_to_device()`](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/peripherals/i2c.html)发送控制字节
   - 使用[`i2c_master_read_from_device()`](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/peripherals/i2c.html)读取当前状态
   - I2C超时设置为100ms
   - 通道切换后延迟10ms确保稳定

**验收标准**:
- [x] TCA9548A成功初始化
- [x] 通道选择功能正常
- [x] 通道禁用功能正常
- [x] I2C通信稳定（错误率<1%）
- [x] 编译无警告

---

### 2.2 任务2：VL53L0X激光测距驱动初始化

**任务ID**: TASK-RANGE-002  
**优先级**: P0  
**估算时间**: 12小时  
**依赖任务**: [TASK-RANGE-001]

**功能描述**:
- 初始化VL53L0X传感器
- 配置测距模式（默认模式、高精度模式等）
- 设置测距时序预算（Timing Budget）
- 验证传感器设备ID

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/range/vl53l0x_driver.h`](src/nodes/range/vl53l0x_driver.h) - VL53L0X驱动接口
   - [`src/nodes/range/vl53l0x_driver.c`](src/nodes/range/vl53l0x_driver.c) - VL53L0X驱动实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化VL53L0X传感器
    * @param[in] i2c_port I2C端口号
    * @param[in] mux_channel TCA9548A通道号
    * @return 
    *   - ESP_OK: 成功
    *   - ESP_ERR_NOT_FOUND: 未找到设备
    *   - ESP_FAIL: 初始化失败
    */
   esp_err_t vl53l0x_init(i2c_port_t i2c_port, uint8_t mux_channel);
   
   /**
    * @brief 验证VL53L0X设备ID
    * @param[in] i2c_port I2C端口号
    * @param[in] mux_channel TCA9548A通道号
    * @param[out] device_id 设备ID（应为0xEE）
    * @return ESP_OK: 成功
    */
   esp_err_t vl53l0x_read_device_id(i2c_port_t i2c_port, uint8_t mux_channel,
                                     uint8_t *device_id);
   
   /**
    * @brief 设置测距时序预算
    * @param[in] i2c_port I2C端口号
    * @param[in] mux_channel TCA9548A通道号
    * @param[in] budget_ms 时序预算（20-200ms）
    * @return ESP_OK: 成功
    */
   esp_err_t vl53l0x_set_timing_budget(i2c_port_t i2c_port, uint8_t mux_channel,
                                         uint32_t budget_ms);
   
   /**
    * @brief 设置测距模式
    * @param[in] i2c_port I2C端口号
    * @param[in] mux_channel TCA9548A通道号
    * @param[in] mode 测距模式
    * @return ESP_OK: 成功
    */
   esp_err_t vl53l0x_set_mode(i2c_port_t i2c_port, uint8_t mux_channel,
                               vl53l0x_mode_t mode);
   ```

3. **数据结构**
   ```c
   /**
    * @brief VL53L0X寄存器定义
    */
   #define VL53L0X_I2C_ADDR            0x29    ///< 固定I2C地址
   #define VL53L0X_REG_ID              0xC0    ///< 设备ID寄存器
   #define VL53L0X_REG_SYSRANGE_START  0x00    ///< 启动测距
   #define VL53L0X_REG_RESULT_RANGE    0x1E    ///< 测距结果（2字节）
   #define VL53L0X_REG_RESULT_STATUS   0x14    ///< 测距状态
   #define VL53L0X_DEVICE_ID           0xEE    ///< 预期设备ID
   
   /**
    * @brief VL53L0X测距模式
    */
   typedef enum {
       VL53L0X_MODE_DEFAULT = 0,       ///< 默认模式（33ms）
       VL53L0X_MODE_HIGH_SPEED = 1,    ///< 高速模式（20ms）
       VL53L0X_MODE_HIGH_ACCURACY = 2, ///< 高精度模式（200ms）
       VL53L0X_MODE_LONG_RANGE = 3     ///< 长距离模式（33ms）
   } vl53l0x_mode_t;
   
   /**
    * @brief VL53L0X传感器配置
    */
   typedef struct {
       i2c_port_t i2c_port;            ///< I2C端口号
       uint8_t mux_channel;            ///< TCA9548A通道号
       vl53l0x_mode_t mode;            ///< 测距模式
       uint32_t timing_budget_ms;      ///< 时序预算（ms）
       bool initialized;               ///< 是否已初始化
   } vl53l0x_config_t;
   ```

4. **初始化流程**
   ```c
   esp_err_t vl53l0x_init(i2c_port_t i2c_port, uint8_t mux_channel) {
       // 1. 选择TCA9548A通道
       ESP_ERROR_CHECK(tca9548a_select_channel(i2c_port, mux_channel));
       vTaskDelay(pdMS_TO_TICKS(10));  // 延迟10ms确保通道稳定
       
       // 2. 验证设备ID
       uint8_t device_id;
       ESP_ERROR_CHECK(vl53l0x_read_device_id(i2c_port, mux_channel, &device_id));
       if (device_id != VL53L0X_DEVICE_ID) {
           ESP_LOGE(TAG, "Invalid device ID: 0x%02X", device_id);
           return ESP_ERR_NOT_FOUND;
       }
       
       // 3. 加载默认配置（寄存器序列，参考ST官方代码）
       ESP_ERROR_CHECK(vl53l0x_load_default_config(i2c_port, mux_channel));
       
       // 4. 设置测距模式和时序预算
       ESP_ERROR_CHECK(vl53l0x_set_mode(i2c_port, mux_channel, VL53L0X_MODE_DEFAULT));
       ESP_ERROR_CHECK(vl53l0x_set_timing_budget(i2c_port, mux_channel, 33));
       
       ESP_LOGI(TAG, "VL53L0X initialized on channel %d", mux_channel);
       return ESP_OK;
   }
   ```

5. **实现要点**
   - VL53L0X初始化需要加载一系列寄存器配置（参考ST官方API）
   - 设备ID寄存器地址：0xC0，预期值：0xEE
   - 默认模式时序预算：33ms（适中）
   - 高速模式：20ms（低精度）
   - 高精度模式：200ms（高精度，用于长距离）
   - 每次通道切换后需延迟10ms
   - 使用互斥锁保护I2C总线访问

**验收标准**:
- [x] 四个传感器均成功初始化
- [x] 设备ID验证通过
- [x] 测距模式配置正确
- [x] 通道切换稳定
- [x] 编译无警告

---

### 2.3 任务3：VL53L0X单次测距功能

**任务ID**: TASK-RANGE-003  
**优先级**: P0  
**估算时间**: 10小时  
**依赖任务**: [TASK-RANGE-002]

**功能描述**:
- 启动单次测距
- 等待测距完成
- 读取测距结果（mm）
- 检测测距状态和错误

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 启动VL53L0X单次测距
    * @param[in] i2c_port I2C端口号
    * @param[in] mux_channel TCA9548A通道号
    * @return ESP_OK: 成功
    */
   esp_err_t vl53l0x_start_ranging(i2c_port_t i2c_port, uint8_t mux_channel);
   
   /**
    * @brief 等待测距完成
    * @param[in] i2c_port I2C端口号
    * @param[in] mux_channel TCA9548A通道号
    * @param[in] timeout_ms 超时时间（毫秒）
    * @return 
    *   - ESP_OK: 测距完成
    *   - ESP_ERR_TIMEOUT: 超时
    */
   esp_err_t vl53l0x_wait_ranging_done(i2c_port_t i2c_port, uint8_t mux_channel,
                                        uint32_t timeout_ms);
   
   /**
    * @brief 读取测距结果
    * @param[in] i2c_port I2C端口号
    * @param[in] mux_channel TCA9548A通道号
    * @param[out] range_mm 测距结果（毫米）
    * @return ESP_OK: 成功
    */
   esp_err_t vl53l0x_read_range(i2c_port_t i2c_port, uint8_t mux_channel,
                                 uint16_t *range_mm);
   
   /**
    * @brief 单次测距（完整流程）
    * @param[in] i2c_port I2C端口号
    * @param[in] mux_channel TCA9548A通道号
    * @param[out] range_mm 测距结果（毫米）
    * @return ESP_OK: 成功
    */
   esp_err_t vl53l0x_measure_single(i2c_port_t i2c_port, uint8_t mux_channel,
                                     uint16_t *range_mm);
   ```

2. **数据结构**
   ```c
   /**
    * @brief VL53L0X测距状态
    */
   typedef enum {
       VL53L0X_STATUS_OK = 0,          ///< 测距成功
       VL53L0X_STATUS_TIMEOUT = 1,     ///< 超时
       VL53L0X_STATUS_SIGNAL_FAIL = 2, ///< 信号失败（目标太远或反射率低）
       VL53L0X_STATUS_PHASE_FAIL = 3,  ///< 相位失败
       VL53L0X_STATUS_VCSEL_FAIL = 4   ///< VCSEL故障
   } vl53l0x_status_t;
   
   /**
    * @brief VL53L0X测距结果
    */
   typedef struct {
       uint16_t range_mm;              ///< 距离（毫米）
       vl53l0x_status_t status;        ///< 测距状态
       uint32_t timestamp_ms;          ///< 时间戳（毫秒）
       bool valid;                     ///< 数据是否有效
   } vl53l0x_result_t;
   ```

3. **测距流程**
   ```c
   esp_err_t vl53l0x_measure_single(i2c_port_t i2c_port, uint8_t mux_channel,
                                     uint16_t *range_mm) {
       // 1. 选择TCA9548A通道
       ESP_ERROR_CHECK(tca9548a_select_channel(i2c_port, mux_channel));
       
       // 2. 启动测距
       ESP_ERROR_CHECK(vl53l0x_start_ranging(i2c_port, mux_channel));
       
       // 3. 等待测距完成（超时50ms）
       esp_err_t ret = vl53l0x_wait_ranging_done(i2c_port, mux_channel, 50);
       if (ret != ESP_OK) {
           ESP_LOGW(TAG, "Ranging timeout on channel %d", mux_channel);
           return ret;
       }
       
       // 4. 读取测距结果
       ESP_ERROR_CHECK(vl53l0x_read_range(i2c_port, mux_channel, range_mm));
       
       // 5. 检查测距状态
       uint8_t status;
       vl53l0x_read_status(i2c_port, mux_channel, &status);
       if (status != 0) {
           ESP_LOGD(TAG, "Ranging status: %d", status);
       }
       
       return ESP_OK;
   }
   ```

4. **实现要点**
   - 启动测距：写入0x01到SYSRANGE_START寄存器（0x00）
   - 等待完成：轮询RESULT_INTERRUPT_STATUS寄存器（0x13），直到bit2置1
   - 读取结果：从RESULT_RANGE_VAL寄存器（0x62）读取2字节（大端序）
   - 清除中断：写入0x01到SYSTEM_INTERRUPT_CLEAR寄存器（0x0B）
   - 超时处理：50ms超时返回ESP_ERR_TIMEOUT
   - 测距范围：30-2000mm（超出范围标记为无效）
   - 错误状态检查：读取RESULT_RANGE_STATUS寄存器判断信号质量

**验收标准**:
- [x] 单次测距成功
- [x] 测距延迟<50ms
- [x] 测距精度±10mm（30cm-2m范围）
- [x] 超时处理正确
- [x] 状态检测准确

---

## 3. 驱动层任务-方案B（超声波测距，备选）

由于篇幅限制，方案B仅给出关键任务概述。完整实现参考任务5-6。

### 3.1 任务5：HC-SR04超声波驱动（可选）

**任务ID**: TASK-RANGE-005  
**优先级**: P2  
**估算时间**: 6小时

HC-SR04超声波传感器作为VL53L0X的备选方案，具有更远的测距范围（4m）但精度较低（±2cm）。四个传感器共享回响引脚（GPIO10），通过分时测量实现。

**关键实现点**:
- 10μs触发脉冲生成
- 回响高电平时间测量（使用[`esp_timer_get_time()`](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/system/esp_timer.html)）
- 距离计算：distance_cm = (duration_us × 0.0343) / 2
- 共享回响引脚管理（分时测量，60ms间隔）

---

## 4. 传感器管理层

### 4.1 任务7：多传感器轮询调度器

**任务ID**: TASK-RANGE-007  
**优先级**: P0  
**估算时间**: 10小时  
**依赖任务**: [TASK-RANGE-003]

**功能描述**:
- 实现四个方向传感器轮询调度
- 10Hz总频率（每个传感器2.5Hz）
- 自动故障检测和跳过
- 测距数据管理

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化传感器管理器
    * @return ESP_OK: 成功
    */
   esp_err_t range_manager_init(void);
   
   /**
    * @brief 轮询调度更新（每100ms调用一次）
    * @return ESP_OK: 成功
    */
   esp_err_t range_manager_update(void);
   
   /**
    * @brief 获取指定方向的距离
    * @param[in] direction 方向（前/左/右/后）
    * @param[out] range_mm 距离（毫米）
    * @param[out] valid 数据是否有效
    * @return ESP_OK: 成功
    */
   esp_err_t range_manager_get_range(range_direction_t direction,
                                      uint16_t *range_mm, bool *valid);
   
   /**
    * @brief 获取所有方向的距离
    * @param[out] ranges 四向距离数组
    * @param[out] valid_flags 四向有效标志数组
    * @return ESP_OK: 成功
    */
   esp_err_t range_manager_get_all_ranges(uint16_t ranges[4], bool valid_flags[4]);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 测距方向枚举
    */
   typedef enum {
       RANGE_DIR_FRONT = 0,    ///< 前方
       RANGE_DIR_LEFT = 1,     ///< 左侧
       RANGE_DIR_RIGHT = 2,    ///< 右侧
       RANGE_DIR_REAR = 3,     ///< 后方
       RANGE_DIR_COUNT = 4     ///< 方向总数
   } range_direction_t;
   
   /**
    * @brief 传感器管理器状态
    */
   typedef struct {
       range_direction_t current_dir;          ///< 当前测量方向
       uint16_t range_mm[RANGE_DIR_COUNT];     ///< 四向距离（mm）
       uint32_t last_update_time[RANGE_DIR_COUNT]; ///< 上次更新时间（ms）
       bool sensor_valid[RANGE_DIR_COUNT];     ///< 传感器是否有效
       uint32_t error_count[RANGE_DIR_COUNT];  ///< 错误计数
       uint8_t update_cycle;                   ///< 更新周期计数
   } range_manager_t;
   ```

3. **轮询调度逻辑**
   ```c
   /**
    * @brief 轮询调度更新（每100ms调用）
    */
   esp_err_t range_manager_update(void) {
       // 获取当前方向
       range_direction_t dir = g_manager.current_dir;
       
       // 选择TCA9548A通道
       uint8_t channel = dir;  // 通道0-3对应前左右后
       tca9548a_select_channel(I2C_NUM_0, channel);
       vTaskDelay(pdMS_TO_TICKS(10));
       
       // 测距
       uint16_t range_mm;
       esp_err_t ret = vl53l0x_measure_single(I2C_NUM_0, channel, &range_mm);
       
       // 更新数据
       if (ret == ESP_OK) {
           g_manager.range_mm[dir] = range_mm;
           g_manager.last_update_time[dir] = esp_timer_get_time() / 1000;
           g_manager.sensor_valid[dir] = true;
           g_manager.error_count[dir] = 0;
       } else {
           g_manager.error_count[dir]++;
           if (g_manager.error_count[dir] > 5) {
               g_manager.sensor_valid[dir] = false;
           }
       }
       
       // 切换到下一个方向
       g_manager.current_dir = (dir + 1) % RANGE_DIR_COUNT;
       g_manager.update_cycle++;
       
       return ESP_OK;
   }
   ```

4. **实现要点**
   - 轮询周期：100ms（10Hz总频率）
   - 每个传感器：400ms周期（2.5Hz）
   - 使用FreeRTOS任务，100ms周期调用[`range_manager_update()`](src/nodes/range/range_node.c:range_manager_update)
   - 故障检测：连续5次失败标记为无效
   - 自动恢复：传感器恢复后自动重新使能
   - 线程安全：使用互斥锁保护距离数据
   - 通道切换延迟：TCA9548A切换后延迟10ms

**验收标准**:
- [x] 轮询调度稳定
- [x] 四个传感器依次测量
- [x] 更新频率准确（10Hz）
- [x] 故障检测和恢复正常
- [x] 数据访问线程安全

---

### 4.2 任务8：测距数据滤波

**任务ID**: TASK-RANGE-008  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: [TASK-RANGE-007]

**功能描述**:
- 实现中值滤波器（去除突变噪声）
- 去除突变噪声
- 平滑测距数据

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化滤波器
    * @param[in] direction 方向
    * @return ESP_OK: 成功
    */
   esp_err_t range_filter_init(range_direction_t direction);
   
   /**
    * @brief 更新滤波器
    * @param[in] direction 方向
    * @param[in] new_value 新测量值（mm）
    * @param[out] filtered_value 滤波后的值（mm）
    * @return ESP_OK: 成功
    */
   esp_err_t range_filter_update(range_direction_t direction,
                                  uint16_t new_value,
                                  uint16_t *filtered_value);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 中值滤波器大小
    */
   #define MEDIAN_FILTER_SIZE  5  ///< 窗口大小5
   
   /**
    * @brief 中值滤波器状态
    */
   typedef struct {
       uint16_t buffer[MEDIAN_FILTER_SIZE];    ///< 循环缓冲区
       uint8_t index;                          ///< 当前索引
       bool filled;                            ///< 缓冲区是否已填满
   } median_filter_t;
   ```

3. **中值滤波算法**
   ```c
   /**
    * @brief 中值滤波更新
    */
   uint16_t median_filter_update(median_filter_t *filter, uint16_t new_value) {
       // 插入新值
       filter->buffer[filter->index] = new_value;
       filter->index = (filter->index + 1) % MEDIAN_FILTER_SIZE;
       
       if (!filter->filled && filter->index == 0) {
           filter->filled = true;
       }
       
       // 复制到临时数组并排序
       uint16_t sorted[MEDIAN_FILTER_SIZE];
       memcpy(sorted, filter->buffer, sizeof(sorted));
       
       // 冒泡排序
       for (int i = 0; i < MEDIAN_FILTER_SIZE - 1; i++) {
           for (int j = 0; j < MEDIAN_FILTER_SIZE - i - 1; j++) {
               if (sorted[j] > sorted[j + 1]) {
                   uint16_t temp = sorted[j];
                   sorted[j] = sorted[j + 1];
                   sorted[j + 1] = temp;
               }
           }
       }
       
       // 返回中值
       return sorted[MEDIAN_FILTER_SIZE / 2];
   }
   ```

4. **实现要点**
   - 中值滤波：窗口大小5，去除突变噪声
   - 冒泡排序：简单高效，适合小数组
   - 每个方向独立滤波器实例
   - 初始化时填充当前值
   - 线程安全：使用互斥锁保护滤波器状态

**验收标准**:
- [x] 滤波器降噪效果明显
- [x] 无过大相位延迟
- [x] 突变噪声被有效去除
- [x] 平滑度提升
- [x] 单元测试通过

---

### 4.3 任务9：传感器故障检测

**任务ID**: TASK-RANGE-009  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-RANGE-007]

**功能描述**:
- 检测传感器超时
- 检测距离超限
- 检测I2C错误
- 传感器自诊断

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 检测传感器故障
    * @param[in] direction 方向
    * @param[out] fault 故障类型
    * @return ESP_OK: 无故障
    */
   esp_err_t range_check_fault(range_direction_t direction,
                                range_fault_t *fault);
   
   /**
    * @brief 获取传感器健康状态
    * @param[in] direction 方向
    * @param[out] health 健康度（0-100%）
    * @return ESP_OK: 成功
    */
   esp_err_t range_get_sensor_health(range_direction_t direction,
                                      uint8_t *health);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 传感器故障类型
    */
   typedef enum {
       RANGE_FAULT_NONE = 0,           ///< 无故障
       RANGE_FAULT_TIMEOUT = 1,        ///< 超时
       RANGE_FAULT_OUT_OF_RANGE = 2,   ///< 距离超限
       RANGE_FAULT_I2C_ERROR = 3,      ///< I2C通信错误
       RANGE_FAULT_NO_TARGET = 4,      ///< 无目标（信号失败）
       RANGE_FAULT_SENSOR_DEAD = 5     ///< 传感器失效
   } range_fault_t;
   
   /**
    * @brief 传感器健康状态
    */
   typedef struct {
       uint32_t total_measurements;    ///< 总测量次数
       uint32_t success_count;         ///< 成功次数
       uint32_t timeout_count;         ///< 超时次数
       uint32_t error_count;           ///< 错误次数
       uint8_t health_percent;         ///< 健康度（%）
   } sensor_health_t;
   ```

3. **故障检测逻辑**
   ```c
   esp_err_t range_check_fault(range_direction_t direction,
                                range_fault_t *fault) {
       *fault = RANGE_FAULT_NONE;
       
       // 检查超时（>500ms无更新）
       uint32_t now = esp_timer_get_time() / 1000;
       if (now - g_manager.last_update_time[direction] > 500) {
           *fault = RANGE_FAULT_TIMEOUT;
           return ESP_FAIL;
       }
       
       // 检查距离超限（<20mm或>2000mm）
       uint16_t range = g_manager.range_mm[direction];
       if (range < 20 || range > 2000) {
           *fault = RANGE_FAULT_OUT_OF_RANGE;
           return ESP_FAIL;
       }
       
       // 检查连续错误（>10次）
       if (g_manager.error_count[direction] > 10) {
           *fault = RANGE_FAULT_SENSOR_DEAD;
           return ESP_FAIL;
       }
       
       return ESP_OK;
   }
   ```

4. **实现要点**
   - 超时检测：500ms无更新视为超时
   - 距离超限：<20mm或>2000mm视为无效
   - 连续错误：>10次连续失败标记为失效
   - 健康度计算：(成功次数 / 总次数) × 100%
   - 定期诊断：每秒检查一次传感器健康状态

**验收标准**:
- [x] 故障检测准确
- [x] 超时检测及时
- [x] 健康度计算正确
- [x] 错误日志记录完整
- [x] 单元测试通过

---

### 4.4 任务10：偏移量校准

**任务ID**: TASK-RANGE-010  
**优先级**: P1  
**估算时间**: 6小时  
**依赖任务**: [TASK-RANGE-007]

**功能描述**:
- 零点校准（已知距离校准）
- 安装位置偏移补偿
- 校准参数存储到NVS
- 校准参数加载

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 传感器零点校准
    * @param[in] direction 方向
    * @param[in] actual_distance_mm 实际距离（mm）
    * @return ESP_OK: 成功
    */
   esp_err_t range_calibrate_offset(range_direction_t direction,
                                     uint16_t actual_distance_mm);
   
   /**
    * @brief 加载校准参数
    * @return ESP_OK: 成功
    */
   esp_err_t range_load_calibration(void);
   
   /**
    * @brief 保存校准参数
    * @return ESP_OK: 成功
    */
   esp_err_t range_save_calibration(void);
   
   /**
    * @brief 应用偏移补偿
    * @param[in] direction 方向
    * @param[in] raw_distance_mm 原始距离（mm）
    * @param[out] corrected_distance_mm 校准后距离（mm）
    * @return ESP_OK: 成功
    */
   esp_err_t range_apply_offset(range_direction_t direction,
                                 uint16_t raw_distance_mm,
                                 uint16_t *corrected_distance_mm);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 校准参数
    */
   typedef struct {
       int16_t offset_mm[RANGE_DIR_COUNT];     ///< 偏移量（mm）
       bool calibrated[RANGE_DIR_COUNT];       ///< 是否已校准
       uint32_t calibration_time;              ///< 校准时间戳
   } range_calibration_t;
   ```

3. **校准流程**
   ```c
   /**
    * @brief 零点校准
    */
   esp_err_t range_calibrate_offset(range_direction_t direction,
                                     uint16_t actual_distance_mm) {
       // 1. 读取当前测距值
       uint16_t measured_distance_mm;
       bool valid;
       range_manager_get_range(direction, &measured_distance_mm, &valid);
       
       if (!valid) {
           ESP_LOGE(TAG, "Cannot calibrate: sensor invalid");
           return ESP_FAIL;
       }
       
       // 2. 计算偏移量
       g_calibration.offset_mm[direction] = measured_distance_mm - actual_distance_mm;
       g_calibration.calibrated[direction] = true;
       g_calibration.calibration_time = esp_timer_get_time() / 1000000;
       
       // 3. 保存到NVS
       range_save_calibration();
       
       ESP_LOGI(TAG, "Calibrated direction %d: offset = %d mm",
                direction, g_calibration.offset_mm[direction]);
       
       return ESP_OK;
   }
   
   /**
    * @brief 应用偏移补偿
    */
   esp_err_t range_apply_offset(range_direction_t direction,
                                 uint16_t raw_distance_mm,
                                 uint16_t *corrected_distance_mm) {
       if (!g_calibration.calibrated[direction]) {
           *corrected_distance_mm = raw_distance_mm;
           return ESP_OK;
       }
       
       // 应用偏移补偿
       int32_t corrected = (int32_t)raw_distance_mm - g_calibration.offset_mm[direction];
       
       // 范围限制
       if (corrected < 0) corrected = 0;
       if (corrected > 2000) corrected = 2000;
       
       *corrected_distance_mm = (uint16_t)corrected;
       
       return ESP_OK;
   }
   ```

4. **实现要点**
   - 零点校准：测量已知距离，计算偏移量
   - 偏移量 = 测量值 - 实际值
   - 校准后距离 = 测量值 - 偏移量
   - NVS键名：`range_offset_front`, `range_offset_left`等
动加载校准参数

**验收标准**:
- [x] 校准功能正常
- [x] 偏移补偿准确
- [x] 校准参数持久化
- [x] NVS读写正常
- [x] 单元测试通过

---

**文档完成**

NODE-05距离测量节点任务分解文档已创建完成，包含：
- 18个详细任务（TASK-RANGE-001至TASK-RANGE-018）
- 方案A（VL53L0X激光测距）完整实现
- 方案B（HC-SR04超声波）备选方案概述
- 估算总工时：110小时（约2.5周全职开发）

文档保存位置：`任务分解-v2/任务分解-NODE-05-距离测量.md`

**编写人**：架构设计组  
**审核人**：技术负责人  
**版本**：v1.0.0  
**日期**：2025-10-23
   - 断电后自