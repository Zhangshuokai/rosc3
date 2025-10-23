
# 任务分解：NODE-07 电源管理节点

## 文档信息

| 项目 | 内容 |
|------|------|
| **文档标题** | NODE-07 电源管理节点任务分解文档 |
| **文档版本** | v1.0.0 |
| **编制日期** | 2025-10-23 |
| **适用范围** | NODE-07 电源管理节点 |
| **节点职责** | 电池监测和电源管理 |
| **硬件平台** | Adafruit QT Py ESP32-C3 |

---

## 目录

- [1. 节点概述](#1-节点概述)
- [2. ADC采样层任务](#2-adc采样层任务)
- [3. 驱动层任务（INA219，可选）](#3-驱动层任务ina219可选)
- [4. 电池监测层](#4-电池监测层)
- [5. SOC估算层](#5-soc估算层)
- [6. 电源管理层](#6-电源管理层)
- [7. 节点应用层（power_node模块）](#7-节点应用层power_node模块)
- [8. 集成测试](#8-集成测试)
- [9. 依赖关系图](#9-依赖关系图)

---

## 1. 节点概述

### 1.1 节点职责

NODE-07 电源管理节点负责监测建筑喷涂施工机器人的电池状态，管理电源策略和低电量保护，确保系统安全稳定运行。

**管理硬件**：
- 电池电压监测电路（分压采样）
- 电池电流监测（霍尔电流传感器或INA219）
- 充电状态检测（GPIO输入）

**主要功能**：
1. 实时监测电池电压（V）
2. 实时监测电池电流（A，负值=放电）
3. 计算剩余电量百分比（%）
4. 发布电池状态到ROS话题（1Hz）
5. 低电量分级报警（50%/20%/10%阈值）
6. 充电状态检测和管理
7. OLED实时显示电压、电量、续航时间
8. 电量校准和容量学习

### 1.2 硬件接口（Adafruit QT Py ESP32-C3）

| 接口类型 | 引脚分配 | 连接设备 | 说明 |
|---------|---------|---------|------|
| ADC输入 | GPIO0 | 电压分压电路 | 测量0-54.6V电池电压 |
| ADC输入 | GPIO1 | 霍尔电流传感器 | 测量-50A ~ +50A电流 |
| GPIO输入 | GPIO4 | 充电状态检测 | 高电平=充电中 |
| I2C总线（可选） | GPIO8 (SDA), GPIO9 (SCL) | INA219传感器 | 精确电压电流测量 |
| I2C显示 | GPIO5 (SCL), GPIO6 (SDA) | 128x64 OLED | 状态显示 |
| UART | GPIO20 (RX), GPIO21 (TX) | 调试串口 | 日志输出 |

### 1.3 电池规格（48V锂电池系统）

| 参数 | 数值 | 说明 |
|------|------|------|
| **标称电压** | 48V | 13S锂电池组 |
| **满电电压** | 54.6V | 4.2V × 13 |
| **截止电压** | 39.0V | 3.0V × 13 |
| **电池容量** | 20Ah | 示例值 |
| **充电电流** | ≤10A | 最大充电电流 |
| **放电电流** | ≤30A | 最大放电电流 |

### 1.4 性能指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| **采样频率** | 1 Hz | 电池状态监测频率 |
| **电压精度** | ±100mV (ADC) / ±4mV (INA219) | 测量精度 |
| **电流精度** | ±0.5A (霍尔) / ±1mA (INA219) | 测量精度 |
| **SOC精度** | ±5% | 混合估算方法 |
| **报警响应** | <2s | 低电量检测到报警 |

### 1.5 代码结构

```
src/nodes/power/
├── power_node.h              # 节点主接口
├── power_node.c              # 节点主逻辑
├── battery_monitor.h         # 电池监测接口
├── battery_monitor.c         # 电池监测实现
├── voltage_adc.h             # 电压ADC采样接口
├── voltage_adc.c             # 电压ADC采样实现
├── current_adc.h             # 电流ADC采样接口
├── current_adc.c             # 电流ADC采样实现
├── ina219_driver.h           # INA219驱动接口（可选）
├── ina219_driver.c           # INA219驱动实现（可选）
├── soc_estimator.h           # 电量SOC估算接口
├── soc_estimator.c           # 电量SOC估算实现
└── README.md                 # 节点说明
```

---

## 2. ADC采样层任务

### 2.1 任务1：电压ADC采样（分压电路）

**任务ID**: TASK-PWR-001  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: 无

**功能描述**:
- 配置GPIO0的ADC功能
- 实现12位ADC采样（11dB衰减）
- 分压电路电压还原计算
- 多点采样平均滤波

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/power/voltage_adc.h`](src/nodes/power/voltage_adc.h) - 电压ADC接口
   - [`src/nodes/power/voltage_adc.c`](src/nodes/power/voltage_adc.c) - 电压ADC实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化电压ADC
    * @param[out] adc 电压ADC结构体指针
    * @return 
    *   - ESP_OK: 成功
    *   - ESP_FAIL: 初始化失败
    */
   esp_err_t voltage_adc_init(voltage_adc_t *adc);
   
   /**
    * @brief 读取电池电压
    * @param[in] adc 电压ADC结构体指针
    * @return 电池电压（V）
    */
   float voltage_adc_read(voltage_adc_t *adc);
   
   /**
    * @brief 读取ADC原始值
    * @param[in] adc 电压ADC结构体指针
    * @return ADC原始值（0-4095）
    */
   int voltage_adc_read_raw(voltage_adc_t *adc);
   
   /**
    * @brief 电压校准
    * @param[in] adc 电压ADC结构体指针
    * @param[in] actual_voltage 实际电压（V）
    * @param[in] measured_voltage 测量电压（V）
    * @return ESP_OK: 成功
    */
   esp_err_t voltage_adc_calibrate(voltage_adc_t *adc, 
                                   float actual_voltage,
                                   float measured_voltage);
   ```

3. **数据结构**
   ```c
   /**
    * @brief 电压ADC配置
    */
   #define VOLTAGE_DIV_RATIO       16.0f       // 分压比（R1=100kΩ, R2=6.8kΩ）
   #define ADC_MAX_VALUE           4095.0f     // 12位ADC最大值
   #define ADC_VREF_MV             3300.0f     // 参考电压（mV）
   #define VOLTAGE_CALIBRATION_K   1.0f        // 校准系数K（默认）
   #define VOLTAGE_CALIBRATION_B   0.0f        // 校准系数B（默认）
   
   /**
    * @brief 电压ADC结构体
    */
   typedef struct {
       adc_oneshot_unit_handle_t adc_handle;  // ADC单元句柄
       adc_channel_t channel;                  // ADC通道
       float calibration_k;                    // 校准系数K（y = kx + b）
       float calibration_b;                    // 校准系数B
   } voltage_adc_t;
   ```

4. **分压电路设计**
   ```
   电池48V → R1(100kΩ) → 采样点 → R2(6.8kΩ) → GND
   分压比：6.8 / (100 + 6.8) ≈ 1/16
   
   54.6V（满电） → 3.41V（需11dB衰减）
   39.0V（空电） → 2.44V
   ```

5. **实现要点**
   - 使用[`adc_oneshot_new_unit()`](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/peripherals/adc_oneshot.html)创建ADC单元
   - 配置11dB衰减（测量范围0-3.3V）
   - 多次采样取平均（建议10次）
   - 应用校准系数：`V_battery = (V_adc * DIV_RATIO) * K + B`
   - 校准参数保存到NVS

**验收标准**:
- [x] ADC采样稳定
- [x] 电压测量精度±100mV
- [x] 分压计算正确
- [x] 校准功能正常
- [x] 编译无警告

---

### 2.2 任务2：电流ADC采样（霍尔传感器）

**任务ID**: TASK-PWR-002  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: 无

**功能描述**:
- 配置GPIO1的ADC功能
- 读取霍尔电流传感器输出
- 电压转换为电流值
- 零点校准和灵敏度校准

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/power/current_adc.h`](src/nodes/power/current_adc.h) - 电流ADC接口
   - [`src/nodes/power/current_adc.c`](src/nodes/power/current_adc.c) - 电流ADC实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化电流ADC
    * @param[out] adc 电流ADC结构体指针
    * @return ESP_OK: 成功
    */
   esp_err_t current_adc_init(current_adc_t *adc);
   
   /**
    * @brief 读取电池电流
    * @param[in] adc 电流ADC结构体指针
    * @return 电池电流（A，负值=放电，正值=充电）
    */
   float current_adc_read(current_adc_t *adc);
   
   /**
    * @brief 电流零点校准
    * @param[in] adc 电流ADC结构体指针
    * @return ESP_OK: 成功
    */
   esp_err_t current_adc_calibrate_zero(current_adc_t *adc);
   ```

3. **数据结构**
   ```c
   /**
    * @brief 霍尔传感器配置（ACS712-50A）
    */
   #define CURRENT_ZERO_POINT_MV   1650.0f     // 0A对应电压（mV）
   #define CURRENT_SENSITIVITY_MV  33.0f       // 灵敏度（mV/A）
   
   /**
    * @brief 电流ADC结构体
    */
   typedef struct {
       adc_oneshot_unit_handle_t adc_handle;  // ADC单元句柄
       adc_channel_t channel;                  // ADC通道
       float zero_point_mv;                    // 零点电压（mV）
       float sensitivity_mv_a;                 // 灵敏度（mV/A）
   } current_adc_t;
   ```

4. **霍尔传感器特性**
   ```
   ACS712-50A输出特性：
   - 测量范围：-50A ~ +50A
   - 输出电压：0V(-50A) ~ 1.65V(0A) ~ 3.3V(+50A)
   - 灵敏度：33mV/A
   - 零点漂移：需要定期校准
   ```

5. **实现要点**
   - 零点校准：断开电池时采样，记录零点电压
   - 电流计算：`I = (V_adc - V_zero) / Sensitivity`
   - 多次采样滤波（10次平均）
   - 校准参数持久化到NVS
   - 负值表示放电，正值表示充电

**验收标准**:
- [x] 电流测量准确（±0.5A）
- [x] 零点校准有效
- [x] 正负电流识别正确
- [x] 滤波效果良好
- [x] 功能测试通过

---

### 2.3 任务3：ADC多点校准

**任务ID**: TASK-PWR-003  
**优先级**: P1  
**估算时间**: 6小时  
**依赖任务**: [TASK-PWR-001, TASK-PWR-002]

**功能描述**:
- 实现两点法校准
- 支持多点法校准（可选）
- 计算校准系数K和B
- 校准数据管理

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 两点法电压校准
    * @param[in] v1_actual 第一点实际电压（V）
    * @param[in] v1_measured 第一点测量电压（V）
    * @param[in] v2_actual 第二点实际电压（V）
    * @param[in] v2_measured 第二点测量电压（V）
    * @param[out] k 校准系数K
    * @param[out] b 校准系数B
    * @return ESP_OK: 成功
    */
   esp_err_t adc_calibrate_two_point(float v1_actual, float v1_measured,
                                      float v2_actual, float v2_measured,
                                      float *k, float *b);
   
   /**
    * @brief 保存校准参数到NVS
    * @param[in] namespace NVS命名空间
    * @param[in] k 校准系数K
    * @param[in] b 校准系数B
    * @return ESP_OK: 成功
    */
   esp_err_t adc_save_calibration(const char *namespace, float k, float b);
   ```

2. **两点法校准算法**
   ```c
   /**
    * @brief 两点法校准计算
    * 
    * 已知两点：(x1, y1), (x2, y2)
    * 求线性方程：y = kx + b
    * 
    * k = (y2 - y1) / (x2 - x1)
    * b = y1 - k * x1
    */
   void calculate_two_point_calibration(float x1, float y1,
                                         float x2, float y2,
                                         float *k, float *b) {
       *k = (y2 - y1) / (x2 - x1);
       *b = y1 - (*k) * x1;
   }
   ```

3. **校准流程**
   ```
   1. 使用万用表测量实际电压（如54.6V和39.0V）
   2. 记录ADC测量值
   3. 计算校准系数K和B
   4. 保存到NVS
   5. 验证校准效果
   ```

4. **实现要点**
   - 校准点选择：满电和低电两个极端点
   - 避免除零错误检查
   - 校准参数合理性检查（K应接近1.0）
   - 支持恢复出厂校准
   - 校准历史记录（可选）

**验收标准**:
- [x] 两点法校准准确
- [x] 校准参数持久化
- [x] 多次校准一致性好
- [x] 错误处理完善
- [x] 单元测试通过

---

### 2.4 任务4：充电状态GPIO检测

**任务ID**: TASK-PWR-004  
**优先级**: P0  
**估算时间**: 4小时  
**依赖任务**: 无

**功能描述**:
- 配置GPIO4输入模式
- 检测充电器连接状态
- 防抖处理
- 充电状态变化回调

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化充电状态检测
    * @return ESP_OK: 成功
    */
   esp_err_t charging_status_init(void);
   
   /**
    * @brief 读取充电状态
    * @return true: 充电中，false: 未充电
    */
   bool charging_status_read(void);
   
   /**
    * @brief 注册充电状态变化回调
    * @param[in] callback 回调函数
    * @param[in] user_data 用户数据
    * @return ESP_OK: 成功
    */
   esp_err_t charging_status_register_callback(
       void (*callback)(bool is_charging, void *user_data),
       void *user_data);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 充电状态检测配置
    */
   #define CHARGING_STATUS_GPIO    GPIO_NUM_4
   #define CHARGING_DEBOUNCE_MS    100         // 防抖时间（毫秒）
   
   /**
    * @brief 充电状态结构
    */
   typedef struct {
       bool is_charging;           // 当前充电状态
       uint32_t last_change_ms;    // 上次状态变化时间
       void (*callback)(bool, void*);  // 状态变化回调
       void *user_data;            // 用户数据
   } charging_status_t;
   ```

3. **防抖算法**
   ```c
   /**
    * @brief GPIO防抖读取
    */
   bool charging_status_read_debounced(void) {
       static bool last_state = false;
       static uint32_t last_change = 0;
       
       bool current_state = gpio_get_level(CHARGING_STATUS_GPIO);
       uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
       
       if (current_state != last_state) {
           if (now - last_change > CHARGING_DEBOUNCE_MS) {
               last_state = current_state;
               last_change = now;
               // 触发回调
               if (g_charging.callback) {
                   g_charging.callback(current_state, g_charging.user_data);
               }
           }
       }
       
       return last_state;
   }
   ```

4. **实现要点**
   - GPIO配置为输入上拉模式
   - 高电平=充电中，低电平=未充电
   - 防抖时间100ms，避免误触发
   - 支持中断方式检测（可选）
   - 状态变化日志记录

**验收标准**:
- [x] 充电状态检测准确
- [x] 防抖处理有效
- [x] 回调机制正常
- [x] 无误触发
- [x] 功能测试通过

---

## 3. 驱动层任务（INA219，可选）

### 3.1 任务5：INA219初始化和配置

**任务ID**: TASK-PWR-005  
**优先级**: P2  
**估算时间**: 8小时  
**依赖任务**: 无

**功能描述**:
- 初始化I2C通信
- 配置INA219寄存器
- 设置分流电阻值
- 配置测量范围和精度

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/power/ina219_driver.h`](src/nodes/power/ina219_driver.h) - INA219驱动接口
   - [`src/nodes/power/ina219_driver.c`](src/nodes/power/ina219_driver.c) - INA219驱动实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化INA219
    * @param[out] ina INA219结构体指针
    * @param[in] i2c_port I2C端口号
    * @param[in] shunt_ohm 分流电阻值（Ω）
    * @return ESP_OK: 成功
    */
   esp_err_t ina219_init(ina219_t *ina, uint8_t i2c_port, float shunt_ohm);
   
   /**
    * @brief 配置INA219
    * @param[in] ina INA219结构体指针
    * @param[in] bus_voltage_range 总线电压范围
    * @param[in] gain PGA增益
    * @param[in] adc_resolution ADC分辨率
    * @return ESP_OK: 成功
    */
   esp_err_t ina219_config(ina219_t *ina,
                          ina219_bus_voltage_range_t bus_voltage_range,
                          ina219_gain_t gain,
                          ina219_adc_resolution_t adc_resolution);
   
   /**
    * @brief 设置校准寄存器
    * @param[in] ina INA219结构体指针
    * @param[in] max_current 最大预期电流（A）
    * @return ESP_OK: 成功
    */
   esp_err_t ina219_set_calibration(ina219_t *ina, float max_current);
   ```

3. **数据结构**
   ```c
   /**
    * @brief INA219寄存器地址
    */
   #define INA219_I2C_ADDR         0x40
   #define INA219_REG_CONFIG       0x00
   #define INA219_REG_SHUNT_V      0x01
   #define INA219_REG_BUS_V        0x02
   #define INA219_REG_POWER        0x03
   #define INA219_REG_CURRENT      0x04
   #define INA219_REG_CALIBRATION  0x05
   
   /**
    * @brief 总线电压范围
    */
   typedef enum {
       INA219_BUS_VOLTAGE_16V = 0,
       INA219_BUS_VOLTAGE_32V = 1
   } ina219_bus_voltage_range_t;
   
   /**
    * @brief PGA增益
    */
   typedef enum {
       INA219_GAIN_40MV  = 0,   // ±40mV
       INA219_GAIN_80MV  = 1,   // ±80mV
       INA219_GAIN_160MV = 2,   // ±160mV
       INA219_GAIN_320MV = 3    // ±320mV
   } ina219_gain_t;
   
   /**
    * @brief INA219结构体
    */
   typedef struct {
       uint8_t i2c_port;               // I2C端口
       uint8_t i2c_addr;               // I2C地址
       float shunt_resistor_ohm;       // 分流电阻（Ω）
       float current_lsb;              // 电流LSB
       uint16_t calibration_value;     // 校准寄存器值
   } ina219_t;
   ```

4. **配置计算**
   ```c
   /**
    * @brief 计算校准寄存器值
    * 
    * Current_LSB = Max_Expected_Current / 32768
    * Calibration = 0.04096 / (Current_LSB * Rshunt)
    */
   uint16_t calculate_calibration(float max_current, float shunt_ohm) {
       float current_lsb = max_current / 32768.0f;
       uint16_t calibration = (uint16_t)(0.04096f / (current_lsb * shunt_ohm));
       return calibration;
   }
   ```

5. **实现要点**
   - 推荐配置：32V总线，±320mV分流，12位ADC
   - 分流电阻选择0.1Ω（30A时压降3V）
   - 校准寄存器计算需考虑电流LSB
   - I2C通信超时设置100ms
   - 器件ID验证（可选）

**验收标准**:
- [x] I2C通信正常
- [x] 配置寄存器正确
- [x] 校准计算准确
- [x] 编译无警告
- [x] 功能测试通过

---

### 3.2 任务6：INA219电压电流读取

**任务ID**: TASK-PWR-006  
**优先级**: P2  
**估算时间**: 6小时  
**依赖任务**: [TASK-PWR-005]

**功能描述**:
- 读取总线电压
- 读取分流电压
- 读取电流值
- 读取功率值

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 读取电压和电流
    * @param[in] ina INA219结构体指针
    * @param[out] voltage 电压（V）
    * @param[out] current 电流（A）
    * @return ESP_OK: 成功
    */
   esp_err_t ina219_read_voltage_current(ina219_t *ina,
                                          float *voltage,
                                          float *current);
   
   /**
    * @brief 读取功率
    * @param[in] ina INA219结构体指针
    * @return 功率（W）
    */
   float ina219_read_power(ina219_t *ina);
   
   /**
    * @brief 读取分流电压
    * @param[in] ina INA219结构体指针
    * @return 分流电压（mV）
    */
   float ina219_read_shunt_voltage(ina219_t *ina);
   ```

2. **读取实现**
   ```c
   /**
    * @brief INA219电压电流读取
    */
   esp_err_t ina219_read_voltage_current(ina219_t *ina,
                                          float *voltage,
                                          float *current) {
       uint8_t buf[2];
       
       // 读取总线电压
       uint8_t reg = INA219_REG_BUS_V;
       ESP_ERROR_CHECK(i2c_master_write_read_device(
           ina->i2c_port, ina->i2c_addr,
           &reg, 1, buf, 2, pdMS_TO_TICKS(100)));
       
       int16_t bus_v_raw = (buf[0] << 8) | buf[1];
       *voltage = (bus_v_raw >> 3) * 0.004f;  // LSB=4mV
       
       // 读取电流
       reg = INA219_REG_CURRENT;
       ESP_ERROR_CHECK(i2c_master_write_read_device(
           ina->i2c_port, ina->i2c_addr,
           &reg, 1, buf, 2, pdMS_TO_TICKS(100)));
       
       int16_t current_raw = (buf[0] << 8) | buf[1];
       *current = current_raw * ina->current_lsb;
       
       return ESP_OK;
   }
   ```

3. **实现要点**
   - 总线电压LSB=4mV，需右移3位
   - 电流值根据校准寄存器计算
   - 功率=电压×电流
   - 大端序数据转换
   - I2C错误处理和重试

**验收标准**:
- [x] 电压读取准确（±4mV）
- [x] 电流读取准确（±1mA）
- [x] 功率计算正确
- [x] I2C通信稳定
- [x] 单元测试通过

---

## 4. 电池监测层

### 4.1 任务7：电压数据滤波

**任务ID**: TASK-PWR-007  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-PWR-001]

**功能描述**:
- 实现移动平均滤波
- 实现中值滤波（可选）
- 降低电压噪声
- 保持响应速度

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化电压滤波器
    * @param[in] window_size 滤波窗口大小
    * @return ESP_OK: 成功
    */
   esp_err_t voltage_filter_init(uint8_t window_size);
   
   /**
    * @
brief 应用电压滤波
    * @param[in] raw_voltage 原始电压（V）
    * @return 滤波后电压（V）
    */
   float voltage_filter_apply(float raw_voltage);
   ```

2. **移动平均滤波器**
   ```c
   /**
    * @brief 移动平均滤波器
    */
   #define VOLTAGE_FILTER_SIZE  10
   
   typedef struct {
       float buffer[VOLTAGE_FILTER_SIZE];
       uint8_t index;
       bool filled;
   } voltage_filter_t;
   
   float voltage_filter_apply(float raw_voltage) {
       static voltage_filter_t filter = {0};
       
       // 更新缓冲区
       filter.buffer[filter.index] = raw_voltage;
       filter.index = (filter.index + 1) % VOLTAGE_FILTER_SIZE;
       
       if (filter.index == 0) {
           filter.filled = true;
       }
       
       // 计算平均值
       float sum = 0;
       uint8_t count = filter.filled ? VOLTAGE_FILTER_SIZE : filter.index;
       for (uint8_t i = 0; i < count; i++) {
           sum += filter.buffer[i];
       }
       
       return sum / count;
   }
   ```

3. **实现要点**
   - 滤波窗口10个采样点
   - 冷启动处理：缓冲区未满时使用已有数据
   - 可选中值滤波抑制尖峰干扰
   - 滤波延迟约10秒（1Hz采样）
   - 静态全局变量存储滤波器状态

**验收标准**:
- [x] 噪声降低>50%
- [x] 响应延迟可接受
- [x] 无相位失真
- [x] 滤波稳定
- [x] 单元测试通过

---

### 4.2 任务8：电流数据滤波和积分

**任务ID**: TASK-PWR-008  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: [TASK-PWR-002]

**功能描述**:
- 电流数据滤波
- 电流积分计算电量消耗
- 累积电量统计
- 积分误差补偿

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化电流滤波器
    * @return ESP_OK: 成功
    */
   esp_err_t current_filter_init(void);
   
   /**
    * @brief 应用电流滤波
    * @param[in] raw_current 原始电流（A）
    * @return 滤波后电流（A）
    */
   float current_filter_apply(float raw_current);
   
   /**
    * @brief 电流积分（计算电量消耗）
    * @param[in] current 电流（A）
    * @param[in] dt 时间间隔（秒）
    * @return 电量消耗（Ah）
    */
   float current_integrate(float current, float dt);
   ```

2. **电流积分实现**
   ```c
   /**
    * @brief 库仑计法电流积分
    */
   static float g_consumed_ah = 0.0f;
   
   float current_integrate(float current, float dt) {
       // 电量消耗（Ah）= 电流（A）× 时间（h）
       float dt_h = dt / 3600.0f;  // 秒转小时
       float charge_ah = current * dt_h;
       
       // 累积消耗（仅累积放电）
       if (current < 0) {
           g_consumed_ah += fabsf(charge_ah);
       }
       
       return charge_ah;
   }
   ```

3. **实现要点**
   - 电流滤波窗口5个采样点（响应更快）
   - 时间间隔使用高精度定时器
   - 积分误差随时间累积，需定期校正
   - 充电时重置累积电量
   - 积分结果保存到NVS（断电保护）

**验收标准**:
- [x] 电流滤波有效
- [x] 积分计算准确
- [x] 累积误差<5%
- [x] 断电恢复正常
- [x] 功能测试通过

---

### 4.3 任务9：电池温度监测（可选）

**任务ID**: TASK-PWR-009  
**优先级**: P2  
**估算时间**: 6小时  
**依赖任务**: 无

**功能描述**:
- 读取电池温度（NTC或INA219内部温度）
- 温度过高报警
- 温度补偿SOC估算
- 温度历史记录

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 读取电池温度
    * @return 温度（°C）
    */
   float battery_read_temperature(void);
   
   /**
    * @brief 检查温度异常
    * @param[in] temperature 温度（°C）
    * @return 
    *   - ESP_OK: 正常
    *   - ESP_ERR_INVALID_STATE: 温度异常
    */
   esp_err_t battery_check_temperature(float temperature);
   ```

2. **温度阈值**
   ```c
   #define BATTERY_TEMP_MIN        0.0f    // 最低工作温度（°C）
   #define BATTERY_TEMP_MAX        45.0f   // 最高工作温度（°C）
   #define BATTERY_TEMP_ALARM      50.0f   // 温度报警（°C）
   ```

3. **实现要点**
   - NTC温度传感器需要ADC采样和查表
   - INA219内部温度传感器精度较低
   - 温度过高时降低最大充放电电流
   - 温度补偿SOC估算（高温容量降低）

**验收标准**:
- [x] 温度读取准确（±2°C）
- [x] 温度报警及时
- [x] 温度补偿有效
- [x] 功能测试通过

---

### 4.4 任务10：电池健康状态评估

**任务ID**: TASK-PWR-010  
**优先级**: P1  
**估算时间**: 6小时  
**依赖任务**: [TASK-PWR-007, TASK-PWR-008]

**功能描述**:
- 检测电压异常（过压、欠压）
- 检测过流保护
- 评估电池健康度
- 记录异常事件

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 检查电池健康状态
    * @param[in] voltage 电压（V）
    * @param[in] current 电流（A）
    * @param[out] health 健康状态
    * @return ESP_OK: 成功
    */
   esp_err_t battery_check_health(float voltage, float current,
                                   battery_health_t *health);
   
   /**
    * @brief 获取电池健康统计
    * @param[out] stats 健康统计
    * @return ESP_OK: 成功
    */
   esp_err_t battery_get_health_stats(battery_health_stats_t *stats);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 电池健康状态
    */
   typedef enum {
       BATTERY_HEALTH_GOOD = 0,         // 健康
       BATTERY_HEALTH_OVERVOLTAGE = 1,  // 过压
       BATTERY_HEALTH_UNDERVOLTAGE = 2, // 欠压
       BATTERY_HEALTH_OVERCURRENT = 3,  // 过流
       BATTERY_HEALTH_OVERHEAT = 4      // 过热
   } battery_health_t;
   
   /**
    * @brief 健康阈值
    */
   #define BATTERY_OVERVOLTAGE_V   55.0f   // 过压阈值
   #define BATTERY_UNDERVOLTAGE_V  38.0f   // 欠压阈值
   #define BATTERY_OVERCURRENT_A   35.0f   // 过流阈值
   ```

3. **健康检查**
   ```c
   battery_health_t check_battery_health(float voltage, float current) {
       if (voltage > BATTERY_OVERVOLTAGE_V) {
           ESP_LOGE(TAG, "Battery overvoltage: %.2fV", voltage);
           return BATTERY_HEALTH_OVERVOLTAGE;
       }
       
       if (voltage < BATTERY_UNDERVOLTAGE_V) {
           ESP_LOGE(TAG, "Battery undervoltage: %.2fV", voltage);
           return BATTERY_HEALTH_UNDERVOLTAGE;
       }
       
       if (fabsf(current) > BATTERY_OVERCURRENT_A) {
           ESP_LOGE(TAG, "Battery overcurrent: %.2fA", current);
           return BATTERY_HEALTH_OVERCURRENT;
       }
       
       return BATTERY_HEALTH_GOOD;
   }
   ```

4. **实现要点**
   - 过压保护防止电池损坏
   - 欠压保护防止过放
   - 过流保护防止短路
   - 异常时触发紧急停机
   - 健康统计记录到NVS

**验收标准**:
- [x] 异常检测准确
- [x] 保护动作及时
- [x] 日志记录完整
- [x] 功能测试通过

---

## 5. SOC估算层

### 5.1 任务11：电压-SOC查找表（LUT法）

**任务ID**: TASK-PWR-011  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-PWR-007]

**功能描述**:
- 建立电压-SOC查找表
- 线性插值计算SOC
- 支持不同电池类型配置
- 查找表校准

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/power/soc_estimator.h`](src/nodes/power/soc_estimator.h) - SOC估算接口
   - [`src/nodes/power/soc_estimator.c`](src/nodes/power/soc_estimator.c) - SOC估算实现

2. **关键函数**
   ```c
   /**
    * @brief 电压转SOC（查找表法）
    * @param[in] voltage 电压（V）
    * @return SOC（%）
    */
   float voltage_to_soc(float voltage);
   
   /**
    * @brief 线性插值
    * @param[in] x 输入值
    * @param[in] x1 点1的x坐标
    * @param[in] y1 点1的y坐标
    * @param[in] x2 点2的x坐标
    * @param[in] y2 点2的y坐标
    * @return 插值结果
    */
   float linear_interpolate(float x, float x1, float y1, float x2, float y2);
   ```

3. **数据结构**
   ```c
   /**
    * @brief 电压-SOC查找表
    */
   typedef struct {
       float voltage;  // 电压（V）
       float soc;      // SOC（%）
   } voltage_soc_lut_t;
   
   /**
    * @brief 48V锂电池（13S）电压-SOC表
    */
   static const voltage_soc_lut_t g_voltage_soc_table[] = {
       {54.6f, 100.0f},  // 满电
       {52.0f, 90.0f},
       {49.4f, 80.0f},
       {48.1f, 70.0f},
       {46.8f, 60.0f},
       {45.5f, 50.0f},
       {44.2f, 40.0f},
       {42.9f, 30.0f},
       {41.6f, 20.0f},
       {40.3f, 10.0f},
       {39.0f, 0.0f}     // 空电
   };
   ```

4. **线性插值实现**
   ```c
   float voltage_to_soc(float voltage) {
       int table_size = sizeof(g_voltage_soc_table) / sizeof(voltage_soc_lut_t);
       
       // 边界检查
       if (voltage >= g_voltage_soc_table[0].voltage) {
           return 100.0f;
       }
       if (voltage <= g_voltage_soc_table[table_size - 1].voltage) {
           return 0.0f;
       }
       
       // 查找区间并线性插值
       for (int i = 0; i < table_size - 1; i++) {
           if (voltage >= g_voltage_soc_table[i + 1].voltage) {
               float v1 = g_voltage_soc_table[i].voltage;
               float v2 = g_voltage_soc_table[i + 1].voltage;
               float soc1 = g_voltage_soc_table[i].soc;
               float soc2 = g_voltage_soc_table[i + 1].soc;
               
               return soc1 + (voltage - v1) * (soc2 - soc1) / (v2 - v1);
           }
       }
       
       return 0.0f;
   }
   ```

5. **实现要点**
   - 查找表根据实际电池特性调整
   - 静态状态（电流≈0）时精度最高
   - 负载下电压会跌落，需补偿
   - 支持配置不同电池类型
   - 查找表可存储在NVS

**验收标准**:
- [x] SOC计算准确（±5%）
- [x] 插值算法正确
- [x] 边界处理完善
- [x] 单元测试通过

---

### 5.2 任务12：库仑计法（电流积分）

**任务ID**: TASK-PWR-012  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: [TASK-PWR-008]

**功能描述**:
- 实现库仑计算法
- 电流积分计算剩余电量
- 容量追踪和更新
- 积分误差管理

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化库仑计
    * @param[out] cc 库仑计结构体
    * @param[in] capacity_ah 电池容量（Ah）
    * @param[in] initial_soc 初始SOC（%）
    * @return ESP_OK: 成功
    */
   esp_err_t coulomb_counter_init(coulomb_counter_t *cc,
                                  float capacity_ah,
                                  float initial_soc);
   
   /**
    * @brief 更新库仑计
    * @param[in] cc 库仑计结构体
    * @param[in] current 电流（A）
    * @return ESP_OK: 成功
    */
   esp_err_t coulomb_counter_update(coulomb_counter_t *cc, float current);
   
   /**
    * @brief 获取库仑计SOC
    * @param[in] cc 库仑计结构体
    * @return SOC（%）
    */
   float coulomb_counter_get_soc(coulomb_counter_t *cc);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 库仑计结构体
    */
   typedef struct {
       float capacity_ah;           // 电池容量（Ah）
       float remaining_ah;          // 剩余电量（Ah）
       float consumed_ah;           // 已消耗电量（Ah）
       uint64_t last_update_us;     // 上次更新时间（微秒）
   } coulomb_counter_t;
   ```

3. **库仑计实现**
   ```c
   esp_err_t coulomb_counter_update(coulomb_counter_t *cc, float current) {
       // 计算时间间隔（小时）
       uint64_t now_us = esp_timer_get_time();
       float dt_h = (now_us - cc->last_update_us) / 3600000000.0f;
       cc->last_update_us = now_us;
       
       // 电流积分（Ah = A × h）
       float charge_ah = current * dt_h;  // 负值=放电，正值=充电
       
       // 更新剩余电量
       cc->remaining_ah -= charge_ah;  // 放电减少，充电增加
       if (charge_ah < 0) {
           cc->consumed_ah += fabsf(charge_ah);
       }
       
       // 限制范围
       if (cc->remaining_ah > cc->capacity_ah) {
           cc->remaining_ah = cc->capacity_ah;  // 充满
       }
       if (cc->remaining_ah < 0.0f) {
           cc->remaining_ah = 0.0f;  // 耗尽
       }
       
       return ESP_OK;
   }
   
   float coulomb_counter_get_soc(coulomb_counter_t *cc) {
       return (cc->remaining_ah / cc->capacity_ah) * 100.0f;
   }
   ```

4. **实现要点**
   - 使用高精度定时器（微秒级）
   - 时间间隔转换为小时
   - 剩余电量持久化到NVS
   - 充电时更新剩余电量
   - 定期保存防止断电丢失

**验收标准**:
- [x] 积分计算准确
- [x] 时间精度高
- [x] 断电恢复正常
- [x] 长期误差<5%
- [x] 单元测试通过

---

### 5.3 任务13：混合SOC估算（电压+库仑计融合）

**任务ID**: TASK-PWR-013  
**优先级**: P0  
**估算时间**: 10小时  
**依赖任务**: [TASK-PWR-011, TASK-PWR-012]

**功能描述**:
- 融合电压法和库仑计法
- 静态/动态状态自适应切换
- 定期校正累积误差
- 提高SOC估算精度

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化SOC估算器
    * @param[out] estimator SOC估算器
    * @param[in] capacity_ah 电池容量（Ah）
    * @return ESP_OK: 成功
    */
   esp_err_t soc_estimator_init(soc_estimator_t *estimator, float capacity_ah);
   
   /**
    * @brief 更新SOC估算
    * @param[in] estimator SOC估算器
    * @param[in] voltage 电压（V）
    * @param[in] current 电流（A）
    * @return ESP_OK: 成功
    */
   esp_err_t soc_estimator_update(soc_estimator_t *estimator,
                                   float voltage,
                                   float current);
   
   /**
    * @brief 获取混合SOC
    * @param[in] estimator SOC估算器
    * @return SOC（%）
    */
   float soc_estimator_get_soc(soc_estimator_t *estimator);
   ```

2. **数据结构**
   ```c
   /**
    * @brief SOC估算器
    */
   typedef struct {
       coulomb_counter_t cc;       // 库仑计
       float voltage_soc;          // 电压法SOC
       float hybrid_soc;           // 混合SOC
       bool is_static;             // 静态状态标志
       uint32_t static_count;      // 静态状态计数
   } soc_estimator_t;
   
   /**
    * @brief 静态状态阈值
    */
   #define SOC_STATIC_CURRENT_THRESHOLD  0.5f  // 电流<0.5A视为静态
   #define SOC_STATIC_COUNT_THRESHOLD    10    // 连续10次静态才校正
   ```

3. **混合估算算法**
   ```c
   esp_err_t soc_estimator_update(soc_estimator_t *estimator,
                                   float voltage,
                                   float current) {
       // 1. 更新库仑计
       coulomb_counter_update(&estimator->cc, current);
       float cc_soc = coulomb_counter_get_soc(&estimator->cc);
       
       // 2. 更新电压SOC
       estimator->voltage_soc = voltage_to_soc(voltage);
       
       // 3. 判断静态/动态状态
       if (fabsf(current) < SOC_STATIC_CURRENT_THRESHOLD) {
           estimator->static_count++;
       } else {
           estimator->static_count = 0;
       }
       
       estimator->is_static = (estimator->static_count >= 
                               SOC_STATIC_COUNT_THRESHOLD);
       
       // 4. 融合策略
       if (estimator->is_static) {
           // 静态：使用电压SOC，并校正库仑计
           estimator->hybrid_soc = estimator->voltage_soc;
           
           // 缓慢校正库仑计（10%权重）
           float correction = (estimator->voltage_soc - cc_soc) * 0.1f;
           estimator->cc.remaining_ah += correction * 
                                         estimator->cc.capacity_ah / 100.0f;
       } else {
           // 动态：使用库仑计SOC
           estimator->hybrid_soc = cc_soc;
       }
       
       // 5. 限制范围
       if (estimator->hybrid_soc > 100.0f) estimator->hybrid_soc = 100.0f;
       if (estimator->hybrid_soc < 0.0f) estimator->hybrid_soc = 0.0f;
       
       return ESP_OK;
   }
   ```

4. **实现要点**
   - 静态判断：电流<0.5A且持续10秒
   - 动态响应：优先使用库仑计
   - 定期校正：静态时用电压法修正库仑计
   - 校正权重：10%避免突变
   - 充电检测：充满时重置SOC=100%

**验收标准**:
- [x] 混合SOC精度±3%
- [x] 静态/动态切换平滑
- [x] 累积误差校正有效
- [x] 长时间稳定性好
- [x] 单元测试通过

---

### 5.4 任务14：容量学习和校准

**任务ID**: TASK-PWR-014  
**优先级**: P1  
**估算时间**: 8小时  
**依赖任务**: [TASK-PWR-013]

**功能描述**:
- 学习电池实际容量
- 完整充放电循环记录
- 容量衰减检测
- 自适应容量更新

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 开始容量学习
    * @return ESP_OK: 成功
    */
   esp_err_t capacity_learning_start(void);
   
   /**
    * @brief 更新容量学习
    * @param[in] voltage 电压（V）
    * @param[in] current 电流（A）
    * @return ESP_OK: 成功
    */
   esp_err_t capacity_learning_update(float voltage, float current);
   
   /**
    * @brief 完成容量学习
    * @param[out] learned_capacity 学习到的容量（Ah）
    * @return ESP_OK: 成功
    */
   esp_err_t capacity_learning_finish(float *learned_capacity);
   ```

2. **容量学习流程**
   ```c
   /**
    * @brief 容量学习状态机
    */
   typedef enum {
       CAP_LEARNING_IDLE = 0,      // 空闲
       CAP_LEARNING_CHARGING = 1,  // 充电中
       CAP_LEARNING_FULL = 2,      // 充满
       CAP_LEARNING_DISCHARGING = 3, // 放电中
       CAP_LEARNING_EMPTY = 4      // 放空
   } capacity_learning_state_t;
   
   esp_err_t capacity_learning_update(float voltage, float current) {
       static capacity_learning_state_t state = CAP_LEARNING_IDLE;
       static float discharge_ah = 0.0f;
       
       switch (state) {
           case CAP_LEARNING_IDLE:
               if (current > 0.5f) {  // 开始充电
                   state = CAP_LEARNING_CHARGING;
                   discharge_ah = 0.0f;
               }
               break;
               
           case CAP_LEARNING_CHARGING:
               if (voltage >= 54.6f) {  // 充满
                   state = CAP_LEARNING_FULL;
               }
               break;
               
           case CAP_LEARNING_FULL:
               if (current < -0.5f) {  // 开始放电
                   state = CAP_LEARNING_DISCHARGING;
                   discharge_ah = 0.0f;
               }
               break;
               
           case CAP_LEARNING_DISCHARGING:
               // 累积放电量
               discharge_ah += fabsf(current) * (1.0f / 3600.0f);
               
               if (voltage <= 39.0f) {  // 放空
                   state = CAP_LEARNING_EMPTY;
                   ESP_LOGI(TAG, "Capacity learned: %.2f Ah", discharge_ah);
                   
                   // 更新电池容量
                   g_battery_capacity = discharge_ah;
                   
                   state = CAP_LEARNING_IDLE;
               }
               break;
       }
       
       return ESP_OK;
   }
   ```

3. **实现要点**
   - 需要完整充放电循环（0-100%-0）
   - 累积放电量作为实际容量
   - 容量变化>10%时更新
   - 容量保存到NVS
   - 容量衰减警告（<80%标称容量）

**验收标准**:
- [x] 容量学习准确（±2%）
- [x] 完整循环检测正确
- [x] 容量更新有效
- [x] 衰减检测准确
- [x] 功能测试通过

---

## 6. 电源管理层

### 6.1 任务15：低电量分级报警

**任务ID**: TASK-PWR-015  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-PWR-013]

**功能描述**:
- 三级低电量报警（50%/20%/10%）
- 报警状态管理
- 报警信息发布
- 防抖处理

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 检查电池电量级别
    * @param[in] soc 电量（%）
    * @return 电量级别
    */
   battery_level_t check_battery_level(float soc);
   
   /**
    * @brief 处理电量报警
    * @param[in] level 电量级别
    * @return ESP_OK: 成功
    */
   esp_err_t handle_battery_alarm(battery_level_t level);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 电池电量级别
    */
   typedef enum {
       BATTERY_LEVEL_NORMAL = 0,     // >50%
       BATTERY_LEVEL_LOW_WARN = 1,   // 20-50%
       BATTERY_LEVEL_LOW_ALARM = 2,  // 10-20%
       BATTERY_LEVEL_CRITICAL = 3    // <10%
   } battery_level_t;
   
   /**
    * @brief 电量阈值
    */
   #define BATTERY_SOC_LOW_WARN    50.0f   // 低电量警告
   #define BATTERY_SOC_LOW_ALARM   20.0f   // 低电量报警
   #define BATTERY_SOC_CRITICAL    10.0f   // 极低电量
   ```

3. **电量检查实现**
   ```c
   battery_level_t check_battery_level(float soc) {
       if (soc >= BATTERY_SOC_LOW_WARN) {
           return BATTERY_LEVEL_NORMAL;
       } else if (soc >= BATTERY_SOC_LOW_ALARM) {
           return BATTERY_LEVEL_LOW_WARN;
       } else if (soc >= BATTERY_SOC_CRITICAL) {
           return BATTERY_LEVEL_LOW_ALARM;
       } else {
           return BATTERY_LEVEL_CRITICAL;
       }
   }
   
   esp_err_t handle_battery_alarm(battery_level_t level) {
       static battery_level_t last_level = BATTERY_LEVEL_NORMAL;
       
       // 状态变化时才触发
       if (level != last_level) {
           switch (level) {
               case BATTERY_LEVEL_LOW_WARN:
                   ESP_LOGW(TAG, "Battery low warning: 20-50%%");
                   // 发布警告到ROS
                   break;
                   
               case BATTERY_LEVEL_LOW_ALARM:
                   ESP_LOGE(TAG, "Battery low alarm: 10-20%%");
                   // 降低功耗模式
                   // 发布警告到ROS
                   break;
                   
               case BATTERY_LEVEL_CRITICAL:
                   ESP_LOGE(TAG, "Battery critical: <10%%");
                   // 自动返回充电站/停止作业
                   // 发布紧急消息到ROS
                   break;
                   
               default:
                   break;
           }
           
           last_level = level;
       }
       
       return ESP_OK;
   }
   ```

4. **实现要点**
   - 状态变化时才触发报警
   - 警告级别递进提示
   - 极低电量触发保护动作
   - 报警信息通过ROS发布
   - OLED显示电量图标

**验收标准**:
- [x] 报警触发准确
- [x] 分级报警明确
- [x] 响应时间<2秒
- [x] 无误报
- [x] 功能测试通过

---

### 6.2 任务16：充电管理

**任务ID**: TASK-PWR-016  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-PWR-004, TASK-PWR-013]

**功能描述**:
- 检测充电状态
- 充满检测和保护
- 充电统计
- 充电完成通知

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 充电管理更新
    * @param[in] voltage 电压（V）
    * @param[in] current 电流（A）
    * @param[in] is_charging 充电状态
    * @return ESP_OK: 成功
    */
   esp_err_t charging_manager_update(float voltage, float current,
                                      bool is_charging);
   
   /**
    * @brief 检查充电完成
    * @param[in] voltage 电压（V）
    * @param[in] current 电流（A）
    * @return true: 充满，false: 未充满
    */
   bool check_charging_complete(float voltage, float current);
   ```

2. **充电状态机**
   ```c
   /**
    * @brief 充电状态
    */
   typedef enum {
       CHARGING_STATE_IDLE = 0,        // 空闲
       CHARGING_STATE_CHARGING = 1,    // 充电中
       CHARGING_STATE_FULL = 2,        // 充满
       CHARGING_STATE_ERROR = 3        // 充电异常
   } charging_state_t;
   
   esp_err_t charging_manager_update(float voltage, float current,
                                      bool is_charging) {
       static charging_state_t state = CHARGING_STATE_IDLE;
       
       if (!is_charging) {
           if (state != CHARGING_STATE_IDLE) {
               ESP_LOGI(TAG, "Charging stopped");
               state = CHARGING_STATE_IDLE;
           }
           return ESP_OK;
       }
       
       // 充电中
       if (state == CHARGING_STATE_IDLE) {
           ESP_LOGI(TAG, "Charging started");
           state = CHARGING_STATE_CHARGING;
       }
       
       // 检查充满
       if (state == CHARGING_STATE_CHARGING) {
           if (voltage >= 54.6f && current < 0.5f) {
               ESP_LOGI(TAG, "Charging complete");
               state = CHARGING_STATE_FULL;
               
               // 重置SOC为100%
               soc_estimator_reset(&g_soc_estimator, 100.0f);
           }
       }
       
       return ESP_OK;
   }
   ```

3. **实现要点**
   - 充满判断：电压≥54.6V且电流<0.5A
   - 充满后重置SOC=100%校正误差
   - 充电时长统计
   - 异常充电检测（电压过高）
   - 充电完成通知到ROS

**验收标准**:
- [x] 充电检测准确
- [x] 充满判断正确
- [x] SOC校正有效
- [x] 充电统计准确
- [x] 功能测试通过

---

### 6.3 任务17：剩余续航时间估算

**任务ID**: TASK-PWR-017  
**优先级**: P1  
**估算时间**: 4小时  
**依赖任务**: [TASK-PWR-013]

**功能描述**:
- 计算剩余续航时间
- 基于平均电流估算
- 动态更新预测
- 续航不足预警

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 估算剩余续航时间
    * @param[in] soc 当前SOC（%）
    * @param[in] current_avg 平均电流（A）
    * @return 续航时间（小时）
    */
   float estimate_remaining_runtime(float soc, float current_avg);
   
   /**
    * @brief 更新平均电流
    * @param[in] current 当前电流（A）
    * @return 平均电流（A）
    */
   float update_average_current(float current);
   ```

2. **续航时间计算**
   ```c
   /**
    * @brief 剩余续航时间估算
    */
   float estimate_remaining_runtime(float soc, float current_avg) {
       // 充电中或静止
       if (current_avg <= 0.0f) {
           return INFINITY;
       }
       
       // 剩余电量（Ah）
       float remaining_ah = (soc / 100.0f) * BATTERY_CAPACITY_AH;
       
       // 续航时间（小时）= 剩余电量 / 平均电流
       float runtime_h = remaining_ah / current_avg;
       
       return runtime_h;
   }
   
   /**
    * @brief 移动平均电流
    */
   float update_average_current(float current) {
       #define AVG_CURRENT_WINDOW  60  // 60秒窗口
       static float buffer[AVG_CURRENT_WINDOW] = {0};
       static uint8_t index = 0;
       
       // 仅统计放电电流
       if (current < 0) {
           buffer[index] = fabsf(current);
           index = (index + 1) % AVG_CURRENT_WINDOW;
       }
       
       // 计算平均值
       float sum = 0;
       for (int i = 0; i < AVG_CURRENT_WINDOW; i++) {
           sum += buffer[i];
       }
       
       return sum / AVG_CURRENT_WINDOW;
   }
   ```

3. **实现要点**
   - 使用60秒移动平均电流
   - 仅统计放电电流
   - 充电时返回无穷大
   - 续航<30分钟时预警
   - 续航时间显示在OLED

**验收标准**:
- [x] 续航估算合理（±10分钟）
- [x] 动态更新及时
- [x] 预警功能正常
- [x] 功能测试通过

---

## 7. 节点应用层（power_node模块）

### 7.1 任务18：节点初始化和主循环

**任务ID**: TASK-PWR-018  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: [TASK-PWR-001~017]

**功能描述**:
- 初始化所有模块
- 创建ROS节点
- 启动电池监测任务（1Hz）
- 实现主循环

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/power/power_node.h`](src/nodes/power/power_node.h) - 节点主接口
   - [`src/nodes/power/power_node.c`](src/nodes/power/power_node.c) - 节点主逻辑

2. **关键函数**
   ```c
   /**
    * @brief 初始化电源节点
    * @return ESP_OK: 成功
    */
   esp_err_t power_node_init(void);
   
   /**
    * @brief 电源节点主函数
    */
   void power_node_main(void);
   
   /**
    * @brief 停止电源节点
    * @return ESP_OK: 成功
    */
   esp_err_t power_node_stop(void);
   ```

3. **初始化流程**
   ```c
   esp_err_t power_node_init(void) {
       ESP_LOGI(TAG, "Initializing Power node...");
       
       // 1. 加载配置
       ESP_ERROR_CHECK(config_load(&g_node_config));
       
       // 2. 初始化通用模块
       ESP_ERROR_CHECK(wifi_manager_init(&g_node_config.wifi));
       ESP_ERROR_CHECK(wifi_manager_connect());
       ESP_ERROR_CHECK(ros_comm_init(&g_node_config.ros));
       ESP_ERROR_CHECK(ros_comm_connect(5000));
       
       // 3. 初始化OLED显示
       ESP_ERROR_CHECK(oled_display_init());
       ESP_ERROR_CHECK(oled_ui_create_status_screen());
       
       // 4. 初始化电源专用模块
       ESP_ERROR_CHECK(voltage_adc_init(&g_voltage_adc));
       ESP_ERROR_CHECK(current_adc_init(&g_current_adc));
       ESP_ERROR_CHECK(charging_status_init());
       
       // 5. 可选INA219初始化
       #ifdef CONFIG_POWER_USE_INA219
       ESP_ERROR_CHECK(ina219_init(&g_ina219, I2C_NUM_0, 0.1f));
       #endif
       
       // 6. 初始化SOC估算器
       ESP_ERROR_CHECK(soc_estimator_init(&g_soc_estimator, 
                                          BATTERY_CAPACITY_AH));
       
       // 7. 创建ROS接口
       ESP_ERROR_CHECK(power_create_ros_interfaces());
       
       ESP_LOGI(TAG, "Power node initialized successfully");
       return ESP_OK;
   }
   ```

4. **主循环**
   ```c
   void power_node_main(void) {
       power_node_init();
       
       // 启动监测任务（1Hz）
       xTaskCreate(battery_monitoring_task, "bat_mon", 4096, NULL, 10, NULL);
       xTaskCreate(battery_publish_task, "bat_pub", 4096, NULL, 10, NULL);
       xTaskCreate(oled_update_task, "oled_upd", 2048, NULL, 5, NULL);
       
       // 主循环：ROS spin
       while (1) {
           ros_comm_spin_once(100);
           
           // 检查连接状态
           if (!wifi_manager_is_connected()) {
               ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
               wifi_manager_connect();
           }
           
           if (!ros_comm_is_connected()) {
               ESP_LOGW(TAG, "ROS disconnected, reconnecting...");
               ros_comm_connect(5000);
           }
           
           vTaskDelay(pdMS_TO_TICKS(10));
       }
   }
   ```

**验收标准**:
- [x] 所有模块成功初始化
- [x] ROS节点正常运行
- [x] 任务创建成功
- [x] 主循环稳定
- [x] 错误恢复正常

---

### 7.2 任务19：ROS话题发布（/battery/state）

**任务ID**: TASK-PWR-019  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: [TASK-PWR-018]

**功能描述**:
- 创建电池状态发布者
- 以1Hz频率发布电池消息
- 构造sensor_msgs/BatteryState消息
- 填充所有状态字段

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 创建电池发布者
    * @return ESP_OK: 成功
    */
   esp_err_t battery_create_publisher(void);
   
   /**
    * @brief 发布电池状态
    * @param[in] state 电池状态
    * @return ESP_OK: 成功
    */
   esp_err_t battery_publish_state(const battery_state_t *state);
   ```

2. **ROS消息构造**
   ```c
   // 使用sensor_msgs/msg/BatteryState消息类型
   sensor_msgs__msg__BatteryState msg;
   
   // Header
   msg.header.stamp.sec = timestamp_us / 1000000;
   msg.header.stamp.nanosec = (timestamp_us % 1000000) * 1000;
   msg.header.frame_id = "battery_link";
   
   // 电压和电流
   msg.voltage = battery_voltage;                // 电压（V）
   msg.temperature = NAN;                        // 温度（°C），未测量
   msg.current = battery_current;                // 电流（A），负值=放电
   msg.charge = remaining_ah;                    // 剩余电量（Ah）
   msg.capacity = BATTERY_CAPACITY_AH;           // 总容量（Ah）
   msg.design_capacity = BATTERY_CAPACITY_AH;
   msg.percentage = soc / 100.0f;                // 电量百分比（0.0-1.0）
   
   // 充电状态
   if (is_charging) {
       msg.power_supply_status = POWER_SUPPLY_STATUS_CHARGING;
   } else if (soc >= 99.0f) {
       msg.power_supply_status = POWER_SUPPLY_STATUS_FULL;
   } else {
       msg.power_supply_status = POWER_SUPPLY_STATUS_DISCHARGING;
   }
   
   // 健康状态
   msg.power_supply_health = POWER_SUPPLY_HEALTH_GOOD;
   msg.power_supply_technology = POWER_SUPPLY_TECHNOLOGY_LIPO;
   msg.present = true;
   msg.location = "Main Battery";
   ```

3. **发布任务**
   ```c
   void battery_publish_task(void *pvParameters) {
       TickType_t last_wake_time = xTaskGetTickCount();
       const TickType_t frequency = pdMS_TO_TICKS(1000);  // 1Hz
       
       while (1) {
           // 获取最新电池状态
           battery_state_t state;
           battery_get_current_state(&state);
           
           // 发布到ROS
           battery_publish_state(&state);
           
           vTaskDelayUntil(&last_wake_time, frequency);
       }
   }
   ```

4. **实现要点**
   - 使用`sensor_msgs/msg/BatteryState`标准消息
   - 时间戳使用`esp_timer_get_time()`
   - percentage字段为0.0-1.0（不是0-100）
   - QoS：RELIABLE，确保数据可靠
   - 线程安全：使用互斥锁保护状态

**验收标准**:
- [x] 话题正常发布
- [x] 发布频率稳定1Hz
- [x] 消息格式正确
- [x] `ros2 topic hz`验证通过
- [x] 数据准确性验证

---

### 7.3 任务20：ROS服务实现

**任务ID**: TASK-PWR-020  
**优先级**: P1  
**估算时间**: 6小时  
**依赖任务**: [TASK-PWR-018]

**功能描述**:
- 实现校准服务（/battery/calibrate）
- 提供电池信息查询
- 支持远程触发校准
- 容量重置功能

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 创建所有服务
    * @return ESP_OK: 成功
    */
   esp_err_t battery_create_services(void);
   
   /**
    * @brief 校准服务回调
    */
   void battery_calibrate_service_callback(const void *request,
                                            void *response,
                                            void *user_data);
   ```

2. **服务定义**（需要自定义srv文件）
   ```
   # battery_interfaces/srv/Calibrate.srv
   uint8 calibration_type  # 0=电压, 1=电流零点, 2=容量
   ---
   bool success
   string message
   float32 voltage_k       # 电压校准系数K
   float32 voltage_b       # 电压校准系数B
   float32 current_zero    # 电流零点（mV）
   float32 capacity_ah     # 学习到的容量（Ah）
   ```

3. **实现要点**
   - 校准服务：支持电压、电流、容量校准
   - 校准时暂停监测任务
   - 校准完成后保存到NVS
   - 返回校准结果供用户确认
   - 错误处理完善

**验收标准**:
- [x] 服务正常响应
- [x] 校准功能正确
- [x] `ros2 service call`测试通过
- [x] 参数持久化正常

---

### 7.4 任务21：OLED实时显示

**任务ID**: TASK-PWR-021  
**优先级**: P1  
**估算时间**: 6小时  
**依赖任务**: [TASK-PWR-018]

**功能描述**:
- 显示电池电压
- 显示剩余电量（百分比和图标）
- 显示续航时间
- 显示充电状态

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 更新OLED显示
    * @return ESP_OK: 成功
    */
   esp_err_t battery_update_oled_display(void);
   ```

2. **显示布局**
   ```
   ┌────────────────────────┐
   │ POWER NODE-07          │  ← 标题
   ├────────────────────────┤
   │ ████░ 75% (48.2V)      │  ← 电量图标+百分比+电压
   │ 12.5A  Runtime: 1.2h   │  ← 电流+续航时间
   │ ⚡ Charging             │  ← 充电状态
   │ ⚠ Low Battery          │  ← 低电量警告
   └────────────────────────┘
   ```

3. **显示任务**
   ```c
   void oled_update_task(void *pvParameters) {
       char line_buf[32];
       
       while (1) {
           // 获取电池状态
           float voltage, current, soc, runtime;
           bool is_charging;
           battery_level_t level;
           
           battery_get_display_data(&voltage, &current, &soc,
                                    &runtime, &is_charging, &level);
           
           // 电量图标（0-5格）
           uint8_t bars = (uint8_t)(soc / 20.0f);
           char icon[6] = "░░░░░";
           for (uint8_t i = 0; i < bars && i < 5; i++) {
               icon[i] = '█';
           }
           
           // 电压和SOC
           snprintf(line_buf, sizeof(line_buf), 
                    "%s %.0f%% (%.1fV)", icon, soc, voltage);
           oled_ui_show_custom_text(2, line_buf);
           
           // 电流和续航
           if (is_charging) {
               snprintf(line_buf, sizeof(line_buf), 
                        "%.1fA  Charging...", current);
           } else {
               snprintf(line_buf, sizeof(line_buf), 
                        "%.1fA  Runtime: %.1fh", 
                        fabsf(current), runtime);
           }
           oled_ui_show_custom_text(3, line_buf);
           
           // 低电量警告
           if (level >= BATTERY_LEVEL_LOW_WARN) {
               oled_ui_show_alert("⚠ Low Battery", level == BATTERY_LEVEL_CRITICAL);
           }
           
           vTaskDelay(pdMS_TO_TICKS(1000));  // 1Hz更新
       }
   }
   ```

4. **实现要点**
   - 电量图标：5格电池符号（0-5格）
   - 更新频率：1Hz
   - 充电时显示闪电图标⚡
   - 低电量时红色背景闪烁
   - 布局紧凑清晰

**验收标准**:
- [x] 显示内容准确
- [x] 更新及时（1Hz）
- [x] 电量图标美观
- [x] 警告明显
- [x] CPU占用<10%

---

## 8. 集成测试

### 8.1 任务22：单元测试

**任务ID**: TASK-PWR-022  
**优先级**: P1  
**估算时间**: 12小时  
**依赖任务**: [TASK-PWR-001~021]

**功能描述**:
- ADC采样单元测试
- SOC估算算法单元测试
- 滤波器单元测试
- 校准算法单元测试
- 覆盖率>80%

**技术实现细节**:

1. **测试文件**
   - [`test/unit/test_voltage_adc.c`](test/unit/test_voltage_adc.c)
   - [`test/unit/test_current_adc.c`](test/unit/test_current_adc.c)
   - [`test/unit/test_soc_estimator.c`](test/unit/test_soc_estimator.c)
   - [`test/unit/test_battery_monitor.c`](test/unit/test_battery_monitor.c)

2. **测试用例**
   ```c
   // test_voltage_adc.c
   void test_voltage_adc_init(void);
   void test_voltage_adc_read(void);
   void test_voltage_adc_calibration(void);
   void test_voltage_division_ratio(void);
   
   // test_soc_estimator.c
   void test_voltage_to_soc_lut(void);
   void test_coulomb_counter(void);
   void test_hybrid_soc(void);
   void test_capacity_learning(void);
   
   // test_battery_monitor.c
   void test_low_battery_alarm(void);
   void test_charging_detection(void);
   void test_battery_health(void);
   ```

3. **实现要点**
   - 使用Unity测试框架
   - Mock ADC硬件接口
   - 测试边界条件
   - 浮点数比较使用容差
   - 自动化测试脚本

**验收标准**:
- [x] 所有测试用例通过
- [x] 代码覆盖率>80%
- [x] 无内存泄漏
- [x] 测试报告生成
- [x] CI集成

---

### 8.2 任务23：硬件在环测试

**任务ID**: TASK-PWR-023  
**优先级**: P0  
**估算时间**: 16小时  
**依赖任务**: [TASK-PWR-001~021]

**功能描述**:
- 实际电池硬件测试
- SOC估算精度验证
- 充放电循环测试
- 长时间稳定性测试

**技术实现细节**:

1. **测试场景**
   - 满电到空电放电测试
   - 空电到满电充电测试
   - 低电量报警功能测试
   - 充电状态检测测试
   - SOC精度验证
   - 长时间运行（24小时）

2. **测试指标**
   ```
   ┌──────────────────┬──────────┬──────────┐
   │ 测试项           │ 目标值   │ 实测值   │
   ├──────────────────┼──────────┼──────────┤
   │ 电压测量精度     │ ±100mV   │          │
   │ 电流测量精度     │ ±0.5A    │          │
   │ SOC估算精度      │ ±5%      │          │
   │ 充满检测准确性   │ 100%     │          │
   │ 低电量报警       │ <2s      │          │
   │ 采样频率         │ 1±0.1Hz  │          │
   │ 长期稳定性       │ >24h     │          │
   │ CPU占用          │ <30%     │          │
   │ 内存占用         │ <80KB    │          │
   └──────────────────┴──────────┴──────────┘
   ```

3. **测试工具**
   - 万用表：验证电压电流精度
   - 电子负载：模拟负载放电
   - 充电器：测试充电检测
   - ROS工具：`ros2 topic echo /battery/state`
   - `ros2 topic hz`：验证发布频率
   - 数据记录脚本：记录SOC变化

4. **测试步骤**
   ```bash
   # 1. 启动ROS Agent
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   
   # 2. 烧录并启动节点
   pio run -e power_node -t upload
   
   # 3. 监控电池话题
   ros2 topic echo /battery/state
   
   # 4. 验证发布频率
   ros2 topic hz /battery/state
   
   # 5. 校准测试
   ros2 service call /battery/calibrate \
       battery_interfaces/srv/Calibrate "{calibration_type: 0}"
   
   # 6. 放电测试
   # 连接电子负载，设置恒流10A放电
   # 记录SOC变化，验证精度
   
   # 7. 充电测试
   # 连接充电器，观察充电检测和充满判断
   
   # 8. 低电量报警测试
   # 放电至50%/20%/10%，验证报警触发
   ```

**验收标准**:
- [x] 所有测试场景通过
- [x] 性能指标达标
- [x] SOC精度±5%
- [x] 充放
电循环稳定
- [x] 长时间运行稳定（>24小时）
- [x] 无内存泄漏
- [x] 测试报告完整

---

## 9. 依赖关系图

### 9.1 任务依赖关系（DAG图）

```
初始化顺序（从上到下）：

1. ADC采样层（并行）
   ├─→ TASK-001: 电压ADC采样
   ├─→ TASK-002: 电流ADC采样 [并行001]
   ├─→ TASK-003: ADC多点校准 [依赖001, 002]
   └─→ TASK-004: 充电状态GPIO [并行001]

2. 驱动层（可选，并行）
   ├─→ TASK-005: INA219初始化 [并行001]
   └─→ TASK-006: INA219读取 [依赖005]

3. 电池监测层（依赖ADC采样）
   ├─→ TASK-007: 电压滤波 [依赖001]
   ├─→ TASK-008: 电流滤波积分 [依赖002]
   ├─→ TASK-009: 温度监测（可选） [并行007]
   └─→ TASK-010: 健康状态评估 [依赖007, 008]

4. SOC估算层（依赖监测层）
   ├─→ TASK-011: 电压-SOC查找表 [依赖007]
   ├─→ TASK-012: 库仑计法 [依赖008]
   ├─→ TASK-013: 混合SOC估算 [依赖011, 012]
   └─→ TASK-014: 容量学习 [依赖013]

5. 电源管理层（依赖SOC估算）
   ├─→ TASK-015: 低电量报警 [依赖013]
   ├─→ TASK-016: 充电管理 [依赖004, 013]
   └─→ TASK-017: 续航时间估算 [依赖013]

6. 节点应用层（依赖所有底层）
   ├─→ TASK-018: 节点初始化 [依赖001-017]
   ├─→ TASK-019: ROS话题发布 [依赖018]
   ├─→ TASK-020: ROS服务 [依赖018]
   └─→ TASK-021: OLED显示 [依赖018]

7. 测试层（依赖全部）
   ├─→ TASK-022: 单元测试 [依赖001-021]
   └─→ TASK-023: 硬件测试 [依赖001-021]
```

### 9.2 任务依赖矩阵

| 任务ID | 任务名称 | 依赖任务 | 可并行任务 | 估算时间 |
|--------|---------|---------|-----------|---------|
| TASK-PWR-001 | 电压ADC采样 | 无 | 002, 004, 005 | 8h |
| TASK-PWR-002 | 电流ADC采样 | 无 | 001, 004, 005 | 8h |
| TASK-PWR-003 | ADC多点校准 | 001, 002 | 005, 006 | 6h |
| TASK-PWR-004 | 充电状态GPIO | 无 | 001, 002, 005 | 4h |
| TASK-PWR-005 | INA219初始化（可选） | 无 | 001-004 | 8h |
| TASK-PWR-006 | INA219读取（可选） | 005 | 001-004 | 6h |
| TASK-PWR-007 | 电压滤波 | 001 | 008, 009 | 6h |
| TASK-PWR-008 | 电流滤波积分 | 002 | 007, 009 | 8h |
| TASK-PWR-009 | 温度监测（可选） | 无 | 007, 008 | 6h |
| TASK-PWR-010 | 健康状态评估 | 007, 008 | 009 | 6h |
| TASK-PWR-011 | 电压-SOC查找表 | 007 | 012 | 6h |
| TASK-PWR-012 | 库仑计法 | 008 | 011 | 8h |
| TASK-PWR-013 | 混合SOC估算 | 011, 012 | 014 | 10h |
| TASK-PWR-014 | 容量学习 | 013 | - | 8h |
| TASK-PWR-015 | 低电量报警 | 013 | 016, 017 | 6h |
| TASK-PWR-016 | 充电管理 | 004, 013 | 015, 017 | 6h |
| TASK-PWR-017 | 续航时间估算 | 013 | 015, 016 | 4h |
| TASK-PWR-018 | 节点初始化 | 001-017 | 022 | 8h |
| TASK-PWR-019 | ROS话题发布 | 018 | 020, 021 | 8h |
| TASK-PWR-020 | ROS服务 | 018 | 019, 021 | 6h |
| TASK-PWR-021 | OLED显示 | 018 | 019, 020 | 6h |
| TASK-PWR-022 | 单元测试 | 001-021 | 023 | 12h |
| TASK-PWR-023 | 硬件测试 | 001-021 | 022 | 16h |

### 9.3 开发里程碑

**阶段1：ADC采样层（1周）**
- 完成电压电流ADC采样（TASK-001~004）
- 目标：成功读取电池电压和电流

**阶段2：电池监测层（1周）**
- 完成滤波和健康检测（TASK-007~010）
- 目标：获得稳定准确的电池数据

**阶段3：SOC估算层（1.5周）**
- 完成SOC估算算法（TASK-011~014）
- 目标：准确计算剩余电量

**阶段4：电源管理层（1周）**
- 完成报警和充电管理（TASK-015~017）
- 目标：实现完整的电源管理功能

**阶段5：节点应用层（1.5周）**
- 完成节点框架和ROS接口（TASK-018~021）
- 目标：完整的ROS节点功能

**阶段6：测试和调优（2周）**
- 完成单元测试和硬件测试（TASK-022~023）
- 目标：稳定可靠的系统

**总计：8周（约2个月）**

### 9.4 关键路径

```
Critical Path（关键路径）:
TASK-001 → TASK-007 → TASK-011 → TASK-013 → TASK-015 → TASK-018 → TASK-023
(8h)      (6h)        (6h)        (10h)      (6h)       (8h)        (16h)

并行路径：
TASK-002 → TASK-008 → TASK-012 → TASK-013
(8h)      (8h)        (8h)        (10h)

总计：60小时（约7.5个工作日）
```

---

## 附录A：配置参数说明

### A.1 Kconfig配置项

```kconfig
menu "Power Node Configuration"
    
    config POWER_USE_INA219
        bool "Use INA219 for Voltage/Current Measurement"
        default n
        help
            Use INA219 sensor instead of ADC sampling for higher accuracy
    
    config POWER_BATTERY_CAPACITY_AH
        int "Battery Capacity (Ah)"
        range 10 50
        default 20
        help
            Nominal battery capacity in Ah
    
    config POWER_BATTERY_VOLTAGE_MAX
        int "Maximum Battery Voltage (V)"
        range 40 60
        default 54
        help
            Maximum battery voltage (13S: 54.6V, 12S: 50.4V)
    
    config POWER_BATTERY_VOLTAGE_MIN
        int "Minimum Battery Voltage (V)"
        range 30 45
        default 39
        help
            Minimum battery voltage (13S: 39.0V, 12S: 36.0V)
    
    config POWER_VOLTAGE_DIV_RATIO
        int "Voltage Divider Ratio"
        range 10 20
        default 16
        help
            Voltage divider ratio (R1+R2)/R2
    
    config POWER_SOC_LOW_WARN
        int "Low Battery Warning Threshold (%)"
        range 20 60
        default 50
        help
            Low battery warning threshold in percentage
    
    config POWER_SOC_LOW_ALARM
        int "Low Battery Alarm Threshold (%)"
        range 10 30
        default 20
        help
            Low battery alarm threshold in percentage
    
    config POWER_SOC_CRITICAL
        int "Critical Battery Threshold (%)"
        range 5 15
        default 10
        help
            Critical battery threshold in percentage

endmenu
```

### A.2 NVS配置键

| 配置键 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `voltage_k` | float | 1.0 | 电压校准系数K |
| `voltage_b` | float | 0.0 | 电压校准系数B |
| `current_zero` | float | 1650.0 | 电流零点电压（mV） |
| `current_sens` | float | 33.0 | 电流灵敏度（mV/A） |
| `battery_capacity` | float | 20.0 | 电池容量（Ah） |
| `remaining_ah` | float | 20.0 | 剩余电量（Ah） |
| `consumed_ah` | float | 0.0 | 累积消耗（Ah） |
| `calibrated` | uint8 | 0 | 校准标志 |

---

## 附录B：ROS接口总结

### B.1 发布话题

| 话题名称 | 消息类型 | QoS | 频率 | 说明 |
|---------|---------|-----|------|------|
| `/battery/state` | `sensor_msgs/BatteryState` | RELIABLE | 1Hz | 电池状态（电压、电流、SOC等） |
| `/battery/diagnostics` | `diagnostic_msgs/DiagnosticStatus` | RELIABLE | 1Hz | 诊断信息 |

### B.2 服务

| 服务名称 | 服务类型 | 说明 |
|---------|---------|------|
| `/battery/calibrate` | `battery_interfaces/Calibrate` | 电池校准（电压/电流/容量） |

### B.3 BatteryState消息字段

```
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id
float32 voltage              # 电压（V）
float32 temperature          # 温度（°C）
float32 current              # 电流（A，负值=放电）
float32 charge               # 剩余电量（Ah）
float32 capacity             # 总容量（Ah）
float32 design_capacity      # 设计容量（Ah）
float32 percentage           # 电量百分比（0.0-1.0）
uint8 power_supply_status    # 充电状态
uint8 power_supply_health    # 健康状态
uint8 power_supply_technology # 电池类型
bool present                 # 电池存在
string location              # 电池位置
```

---

## 附录C：电池管理策略

### C.1 电源管理策略表

| 电量范围 | 系统状态 | 动作 |
|---------|---------|------|
| **>50%** | 正常工作 | 无限制，全功率运行 |
| **20-50%** | 低电量预警 | 发布警告，提示充电 |
| **10-20%** | 低电量保护 | 降低功耗模式（WiFi省电、降低采样频率） |
| **<10%** | 极低电量 | 自动返回充电站/停止作业 |

### C.2 充电管理策略

| 充电阶段 | 判断条件 | 动作 |
|---------|---------|------|
| **未充电** | 充电GPIO=低 | 正常放电模式 |
| **充电中** | 充电GPIO=高，V<54.6V | 显示充电状态，SOC缓慢上升 |
| **充满** | 充电GPIO=高，V≥54.6V，I<0.5A | 重置SOC=100%，发送充满通知 |

### C.3 保护机制

| 保护项 | 阈值 | 动作 |
|--------|------|------|
| **过压保护** | >55.0V | 停止充电，发送警报 |
| **欠压保护** | <38.0V | 强制停机，防止过放 |
| **过流保护** | >35A | 限流保护，检查短路 |
| **温度保护** | >50°C | 降低电流，强制散热 |

---

## 附录D：校准流程

### D.1 电压校准流程

1. **准备工作**
   - 准备万用表（精度±10mV）
   - 电池电量充至满电（54.6V）

2. **第一点校准（满电）**
   ```bash
   # 1. 用万用表测量实际电压
   实际电压: 54.6V
   
   # 2. 读取节点测量值
   ros2 topic echo /battery/state --field voltage
   测量电压: 54.2V
   
   # 3. 记录数据
   点1: (54.2V, 54.6V)
   ```

3. **第二点校准（低电）**
   ```bash
   # 1. 放电至39.0V
   # 2. 万用表测量
   实际电压: 39.0V
   
   # 3. 读取节点测量值
   ros2 topic echo /battery/state --field voltage
   测量电压: 38.8V
   
   # 4. 记录数据
   点2: (38.8V, 39.0V)
   ```

4. **执行校准**
   ```bash
   # 调用校准服务
   ros2 service call /battery/calibrate \
       battery_interfaces/srv/Calibrate \
       "{calibration_type: 0}"
   
   # 系统自动计算K和B并保存到NVS
   ```

5. **验证校准**
   ```bash
   # 再次测量，误差应<100mV
   ros2 topic echo /battery/state --field voltage
   ```

### D.2 电流零点校准流程

1. **准备工作**
   - 断开电池所有负载
   - 确保电流为0

2. **执行校准**
   ```bash
   # 调用校准服务
   ros2 service call /battery/calibrate \
       battery_interfaces/srv/Calibrate \
       "{calibration_type: 1}"
   
   # 系统采样100次，记录零点电压
   ```

3. **验证校准**
   ```bash
   # 观察电流值，应接近0A
   ros2 topic echo /battery/state --field current
   ```

### D.3 容量学习流程

1. **充满电池**
   - 充电至54.6V
   - 等待充电完成

2. **完整放电**
   ```bash
   # 启动容量学习
   ros2 service call /battery/calibrate \
       battery_interfaces/srv/Calibrate \
       "{calibration_type: 2}"
   
   # 连接恒流负载，放电至39.0V
   # 系统自动记录放电量
   ```

3. **学习完成**
   - 系统自动更新容量
   - 保存到NVS

---

## 附录E：故障诊断

### E.1 常见问题

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 电压读数全为0 | ADC未初始化或分压电路故障 | 检查ADC配置，测试分压电路 |
| 电压波动大 | 滤波器未启用或干扰 | 启用滤波器，检查地线连接 |
| 电流读数偏差大 | 零点漂移 | 执行电流零点校准 |
| SOC跳变 | 库仑计误差累积 | 静态时用电压法校正 |
| 充电检测失败 | GPIO配置错误 | 检查GPIO4配置和接线 |
| 低电量不报警 | 阈值配置错误 | 检查Kconfig配置 |

### E.2 日志分析

```
# 正常启动日志
I (1234) POWER_NODE: Initializing Power node...
I (1245) VOLTAGE_ADC: ADC initialized, channel=0
I (1256) CURRENT_ADC: ADC initialized, channel=1
I (1267) SOC_ESTIMATOR: Capacity=20.0Ah, Initial SOC=75.0%
I (1278) POWER_NODE: Power node initialized successfully

# 低电量警告日志
W (5678) BATTERY: Low battery warning: SOC=45.0%
E (8901) BATTERY: Low battery alarm: SOC=18.0%
E (9012) BATTERY: Battery critical: SOC=8.5%

# 充电日志
I (10123) CHARGING: Charging started
I (15234) CHARGING: Charging complete, SOC reset to 100%
```

---

## 附录F：性能优化建议

1. **降低CPU占用**
   - 使用INA219减少ADC采样计算
   - 降低OLED更新频率到1Hz
   - 优化浮点数运算

2. **提高SOC精度**
   - 定期校准（每月或温度变化>10°C时）
   - 使用混合估算法
   - 容量学习自适应更新

3. **减少内存占用**
   - 使用静态分配
   - 减少滤波器缓冲区
   - 优化数据结构

4. **提高可靠性**
   - 定期保存剩余电量到NVS
   - 看门狗保护
   - 异常自动恢复

---

## 附录G：电压-SOC查找表定制

### G.1 不同电池类型的查找表

**12S锂电池（44.4V标称）**
```c
static const voltage_soc_lut_t g_voltage_soc_table_12s[] = {
    {50.4f, 100.0f},  // 满电
    {48.0f, 90.0f},
    {45.6f, 80.0f},
    {44.4f, 70.0f},
    {43.2f, 60.0f},
    {42.0f, 50.0f},
    {40.8f, 40.0f},
    {39.6f, 30.0f},
    {38.4f, 20.0f},
    {37.2f, 10.0f},
    {36.0f, 0.0f}     // 空电
};
```

**16S锂电池（59.2V标称）**
```c
static const voltage_soc_lut_t g_voltage_soc_table_16s[] = {
    {67.2f, 100.0f},
    {64.0f, 90.0f},
    {60.8f, 80.0f},
    {59.2f, 70.0f},
    {57.6f, 60.0f},
    {56.0f, 50.0f},
    {54.4f, 40.0f},
    {52.8f, 30.0f},
    {51.2f, 20.0f},
    {49.6f, 10.0f},
    {48.0f, 0.0f}
};
```

### G.2 查找表优化

根据实际测试数据调整：
1. 记录完整充放电曲线
2. 绘制电压-SOC关系图
3. 选择关键点（10个点）
4. 更新查找表

---

**文档结束**

**
编写人**：架构设计组  
**审核人**：技术负责人  
**版本**：v1.0.0  
**日期**：2025-10-23