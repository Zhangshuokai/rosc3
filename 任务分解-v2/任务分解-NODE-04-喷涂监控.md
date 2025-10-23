
# 任务分解：NODE-04喷涂监控节点

## 文档信息

| 项目 | 内容 |
|------|------|
| **文档标题** | NODE-04喷涂监控节点任务分解文档 |
| **文档版本** | v1.0.0 |
| **编制日期** | 2025-10-23 |
| **适用范围** | NODE-04喷涂监控节点 |
| **节点职责** | 喷涂作业参数监控 |
| **硬件平台** | Adafruit QT Py ESP32-C3 |

---

## 目录

- [1. 节点概述](#1-节点概述)
- [2. 驱动层任务（adc_sensors模块）](#2-驱动层任务adc_sensors模块)
- [3. 传感器转换层](#3-传感器转换层)
- [4. 监控逻辑层（spray_monitor模块）](#4-监控逻辑层spray_monitor模块)
- [5. 节点应用层（spray_node模块）](#5-节点应用层spray_node模块)
- [6. 集成测试](#6-集成测试)
- [7. 依赖关系图](#7-依赖关系图)

---

## 1. 节点概述

### 1.1 节点职责

NODE-04喷涂监控节点负责监控建筑喷涂施工机器人的喷涂作业关键参数，确保施工质量和安全。

**管理硬件**：
- 喷涂压力传感器（0-10 bar，模拟输出）
- 流量传感器（0-5 L/min，模拟输出）
- 料位传感器（超声波或电容式，模拟输出）

**主要功能**：
1. 实时监测喷涂压力（10Hz）
2. 计量涂料流量（10Hz）
3. 检测料桶余量（1Hz）
4. 异常报警（压力异常、流量异常、料位过低）
5. 施工数据记录（累计流量、施工时长）
6. OLED实时显示（压力、流量、料位）

### 1.2 硬件接口（Adafruit QT Py ESP32-C3）

| 接口类型 | 引脚分配 | 连接设备 | 说明 |
|---------|---------|---------|------|
| ADC输入 | GPIO0 | 压力传感器 | ADC1_CH0, 0-3.3V |
| ADC输入 | GPIO1 | 流量传感器 | ADC1_CH1, 0-3.3V |
| ADC输入 | GPIO2 | 料位传感器 | ADC1_CH2, 0-3.3V |
| GPIO输入 | GPIO10 | 流量脉冲（可选） | 用于高精度流量计量 |
| I2C显示 | GPIO5 (SCL), GPIO6 (SDA) | 128x64 OLED | 状态显示 |
| UART | GPIO20 (RX), GPIO21 (TX) | 调试串口 | 日志输出 |

### 1.3 性能指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| **采样频率** | 压力/流量10Hz, 料位1Hz | 根据需求规格2.4.4节 |
| **压力精度** | ±0.1 bar | 需ADC滤波和校准 |
| **流量精度** | ±0.1 L/min | 需ADC滤波和校准 |
| **料位精度** | ±5% | 传感器本身精度 |
| **ADC噪声** | <10 LSB | 通过滤波实现 |
| **响应时间** | <100ms | 从采样到报警 |

### 1.4 代码结构

```
src/nodes/spray/
├── spray_node.h            # 节点主接口
├── spray_node.c            # 节点主逻辑
├── adc_sensors.h           # ADC传感器驱动接口
├── adc_sensors.c           # ADC采样和转换
├── spray_monitor.h         # 喷涂监控逻辑接口
├── spray_monitor.c         # 异常检测和报警
└── README.md               # 节点说明
```

---

## 2. 驱动层任务（adc_sensors模块）

### 2.1 任务1：ADC初始化和多通道配置

**任务ID**: TASK-SPRAY-001  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: 无

**功能描述**:
- 初始化ESP32-C3 ADC1外设
- 配置3个ADC通道（GPIO0-2）
- 设置ADC分辨率（12位）和衰减系数（11dB）
- 实现ADC校准

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/spray/adc_sensors.h`](src/nodes/spray/adc_sensors.h) - ADC传感器驱动接口
   - [`src/nodes/spray/adc_sensors.c`](src/nodes/spray/adc_sensors.c) - ADC采样实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化ADC传感器
    * @return 
    *   - ESP_OK: 成功
    *   - ESP_ERR_INVALID_ARG: 参数无效
    *   - ESP_FAIL: 初始化失败
    */
   esp_err_t adc_sensors_init(void);
   
   /**
    * @brief 校准ADC
    * @param[in] channel ADC通道
    * @return ESP_OK: 成功
    */
   esp_err_t adc_sensors_calibrate(adc1_channel_t channel);
   
   /**
    * @brief 配置ADC通道
    * @param[in] channel ADC通道
    * @param[in] atten 衰减系数
    * @return ESP_OK: 成功
    */
   esp_err_t adc_sensors_config_channel(adc1_channel_t channel, 
                                         adc_atten_t atten);
   ```

3. **数据结构**
   ```c
   /**
    * @brief ADC通道定义
    */
   #define ADC_CHANNEL_PRESSURE    ADC1_CHANNEL_0  ///< GPIO0
   #define ADC_CHANNEL_FLOW        ADC1_CHANNEL_1  ///< GPIO1
   #define ADC_CHANNEL_LEVEL       ADC1_CHANNEL_2  ///< GPIO2
   
   /**
    * @brief ADC配置参数
    */
   #define ADC_RESOLUTION      ADC_WIDTH_BIT_12    ///< 12位分辨率
   #define ADC_ATTEN           ADC_ATTEN_DB_11     ///< 0-3.3V量程
   #define ADC_MAX_VALUE       4095                ///< 12位最大值
   #define ADC_VREF            3300                ///< 参考电压(mV)
   
   /**
    * @brief ADC传感器配置
    */
   typedef struct {
       adc1_channel_t channel;     ///< ADC通道
       adc_atten_t atten;          ///< 衰减系数
       bool calibrated;            ///< 是否已校准
       uint32_t cal_offset;        ///< 校准偏移
   } adc_sensor_config_t;
   ```

4. **实现要点**
   - 使用[`adc1_config_width()`](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/peripherals/adc.html)设置12位分辨率
   - 使用[`adc1_config_channel_atten()`](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/peripherals/adc.html)设置11dB衰减（支持0-3.3V）
   - ESP32-C3的ADC1有5个通道（GPIO0-4），本项目使用3个
   - 使用eFuse中的校准值提高精度
   - 校准时记录零点偏移（无输入时的ADC读数）

**验收标准**:
- [x] ADC成功初始化
- [x] 3个通道独立采样正常
- [x] ADC读数范围0-4095
- [x] 校准功能正常
- [x] 编译无警告

---

### 2.2 任务2：单次ADC采样和读取

**任务ID**: TASK-SPRAY-002  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-SPRAY-001]

**功能描述**:
- 实现单次ADC采样
- 读取原始ADC值
- 处理采样错误
- 实现超时机制

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 读取ADC原始值（单次采样）
    * @param[in] channel ADC通道
    * @param[out] raw_value 原始ADC值（0-4095）
    * @return 
    *   - ESP_OK: 成功
    *   - ESP_ERR_TIMEOUT: 采样超时
    *   - ESP_FAIL: 采样失败
    */
   esp_err_t adc_sensors_read_raw(adc1_channel_t channel, uint16_t *raw_value);
   
   /**
    * @brief 读取ADC电压值
    * @param[in] channel ADC通道
    * @param[out] voltage 电压值（mV）
    * @return ESP_OK: 成功
    */
   esp_err_t adc_sensors_read_voltage(adc1_channel_t channel, uint32_t *voltage);
   
   /**
    * @brief 读取所有传感器原始值
    * @param[out] pressure_raw 压力传感器ADC值
    * @param[out] flow_raw 流量传感器ADC值
    * @param[out] level_raw 料位传感器ADC值
    * @return ESP_OK: 成功
    */
   esp_err_t adc_sensors_read_all_raw(uint16_t *pressure_raw,
                                       uint16_t *flow_raw,
                                       uint16_t *level_raw);
   ```

2. **实现要点**
   - 使用[`adc1_get_raw()`](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/peripherals/adc.html)读取ADC值
   - 实现重试机制：失败时重试3次
   - 超时设置：单次采样超时100ms
   - ADC值范围检查：确保在0-4095之间
   - 批量读取优化：连续读取3个通道减少开销

**验收标准**:
- [x] ADC读取准确
- [x] 读取延迟<10ms
- [x] 错误处理正确
- [x] 批量读取性能良好
- [x] 单元测试通过

---

### 2.3 任务3：ADC滤波算法实现

**任务ID**: TASK-SPRAY-003  
**优先级**: P0  
**估算时间**: 10小时  
**依赖任务**: [TASK-SPRAY-002]

**功能描述**:
- 实现移动平均滤波
- 实现卡尔曼滤波（可选）
- 降低ADC噪声
- 提高测量稳定性

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化ADC滤波器
    * @param[in] channel ADC通道
    * @param[in] filter_size 滤波器大小
    * @return ESP_OK: 成功
    */
   esp_err_t adc_filter_init(adc1_channel_t channel, uint8_t filter_size);
   
   /**
    * @brief 更新滤波器并获取滤波后的值
    * @param[in] channel ADC通道
    * @param[in] new_value 新采样值
    * @param[out] filtered_value 滤波后的值
    * @return ESP_OK: 成功
    */
   esp_err_t adc_filter_update(adc1_channel_t channel,
                                uint16_t new_value,
                                uint16_t *filtered_value);
   
   /**
    * @brief 重置滤波器
    * @param[in] channel ADC通道
    * @return ESP_OK: 成功
    */
   esp_err_t adc_filter_reset(adc1_channel_t channel);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 移动平均滤波器配置
    */
   #define FILTER_SIZE_DEFAULT  10  ///< 默认窗口大小
   
   /**
    * @brief 移动平均滤波器状态
    */
   typedef struct {
       uint16_t buffer[16];        ///< 循环缓冲区（最大16）
       uint8_t index;              ///< 当前索引
       uint8_t size;               ///< 窗口大小
       uint32_t sum;               ///< 当前累加和
       bool filled;                ///< 缓冲区是否已填满
   } adc_filter_t;
   
   /**
    * @brief 卡尔曼滤波器状态（可选）
    */
   typedef struct {
       float x;                    ///< 状态估计
       float p;                    ///< 协方差估计
       float q;                    ///< 过程噪声
       float r;                    ///< 测量噪声
   } kalman_filter_t;
   ```

3. **滤波算法**
   ```c
   // 移动平均滤波
   uint16_t adc_moving_average_filter(adc_filter_t *filter, uint16_t new_value) {
       // 更新缓冲区
       filter->sum -= filter->buffer[filter->index];
       filter->buffer[filter->index] = new_value;
       filter->sum += new_value;
       
       // 更新索引
       filter->index = (filter->index + 1) % filter->size;
       
       // 计算平均值
       if (!filter->filled && filter->index == 0) {
           filter->filled = true;
       }
       
       uint8_t count = filter->filled ? filter->size : filter->index;
       return (uint16_t)(filter->sum / count);
   }
   
   // 卡尔曼滤波（可选，用于更高精度）
   float kalman_filter_update(kalman_filter_t *kf, float measurement) {
       // 预测
       float p_pred = kf->p + kf->q;
       
       // 更新
       float k = p_pred / (p_pred + kf->r);
       kf->x = kf->x + k * (measurement - kf->x);
       kf->p = (1 - k) * p_pred;
       
       return kf->x;
   }
   ```

4. **实现要点**
   - 移动平均滤波器：窗口大小10，降低噪声约√10倍
   - 使用循环缓冲区节省内存
   - 初始化时填充当前值，避免启动瞬变
   - 卡尔曼滤波器：Q=0.01, R=10（需实测调整）
   - 每个通道独立滤波器实例
   - 线程安全：使用互斥锁保护滤波器状态

**验收标准**:
- [x] 滤波器降噪效果明显（噪声<10 LSB）
- [x] 响应时间可接受（<200ms）
- [x] 无相位延迟问题
- [x] 内存占用<1KB
- [x] 单元测试通过

---

### 2.4 任务4：流量脉冲计数器（可选）

**任务ID**: TASK-SPRAY-004  
**优先级**: P2  
**估算时间**: 6小时  
**依赖任务**: [TASK-SPRAY-001]

**功能描述**:
- GPIO10中断计数
- 脉冲频率测量
- 累计脉冲计数
- 防抖动处理

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化脉冲计数器
    * @param[in] gpio_num GPIO引脚号
    * @return ESP_OK: 成功
    */
   esp_err_t pulse_counter_init(gpio_num_t gpio_num);
   
   /**
    * @brief 获取脉冲计数
    * @param[out] count 累计脉冲数
    * @return ESP_OK: 成功
    */
   esp_err_t pulse_counter_get_count(uint32_t *count);
   
   /**
    * @brief 重置脉冲计数
    * @return ESP_OK: 成功
    */
   esp_err_t pulse_counter_reset(void);
   
   /**
    * @brief 获取脉冲频率
    * @param[out] frequency 频率（Hz）
    * @return ESP_OK: 成功
    */
   esp_err_t pulse_counter_get_frequency(float *frequency);
   ```

2. **实现要点**
   - 使用GPIO中断模式（上升沿触发）
   - 中断服务程序中仅计数，避免长时间操作
   - 使用硬件定时器测量频率
   - 防抖动：最小脉冲间隔1ms
   - 溢出保护：使用64位计数器

**验收标准**:
- [x] 脉冲计数准确
- [x] 频率测量误差<1%
- [x] 防抖动有效
- [x] 中断响应及时
- [x] 功能测试通过

---

## 3. 传感器转换层

### 3.1 任务5：压力传感器线性转换

**任务ID**: TASK-SPRAY-005  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-SPRAY-003]

**功能描述**:
- ADC值转换为压力值
- 线性映射（0-3.3V → 0-10 bar）
- 支持多点校准
- 零点漂移补偿

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief ADC值转换为压力值
    * @param[in] adc_value ADC原始值（0-4095）
    * @param[out] pressure 压力值（bar）
    * @return ESP_OK: 成功
    */
   esp_err_t adc_to_pressure(uint16_t adc_value, float *pressure);
   
   /**
    * @brief 压力传感器校准
    * @param[in] actual_pressure 实际压力（bar）
    * @param[in] adc_value 对应ADC值
    * @return ESP_OK: 成功
    */
   esp_err_t pressure_sensor_calibrate(float actual_pressure, uint16_t adc_value);
   
   /**
    * @brief 零点校准
    * @return ESP_OK: 成功
    */
   esp_err_t pressure_sensor_zero_calibrate(void);
   ```

2. **转换公式**
   ```c
   /**
    * @brief 压力转换函数
    * 假设线性输出：V = (P / P_max) × V_ref
    * 其中：P_max = 10 bar, V_ref = 3.3V
    */
   float adc_to_pressure(uint16_t adc_value) {
       // ADC → 电压
       float voltage = (adc_value / 4095.0f) * 3.3f;
       
       // 电压 → 压力（线性映射）
       float pressure_bar = (voltage / 3.3f) * 10.0f;
       
       // 零点补偿
       pressure_bar -= g_pressure_zero_offset;
       
       // 范围限制
       if (pressure_bar < 0.0f) pressure_bar = 0.0f;
       if (pressure_bar > 10.0f) pressure_bar = 10.0f;
       
       return pressure_bar;
   }
   ```

3. **校准数据结构**
   ```c
   /**
    * @brief 压力传感器校准参数
    */
   typedef struct {
       float zero_offset;          ///< 零点偏移（bar）
       float scale_factor;         ///< 比例系数（默认1.0）
       bool calibrated;            ///< 是否已校准
       // 多点校准（可选）
       struct {
           float pressure;         ///< 标定压力
           uint16_t adc_value;     ///< 对应ADC值
       } calibration_points[5];
       uint8_t point_count;        ///< 校准点数量
   } pressure_calibration_t;
   ```

4. **实现要点**
   - 零点校准：无压力时记录ADC值作为偏移
   - 满量程校准：已知压力下记录ADC值，计算比例系数
   - 多点校准：3点以上使用线性插值
   - 校准参数保存到NVS，断电不丢失
   - 每次启动自动加载校准参数

**验收标准**:
- [x] 转换精度±0.1 bar
- [x] 零点校准有效
- [x] 满量程校准有效
- [x] 校准参数持久化
- [x] 单元测试通过

---

### 3.2 任务6：流量传感器线性转换

**任务ID**: TASK-SPRAY-006  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-SPRAY-003]

**功能描述**:
- ADC值转换为流量值
- 线性映射（0-3.3V → 0-5 L/min）
- 支持校准
- 零流量补偿

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief ADC值转换为流量值
    * @param[in] adc_value ADC原始值
    * @param[out] flow_rate 流量（L/min）
    * @return ESP_OK: 成功
    */
   esp_err_t adc_to_flow_rate(uint16_t adc_value, float *flow_rate);
   
   /**
    * @brief 流量传感器校准
    * @param[in] actual_flow 实际流量（L/min）
    * @param[in] adc_value 对应ADC值
    * @return ESP_OK: 成功
    */
   esp_err_t flow_sensor_calibrate(float actual_flow, uint16_t adc_value);
   ```

2. **转换公式**
   ```c
   /**
    * @brief 流量转换函数
    */
   float adc_to_flow_rate(uint16_t adc_value) {
       // ADC → 电压
       float voltage = (adc_value / 4095.0f) * 3.3f;
       
       // 电压 → 流量（线性映射，0-3.3V对应0-5 L/min）
       float flow_lpm = (voltage / 3.3f) * 5.0f;
       
       // 零点补偿
       flow_lpm -= g_flow_zero_offset;
       
       // 范围限制
       if (flow_lpm < 0.0f) flow_lpm = 0.0f;
       if (flow_lpm > 5.0f) flow_lpm = 5.0f;
       
       return flow_lpm;
   }
   ```

3. **实现要点**
   - 零流量校准：无流量时记录ADC值
   - 支持脉冲计数方式（如果硬件支持）：流量 = 脉冲频率 × 系数
   - 校准参数保存到NVS
   - 低流量时(<0.1 L/min)视为0

**验收标准**:
- [x] 转换精度±0.1 L/min
- [x] 零流量补偿有效
- [x] 校准功能正常
- [x] 低流量处理正确
- [x] 单元测试通过

---

### 3.3 任务7：料位传感器百分比转换

**任务ID**: TASK-SPRAY-007  
**优先级**: P0  
**估算时间**: 4小时  
**依赖任务**: [TASK-SPRAY-003]

**功能描述**:
- ADC值转换为料位百分比
- 线性映射（0-3.3V → 0-100%）
- 支持满/空校准
- 异常值过滤

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief ADC值转换为料位百分比
    * @param[in] adc_value ADC原始值
    * @param[out] level_percent 料位百分比（%）
    * @return ESP_OK: 成功
    */
   esp_err_t adc_to_level_percent(uint16_t adc_value, float *level_percent);
   
   /**
    * @brief 料位传感器校准
    * @param[in] full_adc 满料时ADC值
    * @param[in] empty_adc 空料时ADC值
    * @return ESP_OK: 成功
    */
   esp_err_t level_sensor_calibrate(uint16_t full_adc, uint16_t empty_adc);
   ```

2. **转换公式**
   ```c
   /**
    * @brief 料位转换函数
    */
   float adc_to_level_percent(uint16_t adc_value) {
       // ADC → 电压
       float voltage = (adc_value / 4095.0f) * 3.3f;
       
       // 电压 → 百分比（线性映射）
       float level_pct = (voltage / 3.3f) * 100.0f;
       
       // 使用校准参数（如果有）
       if (g_level_calibrated) {
           level_pct = ((float)adc_value - g_level_empty_adc) /
                       (g_level_full_adc - g_level_empty_adc) * 100.0f;
       }
       
       // 范围限制
       if (level_pct < 0.0f) level_pct = 0.0f;
       if (level_pct > 100.0f) level_pct = 100.0f;
       
       return level_pct;
   }
   ```

3. **实现要点**
   - 满料校准：记录满料时ADC值
   - 空料校准：记录空料时ADC值
   - 非线性传感器：使用查找表或多项式拟合
   - 异常值过滤：连续3次超出范围才更新

**验收标准**:
- [x] 转换精度±5%
- [x] 校准功能正常
- [x] 异常值过滤有效
- [x] 范围限制正确
- [x] 单元测试通过

---

## 4. 监控逻辑层（spray_monitor模块）

### 4.1 任务8：压力异常检测

**任务ID**: TASK-SPRAY-008  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-SPRAY-005]

**功能描述**:
- 实时监测压力范围
- 压力过低/过高报警
- 防抖动机制
- 报警状态管理

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/spray/spray_monitor.h`](src/nodes/spray/spray_monitor.h) - 监控逻辑接口
   - [`src/nodes/spray/spray_monitor.c`](src/nodes/spray/spray_monitor.c) - 异常检测实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化喷涂监控
    * @param[in] config 监控配置
    * @return ESP_OK: 成功
    */
   esp_err_t spray_monitor_init(const spray_monitor_config_t *config);
   
   /**
    * @brief 检测压力异常
    * @param[in] pressure 当前压力（bar）
    * @param[out] alarm 报警标志
    * @return ESP_OK: 成功
    */
   esp_err_t spray_monitor_check_pressure(float pressure, 
                                            spray_alarm_flags_t *alarm);
   
   /**
    * @brief 获取报警状态
    * @param[out] alarms 报警标志集合
    * @return ESP_OK: 成功
    */
   esp_err_t spray_monitor_get_alarms(spray_alarm_flags_t *alarms);
   
   /**
    * @brief 清除报警
    * @param[in] alarm_mask 要清除的报警掩码
    * @return ESP_OK: 成功
    */
   esp_err_t spray_monitor_clear_alarms(spray_alarm_flags_t alarm_mask);
   ```

3. **数据结构**
   ```c
   /**
    * @brief 报警标志枚举
    */
   typedef enum {
       ALARM_NONE = 0,
       ALARM_PRESSURE_LOW = (1 << 0),      ///< 压力过低
       ALARM_PRESSURE_HIGH = (1 << 1),     ///< 压力过高
       ALARM_FLOW_LOW = (1 << 2),          ///< 流量过低（堵塞）
       ALARM_LEVEL_LOW = (1 << 3),         ///< 料位过低
       ALARM_LEVEL_CRITICAL = (1 << 4)     ///< 料位危险低
   } spray_alarm_flags_t;
   
   /**
    * @brief 监控配置
    */
   typedef struct {
       // 压力阈值
       float pressure_min;         ///< 最小压力（默认2 bar）
       float pressure_max;         ///< 最大压力（默认9 bar）
       float pressure_warn_min;    ///< 警告最小压力（3 bar）
       float pressure_warn_max;    ///< 警告最大压力（8 bar）
       
       // 流量阈值
       float flow_min;             ///< 最小流量（0.2 L/min）
       float flow_warn_min;        ///< 警告流量（0.5 L/min）
       
       // 料位阈值
       float level_low;            ///< 低位阈值（10%）
       float level_critical;       ///< 危险低位（5%）
       
       // 防抖动参数
       uint8_t alarm_debounce_count;  ///< 连续N次才触发（默认3）
   } spray_monitor_config_t;
   
   /**
    * @brief 监控状态
    */
   typedef struct {
       spray_alarm_flags_t current_alarms;  ///< 当前报警
       uint8_t pressure_alarm_count;        ///< 压力报警计数
       uint8_t flow_alarm_count;            ///< 流量报警计数
       uint8_t level_alarm_count;           ///< 料位报警计数
       uint32_t last_check_time;            ///< 上次检查时间
   } spray_monitor_state_t;
   ```

4. **检测逻辑**
   ```c
   /**
    * @brief 压力异常检测逻辑
    */
   esp_err_t spray_monitor_check_pressure(float pressure, 
                                            spray_alarm_flags_t *alarm) {
       *alarm = ALARM_NONE;
       
       // 检测压力范围
       if (pressure < g_config.pressure_min || pressure > g_config.pressure_max) {
           g_state.pressure_alarm_count++;
           
           // 防抖动：连续N次才触发
           if (g_state.pressure_alarm_count >= g_config.alarm_debounce_count) {
               if (pressure < g_config.pressure_min) {
                   *alarm = ALARM_PRESSURE_LOW;
               } else {
                   *alarm = ALARM_PRESSURE_HIGH;
               }
               g_state.current_alarms |= *alarm;
               ESP_LOGE(TAG, "Pressure alarm: %.2f bar", pressure);
           }
       } else {
           // 恢复正常，清零计数
           if (g_state.pressure_alarm_count > 0) {
               g_state.pressure_alarm_count = 0;
               g_state.current_alarms &= ~(ALARM_PRESSURE_LOW | ALARM_PRESSURE_HIGH);
               ESP_LOGI(TAG, "Pressure alarm cleared");
           }
       }
       
       return ESP_OK;
   }
   ```

5. **实现要点**
   - 防抖动：连续3次检测超限才触发报警
   - 恢复正常后自动清除报警
   - 使用位掩码管理多个报警
   - 报警触发时记录时间戳和压力值
   - 线程安全：使用互斥锁保护报警状态

**验收标准**:
- [x] 压力异常检测准确
- [x] 防抖动机制有效
- [x] 报警触发及时（<100ms）
- [x] 自动清除功能正常
- [x] 单元测试通过

---

### 4.2 任务9：流量异常检测

**任务ID**: TASK-SPRAY-009  
**优先级**: P0  
**估算时间**: 4小时  
**依赖任务**: [TASK-SPRAY-006]

**功能描述**:
- 监测流量过低（堵塞）
- 流量波动检测
- 堵塞报警
- 流量趋势分析

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 检测流量异常
    * @param[in] flow_rate 当前流量（L/min）
    * @param[out] alarm 报警标志
    * @return ESP_OK: 成功
    */
   esp_err_t spray_monitor_check_flow(float flow_rate, 
                                        spray_alarm_flags_t *alarm);
   
   /**
    * @brief 分析流量趋势
    * @param[out] trend 趋势（-1下降, 0稳定, 1上升）
    * @return ESP_OK: 成功
    */
   esp_err_t spray_monitor_analyze_flow_trend(int8_t *trend);
   ```

2. **检测逻辑**
   ```c
   /**
    * @brief 流量异常检测
    */
   esp_err_t spray_monitor_check_flow(float flow_rate, 
                                        spray_alarm_flags_t *alarm) {
       *alarm = ALARM_NONE;
       
       // 仅在喷涂作业时检测（压力>阈值）
       if (g_current_pressure < g_config.pressure_warn_min) {
           return ESP_OK;  // 未喷涂，不检测流量
       }
       
       // 检测流量过低（可能堵塞）
       if (flow_rate < g_config.flow_min) {
           g_state.flow_alarm_count++;
           
           if (g_state.flow_alarm_count >= g_config.alarm_debounce_count) {
               *alarm = ALARM_FLOW_LOW;
               g_state.current_alarms |= *alarm;
               ESP_LOGE(TAG, "Flow blockage detected: %.2f L/min", flow_rate);
           }
       } else {
           if (g_state.flow_alarm_count > 0) {
               g_state.flow_alarm_count = 0;
               g_state.current_alarms &= ~ALARM_FLOW_LOW;
               ESP_LOGI(TAG, "Flow alarm cleared");
           }
       }
       
       return ESP_OK;
   }
   ```

3. **实现要点**
   - 仅在喷涂作业时检测流量（避免误报）
   - 堵塞检测：流量<0.2 L/min且压力正常
   - 趋势分析：使用滑动窗口计算导数
   - 急剧下降时预警（可能堵塞）

**验收标准**:
- [x] 堵塞检测准确
- [x] 无误报（空闲时不报警）
- [x] 趋势分析有效
- [x] 单元测试通过

---

### 4.3 任务10：料位低位报警

**任务ID**: TASK-SPRAY-010  
**优先级**: P0  
**估算时间**: 4小时  
**依赖任务**: [TASK-SPRAY-007]

**功能描述**:
- 监测料位百分比
- 低位报警（<10%）
- 危险低位报警（<5%）
- 加料提示

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 检测料位
    * @param[in] level_percent 当前料位（%）
    * @param[out] alarm 报警标志
    * @return ESP_OK: 成功
    */
   esp_err_t spray_monitor_check_level(float level_percent,
                                         spray_alarm_flags_t *alarm);
   ```

2. **检测逻辑**
   ```c
   /**
    * @brief 料位检测
    */
   esp_err_t spray_monitor_check_level(float level_percent,
                                         spray_alarm_flags_t *alarm) {
       *alarm = ALARM_NONE;
       
       // 危险低位（立即停止）
       if (level_percent < g_config.level_critical) {
           g_state.current_alarms |= ALARM_LEVEL_CRITICAL;
           *alarm = ALARM_LEVEL_CRITICAL;
           ESP_LOGE(TAG, "CRITICAL: Material level too low: %.1f%%", level_percent);
       }
       // 低位警告（提示加料）
       else if (level_percent < g_config.level_low) {
           g_state.current_alarms |= ALARM_LEVEL_LOW;
           *alarm = ALARM_LEVEL_LOW;
           ESP_LOGW(TAG, "Material level low: %.1f%%", level_percent);
       }
       // 正常
       else {
           g_state.current_alarms &= ~(ALARM_LEVEL_LOW | ALARM_LEVEL_CRITICAL);
       }
       
       return ESP_OK;
   }
   ```

3. **实现要点**
   - 两级报警：低位（10%）和危险低位（5%）
   - 危险低位时建议停止喷涂
   - 低位时提示加料但可继续作业
   - 料位恢复后自动清除报警

**验收标准**:
- [x] 料位检测准确
- [x] 两级报警正确触发
- [x] 报警清除正常
- [x] 单元测试通过

---

### 4.4 任务11：累计流量计算和记录

**任务ID**: TASK-SPRAY-011  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-SPRAY-006]

**功能描述**:
- 流量积分计算总用量
- 施工时长统计
- 数据记录到NVS
- 计数器重置功能

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 更新累计流量
    * @param[in] current_flow 当前流量（L/min）
    * @param[in] dt 时间间隔（秒）
    * @return ESP_OK: 成功
    */
   esp_err_t spray_monitor_update_total_volume(float current_flow, float dt);
   
   /**
    * @brief 获取累计流量
    * @param[out] total_volume 总流量（升）
    * @return ESP_OK: 成功
    */
   esp_err_t spray_monitor_get_total_volume(float *total_volume);
   
   /**
    * @brief 重置累计流量
    * @return ESP_OK: 成功
    */
   esp_err_t spray_monitor_reset_total_volume(void);
   
   /**
    * @brief 获取施工时长
    * @param[out] duration_sec 施工时长（秒）
    * @return ESP_OK: 成功
    */
   esp_err_t spray_monitor_get_work_duration(uint32_t *duration_sec);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 流量积分器状态
    */
   typedef struct {
       float total_volume;         ///< 累计总流量（升）
       float last_flow_rate;       ///< 上次流量（L/min）
       uint32_t last_update_time;  ///< 上次更新时间（ms）
       uint32_t work_duration;     ///< 施工时长（秒）
       bool is_spraying;           ///< 是否正在喷涂
   } flow_integrator_t;
   ```

3. **积分算法**
   ```c
   /**
    * @brief 流量积分更新
    */
   esp_err_t spray_monitor_update_total_volume(float current_flow, float dt) {
       // 梯形积分法：V = (F1 + F2) / 2 * dt
       // F1: 上次流量, F2: 当前流量, dt: 时间间隔（分钟）
       
       float dt_min = dt / 60.0f;  // 秒转分钟
       float delta_volume = (g_integrator.last_flow_rate + current_flow) / 2.0f * dt_min;
       
       g_integrator.total_volume += delta_volume;
       g_integrator.last_flow_rate = current_flow;
       g_integrator.last_update_time = esp_timer_get_time() / 1000;
       
       // 判断是否在喷涂
       if (current_flow > 0.1f) {  // 流量>0.1 L/min视为喷涂
           g_integrator.is_spraying = true;
           g_integrator.work_duration += (uint32_t)dt;
       } else {
           g_integrator.is_spraying = false;
       }
       
       // 定期保存到NVS（每10升或每小时）
       if ((uint32_t)g_integrator.total_volume % 10 == 0 ||
           g_integrator.work_duration % 3600 == 0) {
           spray_monitor_save_to_nvs();
       }
       
       return ESP_OK;
   }
   ```

4. **实现要点**
   - 使用梯形积分法提高精度
   - 采样周期：100ms（10Hz）
   - 累计数据定期保存到NVS（每10升或每小时）
   - 断电后恢复累计值
   - 提供手动重置接口（清零）

**验收标准**:
- [x] 积分计算准确（误差<5%）
- [x] 数据持久化存储
- [x] 施工时长统计正确
- [x] 重置功能正常
- [x] 单元测试通过

---

## 5. 节点应用层（spray_node模块）

### 5.1 任务12：节点初始化和主循环

**任务ID**: TASK-SPRAY-012  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: [TASK-SPRAY-008~011]

**功能描述**:
- 初始化所有模块
- 创建ROS节点
- 启动监控任务
- 实现主循环

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/spray/spray_node.h`](src/nodes/spray/spray_node.h) - 节点主接口
   - [`src/nodes/spray/spray_node.c`](src/nodes/spray/spray_node.c) - 节点主逻辑

2. **关键函数**
   ```c
   /**
    * @brief 初始化喷涂监控节点
    * @return ESP_OK: 成功
    */
   esp_err_t spray_node_init(void);
   
   /**
    * @brief 喷涂监控节点主函数
    */
   void spray_node_main(void);
   
   /**
    * @brief 停止喷涂监控节点
    * @return ESP_OK: 成功
    */
   esp_err_t spray_node_stop(void);
   ```

3. **初始化流程**
   ```c
   esp_err_t spray_node_init(void) {
       ESP_LOGI(TAG, "Initializing spray monitor node...");
       
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
       
       // 4. 初始化喷涂监控专用模块
       ESP_ERROR_CHECK(adc_sensors_init());
       ESP_ERROR_CHECK(spray_monitor_init(&g_monitor_config));
       
       // 5. 加载校准参数
       spray_load_calibration_from_nvs();
       
       // 6. 创建ROS接口
       ESP_ERROR_CHECK(spray_create_ros_interfaces());
       
       ESP_LOGI(TAG, "Spray monitor node initialized successfully");
       return ESP_OK;
   }
   ```

4. **主循环**
   ```c
   void spray_node_main(void) {
       spray_node_init();
       
       // 启动监控任务
       xTaskCreate(spray_monitor_task, "spray_mon", 4096, NULL, 10, NULL);
       xTaskCreate(spray_publish_task, "spray_pub", 4096, NULL, 10, NULL);
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

### 5.2 任务13：ROS话题发布

**任务ID**: TASK-SPRAY-013  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: [TASK-SPRAY-012]

**功能描述**:
- 发布压力话题（10Hz）
- 发布流量话题（10Hz）
- 发布料位话题（1Hz）
- 发布报警状态

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 创建ROS发布者
    * @return ESP_OK: 成功
    */
   esp_err_t spray_create_publishers(void);
   
   /**
    * @brief 发布压力数据
    * @param[in] pressure 压力值（bar转换为Pa）
    * @return ESP_OK: 成功
    */
   esp_err_t spray_publish_pressure(float pressure);
   
   /**
    * @brief 发布流量数据
    * @param[in] flow_rate 流量（L/min）
    * @return ESP_OK: 成功
    */
   esp_err_t spray_publish_flow_rate(float flow_rate);
   
   /**
    * @brief 发布料位数据
    * @param[in] level_percent 料位百分比
    * @return ESP_OK: 成功
    */
   esp_err_t spray_publish_material_level(float level_percent);
   ```

2. **ROS消息构造**
   ```c
   // 压力消息（sensor_msgs/msg/FluidPressure）
   sensor_msgs__msg__FluidPressure pressure_msg;
   pressure_msg.header.stamp = current_time;
   pressure_msg.header.frame_id = "spray_sensor";
   pressure_msg.fluid_pressure = pressure_bar * 100000.0f;  // bar → Pa
   pressure_msg.variance = 0.01f;  // 压力方差
   
   // 流量消息（std_msgs/msg/Float32）
   std_msgs__msg__Float32 flow_msg;
   flow_msg.data = flow_rate;  // L/min
   
   // 料位消息（std_msgs/msg/Float32）
   std_msgs__msg__Float32 level_msg;
   level_msg.data = level_percent;  // %
   ```

3. **发布任务**
   ```c
   void spray_publish_task(void *pvParameters) {
       TickType_t last_wake_time = xTaskGetTickCount();
       const TickType_t frequency_10hz = pdMS_TO_TICKS(100);  // 10Hz
       const TickType_t frequency_1hz = pdMS_TO_TICKS(1000);  // 1Hz
       
       uint32_t tick_count = 0;
       
       while (1) {
           // 读取传感器数据（带滤波）
           float pressure, flow_rate, level_percent;
           spray_read_all_sensors(&pressure, &flow_rate, &level_percent);
           
           // 10Hz发布：压力和流量
           spray_publish_pressure(pressure);
           spray_publish_flow_rate(flow_rate);
           
           // 1Hz发布：料位
           if (tick_count % 10 == 0) {
               spray_publish_material_level(level_percent);
           }
           
           tick_count++;
           vTaskDelayUntil(&last_wake_time, frequency_10hz);
       }
   }
   ```

4. **实现要点**
   - 压力：转换为Pa（1 bar = 100000 Pa）
   - 流量：直接使用L/min
   - 料位：百分比（0-100）
   - 使用`vTaskDelayUntil()`确保精确周期
   - 话题QoS：RELIABLE

**验收标准**:
- [x] 话题正常发布
- [x] 发布频率准确（10Hz和1Hz）
- [x] 数据格式正确
- [x] `ros2 topic hz`验证通过
- [x] RViz显示正常

---

### 5.3 任务14：ROS服务实现

**任务ID**: TASK-SPRAY-014  
**优先级**: P1  
**估算时间**: 8小时  
**依赖任务**: [TASK-SPRAY-012]

**功能描述**:
- 实现累计流量重置服务
- 实现传感器校准服务
- 实现报警确认服务
- 提供配置查询服务

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 创建所有服务
    * @return ESP_OK: 成功
    */
   esp_err_t spray_create_services(void);
   
   /**
    * @brief 重置累计流量服务回调
    */
   void spray_reset_counter_callback(const void *request,
                                       void *response,
                                       void *user_data);
   
   /**
    * @brief 传感器校准服务回调
    */
   void spray_calibrate_callback(const void *request,
                                   void *response,
                                   void *user_data);
   ```

2. **服务定义**（需要自定义srv文件）
   ```
   # spray_interfaces/srv/ResetCounter.srv
   ---
   bool success
   string message
   float32 previous_total  # 重置前的累计值
   
   # spray_interfaces/srv/Calibrate.srv
   string sensor_type      # "pressure", "flow", "level"
   float32 actual_value    # 实际值
   uint16 adc_value        # 当前ADC值
   ---
   bool success
   string message
   ```

3. **实现要点**
   - 重置服务：清零累计流量和施工时长
   - 校准服务：更新校准参数并保存到NVS
   - 报警确认：手动清除报警标志
   - 参数验证：检查校准值合理性

**验收标准**:
- [x] 服务正常响应
- [x] 重置功能正确
- [x] 校准参数生效
- [x] `ros2 service call`测试通过

---

### 5.4 任务15：OLED实时显示

**任务ID**: TASK-SPRAY-015  
**优先级**: P1  
**估算时间**: 6小时  
**依赖任务**: [TASK-SPRAY-012]

**功能描述**:
- 显示当前压力
- 显示当前流量
- 显示料位百分比
- 显示报警状态
- 显示累计流量

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 更新OLED显示
    * @return ESP_OK: 成功
    */
   esp_err_t spray_update_oled_display(void);
   ```

2. **显示布局**
   ```
   ┌────────────────────────┐
   │ SPRAY NODE-04          │  ← 标题
   ├────────────────────────┤
   │ P:5.2bar F:1.3L/min    │  ← 压力/流量
   │ Level: 45%             │  ← 料位
   │ Total: 123.5L          │  ← 累计流量
   │ ⚠ Level Low            │  ← 报警状态
   └────────────────────────┘
   ```

3. **显示任务**
   ```c
   void oled_update_task(void *pvParameters) {
       char line_buf[32];
       
       while (1) {
           // 获取传感器数据
           float pressure, flow_rate, level_percent, total_volume;
           spray_get_current_data(&pressure, &flow_rate, 
                                   &level_percent, &total_volume);
           
           // 更新显示
           snprintf(line_buf, sizeof(line_buf), "P:%.1fbar F:%.1fL/min", 
                    pressure, flow_rate);
           oled_ui_show_custom_text(2, line_buf);
           
           snprintf(line_buf, sizeof(line_buf), "Level: %.0f%%", level_percent);
           oled_ui_show_custom_text(3, line_buf);
           
           sn
printf(line_buf, sizeof(line_buf), "Total: %.1fL", total_volume);
           oled_ui_show_custom_text(4, line_buf);
           
           // 显示报警状态
           spray_alarm_flags_t alarms;
           spray_monitor_get_alarms(&alarms);
           if (alarms != ALARM_NONE) {
               if (alarms & ALARM_LEVEL_CRITICAL) {
                   oled_ui_show_alert("⚠ Level CRITICAL!", true);
               } else if (alarms & ALARM_PRESSURE_HIGH) {
                   oled_ui_show_alert("⚠ Pressure HIGH", true);
               } else if (alarms & ALARM_PRESSURE_LOW) {
                   oled_ui_show_alert("⚠ Pressure LOW", false);
               } else if (alarms & ALARM_FLOW_LOW) {
                   oled_ui_show_alert("⚠ Flow Blockage", true);
               } else if (alarms & ALARM_LEVEL_LOW) {
                   oled_ui_show_alert("⚠ Level Low", false);
               }
           }
           
           vTaskDelay(pdMS_TO_TICKS(1000));  // 1Hz更新
       }
   }
   ```

4. **实现要点**
   - 更新频率：1Hz（降低CPU占用）
   - 报警信息闪烁显示（危险级别）
   - 数值精度：压力1位小数，流量1位小数，料位整数
   - 布局紧凑，充分利用128x64空间

**验收标准**:
- [x] 显示内容准确
- [x] 更新及时（1Hz）
- [x] 报警提示明显
- [x] 布局清晰易读
- [x] CPU占用<10%

---

## 6. 集成测试

### 6.1 任务16：单元测试

**任务ID**: TASK-SPRAY-016  
**优先级**: P1  
**估算时间**: 12小时  
**依赖任务**: [TASK-SPRAY-001~015]

**功能描述**:
- ADC转换单元测试
- 滤波算法单元测试
- 异常检测单元测试
- 积分计算单元测试
- 覆盖率>80%

**技术实现细节**:

1. **测试文件**
   - [`test/unit/test_adc_sensors.c`](test/unit/test_adc_sensors.c)
   - [`test/unit/test_spray_monitor.c`](test/unit/test_spray_monitor.c)
   - [`test/unit/test_conversions.c`](test/unit/test_conversions.c)

2. **测试用例**
   ```c
   // test_adc_sensors.c
   void test_adc_init(void);
   void test_adc_read_single_channel(void);
   void test_adc_read_all_channels(void);
   void test_adc_filter_moving_average(void);
   void test_adc_calibration(void);
   
   // test_spray_monitor.c
   void test_pressure_alarm_trigger(void);
   void test_pressure_alarm_debounce(void);
   void test_flow_blockage_detection(void);
   void test_level_alarm_two_stage(void);
   void test_flow_integration(void);
   
   // test_conversions.c
   void test_adc_to_pressure_linear(void);
   void test_adc_to_flow_linear(void);
   void test_adc_to_level_percent(void);
   void test_pressure_calibration(void);
   void test_zero_offset_compensation(void);
   ```

3. **Mock硬件**
   ```c
   // Mock ADC读取
   uint16_t mock_adc_read(adc1_channel_t channel) {
       switch (channel) {
           case ADC_CHANNEL_PRESSURE:
               return g_mock_pressure_adc;
           case ADC_CHANNEL_FLOW:
               return g_mock_flow_adc;
           case ADC_CHANNEL_LEVEL:
               return g_mock_level_adc;
           default:
               return 0;
       }
   }
   ```

4. **实现要点**
   - 使用Unity测试框架
   - Mock ADC硬件接口
   - 测试边界条件（0、最大值、溢出）
   - 浮点数比较使用容差
   - 自动化测试脚本

**验收标准**:
- [x] 所有测试用例通过
- [x] 代码覆盖率>80%
- [x] 无内存泄漏
- [x] 测试报告生成
- [x] CI集成

---

### 6.2 任务17：硬件在环测试

**任务ID**: TASK-SPRAY-017  
**优先级**: P0  
**估算时间**: 16小时  
**依赖任务**: [TASK-SPRAY-001~015]

**功能描述**:
- 信号发生器模拟传感器
- 真实ADC采样测试
- 报警逻辑验证
- 长时间稳定性测试

**技术实现细节**:

1. **测试场景**
   - 压力范围扫描（0-10 bar）
   - 流量范围扫描（0-5 L/min）
   - 料位范围扫描（0-100%）
   - 压力阶跃响应（瞬间变化）
   - 流量堵塞模拟（流量骤降）
   - 料位低位报警触发
   - 长时间连续运行（8小时）

2. **测试指标**
   ```
   ┌──────────────────┬──────────┬──────────┐
   │ 测试项           │ 目标值   │ 实测值   │
   ├──────────────────┼──────────┼──────────┤
   │ 压力精度         │ ±0.1bar  │          │
   │ 流量精度         │ ±0.1L/min│          │
   │ 料位精度         │ ±5%      │          │
   │ ADC噪声(滤波后)  │ <10 LSB  │          │
   │ 采样频率         │ 10±0.5Hz │          │
   │ 报警响应时间     │ <100ms   │          │
   │ 积分误差         │ <5%      │          │
   │ CPU占用          │ <50%     │          │
   │ 内存占用         │ <120KB   │          │
   └──────────────────┴──────────┴──────────┘
   ```

3. **测试工具**
   - 信号发生器：输出0-3.3V可调电压
   - 万用表：验证电压精度
   - ROS工具：`ros2 topic echo`, `ros2 topic hz`
   - 示波器：检查ADC采样波形
   - `top`命令：监控系统资源

4. **测试步骤**
   ```bash
   # 1. 启动ROS Agent
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   
   # 2. 烧录并启动节点
   pio run -e spray_node -t upload
   
   # 3. 监控压力话题
   ros2 topic echo /spray/pressure
   
   # 4. 监控流量话题
   ros2 topic echo /spray/flow_rate
   
   # 5. 监控料位话题
   ros2 topic echo /spray/material_level
   
   # 6. 信号发生器测试
   # 设置0V → 验证压力=0 bar
   # 设置1.65V → 验证压力=5 bar
   # 设置3.3V → 验证压力=10 bar
   
   # 7. 校准测试
   ros2 service call /spray/calibrate \
       spray_interfaces/srv/Calibrate \
       "{sensor_type: 'pressure', actual_value: 5.0, adc_value: 2047}"
   
   # 8. 重置累计流量
   ros2 service call /spray/reset_counter \
       spray_interfaces/srv/ResetCounter "{}"
   ```

**验收标准**:
- [x] 所有测试场景通过
- [x] 性能指标达标
- [x] 报警逻辑正确
- [x] 长时间运行稳定（>8小时）
- [x] 无内存泄漏
- [x] 测试报告完整

---

## 7. 依赖关系图

### 7.1 任务依赖关系（DAG图）

```
初始化顺序（从上到下）：

1. 驱动层（并行）
   ├─→ TASK-001: ADC初始化
   ├─→ TASK-002: 单次采样 [依赖001]
   ├─→ TASK-003: ADC滤波 [依赖002]
   └─→ TASK-004: 脉冲计数（可选） [依赖001]

2. 传感器转换层（依赖驱动层）
   ├─→ TASK-005: 压力转换 [依赖003]
   ├─→ TASK-006: 流量转换 [依赖003]
   └─→ TASK-007: 料位转换 [依赖003]

3. 监控逻辑层（依赖转换层）
   ├─→ TASK-008: 压力异常检测 [依赖005]
   ├─→ TASK-009: 流量异常检测 [依赖006]
   ├─→ TASK-010: 料位报警 [依赖007]
   └─→ TASK-011: 累计流量 [依赖006]

4. 节点应用层（依赖所有底层）
   ├─→ TASK-012: 节点初始化 [依赖008-011]
   ├─→ TASK-013: ROS话题发布 [依赖012]
   ├─→ TASK-014: ROS服务 [依赖012]
   └─→ TASK-015: OLED显示 [依赖012]

5. 测试层（依赖全部）
   ├─→ TASK-016: 单元测试 [依赖001-015]
   └─→ TASK-017: 硬件测试 [依赖001-015]
```

### 7.2 任务依赖矩阵

| 任务ID | 任务名称 | 依赖任务 | 可并行任务 | 估算时间 |
|--------|---------|---------|-----------|---------|
| TASK-SPRAY-001 | ADC初始化 | 无 | - | 8h |
| TASK-SPRAY-002 | 单次ADC采样 | 001 | 004 | 6h |
| TASK-SPRAY-003 | ADC滤波 | 002 | 004 | 10h |
| TASK-SPRAY-004 | 脉冲计数（可选） | 001 | 002, 003 | 6h |
| TASK-SPRAY-005 | 压力转换 | 003 | 006, 007 | 6h |
| TASK-SPRAY-006 | 流量转换 | 003 | 005, 007 | 6h |
| TASK-SPRAY-007 | 料位转换 | 003 | 005, 006 | 4h |
| TASK-SPRAY-008 | 压力异常检测 | 005 | 009, 010 | 6h |
| TASK-SPRAY-009 | 流量异常检测 | 006 | 008, 010 | 4h |
| TASK-SPRAY-010 | 料位报警 | 007 | 008, 009 | 4h |
| TASK-SPRAY-011 | 累计流量 | 006 | 008, 009, 010 | 6h |
| TASK-SPRAY-012 | 节点初始化 | 008-011 | 016 | 8h |
| TASK-SPRAY-013 | ROS话题发布 | 012 | 014, 015 | 8h |
| TASK-SPRAY-014 | ROS服务 | 012 | 013, 015 | 8h |
| TASK-SPRAY-015 | OLED显示 | 012 | 013, 014 | 6h |
| TASK-SPRAY-016 | 单元测试 | 001-015 | 017 | 12h |
| TASK-SPRAY-017 | 硬件测试 | 001-015 | 016 | 16h |

### 7.3 开发里程碑

**阶段1：驱动层（1.5周）**
- 完成ADC初始化和采样（TASK-001~003）
- 完成脉冲计数（TASK-004，可选）
- 目标：ADC能够稳定采样，噪声<10 LSB

**阶段2：传感器转换层（1周）**
- 完成三种传感器转换（TASK-005~007）
- 目标：转换精度满足要求，支持校准

**阶段3：监控逻辑层（1周）**
- 完成异常检测（TASK-008~010）
- 完成累计流量（TASK-011）
- 目标：报警逻辑准确，积分误差<5%

**阶段4：节点应用层（2周）**
- 完成节点框架（TASK-012）
- 完成ROS接口（TASK-013~014）
- 完成OLED显示（TASK-015）
- 目标：完整的ROS节点功能

**阶段5：测试和调优（2周）**
- 完成单元测试（TASK-016）
- 完成硬件测试（TASK-017）
- 目标：稳定可靠的监控系统

**总计：7.5周（约2个月）**

### 7.4 关键路径

```
Critical Path（关键路径）:
TASK-001 → TASK-002 → TASK-003 → TASK-005 → TASK-008 → TASK-012 → TASK-017
(8h)      (6h)        (10h)       (6h)       (6h)       (8h)        (16h)
总计：60小时（约7.5个工作日）
```

---

## 附录A：配置参数说明

### A.1 Kconfig配置项

```kconfig
menu "Spray Monitor Configuration"
    
    config SPRAY_ADC_FILTER_SIZE
        int "ADC Filter Window Size"
        range 5 20
        default 10
        help
            Moving average filter window size
    
    config SPRAY_PRESSURE_MIN
        int "Minimum Pressure (bar x 10)"
        range 0 100
        default 20
        help
            Minimum safe pressure (2.0 bar)
    
    config SPRAY_PRESSURE_MAX
        int "Maximum Pressure (bar x 10)"
        range 0 100
        default 90
        help
            Maximum safe pressure (9.0 bar)
    
    config SPRAY_FLOW_MIN
        int "Minimum Flow Rate (L/min x 10)"
        range 0 50
        default 2
        help
            Minimum flow rate for blockage detection (0.2 L/min)
    
    config SPRAY_LEVEL_LOW
        int "Low Level Threshold (%)"
        range 0 100
        default 10
        help
            Low material level warning threshold
    
    config SPRAY_LEVEL_CRITICAL
        int "Critical Level Threshold (%)"
        range 0 50
        default 5
        help
            Critical low material level threshold

endmenu
```

### A.2 NVS配置键

| 配置键 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `spray_pressure_zero` | float | 0.0 | 压力零点偏移（bar） |
| `spray_pressure_scale` | float | 1.0 | 压力比例系数 |
| `spray_flow_zero` | float | 0.0 | 流量零点偏移（L/min） |
| `spray_flow_scale` | float | 1.0 | 流量比例系数 |
| `spray_level_full_adc` | uint16 | 4095 | 满料ADC值 |
| `spray_level_empty_adc` | uint16 | 0 | 空料ADC值 |
| `spray_total_volume` | float | 0.0 | 累计流量（升） |
| `spray_work_duration` | uint32 | 0 | 施工时长（秒） |

---

## 附录B：ROS接口总结

### B.1 发布话题

| 话题名称 | 消息类型 | QoS | 频率 | 说明 |
|---------|---------|-----|------|------|
| `/spray/pressure` | `sensor_msgs/FluidPressure` | RELIABLE | 10Hz | 喷涂压力（Pa） |
| `/spray/flow_rate` | `std_msgs/Float32` | RELIABLE | 10Hz | 流量（L/min） |
| `/spray/material_level` | `std_msgs/Float32` | RELIABLE | 1Hz | 料位百分比（%） |
| `/spray/diagnostics` | `diagnostic_msgs/DiagnosticStatus` | RELIABLE | 1Hz | 诊断信息 |

### B.2 服务

| 服务名称 | 服务类型 | 说明 |
|---------|---------|------|
| `/spray/reset_counter` | `spray_interfaces/ResetCounter` | 重置累计流量计数器 |
| `/spray/calibrate` | `spray_interfaces/Calibrate` | 传感器校准 |

---

## 附录C：监控逻辑规则

根据需求规格说明书2.4.4节，监控逻辑如下：

| 监控项 | 正常范围 | 警告条件 | 报警条件 | 处理动作 |
|--------|---------|---------|---------|---------|
| **压力** | 3-8 bar | 2-3 bar或8-9 bar | <2 bar或>9 bar | 暂停喷涂，发布ERROR |
| **流量** | 0.5-3 L/min | 0.2-0.5 L/min | <0.2 L/min | 堵塞报警 |
| **料位** | >20% | 10-20% | <10% | 提示加料 |

**防抖动策略**：
- 连续3次（300ms）检测超限才触发报警
- 恢复正常后立即清除报警
- 危险级别（压力极限、料位危险低）立即触发

---

## 附录D：校准流程

### D.1 压力传感器校准

1. **零点校准**
   ```bash
   # 确保无压力状态
   ros2 service call /spray/calibrate \
       spray_interfaces/srv/Calibrate \
       "{sensor_type: 'pressure', actual_value: 0.0, adc_value: 0}"
   ```

2. **满量程校准**
   ```bash
   # 施加已知压力（如