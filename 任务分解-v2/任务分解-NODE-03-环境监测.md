
# 任务分解：NODE-03环境监测节点

## 文档信息

| 项目 | 内容 |
|------|------|
| **文档标题** | NODE-03环境监测节点任务分解文档 |
| **文档版本** | v1.0.0 |
| **编制日期** | 2025-10-23 |
| **适用范围** | NODE-03环境监测节点 |
| **节点职责** | 施工现场环境监测 |
| **硬件平台** | Adafruit QT Py ESP32-C3 |

---

## 目录

- [1. 节点概述](#1-节点概述)
- [2. 驱动层任务（bme280_driver模块）](#2-驱动层任务bme280_driver模块)
- [3. 监测逻辑层（environment_monitor模块）](#3-监测逻辑层environment_monitor模块)
- [4. 节点应用层（environment_node模块）](#4-节点应用层environment_node模块)
- [5. 集成测试](#5-集成测试)
- [6. 依赖关系图](#6-依赖关系图)

---

## 1. 节点概述

### 1.1 节点职责

NODE-03环境监测节点负责监测建筑喷涂施工现场的环境参数，为施工决策提供数据支持。

**管理硬件**：
- 温湿度传感器（BME280或SHT3x）
- 气压传感器（BME280集成）

**主要功能**：
1. 定期采集环境温度、湿度、气压
2. 发布环境数据到ROS话题（`/environment/temperature`、`/environment/humidity`、`/environment/pressure`）
3. 施工适宜性判断（温度5-35°C，湿度40-85%）
4. 超限报警（温度<0°C或>40°C，湿度>90%）
5. 数据记录（用于施工记录）
6. OLED显示环境参数和适宜性

### 1.2 硬件接口（Adafruit QT Py ESP32-C3）

| 接口类型 | 引脚分配 | 连接设备 | 说明 |
|---------|---------|---------|------|
| I2C总线 | GPIO8 (SDA), GPIO9 (SCL) | BME280传感器 | I2C地址0x76或0x77 |
| I2C显示 | GPIO5 (SCL), GPIO6 (SDA) | 128x64 OLED | 状态显示 |
| UART | GPIO20 (RX), GPIO21 (TX) | 调试串口 | 日志输出 |

### 1.3 性能指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| **采样频率** | 0.5 Hz | 每2秒采样一次 |
| **温度精度** | ±0.1°C | 需校准 |
| **湿度精度** | ±2% RH | BME280典型精度±3% |
| **气压精度** | ±1 hPa | BME280典型精度 |
| **响应时间** | <100ms | 传感器读取延迟 |

### 1.4 代码结构

```
src/nodes/environment/
├── environment_node.h      # 节点主接口
├── environment_node.c      # 节点主逻辑
├── bme280_driver.h         # BME280驱动接口
├── bme280_driver.c         # I2C BME280驱动实现
├── environment_monitor.h   # 环境监测逻辑接口
├── environment_monitor.c   # 适宜性判断和报警
└── README.md               # 节点说明
```

---

## 2. 驱动层任务（bme280_driver模块）

### 2.1 任务1：BME280传感器I2C驱动初始化

**任务ID**: TASK-ENV-001  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: 无

**功能描述**:
- 初始化I2C总线
- 检测BME280传感器（Chip ID验证）
- 读取出厂校准参数（dig_T1~dig_H6）
- 配置传感器工作模式

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/environment/bme280_driver.h`](src/nodes/environment/bme280_driver.h) - BME280驱动接口定义
   - [`src/nodes/environment/bme280_driver.c`](src/nodes/environment/bme280_driver.c) - BME280驱动实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化BME280传感器
    * @return 
    *   - ESP_OK: 成功
    *   - ESP_ERR_NOT_FOUND: 传感器未检测到
    *   - ESP_FAIL: 初始化失败
    */
   esp_err_t bme280_init(void);
   
   /**
    * @brief 读取Chip ID
    * @param[out] chip_id Chip ID（应为0x60）
    * @return ESP_OK: 成功
    */
   esp_err_t bme280_read_chip_id(uint8_t *chip_id);
   
   /**
    * @brief 读取校准参数
    * @param[out] calib_data 校准数据结构体
    * @return ESP_OK: 成功
    */
   esp_err_t bme280_read_calibration_data(bme280_calib_data_t *calib_data);
   
   /**
    * @brief 配置传感器
    * @param[in] config 配置参数
    * @return ESP_OK: 成功
    */
   esp_err_t bme280_configure(const bme280_config_t *config);
   ```

3. **数据结构**
   ```c
   /**
    * @brief BME280 I2C地址
    */
   #define BME280_I2C_ADDR_PRIMARY   0x76  ///< SDO接GND
   #define BME280_I2C_ADDR_SECONDARY 0x77  ///< SDO接VCC
   
   /**
    * @brief BME280寄存器地址
    */
   #define BME280_REG_CHIP_ID      0xD0  ///< Chip ID寄存器
   #define BME280_REG_RESET        0xE0  ///< 软复位寄存器
   #define BME280_REG_CTRL_HUM     0xF2  ///< 湿度控制寄存器
   #define BME280_REG_STATUS       0xF3  ///< 状态寄存器
   #define BME280_REG_CTRL_MEAS    0xF4  ///< 测量控制寄存器
   #define BME280_REG_CONFIG       0xF5  ///< 配置寄存器
   #define BME280_REG_PRESS_MSB    0xF7  ///< 气压数据起始
   #define BME280_REG_TEMP_MSB     0xFA  ///< 温度数据起始
   #define BME280_REG_HUM_MSB      0xFD  ///< 湿度数据起始
   
   /**
    * @brief BME280校准数据结构体
    */
   typedef struct {
       // 温度校准参数
       uint16_t dig_T1;
       int16_t dig_T2;
       int16_t dig_T3;
       
       // 气压校准参数
       uint16_t dig_P1;
       int16_t dig_P2;
       int16_t dig_P3;
       int16_t dig_P4;
       int16_t dig_P5;
       int16_t dig_P6;
       int16_t dig_P7;
       int16_t dig_P8;
       int16_t dig_P9;
       
       // 湿度校准参数
       uint8_t dig_H1;
       int16_t dig_H2;
       uint8_t dig_H3;
       int16_t dig_H4;
       int16_t dig_H5;
       int8_t dig_H6;
   } bme280_calib_data_t;
   
   /**
    * @brief BME280配置结构体
    */
   typedef struct {
       uint8_t i2c_addr;           ///< I2C地址（0x76或0x77）
       uint8_t mode;               ///< 工作模式（SLEEP/FORCED/NORMAL）
       uint8_t osrs_t;             ///< 温度过采样（0-5）
       uint8_t osrs_p;             ///< 气压过采样（0-5）
       uint8_t osrs_h;             ///< 湿度过采样（0-5）
       uint8_t filter;             ///< IIR滤波器系数（0-4）
       uint8_t standby_time;       ///< 待机时间（0-7）
   } bme280_config_t;
   
   /**
    * @brief 默认配置（适用于环境监测）
    */
   #define BME280_MODE_NORMAL      0x03  ///< 正常模式（连续测量）
   #define BME280_OSRS_X1          0x01  ///< 过采样×1
   #define BME280_OSRS_X2          0x02  ///< 过采样×2
   #define BME280_OSRS_X16         0x05  ///< 过采样×16
   #define BME280_FILTER_COEFF_16  0x04  ///< IIR滤波器系数16
   #define BME280_STANDBY_2000MS   0x06  ///< 待机时间2000ms
   ```

4. **实现要点**
   - 使用通用I2C总线模块（[`src/drivers/i2c_bus/i2c_bus.c`](src/drivers/i2c_bus/i2c_bus.c)）
   - 初始化时先软复位（写0xB6到RESET寄存器）
   - 读取Chip ID验证（应返回0x60，BME280特征值）
   - 从0x88-0xA1和0xE1-0xF0读取校准参数（26+7=33字节）
   - 配置顺序：先设置ctrl_hum，再设置ctrl_meas（触发生效）
   - 推荐配置：过采样×2（温湿度）、×16（气压）、IIR滤波系数16
   - I2C超时设置为100ms

**验收标准**:
- [x] 成功检测BME280传感器（Chip ID = 0x60）
- [x] 校准参数读取正确
- [x] 配置写入成功
- [x] I2C通信稳定（错误率<1%）
- [x] 编译无警告

---

### 2.2 任务2：温度读取和补偿计算

**任务ID**: TASK-ENV-002  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-ENV-001]

**功能描述**:
- 读取温度ADC原始值（20位）
- 使用校准参数进行补偿计算
- 转换为摄氏度（°C）
- 处理异常值

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 读取温度原始数据
    * @param[out] adc_T 温度ADC值（20位）
    * @return ESP_OK: 成功
    */
   esp_err_t bme280_read_temperature_raw(int32_t *adc_T);
   
   /**
    * @brief 温度补偿计算
    * @param[in] adc_T 温度ADC值
    * @param[in] calib_data 校准数据
    * @param[out] t_fine 精细温度值（用于其他补偿）
    * @return 温度值（°C × 100，如2534表示25.34°C）
    */
   int32_t bme280_compensate_temperature(int32_t adc_T, 
                                          const bme280_calib_data_t *calib_data,
                                          int32_t *t_fine);
   
   /**
    * @brief 读取温度（高层接口）
    * @param[out] temperature 温度值（°C）
    * @return ESP_OK: 成功
    */
   esp_err_t bme280_read_temperature(float *temperature);
   ```

2. **实现要点**
   - 从0xFA-0xFC读取3字节温度数据
   - 组合为20位ADC值：`adc_T = (msb << 12) | (lsb << 4) | (xlsb >> 4)`
   - 使用补偿公式计算（参考BME280数据手册）
   - t_fine值需保存，用于气压和湿度补偿
   - 转换为浮点数：`temp_C = (float)T / 100.0f`
   - 异常检测：温度超出-40~85°C范围时返回错误

**验收标准**:
- [x] 温度读取准确（与实际温度误差<1°C）
- [x] 补偿算法正确实现
- [x] 异常值正确处理
- [x] 单元测试通过
- [x] 编译无警告

---

### 2.3 任务3：湿度读取和补偿计算

**任务ID**: TASK-ENV-003  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-ENV-001, TASK-ENV-002]

**功能描述**:
- 读取湿度ADC原始值（16位）
- 使用校准参数和t_fine进行补偿
- 转换为相对湿度（%RH）
- 范围限制（0-100%）

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 读取湿度原始数据
    * @param[out] adc_H 湿度ADC值（16位）
    * @return ESP_OK: 成功
    */
   esp_err_t bme280_read_humidity_raw(int32_t *adc_H);
   
   /**
    * @brief 湿度补偿计算
    * @param[in] adc_H 湿度ADC值
    * @param[in] t_fine 精细温度值（来自温度补偿）
    * @param[in] calib_data 校准数据
    * @return 湿度值（%RH × 1024，如51200表示50.0%）
    */
   uint32_t bme280_compensate_humidity(int32_t adc_H,
                                        int32_t t_fine,
                                        const bme280_calib_data_t *calib_data);
   
   /**
    * @brief 读取湿度（高层接口）
    * @param[out] humidity 湿度值（%RH）
    * @return ESP_OK: 成功
    */
   esp_err_t bme280_read_humidity(float *humidity);
   ```

2. **实现要点**
   - 从0xFD-0xFE读取2字节湿度数据
   - 组合为16位ADC值：`adc_H = (msb << 8) | lsb`
   - 必须先计算温度获得t_fine，再计算湿度
   - 使用补偿公式计算（参考BME280数据手册）
   - 转换为浮点数：`humidity = (float)H / 1024.0f`
   - 范围限制：湿度值限制在0-100%之间

**验收标准**:
- [x] 湿度读取准确（误差<3%）
- [x] 补偿算法正确实现
- [x] 范围限制生效
- [x] 依赖t_fine正确处理
- [x] 单元测试通过

---

### 2.4 任务4：气压读取和补偿计算

**任务ID**: TASK-ENV-004  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-ENV-001, TASK-ENV-002]

**功能描述**:
- 读取气压ADC原始值（20位）
- 使用校准参数和t_fine进行补偿
- 转换为气压值（Pa）
- 海拔高度补偿（可选）

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 读取气压原始数据
    * @param[out] adc_P 气压ADC值（20位）
    * @return ESP_OK: 成功
    */
   esp_err_t bme280_read_pressure_raw(int32_t *adc_P);
   
   /**
    * @brief 气压补偿计算
    * @param[in] adc_P 气压ADC值
    * @param[in] t_fine 精细温度值（来自温度补偿）
    * @param[in] calib_data 校准数据
    * @return 气压值（Pa，如101325表示1013.25 hPa）
    */
   uint32_t bme280_compensate_pressure(int32_t adc_P,
                                        int32_t t_fine,
                                        const bme280_calib_data_t *calib_data);
   
   /**
    * @brief 读取气压（高层接口）
    * @param[out] pressure 气压值（Pa）
    * @return ESP_OK: 成功
    */
   esp_err_t bme280_read_pressure(float *pressure);
   
   /**
    * @brief 读取所有传感器数据
    * @param[out] temperature 温度（°C）
    * @param[out] humidity 湿度（%RH）
    * @param[out] pressure 气压（Pa）
    * @return ESP_OK: 成功
    */
   esp_err_t bme280_read_all(float *temperature, float *humidity, float *pressure);
   ```

2. **实现要点**
   - 从0xF7-0xF9读取3字节气压数据
   - 组合为20位ADC值：`adc_P = (msb << 12) | (lsb << 4) | (xlsb >> 4)`
   - 必须先计算温度获得t_fine，再计算气压
   - 使用64位整数避免溢出
   - 气压范围验证：80000-110000 Pa（800-1100 hPa）
   - [`bme280_read_all()`](src/nodes/environment/bme280_driver.c)函数一次性读取所有数据（优化性能）

**验收标准**:
- [x] 气压读取准确（误差<1 hPa）
- [x] 补偿算法正确实现
- [x] 避免整数溢出
- [x] 批量读取功能正常
- [x] 单元测试通过

---

## 3. 监测逻辑层（environment_monitor模块）

### 3.1 任务5：施工适宜性判断算法

**任务ID**: TASK-ENV-005  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-ENV-002, TASK-ENV-003]

**功能描述**:
- 实现施工适宜性判断逻辑
- 根据温湿度范围判断施工条件
- 提供三级适宜性评估（适宜/警告/不适宜）
- 支持阈值配置

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/environment/environment_monitor.h`](src/nodes/environment/environment_monitor.h) - 监测逻辑接口
   - [`src/nodes/environment/environment_monitor.c`](src/nodes/environment/environment_monitor.c) - 适宜性判断实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化环境监测
    * @param[in] threshold 阈值配置
    * @return ESP_OK: 成功
    */
   esp_err_t environment_monitor_init(const environment_threshold_t *threshold);
   
   /**
    * @brief 判断施工适宜性
    * @param[in] temperature 当前温度（°C）
    * @param[in] humidity 当前湿度（%）
    * @return 适宜性等级
    */
   environment_suitability_t environment_judge_suitability(float temperature, 
                                                           float humidity);
   
   /**
    * @brief 获取适宜性描述
    * @param[in] suitability 适宜性等级
    * @return 描述字符串
    */
   const char* environment_get_suitability_desc(environment_suitability_t suitability);
   
   /**
    * @brief 设置阈值
    * @param[in] threshold 新阈值
    * @return ESP_OK: 成功
    */
   esp_err_t environment_set_threshold(const environment_threshold_t *threshold);
   ```

3. **数据结构**
   ```c
   /**
    * @brief 施工适宜性枚举
    */
   typedef enum {
       ENV_SUITABLE = 0,        ///< 适宜施工（绿色）
       ENV_WARNING = 1,         ///< 可施工但需注意（黄色）
       ENV_UNSUITABLE = 2       ///< 不适宜施工（红色）
   } environment_suitability_t;
   
   /**
    * @brief 环境阈值配置
    */
   typedef struct {
       // 适宜温度范围
       float temp_suitable_min;     ///< 适宜最低温度（默认5°C）
       float temp_suitable_max;     ///< 适宜最高温度（默认35°C）
       
       // 警告温度范围
       float temp_warning_min;      ///< 警告最低温度（默认0°C）
       float temp_warning_max;      ///< 警告最高温度（默认40°C）
       
       // 适宜湿度范围
       float humidity_suitable_min; ///< 适宜最低湿度（默认40%）
       float humidity_suitable_max; ///< 适宜最高湿度（默认85%）
       
       // 警告湿度范围
       float humidity_warning_min;  ///< 警告最低湿度（默认30%）
       float humidity_warning_max;  ///< 警告最高湿度（默认90%）
   } environment_threshold_t;
   
   /**
    * @brief 默认阈值（参考需求规格说明书2.3.4节）
    */
   #define ENV_TEMP_SUITABLE_MIN    5.0f   ///< 5°C
   #define ENV_TEMP_SUITABLE_MAX    35.0f  ///< 35°C
   #define ENV_TEMP_WARNING_MIN     0.0f   ///< 0°C
   #define ENV_TEMP_WARNING_MAX     40.0f  ///< 40°C
   
   #define ENV_HUMIDITY_SUITABLE_MIN  40.0f  ///< 40%
   #define ENV_HUMIDITY_SUITABLE_MAX  85.0f  ///< 85%
   #define ENV_HUMIDITY_WARNING_MIN   30.0f  ///< 30%
   #define ENV_HUMIDITY_WARNING_MAX   90.0f  ///< 90%
   ```

4. **实现要点**
   - 判断顺序：先检查不适宜，再检查警告，最后为适宜
   - 温度和湿度同时满足条件才为适宜
   - 任一参数超限则降级评估
   - 阈值从NVS加载，支持运行时修改
   - 线程安全：使用互斥锁保护阈值

**验收标准**:
- [x] 判断逻辑正确
- [x] 边界条件处理正确
- [x] 阈值配置生效
- [x] 单元测试覆盖所有场景
- [x] 编译无警告

---

### 3.2 任务6：环境参数超限检测和报警

**任务ID**: TASK-ENV-006  
**优先级**: P0  
**估算时间**: 