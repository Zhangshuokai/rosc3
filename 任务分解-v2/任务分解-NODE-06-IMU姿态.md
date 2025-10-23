
# 任务分解：NODE-06 IMU姿态节点

## 文档信息

| 项目 | 内容 |
|------|------|
| **文档标题** | NODE-06 IMU姿态节点任务分解文档 |
| **文档版本** | v1.0.0 |
| **编制日期** | 2025-10-23 |
| **适用范围** | NODE-06 IMU姿态节点 |
| **节点职责** | 姿态测量和平衡监测 |
| **硬件平台** | Adafruit QT Py ESP32-C3 |

---

## 目录

- [1. 节点概述](#1-节点概述)
- [2. 驱动层任务（MPU6050模块）](#2-驱动层任务mpu6050模块)
- [3. 传感器管理层](#3-传感器管理层)
- [4. 姿态融合层](#4-姿态融合层)
- [5. 应用逻辑层](#5-应用逻辑层)
- [6. 节点应用层（imu_node模块）](#6-节点应用层imu_node模块)
- [7. 集成测试](#7-集成测试)
- [8. 依赖关系图](#8-依赖关系图)

---

## 1. 节点概述

### 1.1 节点职责

NODE-06 IMU姿态节点负责测量建筑喷涂施工机器人的姿态和运动状态，用于平衡监测、姿态校正和振动检测。

**管理硬件**：
- IMU惯性测量单元（MPU6050或ICM20948）
  - 三轴加速度计
  - 三轴陀螺仪
  - 温度传感器（内置）

**主要功能**：
1. 采集三轴加速度数据（m/s²）
2. 采集三轴角速度数据（rad/s）
3. 计算姿态角（Roll、Pitch、Yaw）
4. 发布IMU数据到ROS话题（50Hz）
5. 倾斜监测和报警（>5°警告，>10°紧急停止）
6. OLED实时显示姿态角和倾斜警告
7. 零点校准（陀螺仪漂移补偿）

### 1.2 硬件接口（Adafruit QT Py ESP32-C3）

| 接口类型 | 引脚分配 | 连接设备 | 说明 |
|---------|---------|---------|------|
| I2C总线 | GPIO8 (SDA), GPIO9 (SCL) | MPU6050/ICM20948 | I2C地址0x68或0x69 |
| I2C显示 | GPIO5 (SCL), GPIO6 (SDA) | 128x64 OLED | 状态显示 |
| UART | GPIO20 (RX), GPIO21 (TX) | 调试串口 | 日志输出 |

### 1.3 性能指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| **采样频率** | 50 Hz | IMU数据采集频率 |
| **加速度量程** | ±8g | 推荐配置 |
| **陀螺仪量程** | ±1000°/s | 推荐配置 |
| **加速度精度** | ±0.01 m/s² | 静态精度 |
| **角速度精度** | ±0.1°/s | 校准后漂移 |
| **姿态角精度** | ±1° | 互补滤波 |
| **倾斜响应** | <100ms | 从检测到报警 |

### 1.4 代码结构

```
src/nodes/imu/
├── imu_node.h              # 节点主接口
├── imu_node.c              # 节点主逻辑
├── mpu6050_driver.h        # MPU6050驱动接口
├── mpu6050_driver.c        # MPU6050驱动实现
├── icm20948_driver.h       # ICM20948驱动接口（可选）
├── icm20948_driver.c       # ICM20948驱动实现（可选）
├── attitude_fusion.h       # 姿态融合算法接口
├── attitude_fusion.c       # 姿态融合算法实现
└── README.md               # 节点说明
```

---

## 2. 驱动层任务（MPU6050模块）

### 2.1 任务1：MPU6050初始化和配置

**任务ID**: TASK-IMU-001  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: 无

**功能描述**:
- 初始化I2C总线通信
- 检测MPU6050设备ID
- 配置加速度计量程（±8g）
- 配置陀螺仪量程（±1000°/s）
- 配置数字低通滤波器（DLPF）

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/imu/mpu6050_driver.h`](src/nodes/imu/mpu6050_driver.h) - MPU6050驱动接口
   - [`src/nodes/imu/mpu6050_driver.c`](src/nodes/imu/mpu6050_driver.c) - MPU6050驱动实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化MPU6050
    * @param[in] i2c_port I2C端口号
    * @param[in] i2c_addr I2C地址（0x68或0x69）
    * @return 
    *   - ESP_OK: 成功
    *   - ESP_ERR_NOT_FOUND: 设备未找到
    *   - ESP_FAIL: 初始化失败
    */
   esp_err_t mpu6050_init(uint8_t i2c_port, uint8_t i2c_addr);
   
   /**
    * @brief 配置MPU6050参数
    * @param[in] accel_range 加速度计量程
    * @param[in] gyro_range 陀螺仪量程
    * @param[in] dlpf DLPF配置
    * @return ESP_OK: 成功
    */
   esp_err_t mpu6050_config(mpu6050_accel_range_t accel_range,
                            mpu6050_gyro_range_t gyro_range,
                            mpu6050_dlpf_t dlpf);
   
   /**
    * @brief 复位MPU6050
    * @return ESP_OK: 成功
    */
   esp_err_t mpu6050_reset(void);
   
   /**
    * @brief 检查MPU6050设备ID
    * @param[out] device_id 设备ID（应为0x68）
    * @return ESP_OK: 成功
    */
   esp_err_t mpu6050_check_device_id(uint8_t *device_id);
   ```

3. **数据结构**
   ```c
   /**
    * @brief MPU6050关键寄存器定义
    */
   #define MPU6050_I2C_ADDR        0x68  ///< AD0=GND
   #define MPU6050_I2C_ADDR_ALT    0x69  ///< AD0=VCC
   #define MPU6050_REG_WHO_AM_I    0x75  ///< 设备ID
   #define MPU6050_REG_PWR_MGMT_1  0x6B  ///< 电源管理1
   #define MPU6050_REG_CONFIG      0x1A  ///< 配置（DLPF）
   #define MPU6050_REG_GYRO_CONFIG 0x1B  ///< 陀螺仪配置
   #define MPU6050_REG_ACCEL_CONFIG 0x1C ///< 加速度计配置
   #define MPU6050_REG_ACCEL_XOUT_H 0x3B ///< 加速度数据起始
   #define MPU6050_REG_TEMP_OUT_H  0x41  ///< 温度数据
   #define MPU6050_REG_GYRO_XOUT_H 0x43  ///< 陀螺仪数据起始
   
   /**
    * @brief 加速度计量程枚举
    */
   typedef enum {
       MPU6050_ACCEL_RANGE_2G  = 0,  ///< ±2g
       MPU6050_ACCEL_RANGE_4G  = 1,  ///< ±4g
       MPU6050_ACCEL_RANGE_8G  = 2,  ///< ±8g（推荐）
       MPU6050_ACCEL_RANGE_16G = 3   ///< ±16g
   } mpu6050_accel_range_t;
   
   /**
    * @brief 陀螺仪量程枚举
    */
   typedef enum {
       MPU6050_GYRO_RANGE_250DPS  = 0,  ///< ±250°/s
       MPU6050_GYRO_RANGE_500DPS  = 1,  ///< ±500°/s
       MPU6050_GYRO_RANGE_1000DPS = 2,  ///< ±1000°/s（推荐）
       MPU6050_GYRO_RANGE_2000DPS = 3   ///< ±2000°/s
   } mpu6050_gyro_range_t;
   
   /**
    * @brief DLPF数字低通滤波器
    */
   typedef enum {
       MPU6050_DLPF_260HZ = 0,  ///< 带宽260Hz
       MPU6050_DLPF_184HZ = 1,  ///< 带宽184Hz
       MPU6050_DLPF_94HZ  = 2,  ///< 带宽94Hz
       MPU6050_DLPF_44HZ  = 3,  ///< 带宽44Hz
       MPU6050_DLPF_21HZ  = 4,  ///< 带宽21Hz（推荐）
       MPU6050_DLPF_10HZ  = 5,  ///< 带宽10Hz
       MPU6050_DLPF_5HZ   = 6   ///< 带宽5Hz
   } mpu6050_dlpf_t;
   ```

4. **实现要点**
   - 使用通用I2C总线模块进行通信
   - 初始化流程：检查设备ID → 复位 → 退出睡眠 → 配置DLPF → 配置量程
   - DLPF推荐21Hz，平衡延迟和噪声抑制
   - 复位后需延迟100ms等待设备稳定
   - 使用[`i2c_master_write_to_device()`](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/peripherals/i2c.html)写寄存器
   - 使用[`i2c_master_read_from_device()`](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/peripherals/i2c.html)读寄存器

**验收标准**:
- [x] 成功检测MPU6050设备（ID=0x68）
- [x] I2C通信稳定
- [x] 配置参数正确写入
- [x] 复位功能正常
- [x] 编译无警告

---

### 2.2 任务2：MPU6050原始数据读取

**任务ID**: TASK-IMU-002  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-IMU-001]

**功能描述**:
- 读取三轴加速度原始值
- 读取三轴角速度原始值
- 读取温度传感器值
- 处理大端序数据转换

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 读取MPU6050原始数据
    * @param[out] raw 原始数据结构体
    * @return 
    *   - ESP_OK: 成功
    *   - ESP_ERR_TIMEOUT: I2C超时
    *   - ESP_FAIL: 读取失败
    */
   esp_err_t mpu6050_read_raw(mpu6050_raw_data_t *raw);
   
   /**
    * @brief 读取加速度原始数据
    * @param[out] accel_x X轴加速度
    * @param[out] accel_y Y轴加速度
    * @param[out] accel_z Z轴加速度
    * @return ESP_OK: 成功
    */
   esp_err_t mpu6050_read_accel_raw(int16_t *accel_x, int16_t *accel_y, 
                                     int16_t *accel_z);
   
   /**
    * @brief 读取陀螺仪原始数据
    * @param[out] gyro_x X轴角速度
    * @param[out] gyro_y Y轴角速度
    * @param[out] gyro_z Z轴角速度
    * @return ESP_OK: 成功
    */
   esp_err_t mpu6050_read_gyro_raw(int16_t *gyro_x, int16_t *gyro_y,
                                    int16_t *gyro_z);
   
   /**
    * @brief 读取温度原始数据
    * @param[out] temp 温度原始值
    * @return ESP_OK: 成功
    */
   esp_err_t mpu6050_read_temp_raw(int16_t *temp);
   ```

2. **数据结构**
   ```c
   /**
    * @brief MPU6050原始数据结构
    */
   typedef struct {
       int16_t accel_x, accel_y, accel_z;  ///< 加速度原始值
       int16_t temp;                        ///< 温度原始值
       int16_t gyro_x, gyro_y, gyro_z;      ///< 角速度原始值
   } mpu6050_raw_data_t;
   ```

3. **实现要点**
   - 从寄存器0x3B连续读取14字节（加速度6 + 温度2 + 陀螺仪6）
   - 大端序转换：`value = (high_byte << 8) | low_byte`
   - I2C超时设置为100ms
   - 失败时重试3次
   - 使用`i2c_master_read_from_device()`一次性读取所有数据
   - 原始值为有符号16位整数（-32768 ~ 32767）

**验收标准**:
- [x] 数据读取准确
- [x] 大端序转换正确
- [x] I2C错误处理正确
- [x] 读取延迟<10ms
- [x] 功能测试通过

---

### 2.3 任务3：MPU6050单位转换

**任务ID**: TASK-IMU-003  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-IMU-002]

**功能描述**:
- 加速度转换为m/s²
- 角速度转换为rad/s
- 温度转换为°C
- 支持不同量程配置

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 转换MPU6050原始数据为物理量
    * @param[in] raw 原始数据
    * @param[out] data 物理量数据
    * @param[in] accel_range 加速度量程
    * @param[in] gyro_range 陀螺仪量程
    * @return ESP_OK: 成功
    */
   esp_err_t mpu6050_convert_data(const mpu6050_raw_data_t *raw,
                                   imu_data_t *data,
                                   mpu6050_accel_range_t accel_range,
                                   mpu6050_gyro_range_t gyro_range);
   
   /**
    * @brief 加速度原始值转换为m/s²
    * @param[in] raw_value 原始值
    * @param[in] range 量程配置
    * @return 加速度（m/s²）
    */
   float mpu6050_accel_to_ms2(int16_t raw_value, mpu6050_accel_range_t range);
   
   /**
    * @brief 角速度原始值转换为rad/s
    * @param[in] raw_value 原始值
    * @param[in] range 量程配置
    * @return 角速度（rad/s）
    */
   float mpu6050_gyro_to_rads(int16_t raw_value, mpu6050_gyro_range_t range);
   
   /**
    * @brief 温度原始值转换为°C
    * @param[in] raw_value 原始值
    * @return 温度（°C）
    */
   float mpu6050_temp_to_celsius(int16_t raw_value);
   ```

2. **数据结构**
   ```c
   /**
    * @brief IMU物理量数据
    */
   typedef struct {
       float accel_x, accel_y, accel_z;  ///< 加速度（m/s²）
       float gyro_x, gyro_y, gyro_z;      ///< 角速度（rad/s）
       float temperature;                 ///< 温度（°C）
   } imu_data_t;
   
   /**
    * @brief 转换系数定义
    */
   // 加速度转换（原始值→m/s²）
   #define ACCEL_SCALE_2G   (2.0f * 9.81f / 32768.0f)
   #define ACCEL_SCALE_4G   (4.0f * 9.81f / 32768.0f)
   #define ACCEL_SCALE_8G   (8.0f * 9.81f / 32768.0f)
   #define ACCEL_SCALE_16G  (16.0f * 9.81f / 32768.0f)
   
   // 角速度转换（原始值→rad/s）
   #define GYRO_SCALE_250DPS  (250.0f / 32768.0f * M_PI / 180.0f)
   #define GYRO_SCALE_500DPS  (500.0f / 32768.0f * M_PI / 180.0f)
   #define GYRO_SCALE_1000DPS (1000.0f / 32768.0f * M_PI / 180.0f)
   #define GYRO_SCALE_2000DPS (2000.0f / 32768.0f * M_PI / 180.0f)
   ```

3. **转换公式**
   ```c
   // 加速度转换示例（±8g量程）
   float accel_ms2 = raw_accel * ACCEL_SCALE_8G;
   
   // 角速度转换示例（±1000°/s量程）
   float gyro_rads = raw_gyro * GYRO_SCALE_1000DPS;
   
   // 温度转换（MPU6050公式）
   float temp_celsius = (raw_temp / 340.0f) + 36.53f;
   ```

4. **实现要点**
   - 根据配置的量程选择对应的转换系数
   - 加速度转换：考虑重力加速度9.81 m/s²
   - 角速度转换：将°/s转换为rad/s（乘以π/180）
   - 温度公式：`T = TEMP_OUT/340 + 36.53`
   - 使用浮点数提高精度

**验收标准**:
- [x] 单位转换准确（误差<1%）
- [x] 不同量程配置正确
- [x] 温度转换准确（±1°C）
- [x] 单元测试通过
- [x] 编译无警告

---

### 2.4 任务4：MPU6050自检和故障检测

**任务ID**: TASK-IMU-004  
**优先级**: P1  
**估算时间**: 6小时  
**依赖任务**: [TASK-IMU-002]

**功能描述**:
- 实现MPU6050自检功能
- 检测I2C通信故障
- 检测数据异常（卡死、饱和）
- 提供故障诊断信息

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief MPU6050自检
    * @return 
    *   - ESP_OK: 自检通过
    *   - ESP_FAIL: 自检失败
    */
   esp_err_t mpu6050_self_test(void);
   
   /**
    * @brief 检查通信状态
    * @return 
    *   - ESP_OK: 通信正常
    *   - ESP_ERR_TIMEOUT: 通信超时
    */
   esp_err_t mpu6050_check_communication(void);
   
   /**
    * @brief 检测数据异常
    * @param[in] raw 原始数据
    * @return 
    *   - ESP_OK: 数据正常
    *   - ESP_ERR_INVALID_STATE: 数据异常
    */
   esp_err_t mpu6050_check_data_valid(const mpu6050_raw_data_t *raw);
   ```

2. **实现要点**
   - 自检：读取设备ID并验证
   - 通信检测：连续读取寄存器，检查成功率
   - 数据饱和检测：检查是否长时间停留在最大值/最小值
   - 数据卡死检测：检查是否长时间无变化
   - 异常时输出详细日志便于诊断

**验收标准**:
- [x] 自检功能正常
- [x] 通信故障检测准确
- [x] 数据异常检测有效
- [x] 诊断信息清晰
- [x] 功能测试通过

---

## 3. 传感器管理层

### 3.1 任务5：零点校准（陀螺仪偏移补偿）

**任务ID**: TASK-IMU-005  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: [TASK-IMU-003]

**功能描述**:
- 静态状态下采集陀螺仪数据
- 计算陀螺仪零点偏移
- 保存校准参数到NVS
- 启动时自动加载校准参数

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 陀螺仪零点校准
    * @param[in] sample_count 采样次数（推荐100）
    * @return ESP_OK: 成功
    */
   esp_err_t imu_calibrate_gyro(uint16_t sample_count);
   
   /**
    * @brief 加速度计校准
    * @param[in] sample_count 采样次数
    * @return ESP_OK: 成功
    */
   esp_err_t imu_calibrate_accel(uint16_t sample_count);
   
   /**
    * @brief 保存校准参数到NVS
    * @return ESP_OK: 成功
    */
   esp_err_t imu_save_calibration(void);
   
   /**
    * @brief 从NVS加载校准参数
    * @return ESP_OK: 成功
    */
   esp_err_t imu_load_calibration(void);
   
   /**
    * @brief 应用校准参数到数据
    * @param[in,out] data IMU数据
    * @return ESP_OK: 成功
    */
   esp_err_t imu_apply_calibration(imu_data_t *data);
   ```

2. **数据结构**
   ```c
   /**
    * @brief IMU校准参数
    */
   typedef struct {
       float gyro_offset_x, gyro_offset_y, gyro_offset_z;  ///< 陀螺仪零点偏移（rad/s）
       float accel_offset_x, accel_offset_y, accel_offset_z; ///< 加速度零点偏移（m/s²）
       float accel_scale_x, accel_scale_y, accel_scale_z;  ///< 加速度比例因子
       bool calibrated;                                     ///< 是否已校准
       uint32_t timestamp;                                  ///< 校准时间戳
   } imu_calibration_t;
   
   /**
    * @brief NVS键名定义
    */
   #define NVS_KEY_GYRO_OFFSET_X   "gyro_off_x"
   #define NVS_KEY_GYRO_OFFSET_Y   "gyro_off_y"
   #define NVS_KEY_GYRO_OFFSET_Z   "gyro_off_z"
   #define NVS_KEY_ACCEL_OFFSET_X  "accel_off_x"
   #define NVS_KEY_ACCEL_OFFSET_Y  "accel_off_y"
   #define NVS_KEY_ACCEL_OFFSET_Z  "accel_off_z"
   ```

3. **校准算法**
   ```c
   /**
    * @brief 陀螺仪零点校准实现
    */
   esp_err_t imu_calibrate_gyro(uint16_t sample_count) {
       float sum_x = 0, sum_y = 0, sum_z = 0;
       
       ESP_LOGI(TAG, "Gyro calibration: keep IMU still for %d samples", 
                sample_count);
       
       for (int i = 0; i < sample_count; i++) {
           mpu6050_raw_data_t raw;
           imu_data_t data;
           
           mpu6050_read_raw(&raw);
           mpu6050_convert_data(&raw, &data, 
                               MPU6050_ACCEL_RANGE_8G,
                               MPU6050_GYRO_RANGE_1000DPS);
           
           sum_x += data.gyro_x;
           sum_y += data.gyro_y;
           sum_z += data.gyro_z;
           
           vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz采样
       }
       
       g_calibration.gyro_offset_x = sum_x / sample_count;
       g_calibration.gyro_offset_y = sum_y / sample_count;
       g_calibration.gyro_offset_z = sum_z / sample_count;
       g_calibration.calibrated = true;
       
       ESP_LOGI(TAG, "Gyro offset: x=%.4f, y=%.4f, z=%.4f rad/s",
                g_calibration.gyro_offset_x,
                g_calibration.gyro_offset_y,
                g_calibration.gyro_offset_z);
       
       return imu_save_calibration();
   }
   ```

4. **实现要点**
   - 校准时IMU必须静止
   - 采样100次取平均值作为零点偏移
   - 校准参数保存到NVS命名空间"imu_calib"
   - 启动时自动加载，若无校准数据则提示用户校准
   - 应用校准：`corrected_value = measured_value - offset`
   - 加速度计校准可使用六面法（可选）

**验收标准**:
- [x] 零点校准准确（漂移<0.01 rad/s）
- [x] 校准参数持久化存储
- [x] 断电后恢复校准参数
- [x] 校准流程用户友好
- [x] 单元测试通过

---

### 3.2 任务6：数字低通滤波

**任务ID**: TASK-IMU-006  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: [TASK-IMU-003]

**功能描述**:
- 实现移动平均滤波器
- 实现一阶低通滤波器
- 降低传感器噪声
- 保持合理的响应速度

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化IMU滤波器
    * @param[in] alpha 滤波器系数（0-1）
    * @return ESP_OK: 成功
    */
   esp_err_t imu_filter_init(float alpha);
   
   /**
    * @brief 应用低通滤波器
    * @param[in,out] data IMU数据
    * @return ESP_OK: 成功
    */
   esp_err_t imu_filter_apply(imu_data_t *data);
   
   /**
    * @brief 重置滤波器状态
    * @return ESP_OK: 成功
    */
   esp_err_t imu_filter_reset(void);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 低通滤波器状态
    */
   typedef struct {
       float alpha;                      ///< 滤波系数（0.1-0.3推荐）
       imu_data_t filtered;              ///< 滤波后的值
       bool initialized;                 ///< 是否已初始化
   } imu_filter_t;
   
   /**
    * @brief 滤波器配置
    */
   #define IMU_FILTER_ALPHA_DEFAULT  0.2f  ///< 默认滤波系数
   ```

3. **滤波算法**
   ```c
   /**
    * @brief 一阶低通滤波器
    * filtered = alpha * new_value + (1 - alpha) * filtered
    * alpha越小，滤波越强，但响应越慢
    */
   esp_err_t imu_filter_apply(imu_data_t *data) {
       if (!g_filter.initialized) {
           // 首次使用当前值初始化
           memcpy(&g_filter.filtered, data, sizeof(imu_data_t));
           g_filter.initialized = true;
           return ESP_OK;
       }
       
       float alpha = g_filter.alpha;
       
       // 加速度滤波
       g_filter.filtered.accel_x = alpha * data->accel_x + 
                                   (1.0f - alpha) * g_filter.filtered.accel_x;
       g_filter.filtered.accel_y = alpha * data->accel_y + 
                                   (1.0f - alpha) * g_filter.filtered.accel_y;
       g_filter.filtered.accel_z = alpha * data->accel_z + 
                                   (1.0f - alpha) * g_filter.filtered.accel_z;
       
       // 陀螺仪滤波
       g_filter.filtered.gyro_x = alpha * data->gyro_x + 
                                  (1.0f - alpha) * g_filter.filtered.gyro_x;
       g_filter.filtered.gyro_y = alpha * data->gyro_y + 
                                  (1.0f - alpha) * g_filter.filtered.gyro_y;
       g_filter.filtered.gyro_z = alpha * data->gyro_z + 
                                  (1.0f - alpha) * g_filter.filtered.gyro_z;
       
       // 温度滤波
       g_filter.filtered.temperature = alpha * data->temperature + 
                                       (1.0f - alpha) * g_filter.filtered.temperature;
       
       // 更新数据为滤波后的值
       memcpy(data, &g_filter.filtered, sizeof(imu_data_t));
       
       return ESP_OK;
   }
   ```

4. **实现要点**
   - alpha=0.2时，截止频率约为采样频率的1/10
   - 加速度和陀螺仪使用独立滤波器
   - 首次运行时用当前值初始化，避免启动瞬变
   - 可配合硬件DLPF使用，进一步降噪
   - 滤波器状态使用静态全局变量

**验收标准**:
- [x] 滤波效果明显（噪声降低>50%）
- [x] 响应速度可接受（延迟<50ms）
- [x] 无相位失真
- [x] 参数可调整
- [x] 单元测试通过

---

### 3.3 任务7：温度补偿（可选）

**任务ID**: TASK-IMU-007  
**优先级**: P2  
**估算时间**: 8小时  
**依赖任务**: [TASK-IMU-003]

**功能描述**:
- 测量温度对陀螺仪漂移的影响
- 建立温度补偿模型
- 实时温度补偿
- 提高长期稳定性

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 温度补偿校准
    * @return ESP_OK: 成功
    */
   esp_err_t imu_calibrate_temperature_compensation(void);
   
   /**
    * @brief 应用温度补偿
    * @param[in] temperature 当前温度（°C）
    * @param[in,out] data IMU数据
    * @return ESP_OK: 成功
    */
   esp_err_t imu_apply_temperature_compensation(float temperature, 
                                                  imu_data_t *data);
   ```

2. **实现要点**
   - 温度补偿模型：`drift = k * (T - T0)`
   - 需要在不同温度下测量漂移
   - 保存温度系数到NVS
   - 实时补偿：根据当前温度调整陀螺仪输出

**验收标准**:
- [x] 温度补偿模型建立
- [x] 补偿效果验证（漂移降低>30%）
- [x] 不同温度下测试通过

---

## 4. 姿态融合层

### 4.1 任务8：互补滤波器（Roll/Pitch融合）

**任务ID**: TASK-IMU-008  
**优先级**: P0  
**估算时间**: 10小时  
**依赖任务**: [TASK-IMU-006]

**功能描述**:
- 实现互补滤波算法
- 融合加速度计和陀螺仪数据
- 计算Roll和Pitch姿态角
- 抑制加速度计噪声和陀螺仪漂移

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/imu/attitude_fusion.h`](src/nodes/imu/attitude_fusion.h) - 姿态融合接口
   - [`src/nodes/imu/attitude_fusion.c`](src/nodes/imu/attitude_fusion.c) - 姿态融合实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化姿态融合
    * @param[in] alpha 互补滤波系数（推荐0.98）
    * @return ESP_OK: 成功
    */
   esp_err_t attitude_fusion_init(float alpha);
   
   /**
    * @brief 更新姿态角（互补滤波）
    * @param[in] imu IMU数据
    * @param[in] dt 时间间隔（秒）
    * @param[out] angles 姿态角
    * @return ESP_OK: 成功
    */
   esp_err_t attitude_fusion_update(const imu_data_t *imu,
                                      float dt,
                                      euler_angles_t *angles);
   
   /**
    * @brief 重置姿态角
    * @return ESP_OK: 成功
    */
   esp_err_t attitude_fusion_reset(void);
   ```

3. **数据结构**
   ```c
   /**
    * @brief 欧拉角结构
    */
   typedef struct {
       float roll;   ///< 横滚角（rad）
       float pitch;  ///< 俯仰角（rad）
       float yaw;    ///< 偏航角（rad）
   } euler_angles_t;
   
   /**
    * @brief 互补滤波器配置
    */
   #define COMPLEMENTARY_ALPHA  0.98f  ///< 陀螺仪权重98%，加速度2%
   ```

4. **互补滤波算法**
   ```c
   /**
    * @brief 互补滤波器更新
    */
   void complementary_filter_update(euler_angles_t *angles,
                                     const imu_data_t *imu,
                                     float dt) {
       // 1. 从加速度计计算Roll和Pitch（静态分量）
       float accel_roll = atan2f(imu->accel_y, imu->accel_z);
       float accel_pitch = atan2f(-imu->accel_x,
                                  sqrtf(imu->accel_y * imu->accel_y +
                                        imu->accel_z * imu->accel_z));
       
       // 2. 从陀螺仪积分计算Roll和Pitch（动态分量）
       angles->roll += imu->gyro_x * dt;
       angles->pitch += imu->gyro_y * dt;
       angles->yaw += imu->gyro_z * dt;
       
       // 3. 互补滤波融合（98%陀螺仪 + 2%加速度）
       angles->roll = COMPLEMENTARY_ALPHA * angles->roll +
                      (1.0f - COMPLEMENTARY_ALPHA) * accel_roll;
       angles->pitch = COMPLEMENTARY_ALPHA * angles->pitch +
                       (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;
       
       // 注意：Yaw角仅靠陀螺仪积分，会漂移（需磁力计或里程计融合）
   }
   ```

5. **实现要点**
   - 互补滤波系数0.98：陀螺仪提供高频响应，加速度计校正低频漂移
   - 加速度计Roll/Pitch计算：使用反正切函数
   - 陀螺仪积分：`angle += gyro * dt`
   - dt=0.02s（50Hz采样）
   - Yaw角漂移问题：需要磁力计（ICM20948）或里程计融合
   - 姿态角归一化到[-π, π]

**验收标准**:
- [x] Roll/Pitch角计算准确（±1°）
- [x] 融合算法稳定无振荡
- [x] 动态响应快速（<100ms）
- [x] 静态漂移小（<0.5°/min）
- [x] 单元测试通过

---

### 4.2 任务9：卡尔曼滤波器（可选，高精度）

**任务ID**: TASK-IMU-009  
**优先级**: P2  
**估算时间**: 12小时  
**依赖任务**: [TASK-IMU-008]

**功能描述**:
- 实现卡尔曼滤波算法
- 提供比互补滤波更高的精度
- 自适应调整滤波参数
- 优化噪声抑制

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化卡尔曼滤波器
    * @param[in] process_noise 过程噪声
    * @param[in] measurement_noise 测量噪声
    * @return ESP_OK: 成功
    */
   esp_err_t kalman_filter_init(float process_noise, float measurement_noise);
   
   /**
    * @brief 卡尔曼滤波更新
    * @param[in] imu IMU数据
    * @param[in] dt 时间间隔
    * @param[out] angles 姿态角
    * @return ESP_OK: 成功
    */
   esp_err_t kalman_filter_update(const imu_data_t *imu,
                                    float dt,
                                    euler_angles_t *angles);
   ```

2. **实现要点**
   - 状态向量：[angle, bias]
   - 预测：angle_pred = angle + (gyro - bias) * dt
   - 更新：使用加速度计测量值修正
   - 协方差矩阵管理
   - 计算量比互补滤波大，但精度更高

**验收标准**:
- [x] 姿态角精度±0.5°
- [x] 漂移抑制更好
- [x] 计算量可接受（<20% CPU）
- [x] 对比测试优于互补滤波

---

### 4.3 任务10：四元数表示（可选）

**任务ID**: TASK-IMU-010  
**优先级**: P2  
**估算时间**: 10小时  
**依赖任务**: [TASK-IMU-008]

**功能描述**:
- 实现四元数姿态表示
- 避免万向节锁问题
- 四元数与欧拉角相互转换
- 优化计算效率

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 四元数更新
    * @param[in] imu IMU数据
    * @param[in] dt 时间间隔
    * @param[in,out] q 四元数
    * @return ESP_OK: 成功
    */
   esp_err_t quaternion_update(const imu_data_t *imu, float dt, 
                                quaternion_t *q);
   
   /**
    * @brief 四元数转欧拉角
    * @param[in] q 四元数
    * @param[out] angles 欧拉角
    * @return ESP_OK: 成功
    */
   esp_err_t quaternion_to_euler(const quaternion_t *q, 
                                  euler_angles_t *angles);
   
   /**
    * @brief 欧拉角转四元数
    * @param[in] angles 欧拉角
    * @param[out] q 四元数
    * @return ESP_OK: 成功
    */
   esp_err_t euler_to_quaternion(const euler_angles_t *angles,
                                  quaternion_t *q);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 四元数结构
    */
   typedef struct {
       float w, x, y, z;
   } quaternion_t;
   ```

**验收标准**:
- [x] 四元数表示正确
- [x] 转换算法准确
- [x] 无万向节锁
- [x] 性能优化

---

## 5. 应用逻辑层

### 5.1 任务11：倾斜监测和分级报警

**任务ID**: TASK-IMU-011  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-IMU-008]

**功能描述**:
- 实时监测Roll和Pitch角
- 两级报警：警告（>5°）和紧急（>10°）
- 报警状态管理
- 发布报警信息到ROS

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 检查倾斜状态
    * @param[in] angles 姿态角
    * @param[out] level 倾斜级别
    * @return ESP_OK: 成功
    */
   esp_err_t imu_check_tilt(const euler_angles_t *angles, 
                            tilt_level_t *level);
   
   /**
    * @brief 获取倾斜报警状态
    * @param[out] tilt_alarm 倾斜报警标志
    * @return ESP_OK: 成功
    */
   esp_err_t imu_get_tilt_alarm(bool *tilt_alarm);
   
   /**
    * @brief 清除倾斜报警
    * @return ESP_OK: 成功
    */
   esp_err_t imu_clear_tilt_alarm(void);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 倾斜级别枚举
    */
   typedef enum {
       TILT_NONE = 0,   ///< 正常
       TILT_WARN = 1,   ///< 警告（>5°）
       TILT_ALARM = 2   ///< 紧急（>10°）
   } tilt_level_t;
   
   /**
    * @brief 倾斜阈值定义
    */
   #define TILT_WARN_ANGLE_RAD   (5.0f * M_PI / 180.0f)   ///< 5°
   #define TILT_ALARM_ANGLE_RAD  (10.0f * M_PI / 180.0f)  ///< 10°
   ```

3. **检测逻辑**
   ```c
   /**
    * @brief 倾斜检测实现
    */
   tilt_level_t check_tilt(const euler_angles_t *angles) {
       float roll_abs = fabsf(angles->roll);
       float pitch_abs = fabsf(angles->pitch);
       
       // 紧急报警：超过10°
       if (roll_abs > TILT_ALARM_ANGLE_RAD || 
           pitch_abs > TILT_ALARM_ANGLE_RAD) {
           ESP_LOGE(TAG, "TILT ALARM: Roll=%.1f°, Pitch=%.1f°",
                    angles->roll * 180.0f / M_PI,
                    angles->pitch * 180.0f / M_PI);
           return TILT_ALARM;
       }
       
       // 警告：超过5°
       if (roll_abs > TILT_WARN_ANGLE_RAD || 
           pitch_abs > TILT_WARN_ANGLE_RAD) {
           ESP_LOGW(TAG, "TILT WARNING: Roll=%.1f°, Pitch=%.1f°",
                    angles->roll * 180.0f / M_PI,
                    angles->pitch * 180.0f / M_PI);
           return TILT_WARN;
       }
       
       return TILT_NONE;
   }
   ```

4. **实现要点**
   - 使用绝对值判断倾斜（正负倾斜都要检测）
   - 紧急报警时建议停止机器人运动
   - 警告时提示操作员注意
   - 报警状态保持，直到倾斜恢复或手动清除
   - 防抖动：连续3次检测超限才触发

**验收标准**:
- [x] 倾斜检测准确（误差<0.5°）
- [x] 报警触发及时（<100ms）
- [x] 两级报警区分明确
- [x] 防抖动机制有效
- [x] 功能测试通过

---

### 5.2 任务12：振动检测

**任务ID**: TASK-IMU-012  
**优先级**: P1  
**估算时间**: 6小时  
**依赖任务**: [TASK-IMU-006]

**功能描述**:
- 检测异常振动（加速度突变）
- 计算振动幅度和频率
- 振动报警和记录
- 用于设备故障诊断

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 检测振动
    * @param[in] imu IMU数据
    * @param[out] vibration 振动幅度（g）
    * @return ESP_OK: 成功
    */
   esp_err_t imu_detect_vibration(const imu_data_t *imu, float *vibration);
   
   /**
    * @brief 获取振动统计
    * @param[out] stats 振动统计信息
    * @return ESP_OK: 成功
    */
   esp_err_t imu_get_vibration_stats(vibration_stats_t *stats);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 振动统计
    */
   typedef struct {
       float current_vibration;  ///< 当前振动（g）
       float max_vibration;      ///< 最大振动（g）
       float avg_vibration;      ///< 平均振动（g）
       uint32_t alarm_count;     ///< 报警次数
   } vibration_stats_t;
   
   /**
    * @brief 振动阈值
    */
   #define VIBRATION_ALARM_THRESHOLD  2.0f  ///< 2g
   ```

3. **检测算法**
   ```c
   /**
    * @brief 振动检测（加速度幅度）
    */
   float calculate_vibration(const imu_data_t *imu) {
       // 计算加速度向量幅度（去除重力）
       float ax = imu->accel_x;
       float ay = imu->accel_y;
       float az = imu->accel_z - 9.81f;  // 减去重力
       
       float magnitude = sqrtf(ax*ax + ay*ay + az*az);
       
       return magnitude / 9.81f;  // 转换为g
   }
   ```

4. **实现要点**
   - 计算加速度向量幅度（去除重力分量）
   - 使用滑动窗口计算平均振动
   - 超过2g视为异常振动
   - 记录振动历史用于故障诊断
   - 高频振动可能指示机械故障

**验收标准**:
- [x] 振动检测准确
- [x] 异常振动报警及时
- [x] 统计数据正确
- [x] 功能测试通过

---

### 5.3 任务13：校准参数存储和管理

**任务ID**: TASK-IMU-013  
**优先级**: P0  
**估算时间**: 4小时  
**依赖任务**: [TASK-IMU-005]

**功能描述**:
- 保存校准参数到NVS
- 启动时自动加载
- 提供校准参数查询接口
- 支持恢复出厂校准

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 保存所有校准参数
    * @return ESP_OK: 成功
    */
   esp_err_t imu_save_all_calibration(void);
   
   /**
    * @brief 加载所有校准参数
    * @return ESP_OK: 成功
    */
   esp_err_t imu_load_all_calibration(void);
   
   /**
    * @brief 获取校准参数
    * @param[out] calib 校准参数
    * @return ESP_OK: 成功
    */
   esp_err_t imu_get_calibration(imu_calibration_t *calib);
   
   /**
    * @brief 恢复出厂校准
    * @return ESP_OK: 成功
    */
   esp_err_t imu_reset_calibration(void);
   ```

2. **实现要点**
   - 使用NVS命名空间"imu_calib"
   - 保存陀螺仪
零点偏移、加速度偏移和比例因子
   - 每次修改校准参数后立即保存
   - 启动时检查NVS是否有校准数据
   - 若无校准数据，使用默认值并提示用户校准
   - 提供查询接口供诊断使用

**验收标准**:
- [x] 校准参数持久化
- [x] 断电后恢复正常
- [x] 接口功能完善
- [x] 错误处理正确

---

## 6. 节点应用层（imu_node模块）

### 6.1 任务14：节点初始化和主循环

**任务ID**: TASK-IMU-014  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: [TASK-IMU-008, TASK-IMU-011]

**功能描述**:
- 初始化所有模块
- 创建ROS节点
- 启动IMU采样任务（50Hz）
- 实现主循环

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/imu/imu_node.h`](src/nodes/imu/imu_node.h) - 节点主接口
   - [`src/nodes/imu/imu_node.c`](src/nodes/imu/imu_node.c) - 节点主逻辑

2. **关键函数**
   ```c
   /**
    * @brief 初始化IMU节点
    * @return ESP_OK: 成功
    */
   esp_err_t imu_node_init(void);
   
   /**
    * @brief IMU节点主函数
    */
   void imu_node_main(void);
   
   /**
    * @brief 停止IMU节点
    * @return ESP_OK: 成功
    */
   esp_err_t imu_node_stop(void);
   ```

3. **初始化流程**
   ```c
   esp_err_t imu_node_init(void) {
       ESP_LOGI(TAG, "Initializing IMU node...");
       
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
       
       // 4. 初始化IMU专用模块
       ESP_ERROR_CHECK(mpu6050_init(I2C_NUM_0, MPU6050_I2C_ADDR));
       ESP_ERROR_CHECK(mpu6050_config(MPU6050_ACCEL_RANGE_8G,
                                      MPU6050_GYRO_RANGE_1000DPS,
                                      MPU6050_DLPF_21HZ));
       
       // 5. 加载校准参数
       ESP_ERROR_CHECK(imu_load_calibration());
       
       // 6. 初始化姿态融合
       ESP_ERROR_CHECK(attitude_fusion_init(COMPLEMENTARY_ALPHA));
       
       // 7. 初始化滤波器
       ESP_ERROR_CHECK(imu_filter_init(IMU_FILTER_ALPHA_DEFAULT));
       
       // 8. 创建ROS接口
       ESP_ERROR_CHECK(imu_create_ros_interfaces());
       
       ESP_LOGI(TAG, "IMU node initialized successfully");
       return ESP_OK;
   }
   ```

4. **主循环**
   ```c
   void imu_node_main(void) {
       imu_node_init();
       
       // 启动IMU采样任务（50Hz）
       xTaskCreate(imu_sampling_task, "imu_sample", 4096, NULL, 10, NULL);
       xTaskCreate(imu_publish_task, "imu_pub", 4096, NULL, 10, NULL);
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

### 6.2 任务15：ROS话题发布（/imu/data）

**任务ID**: TASK-IMU-015  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: [TASK-IMU-014]

**功能描述**:
- 创建IMU数据发布者
- 以50Hz频率发布IMU消息
- 构造sensor_msgs/Imu消息
- 填充协方差矩阵

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 创建IMU发布者
    * @return ESP_OK: 成功
    */
   esp_err_t imu_create_publisher(void);
   
   /**
    * @brief 发布IMU数据
    * @param[in] imu IMU数据
    * @param[in] angles 姿态角
    * @return ESP_OK: 成功
    */
   esp_err_t imu_publish_data(const imu_data_t *imu, 
                               const euler_angles_t *angles);
   ```

2. **ROS消息构造**
   ```c
   // 使用sensor_msgs/msg/Imu消息类型
   sensor_msgs__msg__Imu imu_msg;
   
   // Header
   imu_msg.header.stamp.sec = timestamp_us / 1000000;
   imu_msg.header.stamp.nanosec = (timestamp_us % 1000000) * 1000;
   imu_msg.header.frame_id = "imu_link";
   
   // 角速度（rad/s）
   imu_msg.angular_velocity.x = imu_data.gyro_x;
   imu_msg.angular_velocity.y = imu_data.gyro_y;
   imu_msg.angular_velocity.z = imu_data.gyro_z;
   imu_msg.angular_velocity_covariance[0] = 0.001;  // 方差
   imu_msg.angular_velocity_covariance[4] = 0.001;
   imu_msg.angular_velocity_covariance[8] = 0.001;
   
   // 线加速度（m/s²）
   imu_msg.linear_acceleration.x = imu_data.accel_x;
   imu_msg.linear_acceleration.y = imu_data.accel_y;
   imu_msg.linear_acceleration.z = imu_data.accel_z;
   imu_msg.linear_acceleration_covariance[0] = 0.01;
   imu_msg.linear_acceleration_covariance[4] = 0.01;
   imu_msg.linear_acceleration_covariance[8] = 0.01;
   
   // 姿态四元数（从欧拉角转换）
   quaternion_t q;
   euler_to_quaternion(&angles, &q);
   imu_msg.orientation.w = q.w;
   imu_msg.orientation.x = q.x;
   imu_msg.orientation.y = q.y;
   imu_msg.orientation.z = q.z;
   imu_msg.orientation_covariance[0] = 0.001;
   imu_msg.orientation_covariance[4] = 0.001;
   imu_msg.orientation_covariance[8] = 0.001;
   
   // 如果没有姿态融合，设orientation_covariance[0]=-1表示无效
   // imu_msg.orientation_covariance[0] = -1.0;
   ```

3. **发布任务**
   ```c
   void imu_publish_task(void *pvParameters) {
       TickType_t last_wake_time = xTaskGetTickCount();
       const TickType_t frequency = pdMS_TO_TICKS(20);  // 50Hz
       
       while (1) {
           // 获取最新IMU数据和姿态角
           imu_data_t imu_data;
           euler_angles_t angles;
           
           imu_get_latest_data(&imu_data, &angles);
           
           // 发布到ROS
           imu_publish_data(&imu_data, &angles);
           
           vTaskDelayUntil(&last_wake_time, frequency);
       }
   }
   ```

4. **实现要点**
   - 使用`sensor_msgs/msg/Imu`标准消息类型
   - 时间戳使用`esp_timer_get_time()`获取微秒精度
   - 欧拉角转四元数：仅绕Z轴旋转时简化计算
   - 协方差矩阵：对角线元素根据传感器精度设置
   - QoS：RELIABLE，确保数据可靠传输
   - 线程安全：使用互斥锁保护IMU数据

**验收标准**:
- [x] 话题正常发布
- [x] 发布频率稳定50Hz
- [x] 消息格式正确
- [x] `ros2 topic hz`验证通过
- [x] RViz可视化正常

---

### 6.3 任务16：ROS服务实现

**任务ID**: TASK-IMU-016  
**优先级**: P1  
**估算时间**: 8小时  
**依赖任务**: [TASK-IMU-014]

**功能描述**:
- 实现校准服务（/imu/calibrate）
- 实现姿态重置服务（/imu/reset_orientation）
- 提供参数查询服务
- 支持远程触发校准

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 创建所有服务
    * @return ESP_OK: 成功
    */
   esp_err_t imu_create_services(void);
   
   /**
    * @brief 校准服务回调
    */
   void imu_calibrate_service_callback(const void *request,
                                        void *response,
                                        void *user_data);
   
   /**
    * @brief 姿态重置服务回调
    */
   void imu_reset_orientation_callback(const void *request,
                                        void *response,
                                        void *user_data);
   ```

2. **服务定义**（需要自定义srv文件）
   ```
   # imu_interfaces/srv/Calibrate.srv
   ---
   bool success
   string message
   float32[3] gyro_offset    # 陀螺仪零点偏移
   float32[3] accel_offset   # 加速度零点偏移
   
   # imu_interfaces/srv/ResetOrientation.srv
   ---
   bool success
   string message
   ```

3. **实现要点**
   - 校准服务：触发陀螺仪零点校准并返回结果
   - 姿态重置：将Roll/Pitch/Yaw归零
   - 服务调用时暂停姿态更新
   - 校准完成后保存到NVS
   - 返回校准参数供用户确认

**验收标准**:
- [x] 服务正常响应
- [x] 校准功能正确
- [x] 姿态重置有效
- [x] `ros2 service call`测试通过

---

### 6.4 任务17：OLED实时显示

**任务ID**: TASK-IMU-017  
**优先级**: P1  
**估算时间**: 6小时  
**依赖任务**: [TASK-IMU-014]

**功能描述**:
- 显示当前姿态角（Roll/Pitch/Yaw）
- 显示倾斜警告状态
- 显示IMU温度
- 显示校准状态

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 更新OLED显示
    * @return ESP_OK: 成功
    */
   esp_err_t imu_update_oled_display(void);
   ```

2. **显示布局**
   ```
   ┌────────────────────────┐
   │ IMU NODE-06            │  ← 标题
   ├────────────────────────┤
   │ R:2.3° P:-1.5° Y:0.0°  │  ← 姿态角
   │ Temp: 36.5°C           │  ← 温度
   │ Status: OK             │  ← 状态
   │ ⚠ TILT WARNING         │  ← 倾斜警告
   └────────────────────────┘
   ```

3. **显示任务**
   ```c
   void oled_update_task(void *pvParameters) {
       char line_buf[32];
       
       while (1) {
           // 获取姿态角
           euler_angles_t angles;
           float temperature;
           tilt_level_t tilt;
           
           imu_get_current_state(&angles, &temperature, &tilt);
           
           // 更新显示
           snprintf(line_buf, sizeof(line_buf), 
                    "R:%.1f° P:%.1f° Y:%.1f°",
                    angles.roll * 180.0f / M_PI,
                    angles.pitch * 180.0f / M_PI,
                    angles.yaw * 180.0f / M_PI);
           oled_ui_show_custom_text(2, line_buf);
           
           snprintf(line_buf, sizeof(line_buf), 
                    "Temp: %.1f°C", temperature);
           oled_ui_show_custom_text(3, line_buf);
           
           // 显示倾斜警告
           if (tilt == TILT_ALARM) {
               oled_ui_show_alert("⚠ TILT ALARM!", true);
           } else if (tilt == TILT_WARN) {
               oled_ui_show_alert("⚠ TILT WARNING", false);
           } else {
               snprintf(line_buf, sizeof(line_buf), "Status: OK");
               oled_ui_show_custom_text(4, line_buf);
           }
           
           vTaskDelay(pdMS_TO_TICKS(1000));  // 1Hz更新
       }
   }
   ```

4. **实现要点**
   - 更新频率：1Hz（降低CPU占用）
   - 姿态角显示：转换为度（°）
   - 倾斜警告闪烁显示（红色背景）
   - 温度显示1位小数
   - 布局紧凑，充分利用空间

**验收标准**:
- [x] 显示内容准确
- [x] 更新及时（1Hz）
- [x] 倾斜警告明显
- [x] 布局清晰易读
- [x] CPU占用<10%

---

## 7. 集成测试

### 7.1 任务18：单元测试

**任务ID**: TASK-IMU-018  
**优先级**: P1  
**估算时间**: 12小时  
**依赖任务**: [TASK-IMU-001~017]

**功能描述**:
- MPU6050驱动单元测试
- 姿态融合算法单元测试
- 倾斜检测单元测试
- 滤波器单元测试
- 覆盖率>80%

**技术实现细节**:

1. **测试文件**
   - [`test/unit/test_mpu6050_driver.c`](test/unit/test_mpu6050_driver.c)
   - [`test/unit/test_attitude_fusion.c`](test/unit/test_attitude_fusion.c)
   - [`test/unit/test_imu_calibration.c`](test/unit/test_imu_calibration.c)

2. **测试用例**
   ```c
   // test_mpu6050_driver.c
   void test_mpu6050_init(void);
   void test_mpu6050_read_raw(void);
   void test_mpu6050_convert_data(void);
   void test_mpu6050_self_test(void);
   
   // test_attitude_fusion.c
   void test_complementary_filter_static(void);
   void test_complementary_filter_dynamic(void);
   void test_euler_to_quaternion(void);
   void test_quaternion_to_euler(void);
   
   // test_imu_calibration.c
   void test_gyro_calibration(void);
   void test_accel_calibration(void);
   void test_calibration_persistence(void);
   ```

3. **实现要点**
   - 使用Unity测试框架
   - Mock I2C硬件接口
   - 测试边界条件（角度±180°，加速度饱和）
   - 浮点数比较使用容差
   - 自动化测试脚本

**验收标准**:
- [x] 所有测试用例通过
- [x] 代码覆盖率>80%
- [x] 无内存泄漏
- [x] 测试报告生成
- [x] CI集成

---

### 7.2 任务19：硬件在环测试

**任务ID**: TASK-IMU-019  
**优先级**: P0  
**估算时间**: 16小时  
**依赖任务**: [TASK-IMU-001~017]

**功能描述**:
- 实际IMU硬件测试
- 姿态角精度验证
- 倾斜检测功能测试
- 长时间稳定性测试

**技术实现细节**:

1. **测试场景**
   - 静止状态姿态角测试（应为0°）
   - 已知角度旋转测试（90°旋转）
   - 倾斜5°和10°触发测试
   - 振动测试（模拟机械振动）
   - 温度漂移测试（不同温度下）
   - 长时间运行（8小时）

2. **测试指标**
   ```
   ┌──────────────────┬──────────┬──────────┐
   │ 测试项           │ 目标值   │ 实测值   │
   ├──────────────────┼──────────┼──────────┤
   │ Roll/Pitch精度   │ ±1°      │          │
   │ 零点漂移         │ <0.5°/min│          │
   │ 倾斜检测精度     │ ±0.5°    │          │
   │ 采样频率         │ 50±1Hz   │          │
   │ 响应时间         │ <100ms   │          │
   │ 长期稳定性       │ >8小时   │          │
   │ CPU占用          │ <50%     │          │
   │ 内存占用         │ <100KB   │          │
   └──────────────────┴──────────┴──────────┘
   ```

3. **测试工具**
   - 角度测量仪：验证姿态角精度
   - ROS工具：`ros2 topic echo /imu/data`
   - `ros2 topic hz`：验证发布频率
   - RViz：可视化姿态
   - 倾斜平台：测试倾斜检测
   - 温度箱：测试温度补偿（可选）

4. **测试步骤**
   ```bash
   # 1. 启动ROS Agent
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   
   # 2. 烧录并启动节点
   pio run -e imu_node -t upload
   
   # 3. 监控IMU话题
   ros2 topic echo /imu/data
   
   # 4. 验证发布频率
   ros2 topic hz /imu/data
   
   # 5. 校准测试
   ros2 service call /imu/calibrate imu_interfaces/srv/Calibrate "{}"
   
   # 6. 姿态重置测试
   ros2 service call /imu/reset_orientation \
       imu_interfaces/srv/ResetOrientation "{}"
   
   # 7. 倾斜测试
   # 使用倾斜平台倾斜5°，观察OLED警告
   # 倾斜10°，观察紧急报警
   ```

**验收标准**:
- [x] 所有测试场景通过
- [x] 性能指标达标
- [x] 倾斜检测准确
- [x] 长时间运行稳定（>8小时）
- [x] 无内存泄漏
- [x] 测试报告完整

---

## 8. 依赖关系图

### 8.1 任务依赖关系（DAG图）

```
初始化顺序（从上到下）：

1. 驱动层（并行）
   ├─→ TASK-001: MPU6050初始化
   ├─→ TASK-002: 原始数据读取 [依赖001]
   ├─→ TASK-003: 单位转换 [依赖002]
   └─→ TASK-004: 自检故障检测 [依赖002]

2. 传感器管理层（依赖驱动层）
   ├─→ TASK-005: 零点校准 [依赖003]
   ├─→ TASK-006: 数字滤波 [依赖003]
   └─→ TASK-007: 温度补偿（可选） [依赖003]

3. 姿态融合层（依赖传感器管理）
   ├─→ TASK-008: 互补滤波器 [依赖006]
   ├─→ TASK-009: 卡尔曼滤波（可选） [依赖008]
   └─→ TASK-010: 四元数表示（可选） [依赖008]

4. 应用逻辑层（依赖姿态融合）
   ├─→ TASK-011: 倾斜监测 [依赖008]
   ├─→ TASK-012: 振动检测 [依赖006]
   └─→ TASK-013: 校准参数管理 [依赖005]

5. 节点应用层（依赖所有底层）
   ├─→ TASK-014: 节点初始化 [依赖008,011]
   ├─→ TASK-015: ROS话题发布 [依赖014]
   ├─→ TASK-016: ROS服务 [依赖014]
   └─→ TASK-017: OLED显示 [依赖014]

6. 测试层（依赖全部）
   ├─→ TASK-018: 单元测试 [依赖001-017]
   └─→ TASK-019: 硬件测试 [依赖001-017]
```

### 8.2 任务依赖矩阵

| 任务ID | 任务名称 | 依赖任务 | 可并行任务 | 估算时间 |
|--------|---------|---------|-----------|---------|
| TASK-IMU-001 | MPU6050初始化 | 无 | - | 8h |
| TASK-IMU-002 | 原始数据读取 | 001 | 004 | 6h |
| TASK-IMU-003 | 单位转换 | 002 | 004 | 6h |
| TASK-IMU-004 | 自检故障检测 | 002 | 003 | 6h |
| TASK-IMU-005 | 零点校准 | 003 | 006, 007 | 8h |
| TASK-IMU-006 | 数字滤波 | 003 | 005, 007 | 8h |
| TASK-IMU-007 | 温度补偿（可选） | 003 | 005, 006 | 8h |
| TASK-IMU-008 | 互补滤波器 | 006 | 009, 010 | 10h |
| TASK-IMU-009 | 卡尔曼滤波（可选） | 008 | 010 | 12h |
| TASK-IMU-010 | 四元数表示（可选） | 008 | 009 | 10h |
| TASK-IMU-011 | 倾斜监测 | 008 | 012, 013 | 6h |
| TASK-IMU-012 | 振动检测 | 006 | 011, 013 | 6h |
| TASK-IMU-013 | 校准参数管理 | 005 | 011, 012 | 4h |
| TASK-IMU-014 | 节点初始化 | 008, 011 | 018 | 8h |
| TASK-IMU-015 | ROS话题发布 | 014 | 016, 017 | 8h |
| TASK-IMU-016 | ROS服务 | 014 | 015, 017 | 8h |
| TASK-IMU-017 | OLED显示 | 014 | 015, 016 | 6h |
| TASK-IMU-018 | 单元测试 | 001-017 | 019 | 12h |
| TASK-IMU-019 | 硬件测试 | 001-017 | 018 | 16h |

### 8.3 开发里程碑

**阶段1：驱动层（1周）**
- 完成MPU6050驱动（TASK-001~004）
- 目标：成功读取IMU原始数据

**阶段2：传感器管理层（1.5周）**
- 完成校准和滤波（TASK-005~006）
- 目标：获得稳定的IMU数据

**阶段3：姿态融合层（1.5周）**
- 完成互补滤波器（TASK-008）
- 目标：准确计算姿态角

**阶段4：应用逻辑层（1周）**
- 完成倾斜监测等功能（TASK-011~013）
- 目标：实现平衡监测功能

**阶段5：节点应用层（2周）**
- 完成节点框架和ROS接口（TASK-014~017）
- 目标：完整的ROS节点功能

**阶段6：测试和调优（2周）**
- 完成单元测试和硬件测试（TASK-018~019）
- 目标：稳定可靠的系统

**总计：9周（约2个月）**

### 8.4 关键路径

```
Critical Path（关键路径）:
TASK-001 → TASK-002 → TASK-003 → TASK-006 → TASK-008 → TASK-011 → TASK-014 → TASK-019
(8h)      (6h)        (6h)        (8h)       (10h)      (6h)       (8h)        (16h)
总计：68小时（约8.5个工作日）
```

---

## 附录A：配置参数说明

### A.1 Kconfig配置项

```kconfig
menu "IMU Node Configuration"
    
    config IMU_I2C_ADDR
        hex "MPU6050 I2C Address"
        range 0x68 0x69
        default 0x68
        help
            MPU6050 I2C address (0x68 if AD0=GND, 0x69 if AD0=VCC)
    
    config IMU_ACCEL_RANGE
        int "Accelerometer Range"
        range 0 3
        default 2
        help
            0=±2g, 1=±4g, 2=±8g, 3=±16g
    
    config IMU_GYRO_RANGE
        int "Gyroscope Range"
        range 0 3
        default 2
        help
            0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s
    
    config IMU_DLPF_CFG
        int "DLPF Configuration"
        range 0 6
        default 4
        help
            0=260Hz, 1=184Hz, 2=94Hz, 3=44Hz, 4=21Hz, 5=10Hz, 6=5Hz
    
    config IMU_SAMPLE_RATE
        int "Sample Rate (Hz)"
        range 10 100
        default 50
        help
            IMU sampling and publishing rate
    
    config IMU_TILT_WARN_ANGLE
        int "Tilt Warning Angle (degrees)"
        range 1 45
        default 5
        help
            Tilt warning threshold in degrees
    
    config IMU_TILT_ALARM_ANGLE
        int "Tilt Alarm Angle (degrees)"
        range 1 45
        default 10
        help
            Tilt alarm threshold in degrees

endmenu
```

### A.2 NVS配置键

| 配置键 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `gyro_offset_x` | float | 0.0 | X轴陀螺仪零点偏移（rad/s） |
| `gyro_offset_y` | float | 0.0 | Y轴陀螺仪零点偏移（rad/s） |
| `gyro_offset_z` | float | 0.0 | Z轴陀螺仪零点偏移（rad/s） |
| `accel_offset_x` | float | 0.0 | X轴加速度零点偏移（m/s²） |
| `accel_offset_y` | float | 0.0 | Y轴加速度零点偏移（m/s²） |
| `accel_offset_z` | float | 0.0 | Z轴加速度零点偏移（m/s²） |
| `calibrated` | uint8 | 0 | 是否已校准标志 |
| `calib_timestamp` | uint32 | 0 | 校准时间戳 |

---

## 附录B：ROS接口总结

### B.1 发布话题

| 话题名称 | 消息类型 | QoS | 频率 | 说明 |
|---------|---------|-----|------|------|
| `/imu/data` | `sensor_msgs/Imu` | RELIABLE | 50Hz | IMU数据（加速度、角速度、姿态） |
| `/imu/diagnostics` | `diagnostic_msgs/DiagnosticStatus` | RELIABLE | 1Hz | 诊断信息 |

### B.2 服务

| 服务名称 | 服务类型 | 说明 |
|---------|---------|------|
| `/imu/calibrate` | `imu_interfaces/Calibrate` | 陀螺仪零点校准 |
| `/imu/reset_orientation` | `imu_interfaces/ResetOrientation` | 重置姿态角为零 |

---

## 附录C：姿态角定义

根据航空惯例定义姿态角：

| 姿态角 | 定义 | 正方向 | 范围 |
|--------|------|--------|------|
| **Roll（横滚）** | 绕X轴旋转 | 右侧向下 | -π ~ π |
| **Pitch（俯仰）** | 绕Y轴旋转 | 抬头向上 | -π/2 ~ π/2 |
| **Yaw（偏航）** | 绕Z轴旋转 | 逆时针 | -π ~ π |

**坐标系定义**：
- X轴：机器人前进方向
- Y轴：机器人左侧方向
- Z轴：机器人向上方向
- 右手坐标系

---

## 附录D：校准流程

### D.1 陀螺仪零点校准

1. **准备工作**
   - 将机器人放置在水平稳定表面
   - 确保IMU完全静止
   - 等待温度稳定（开机后5分钟）

2. **执行校准**
   ```bash
   # 方法1：ROS服务调用
   ros2 service call /imu/calibrate imu_interfaces/srv/Calibrate "{}"
   
   # 方法2：OLED界面操作（如果支持）
   # 进入设置菜单 → 选择"IMU校准" → 确认
   ```

3. **校准过程**
   - 采集100次数据（约2秒）
   - 计算平均值作为零点偏移
   - 保存到NVS
   - 显示校准结果

4. **验证校准**
   ```bash
   # 观察静止时的角速度，应接近0
   ros2 topic echo /imu/data --field angular_velocity
   ```

### D.2 加速度计校准（六面法，可选）

六面法校准需要将IMU依次放置在六个方向：
1. Z轴向上（正常放置）
2. Z轴向下（倒置）
3. X轴向上
4. X轴向下
5. Y轴向上
6. Y轴向下

每个方向采集数据，计算零点偏移和比例因子。

---

## 附录E：故障诊断

### E.1 常见问题

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 设备ID读取失败 | I2C连接问题 | 检查I2C接线，确认地址正确 |
| 姿态角漂移严重 | 未校准或温度变化 | 执行陀螺仪零点校准 |
| 数据全为0 | MPU6050睡眠模式 | 检查电源管理寄存器配置 |
| 倾斜检测误报 | 滤波器系数不当 | 调整互补滤波器alpha值 |
| ROS话题无数据 | 网络连接问题 | 检查WiFi和ROS Agent连接 |
| CPU占用过高 | 采样频率过高 | 降低采样频率到50Hz |

### E.2 日志分析

```
# 正常启动日志
I (1234) IMU_NODE: Initializing IMU node...
I (1245) MPU6050: Device ID: 0x68
I (1256) MPU6050: Configuration: Accel=±8g, Gyro=±1000°/s, DLPF=21Hz
I (1267) IMU_CALIB: Loading calibration from NVS
I (1278) IMU_CALIB: Gyro offset: x=0.0012, y=-0.0008, z=0.0003 rad/s
I (1289) IMU_NODE: IMU node initialized successfully

# 倾斜警告日志
W (5678) IMU_TILT: TILT WARNING: Roll=5.2°, Pitch=1.3°
E (5890) IMU_TILT: TILT ALARM: Roll=11.5°, Pitch=2.1°
```

---

## 附录F：性能优化建议

1. **降低CPU占用**
   - 使用硬件DLPF代替软件滤波
   - 降低OLED更新频率到1Hz
   - 优化浮点数运算（使用查找表）

2. **提高姿态精度**
   - 定期重新校准（温度变化>10°C时）
   - 使用卡尔曼滤波代替互补滤波
   - 添加磁力计融合Yaw角（ICM20948）

3. **减少内存占用**
   - 使用静态分配代替动态分配
   - 减少滤波器缓冲区大小
   - 优化数据结构对齐

4. **提高实时性**
   - 提高IMU任务优先级
   - 使用DMA传输I2C数据
   - 减少日志输出

---

**文档结束**

**编写人**：架构设计组  
**审核人**：技术负责人  
**版本**：v1.0.0  
**日期**：2025-10-23