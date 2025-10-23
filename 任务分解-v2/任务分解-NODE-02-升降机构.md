
# 任务分解：NODE-02升降机构节点

## 文档信息

| 项目 | 内容 |
|------|------|
| **文档标题** | NODE-02升降机构节点任务分解文档 |
| **文档版本** | v1.0.0 |
| **编制日期** | 2025-10-23 |
| **适用范围** | NODE-02升降机构节点 |
| **节点职责** | 垂直升降机构位置控制 |
| **硬件平台** | Adafruit QT Py ESP32-C3 |

---

## 目录

- [1. 节点概述](#1-节点概述)
- [2. 驱动层任务](#2-驱动层任务)
- [3. 控制算法层](#3-控制算法层)
- [4. 节点应用层](#4-节点应用层)
- [5. 集成测试](#5-集成测试)
- [6. 依赖关系图](#6-依赖关系图)

---

## 1. 节点概述

### 1.1 节点职责

NODE-02升降机构节点负责管理建筑喷涂施工机器人的垂直升降机构，实现精确的高度控制和位置跟踪。

**管理硬件**：
- 伺服电机（升降驱动）
- AS5600磁编码器（位置传感器）
- 上限位开关
- 下限位开关

**主要功能**：
1. 接收ROS高度指令（[`/lift/cmd_position`](../需求分析/ROS接口定义文档.md)）
2. 执行伺服电机位置控制
3. 读取AS5600编码器位置反馈（I2C）
4. 发布当前高度（[`/lift/position`](../需求分析/ROS接口定义文档.md) @ 20Hz）
5. 限位保护和缓冲控制
6. 归零校准功能
7. 提供配置和诊断服务

### 1.2 硬件接口（Adafruit QT Py ESP32-C3）

| 接口类型 | 引脚分配 | 连接设备 | 说明 |
|---------|---------|---------|------|
| PWM输出 | GPIO0 | 伺服电机 | LEDC通道0 |
| I2C总线 | GPIO8 (SDA), GPIO9 (SCL) | AS5600磁编码器 | 角度读取 |
| GPIO输入 | GPIO7 | 上限位开关 | 上拉输入 |
| GPIO输入 | GPIO10 | 下限位开关 | 上拉输入 |
| I2C显示 | GPIO5 (SCL), GPIO6 (SDA) | 128x64 OLED | 状态显示 |
| UART | GPIO20 (RX), GPIO21 (TX) | 调试串口 | 日志输出 |

### 1.3 性能指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| **定位精度** | ±5mm | 高度控制精度 |
| **升降速度** | 0.1-0.5 m/s | 可调节范围 |
| **响应时间** | <200ms | 指令到电机启动 |
| **安全停止时间** | <100ms | 限位触发到停止 |
| **发布频率** | 20 Hz | 位置数据发布频率 |

### 1.4 代码结构

```
src/nodes/lift/
├── lift_node.h             # 节点主接口
├── lift_node.c             # 节点主逻辑
├── lift_control.h          # 位置控制算法接口
├── lift_control.c          # PID位置控制实现
├── servo_driver.h          # 伺服电机驱动接口
├── servo_driver.c          # PWM伺服控制
├── as5600_driver.h         # AS5600编码器驱动接口
├── as5600_driver.c         # I2C编码器读取
└── README.md               # 节点说明
```

---

## 2. 驱动层任务

### 2.1 任务1：伺服电机PWM驱动初始化和控制

**任务ID**: TASK-LIFT-001  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: 无

**功能描述**:
- 初始化LEDC外设用于PWM输出
- 配置伺服电机PWM通道（50Hz，脉宽1-2ms）
- 实现伺服角度/速度设置接口
- 实现平滑运动控制

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/lift/servo_driver.h`](src/nodes/lift/servo_driver.h) - 伺服驱动接口定义
   - [`src/nodes/lift/servo_driver.c`](src/nodes/lift/servo_driver.c) - 伺服驱动实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化伺服电机驱动
    * @return 
    *   - ESP_OK: 成功
    *   - ESP_FAIL: 初始化失败
    */
   esp_err_t servo_driver_init(void);
   
   /**
    * @brief 设置伺服角度
    * @param[in] angle 角度值（0.0 ~ 180.0度）
    * @return ESP_OK: 成功
    */
   esp_err_t servo_driver_set_angle(float angle);
   
   /**
    * @brief 设置伺服速度
    * @param[in] speed 速度值（-100.0 ~ 100.0）
    *                  正值=上升，负值=下降，0=停止
    * @return ESP_OK: 成功
    */
   esp_err_t servo_driver_set_speed(float speed);
   
   /**
    * @brief 紧急停止伺服
    * @return ESP_OK: 成功
    */
   esp_err_t servo_driver_emergency_stop(void);
   
   /**
    * @brief 获取伺服状态
    * @param[out] state 伺服状态
    * @return ESP_OK: 成功
    */
   esp_err_t servo_driver_get_state(servo_state_t *state);
   ```

3. **数据结构**
   ```c
   /**
    * @brief 伺服配置结构体
    */
   typedef struct {
       uint8_t pwm_gpio;           ///< PWM输出引脚（GPIO0）
       uint32_t pwm_frequency;     ///< PWM频率（Hz，标准50Hz）
       uint16_t pulse_min_us;      ///< 最小脉宽（微秒，标准1000us）
       uint16_t pulse_max_us;      ///< 最大脉宽（微秒，标准2000us）
       ledc_timer_t timer_num;     ///< LEDC定时器编号
       ledc_channel_t channel;     ///< LEDC通道编号
   } servo_config_t;
   
   /**
    * @brief 伺服状态
    */
   typedef struct {
       float current_angle;        ///< 当前角度（度）
       float target_angle;         ///< 目标角度（度）
       float current_speed;        ///< 当前速度（%）
       bool is_moving;             ///< 是否运动中
       uint16_t pwm_pulse_us;      ///< 当前PWM脉宽（微秒）
   } servo_state_t;
   ```

4. **实现要点**
   - 使用[`ledc_timer_config()`](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/peripherals/ledc.html)配置LEDC定时器
   - PWM频率设置为50Hz（伺服标准频率）
   - 使用16位分辨率（65536级，0-65535）
   - 脉宽计算：`duty = (pulse_us * 65536) / 20000`（20ms周期）
   - 角度到脉宽映射：`pulse_us = pulse_min_us + (angle / 180.0) * (pulse_max_us - pulse_min_us)`
   - 平滑运动：使用线性插值，避免突变
   - 线程安全：使用互斥锁保护伺服状态

**验收标准**:
- [x] PWM输出波形正确（示波器验证50Hz，1-2ms脉宽）
- [x] 伺服角度控制准确（±2度）
- [x] 速度设置响应迅速（<20ms）
- [x] 急停功能立即生效
- [x] 编译无警告

---

### 2.2 任务2：AS5600磁编码器I2C驱动

**任务ID**: TASK-LIFT-002  
**优先级**: P0  
**估算时间**: 10小时  
**依赖任务**: 无

**功能描述**:
- 初始化I2C总线
- 配置AS5600编码器参数
- 读取12位角度值（0-4095）
- 实现角度校准和零点设置
- 检测磁铁状态

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/lift/as5600_driver.h`](src/nodes/lift/as5600_driver.h) - AS5600驱动接口
   - [`src/nodes/lift/as5600_driver.c`](src/nodes/lift/as5600_driver.c) - AS5600驱动实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化AS5600编码器
    * @return ESP_OK: 成功
    */
   esp_err_t as5600_init(void);
   
   /**
    * @brief 读取原始角度值
    * @param[out] angle_raw 12位角度值（0-4095）
    * @return 
    *   - ESP_OK: 成功
    *   - ESP_ERR_TIMEOUT: I2C超时
    *   - ESP_ERR_INVALID_STATE: 磁铁未检测到
    */
   esp_err_t as5600_read_angle_raw(uint16_t *angle_raw);
   
   /**
    * @brief 读取角度（度）
    * @param[out] angle_deg 角度值（0-360度）
    * @return ESP_OK: 成功
    */
   esp_err_t as5600_read_angle_degrees(float *angle_deg);
   
   /**
    * @brief 设置零点位置
    * @return ESP_OK: 成功
    */
   esp_err_t as5600_set_zero_position(void);
   
   /**
    * @brief 检测磁铁状态
    * @param[out] status 磁铁状态
    * @return ESP_OK: 成功
    */
   esp_err_t as5600_get_magnet_status(as5600_magnet_status_t *status);
   ```

3. **数据结构**
   ```c
   /**
    * @brief AS5600寄存器地址
    */
   #define AS5600_I2C_ADDR         0x36        ///< I2C地址
   #define AS5600_REG_RAW_ANGLE    0x0C        ///< 原始角度寄存器（2字节）
   #define AS5600_REG_ANGLE        0x0E        ///< 角度寄存器（2字节）
   #define AS5600_REG_STATUS       0x0B        ///< 状态寄存器
   #define AS5600_REG_AGC          0x1A        ///< 自动增益控制
   #define AS5600_REG_MAGNITUDE    0x1B        ///< 磁场强度（2字节）
   
   /**
    * @brief 磁铁状态
    */
   typedef enum {
       AS5600_MAGNET_NOT_DETECTED = 0,     ///< 磁铁未检测到
       AS5600_MAGNET_TOO_WEAK,             ///< 磁场过弱
       AS5600_MAGNET_TOO_STRONG,           ///< 磁场过强
       AS5600_MAGNET_DETECTED              ///< 磁场正常
   } as5600_magnet_status_t;
   
   /**
    * @brief AS5600配置
    */
   typedef struct {
       uint8_t i2c_addr;           ///< I2C地址（0x36）
       uint32_t i2c_timeout_ms;    ///< I2C超时（毫秒）
       float angle_offset;         ///< 角度偏移（度）
       bool reverse_direction;     ///< 反转方向
   } as5600_config_t;
   ```

4. **实现要点**
   - AS5600是12位磁编码器，分辨率：360°/4096 = 0.0879°/LSB
   - 使用[`i2c_master_write_read_device()`](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/peripherals/i2c.html)读取寄存器
   - 角度寄存器为大端序（MSB first），需字节序转换
   - 角度计算：`angle_deg = (angle_raw / 4096.0) * 360.0`
   - 磁铁检测：读取状态寄存器（0x0B），位[5:3]表示磁铁状态
   - I2C超时设置为100ms
   - 失败时重试3次
   - 记录读取时间戳用于速度计算

**验收标准**:
- [x] I2C通信稳定（错误率<1%）
- [x] 角度读取准确（与实际旋转一致）
- [x] 读取延迟<10ms
- [x] 磁铁状态检测正确
- [x] 异常处理正确（总线错误、超时等）

---

### 2.3 任务3：限位开关GPIO中断处理

**任务ID**: TASK-LIFT-003  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-LIFT-001]

**功能描述**:
- 初始化限位开关GPIO
- 配置中断触发方式
- 实现中断服务程序
- 实现防抖动处理
- 提供限位状态查询接口

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化限位开关
    * @return ESP_OK: 成功
    */
   esp_err_t limit_switch_init(void);
   
   /**
    * @brief 上限位中断处理函数
    * @param[in] arg 参数
    */
   void IRAM_ATTR upper_limit_isr_handler(void *arg);
   
   /**
    * @brief 下限位中断处理函数
    * @param[in] arg 参数
    */
   void IRAM_ATTR lower_limit_isr_handler(void *arg);
   
   /**
    * @brief 获取限位状态
    * @param[out] upper 上限位状态（true=触发）
    * @param[out] lower 下限位状态（true=触发）
    * @return ESP_OK: 成功
    */
   esp_err_t limit_switch_get_state(bool *upper, bool *lower);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 限位开关配置
    */
   #define GPIO_UPPER_LIMIT    7       ///< 上限位GPIO
   #define GPIO_LOWER_LIMIT    10      ///< 下限位GPIO
   
   /**
    * @brief 限位开关状态
    */
   typedef struct {
       bool upper_triggered;       ///< 上限位触发
       bool lower_triggered;       ///< 下限位触发
       uint32_t upper_trigger_time;///< 上限位触发时间（毫秒）
       uint32_t lower_trigger_time;///< 下限位触发时间（毫秒）
       uint32_t debounce_ms;       ///< 防抖时间（毫秒）
   } limit_switch_state_t;
   ```

3. **实现要点**
   ```c
   esp_err_t limit_switch_init(void) {
       // 配置上限位GPIO
       gpio_config_t upper_conf = {
           .pin_bit_mask = (1ULL << GPIO_UPPER_LIMIT),
           .mode = GPIO_MODE_INPUT,
           .pull_up_en = GPIO_PULLUP_ENABLE,
           .pull_down_en = GPIO_PULLDOWN_DISABLE,
           .intr_type = GPIO_INTR_NEGEDGE  // 下降沿触发（按下）
       };
       gpio_config(&upper_conf);
       
       // 配置下限位GPIO（同上）
       // ...
       
       // 安装GPIO ISR服务
       gpio_install_isr_service(0);
       gpio_isr_handler_add(GPIO_UPPER_LIMIT, upper_limit_isr_handler, NULL);
       gpio_isr_handler_add(GPIO_LOWER_LIMIT, lower_limit_isr_handler, NULL);
       
       return ESP_OK;
   }
   
   void IRAM_ATTR upper_limit_isr_handler(void *arg) {
       BaseType_t xHigherPriorityTaskWoken = pdFALSE;
       uint32_t current_time = xTaskGetTickCountFromISR();
       
       // 防抖动：检查距离上次触发是否超过防抖时间
       if (current_time - g_upper_trigger_time > DEBOUNCE_MS) {
           g_upper_triggered = true;
           g_upper_trigger_time = current_time;
           
           // 立即停止伺服
           servo_driver_emergency_stop();
           
           // 通知任务处理
           xTaskNotifyFromISR(g_limit_task_handle, 
                              UPPER_LIMIT_EVENT,
                              eSetBits,
                              &xHigherPriorityTaskWoken);
       }
       
       portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
   }
   ```

4. **安全策略**
   - 限位触发后立即停止伺服电机
   - 防抖时间设置为50ms
   - ISR中仅设置标志和停止电机，详细处理在任务中
   - 限位触发后禁止向该方向运动
   - 提供手动解除限位功能（谨慎使用）

**验收标准**:
- [x] 限位开关触发立即停止（<10ms）
- [x] 中断处理稳定无误
- [x] 防抖动机制有效
- [x] 上下限位独立工作
- [x] 压力测试通过

---

## 3. 控制算法层

### 3.1 任务4：位置PID控制器实现

**任务ID**: TASK-LIFT-004  
**优先级**: P0  
**估算时间**: 12小时  
**依赖任务**: [TASK-LIFT-002]

**功能描述**:
- 实现位置PID控制器
- 支持PID参数配置
- 实现死区处理
- 实现积分抗饱和
- 实现输出限幅

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/lift/lift_control.h`](src/nodes/lift/lift_control.h) - 控制算法接口
   - [`src/nodes/lift/lift_control.c`](src/nodes/lift/lift_control.c) - PID控制实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化位置PID控制器
    * @param[in] params PID参数
    * @return ESP_OK: 成功
    */
   esp_err_t lift_pid_init(const position_pid_params_t *params);
   
   /**
    * @brief 执行位置PID控制（单次迭代）
    * @param[in] target_position 目标位置（m）
    * @param[in] current_position 当前位置（m）
    * @param[in] dt 时间步长（秒）
    * @param[out] velocity_output 速度输出（-100.0 ~ 100.0）
    * @return ESP_OK: 成功
    */
   esp_err_t lift_pid_compute(float target_position, float current_position,
                               float dt, float *velocity_output);
   
   /**
    * @brief 重置PID控制器
    * @return ESP_OK: 成功
    */
   esp_err_t lift_pid_reset(void);
   
   /**
    * @brief 设置PID参数
    * @param[in] params PID参数
    * @return ESP_OK: 成功
    */
   esp_err_t lift_pid_set_params(const position_pid_params_t *params);
   ```

3. **数据结构**
   ```c
   /**
    * @brief 位置PID参数
    */
   typedef struct {
       float kp;               ///< 比例系数
       float ki;               ///< 积分系数
       float kd;               ///< 微分系数
       float output_min;       ///< 输出下限（-100.0）
       float output_max;       ///< 输出上限（100.0）
       float integral_min;     ///< 积分下限（抗饱和）
       float integral_max;     ///< 积分上限（抗饱和）
       float deadzone;         ///< 死区（±5mm）
   } position_pid_params_t;
   
   /**
    * @brief 位置PID控制器状态
    */
   typedef struct {
       position_pid_params_t params;   ///< PID参数
       float target_position;          ///< 目标位置（m）
       float current_position;         ///< 当前位置（m）
       float error;                    ///< 当前误差（m）
       float error_prev;               ///< 上次误差（m）
       float error_sum;                ///< 误差积分（m·s）
       float output;                   ///< 控制输出（%）
       uint32_t sample_count;          ///< 采样计数
   } position_pid_t;
   
   /**
    * @brief 默认PID参数（需要现场调试）
    */
   #define PID_KP_DEFAULT      2.0f    ///< 比例系数
   #define PID_KI_DEFAULT      0.5f    ///< 积分系数
   #define PID_KD_DEFAULT      0.2f    ///< 微分系数
   #define PID_DEADZONE        0.005f  ///< 死区5mm
   ```

4. **实现要点**
   - PID算法：`output = Kp*e + Ki*Σe*dt + Kd*de/dt`
   - 比例项：`p_term = kp * error`
   - 积分项：`i_term += ki * error * dt`，限幅到[integral_min, integral_max]
   - 微分项：`d_term = kd * (error - error_prev) / dt`
   - 死区处理：`if (abs(error) < deadzone) error = 0`
   - 输出限幅：`output = constrain(p_term + i_term + d_term, output_min, output_max)`
   - 抗饱和：当输出饱和时停止积分累加
   - 线程安全：使用互斥锁保护PID状态

**验收标准**:
- [x] PID算法正确实现
- [x] 参数可在线调整
- [x] 积分抗饱和工作正常
- [x] 死区处理正确
- [x] 单元测试通过（阶跃响应）

---

### 3.2 任务5：角度到高度的转换算法

**任务ID**: TASK-LIFT-005  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-LIFT-002]

**功能描述**:
- 实现编码器角度到实际高度的转换
- 考虑链轮/皮带轮直径
- 实现多圈计数（累计旋转角度）
- 处理角度跳变

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化高度转换模块
    * @param[in] params 机械参数
    * @return ESP_OK: 成功
    */
   esp_err_t height_conversion_init(const lift_mech_params_t *params);
   
   /**
    * @brief 角度转换为高度
    * @param[in] angle_deg 编码器角度（度）
    * @param[out] height 高度（m）
    * @return ESP_OK: 成功
    */
   esp_err_t angle_to_height(float angle_deg, float *height);
   
   /**
    * @brief 更新高度（处理多圈计数）
    * @param[in] angle_raw AS5600原始角度值（0-4095）
    * @param[out] height 当前高度（m）
    * @return ESP_OK: 成功
    */
   esp_err_t update_height_from_angle(uint16_t angle_raw, float *height);
   
   /**
    * @brief 重置高度为零点
    * @return ESP_OK: 成功
    */
   esp_err_t reset_height_zero(void);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 升降机构机械参数
    */
   typedef struct {
       float sprocket_diameter;    ///< 链轮直径（m）：0.1m
       float gear_ratio;           ///< 减速比：1.0（直连）
       float max_height;           ///< 最大高度（m）：3.0m
       float min_height;           ///< 最小高度（m）：0.0m
   } lift_mech_params_t;
   
   /**
    * @brief 高度跟踪状态
    */
   typedef struct {
       float current_height;       ///< 当前高度（m）
       uint16_t angle_prev;        ///< 上次角度值（0-4095）
       int32_t revolution_count;   ///< 旋转圈数
       float total_angle;          ///< 累计角度（度）
       lift_mech_params_t mech;    ///< 机械参数
   } height_tracker_t;
   ```

3. **转换公式**
   ```c
   // 角度到高度转换（假设链传动）
   // 1. 计算链轮周长
   float circumference = M_PI * sprocket_diameter;
   
   // 2. 角度转换为高度
   // height = (angle_deg / 360.0) * circumference / gear_ratio
   float height = (angle_deg / 360.0f) * circumference;
   
   // 3. 多圈计数（处理0-360度跳变）
   int16_t angle_delta = (int16_t)angle_raw - (int16_t)angle_prev;
   if (angle_delta > 2048) {
       // 从360跳到0（下降）
       revolution_count--;
   } else if (angle_delta < -2048) {
       // 从0跳到360（上升）
       revolution_count++;
   }
   float total_angle = revolution_count * 360.0 + (angle_raw * 360.0 / 4096.0);
   ```

4. **实现要点**
   - AS5600只能检测0-360度，需要软件实现多圈计数
   - 使用角度差判断旋转方向（>180度视为跳变）
   - 累计角度 = 旋转圈数 × 360° + 当前角度
   - 高度范围限制：[0, max_height]
   - 零点校准：记录当前角度作为零点偏移
   - 参数从配置文件读取（NVS或sdkconfig）

**验收标准**:
- [x] 转换公式正确（数学验证）
- [x] 多圈计数准确
- [x] 跳变处理正确
- [x] 高度范围限制生效
- [x] 单元测试通过

---

### 3.3 任务6：梯形速度规划

**任务ID**: TASK-LIFT-006  
**优先级**: P1  
**估算时间**: 10小时  
**依赖任务**: [TASK-LIFT-004, TASK-LIFT-005]

**功能描述**:
- 实现梯形速度曲线规划
- 支持加速、匀速、减速三阶段
- 计算到达目标位置的最优轨迹
- 提供速度前瞻功能

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化轨迹规划器
    * @param[in] params 轨迹参数
    * @return ESP_OK: 成功
    */
   esp_err_t trajectory_planner_init(const trajectory_params_t *params);
   
   /**
    * @brief 计算梯形速度轨迹
    * @param[in] current_position 当前位置（m）
    * @param[in] target_position 目标位置（m）
    * @param[in] current_velocity 当前速度（m/s）
    * @param[out] target_velocity 目标速度（m/s）
    * @return ESP_OK: 成功
    */
   esp_err_t trajectory_compute(float current_position, 
                                 float target_position,
                                 float current_velocity,
                                 float *target_velocity);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 轨迹规划参数
    */
   typedef struct {
       float max_velocity;         ///< 最大速度（m/s）：0.5
       float max_acceleration;     ///< 最大加速度（m/s²）：0.5
       float deceleration_dist;    ///< 减速距离（m）
   } trajectory_params_t;
   
   /**
    * @brief 轨迹状态
    */
   typedef enum {
       TRAJ_STATE_IDLE,            ///< 空闲
       TRAJ_STATE_ACCELERATING,    ///< 加速阶段
       TRAJ_STATE_CRUISING,        ///< 匀速阶段
       TRAJ_STATE_DECELERATING,    ///< 减速阶段
       TRAJ_STATE_REACHED          ///< 已到达
   } trajectory_state_t;
   ```

3. **梯形轨迹算法**
   ```c
   esp_err_t trajectory_compute(float current_pos, float target_pos,
                                 float current_vel, float *target_vel) {
       float distance = target_pos - current_pos;
       float direction = (distance > 0) ? 1.0f : -1.0f;
       distance = fabsf(distance);
       
       // 计算减速距离（v² = v₀² + 2as）
       float decel_dist = (current_vel * current_vel) / (2.0f * max_acceleration);
       
       if (distance < 0.005f) {
           // 已到达（5mm死区）
           *target_vel = 0.0f;
           return ESP_OK;
       }
       
       if (distance <= decel_dist) {
           // 开始减速
           *target_vel = sqrtf(2.0f * max_acceleration * distance) * direction;
       } else {
           // 加速或匀速
           if (fabsf(current_vel) < max_velocity) {
               // 加速阶段
               *target_vel = max_velocity * direction;
           } else {
               // 匀速阶段
               *target_vel = current_vel;
           }
       }
       
       return ESP_OK;
   }
   ```

4. **实现要点**
   - 三阶段规划：加速→匀速→减速
   - 减速距离计算：`s = v² / (2a)`
   - 考虑当前速度，避免速度跳变
   - 距离目标<5mm时认为到达
   - 与PID控制器配合使用（先规划轨迹，再PID跟踪）

**验收标准**:
- [x] 速度曲线平滑
- [x] 到达目标不超调
- [x] 响应时间合理
- [x] 不同距离测试通过
- [x] 单元测试通过

---

## 4. 节点应用层

### 4.1 任务7：节点初始化和主循环

**任务ID**: TASK-LIFT-007  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: [TASK-LIFT-004, TASK-LIFT-005]

**功能描述**:
- 初始化所有模块
- 创建ROS节点
- 启动控制任务
- 实现主循环
- 错误处理和恢复

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/lift/lift_node.h`](src/nodes/lift/lift_node.h) - 节点主接口
   - [`src/nodes/lift/lift_node.c`](src/nodes/lift/lift_node.c) - 节点主逻辑

2. **关键函数**
   ```c
   /**
    * @brief 初始化升降节点
    * @return ESP_OK: 成功
    */
   esp_err_t lift_node_init(void);
   
   /**
    * @brief 升降节点主函数（不返回）
    */
   void lift_node_main(void);
   
   /**
    * @brief 停止升降节点
    * @return ESP_OK: 成功
    */
   esp_err_t lift_node_stop(void);
   ```

3. **初始化流程**
   ```c
   esp_err_t lift_node_init(void) {
       ESP_LOGI(TAG, "Initializing lift node...");
       
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
       
       // 4. 初始化升降专用模块
       ESP_ERROR_CHECK(servo_driver_init());
       ESP_ERROR_CHECK(as5600_init());
       ESP_ERROR_CHECK(limit_switch_init());
       ESP_ERROR_CHECK(height_conversion_init(&g_mech_params));
       ESP_ERROR_CHECK(lift_pid_init(&g_pid_params));
       ESP_ERROR_CHECK(trajectory_planner_init(&g_traj_params));
       
       // 5. 创建ROS接口
       ESP_ERROR_CHECK(lift_create_ros_interfaces());
       
       ESP_LOGI(TAG, "Lift node initialized successfully");
       return ESP_OK;
   }
   ```

4. **主循环**
   ```c
   void lift_node_main(void) {
       lift_node_init();
       
       // 启动控制和发布任务
       xTaskCreate(position_control_task, "pos_ctrl", 4096, NULL, 10, NULL);
       xTaskCreate(position_publish_task, "pos_pub", 4096, NULL, 10, NULL);
       xTaskCreate(limit_monitor_task, "limit_mon", 2048, NULL, 15, NULL);
       
       // 主循环：ROS spin
       while (1) {
           ros_comm_spin_once(100);
           
           // 检查WiFi和ROS连接状态
           if (!wifi_manager_is_connected()) {
               ESP_LOGW(TAG, "WiFi disconnected, attempting reconnect...");
               wifi_manager_connect();
           }
           
           if (!ros_comm_is_connected()) {
               ESP_LOGW(TAG, "ROS disconnected, attempting reconnect...");
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
- [x] 主循环稳定运行
- [x] 错误恢复正常

---

### 4.2 任务8：ROS话题订阅（/lift/cmd_position）

**任务ID**: TASK-LIFT-008  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-LIFT-007]

**功能描述**:
- 订阅高度指令话题
- 解析Float32消息
- 更新目标高度
- 实现高度范围检查
- 超时保护

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 创建cmd_position订阅者
    * @return ESP_OK: 成功
    */
   esp_err_t lift_create_cmd_position_subscriber(void);
   
   /**
    * @brief cmd_position回调函数
    * @param[in] msg Float32消息
    * @param[in] user_data 用户数据
    */
   void lift_cmd_position_callback(const void *msg, void *user_data);
   ```

2. **实现要点**
   ```c
   void lift_cmd_position_callback(const void *msg, void *user_data) {
       const std_msgs__msg__Float32 *height_msg = 
           (const std_msgs__msg__Float32 *)msg;
       
       float target_height = height_msg->data;
       
       // 高度范围检查
       if (target_height < 0.0f || target_height > g_mech_params.max_height) {
           ESP_LOGW(TAG, "Target height %.2f out of range [0, %.2f]",
                    target_height, g_mech_params.max_height);
           return;
       }
       
       // 检查限位状态
       if (target_height > g_current_height && g_upper_limit_triggered) {
           ESP_LOGW(TAG, "Upper limit triggered, cannot move up");
           return;
       }
       if (target_height < g_current_height && g_lower_limit_triggered) {
           ESP_LOGW(TAG, "Lower limit triggered, cannot move down");
           return;
       }
       
       // 更新目标高度
       g_target_height = target_height;
       g_cmd_timeout = 0;
       
       ESP_LOGI(TAG, "cmd_position: %.3f m", target_height);
   }
   ```

3. **超时保护**
   ```c
   // 在控制任务中检查超时
   #define CMD_POSITION_TIMEOUT_MS  1000  // 1秒无指令则保持
   
   void position_control_task(void *pvParameters) {
       while (1) {
           g_cmd_timeout += 50;  // 每50ms增加
           
           if (g_cmd_timeout > CMD_POSITION_TIMEOUT_MS) {
               // 超时，保持当前位置
               g_target_height = g_current_height;
           }
           
           lift_position_control_step();
           vTaskDelay(pdMS_TO_TICKS(50));
       }
   }
   ```

**验收标准**:
- [x] 成功订阅话题
- [x] 回调函数正确执行
- [x] 高度范围检查生效
- [x] 限位保护正常
- [x] 响应延迟<50ms

---

### 4.3 任务9：ROS话题发布（/lift/position）

**任务ID**: TASK-LIFT-009  
**优先级**: P0  
**估算时间**: 4小时  
**依赖任务**: [TASK-LIFT-007, TASK-LIFT-005]

**功能描述**:
- 创建位置发布者
- 定期发布高度消息
- 保证发布频率20Hz

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 创建position发布者
    * @return ESP_OK: 成功
    */
   esp_err_t lift_create_position_publisher(void);
   
   /**
    * @brief 发布位置消息
    * @return ESP_OK: 成功
    */
   esp_err_t lift_publish_position(void);
   ```

2. **实现要点**
   ```c
   esp_err_t lift_create_position_publisher(void) {
       return ros_comm_create_publisher(
           &g_position_pub,
           "/lift/position",
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
           &QOS_SENSOR_DATA
       );
   }
   
   // 在发布任务中
   void position_publish_task(void *pvParameters) {
       TickType_t last_wake_time = xTaskGetTickCount();
       const TickType_t frequency = pdMS_TO_TICKS(50);  // 20Hz
       
       std_msgs__msg__Float32 pos_msg;
       
       while (1) {
           // 读取当前高度
           pos_msg.data = g_current_height;
           
           // 发布消息
           ros_comm_publish(&g_position_pub, &pos_msg);
           
           vTaskDelayUntil(&last_wake_time, frequency);
       }
   }
   ```

**验收标准**:
- [x] 发布者创建成功
- [x] 消息正常发布
- [x] 频率稳定20Hz
- [x] `ros2 topic hz`验证通过
- [x] 数据准确

---

### 4.4 任务10：ROS服务实现

**任务ID**: TASK-LIFT-010  
**优先级**: P1  
**估算时间**: 10小时  
**依赖任务**: [TASK-LIFT-007]

**功能描述**:
- 实现[`/lift/home`](../需求分析/ROS接口定义文档.md)服务（归零）
- 实现[`/lift/emergency_stop`](../需求分析/ROS接口定义文档.md)服务
- 实现[`/lift/set_speed`](../需求分析/ROS接口定义文档.md)服务
- 提供配置查询服务

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 创建所有服务
    * @return ESP_OK: 成功
    */
   esp_err_t lift_create_services(void);
   
   /**
    * @brief home服务回调（归零）
    */
   void lift_home_service_callback(const void *request, 
                                    void *response, void *user_data);
   
   /**
    * @brief emergency_stop服务回调
    */
   void lift_emergency_stop_service_callback(const void *request,
                                              void *response, void *user_data);
   
   /**
    * @brief set_speed服务回调
    */
   void lift_set_speed_service_callback(const void *request,
                                         void *response, void *user_data);
   ```

2. **归零服务实现**
   ```c
   void lift_home_service_callback(const void *request, 
                                    void *response, void *user_data) {
       std_srvs__srv__Trigger_Response *res = 
           (std_srvs__srv__Trigger_Response *)response;
       
       ESP_LOGI(TAG, "Home service called");
       
       // 1. 缓慢下降到下限位
       g_target_height = 0.0f;
       float slow_speed = 0.1f;  // 10% 慢速
       
       while (!g_lower_limit_triggered) {
           servo_driver_set_speed(-slow_speed);
           vTaskDelay(pdMS_TO_TICKS(100));
       }
       
       // 2. 停止
       servo_driver_emergency_stop();
       
       // 3. 重置高度为零
       reset_height_zero();
       g_current_height = 0.0f;
       g_target_height = 0.0f;
       
       res->success = true;
       strcpy(res->message.data, "Homing completed");
       
       ESP_LOGI(TAG, "Homing completed");
   }
   ```

**验收标准**:
- [x] 服务正常响应
- [x] 归零功能正确
- [x] 紧急停止立即生效
- [x] 速度设置生效
- [x] `ros2 service call`测试通过

---

### 4.5 任务11：限位保护和安全控制

**任务ID**: TASK-LIFT-011  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-LIFT-003, TASK-LIFT-007]

**功能描述**:
- 监控限位开关状态
- 触发限位时立即停止
- 防止向限位方向运动
- 提供手动解除功能
- 发布限位状态

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化限位保护
    * @return ESP_OK: 成功
    */
   esp_err_t lift_limit_protection_init(void);
   
   /**
    * @brief 限位监控任务
    * @param[in] pvParameters 参数
    */
   void limit_monitor_task(void *pvParameters);
   
   /**
    * @brief 检查运动是否允许
    * @param[in] direction 方向（+1上升，-1下降）
    * @return true=允许，false=禁止
    */
   bool lift_is_movement_allowed(int8_t direction);
   ```

2. **实现要点**
   ```c
   void limit_monitor_task(void *pvParameters) {
       uint32_t notification_value;
       
       while (1) {
           // 等待限位中断通知
           if (xTaskNotifyWait(0, 0xFFFFFFFF, &notification_value, 100)) {
               if (notification_value & UPPER_LIMIT_EVENT) {
                   ESP_LOGE(TAG, "UPPER LIMIT TRIGGERED!");
                   servo_driver_emergency_stop();
                   g_upper_limit_triggered = true;
                   
                   // 发布诊断信息
                   lift_publish_diagnostic(DIAGNOSTIC_WARN, 
                                           "Upper limit reached");
               }
               
               if (notification_value & LOWER_LIMIT_EVENT) {
                   ESP_LOGE(TAG, "LOWER LIMIT TRIGGERED!");
                   servo_driver_emergency_stop();
                   g_lower_limit_triggered = true;
                   
                   lift_publish_diagnostic(DIAGNOSTIC_WARN,
                                           "Lower limit reached");
               }
           }
           
           // 周期检查限位状态
           bool upper_active = (gpio_get_level(GPIO_UPPER_LIMIT) == 0);
           bool lower_active = (gpio_get_level(GPIO_LOWER_LIMIT) == 0);
           
           // 释放限位（按钮松开）
           if (!upper_active && g_upper_limit_triggered) {
               ESP_LOGI(TAG, "Upper limit released");
               g_upper_limit_triggered = false;
           }
           if (!lower_active && g_lower_limit_triggered) {
               ESP_LOGI(TAG, "Lower limit released");
               g_lower_limit_triggered = false;
           }
       }
   }
   
   bool lift_is_movement_allowed(int8_t direction) {
       if (direction > 0 && g_upper_limit_triggered) {
           return false;  // 上限位触发，禁止上升
       }
       if (direction < 0 && g_lower_limit_triggered) {
           return false;  // 下限位触发，禁止下降
       }
       return true;
   }
   ```

**验收标准**:
- [x] 限位触发立即停止
- [x] 防止反向运动生效
- [x] 限位释放后恢复正常
- [x] 诊断信息正确发布
- [x] 压力测试通过

---

### 4.6 任务12：OLED状态显示

**任务ID**: TASK-LIFT-012  
**优先级**: P1  
**估算时间**: 6小时  
**依赖任务**: [TASK-LIFT-007]

**功能描述**:
- 显示当前高度
- 显示目标高度
- 显示限位状态
- 显示节点状态
- 显示错误信息

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 更新OLED显示
    * @return ESP_OK: 成功
    */
   esp_err_t lift_update_oled_display(void);
   ```

2. **显示布局**
   ```
   ┌────────────────────────┐
   │ LIFT NODE-02           │  ← 标题
   ├────────────────────────┤
   │ WiFi:✓ ROS:✓           │  ← 连接状态
   │ Cur: 1.25m Tgt: 1.50m  │  ← 当前/目标高度
   │ Limit: ✗U  ✓L          │  ← 限位状态
   │ IP: 192.168.1.102      │  ← IP地址
   └────────────────────────┘
   ```

3. **实现要点**
   ```c
   void oled_display_update_task(void *pvParameters) {
       char line_buf[32];
       
       while (1) {
           // 高度显示
           snprintf(line_buf, sizeof(line_buf), 
                    "Cur:%.2fm Tgt:%.2fm", 
                    g_current_height, g_target_height);
           oled_ui_show_custom_text(2, line_buf);
           
           // 限位状态
           snprintf(line_buf, sizeof(line_buf), 
                    "Limit: %cU  %cL",
                    g_upper_limit_triggered ? '✓' : '✗',
                    g_lower_limit_triggered ? '✓' : '✗');
           oled_ui_show_custom_text(3, line_buf);
           
           vTaskDelay(pdMS_TO_TICKS(1000));  // 1Hz更新
       }
   }
   ```

**验收标准**:
- [x] 显示内容准确
- [x] 更新及时（1Hz）
- [x] 布局清晰易读
- [x] 无闪烁抖动
- [x] CPU占用<5%

---

## 5. 集成测试

### 5.1 任务13：单元测试

**任务ID**: TASK-LIFT-013  
**优先级**: P1  
**估算时间**: 12小时  
**依赖任务**: [TASK-LIFT-001~012]

**功能描述**:
- PID控制器单元测试
- 角度转换算法测试
- 轨迹规划算法测试
- 覆盖率>80%

**技术实现细节**:

1. **测试文件**
   - [`test/unit/test_lift_control.c`](test/unit/test_lift_control.c)
   - [`test/unit/test_height_conversion.c`](test/unit/test_height_conversion.c)
   - [`test/unit/test_trajectory_planner.c`](test/unit/test_trajectory_planner.c)

2. **测试用例**
   ```c
   // test_lift_control.c
   void test_pid_position_control(void);
   void test_pid_deadzone(void);
   void test_pid_integral_limit(void);
   void test_pid_output_limit(void);
   
   // test_height_conversion.c
   void test_angle_to_height_conversion(void);
   void test_multi_revolution_counting(void);
   void test_angle_wraparound(void);
   void test_height_zero_reset(void);
   
   // test_trajectory_planner.c
   void test_trapezoidal_profile(void);
   void test_acceleration_phase(void);
   void test_deceleration_phase(void);
   void test_short_distance_move(void);
   ```

3. **实现要点**
   - 使用Unity测试框架
   - 每个函数至少3个测试用例
   - 使用Mock模拟硬件
   - 浮点数比较使用容差
   - 自动化测试脚本

**验收标准**:
- [x] 所有测试用例通过
- [x] 代码覆盖率>80%
- [x] 无内存泄漏
- [x] 测试报告生成
- [x] CI集成

---

### 5.2 任务14：硬件在环测试

**任务ID**: TASK-LIFT-014  
**优先级**: P0  
**估算时间**: 16小时  
**依赖任务**: [TASK-LIFT-001~012]

**功能描述**:
- 实际伺服运行测试
- 编码器精度测试
- 位置控制性能测试
- 限位保护测试

**技术实现细节**:

1. **测试场景**
   - 升降1米（上升/下降）
   - 多次往返运动（10次）
   - 精确定位测试（多个高度点）
   - 限位触发测试
   - 长时间连续运行（1小时）

2. **测试指标**
   ```
   ┌──────────────────┬──────────┬──────────┐
   │ 测试项           │ 目标值   │ 实测值   │
   ├──────────────────┼──────────┼──────────┤
   │ 定位精度         │ ±5mm     │          │
   │ 上升1m时间       │ 2-5秒    │          │
   │ 响应时间         │ <200ms   │          │
   │ 限位停止时间     │ <100ms   │          │
   │ 位置发布频率     │ 20±1Hz   │          │
   │ CPU占用          │ <60%     │          │
   │ 内存占用         │ <150KB   │          │
   └──────────────────┴──────────┴──────────┘
   ```

3. **测试工具**
   - 卷尺：测量实际高度
   
   - 激光测距仪：精确测量（可选）
   - ROS工具：`ros2 topic hz`, `ros2 topic echo`
   - Rviz：可视化位置数据
   - `top`命令：监控系统资源

4. **测试步骤**
   ```bash
   # 1. 启动ROS Agent
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   
   # 2. 烧录并启动节点
   pio run -e lift_node -t upload
   
   # 3. 监控位置话题
   ros2 topic echo /lift/position
   
   # 4. 发送高度指令测试
   ros2 topic pub /lift/cmd_position std_msgs/Float32 "data: 1.5"
   
   # 5. 测试归零服务
   ros2 service call /lift/home std_srvs/Trigger
   
   # 6. 记录测试结果
   ```

**验收标准**:
- [x] 所有测试场景通过
- [x] 性能指标达标
- [x] 无系统崩溃
- [x] 无内存泄漏（>1小时运行）
- [x] 测试报告完整

---

## 6. 依赖关系图

### 6.1 任务依赖关系（DAG图）

```
初始化顺序（从上到下）：

1. 驱动层（并行）
   ├─→ TASK-001: 伺服PWM驱动
   ├─→ TASK-002: AS5600编码器I2C驱动
   └─→ TASK-003: 限位开关GPIO中断 [依赖001]

2. 控制算法层（依赖驱动层）
   ├─→ TASK-004: 位置PID控制器 [依赖002]
   ├─→ TASK-005: 角度到高度转换 [依赖002]
   └─→ TASK-006: 梯形速度规划 [依赖004,005]

3. 节点应用层（依赖所有底层）
   ├─→ TASK-007: 节点初始化 [依赖004,005]
   ├─→ TASK-008: cmd_position订阅 [依赖007]
   ├─→ TASK-009: position发布 [依赖007,005]
   ├─→ TASK-010: ROS服务 [依赖007]
   ├─→ TASK-011: 限位保护 [依赖003,007]
   └─→ TASK-012: OLED显示 [依赖007]

4. 测试层（依赖全部）
   ├─→ TASK-013: 单元测试 [依赖001-012]
   └─→ TASK-014: 硬件测试 [依赖001-012]
```

### 6.2 任务依赖矩阵

| 任务ID | 任务名称 | 依赖任务 | 可并行任务 | 估算时间 |
|--------|---------|---------|-----------|---------|
| TASK-LIFT-001 | 伺服PWM驱动 | 无 | 002, 005 | 8h |
| TASK-LIFT-002 | AS5600编码器 | 无 | 001, 005 | 10h |
| TASK-LIFT-003 | 限位开关中断 | 001 | 002, 004, 005 | 6h |
| TASK-LIFT-004 | 位置PID控制 | 002 | 005, 006 | 12h |
| TASK-LIFT-005 | 角度高度转换 | 002 | 001, 004, 006 | 6h |
| TASK-LIFT-006 | 梯形速度规划 | 004, 005 | - | 10h |
| TASK-LIFT-007 | 节点初始化 | 004, 005 | 013 | 8h |
| TASK-LIFT-008 | cmd_position订阅 | 007 | 009, 010, 011, 012 | 6h |
| TASK-LIFT-009 | position发布 | 007, 005 | 008, 010, 011, 012 | 4h |
| TASK-LIFT-010 | ROS服务 | 007 | 008, 009, 011, 012 | 10h |
| TASK-LIFT-011 | 限位保护 | 003, 007 | 008, 009, 010, 012 | 6h |
| TASK-LIFT-012 | OLED显示 | 007 | 008, 009, 010, 011 | 6h |
| TASK-LIFT-013 | 单元测试 | 001-012 | 014 | 12h |
| TASK-LIFT-014 | 硬件测试 | 001-012 | 013 | 16h |

### 6.3 开发里程碑

**阶段1：驱动层（1周）**
- 完成伺服驱动（TASK-001）
- 完成AS5600驱动（TASK-002）
- 完成限位开关（TASK-003）
- 目标：伺服能够响应控制，编码器能够读取

**阶段2：控制算法层（2周）**
- 完成PID控制器（TASK-004）
- 完成高度转换（TASK-005）
- 完成速度规划（TASK-006）
- 目标：实现精确的位置控制

**阶段3：节点应用层（2周）**
- 完成节点框架（TASK-007）
- 完成ROS接口（TASK-008~010）
- 完成安全和显示（TASK-011~012）
- 目标：完整的ROS节点功能

**阶段4：测试和调优（1.5周）**
- 完成单元测试（TASK-013）
- 完成硬件测试（TASK-014）
- 目标：稳定可靠的系统

**总计：6.5周（约1.5个月）**

### 6.4 关键路径

```
Critical Path（关键路径）:
TASK-002 → TASK-004 → TASK-006 → TASK-007 → TASK-014
(10h)      (12h)       (10h)       (8h)        (16h)
总计：56小时（约7个工作日）
```

---

## 附录A：配置参数说明

### A.1 Kconfig配置项

```kconfig
menu "Lift Node Configuration"
    
    config LIFT_SPROCKET_DIAMETER
        int "Sprocket Diameter (mm)"
        range 50 200
        default 100
        help
            Diameter of sprocket/pulley in millimeters
    
    config LIFT_MAX_HEIGHT
        int "Max Height (mm)"
        range 1000 5000
        default 3000
        help
            Maximum lift height in millimeters
    
    config LIFT_MAX_VELOCITY
        int "Max Velocity (mm/s)"
        range 50 1000
        default 500
        help
            Maximum lift velocity in mm/s
    
    config LIFT_MAX_ACCELERATION
        int "Max Acceleration (mm/s²)"
        range 100 1000
        default 500
        help
            Maximum acceleration in mm/s²
    
    config LIFT_PID_KP
        int "PID Kp (x1000)"
        range 0 10000
        default 2000
        help
            PID proportional gain (multiplied by 1000)
    
    config LIFT_PID_KI
        int "PID Ki (x1000)"
        range 0 10000
        default 500
        help
            PID integral gain (multiplied by 1000)
    
    config LIFT_PID_KD
        int "PID Kd (x1000)"
        range 0 10000
        default 200
        help
            PID derivative gain (multiplied by 1000)

endmenu
```

### A.2 NVS配置键

| 配置键 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `lift_sprocket_dia` | float | 0.1 | 链轮直径（m） |
| `lift_max_height` | float | 3.0 | 最大高度（m） |
| `lift_max_vel` | float | 0.5 | 最大速度（m/s） |
| `lift_max_accel` | float | 0.5 | 最大加速度（m/s²） |
| `lift_pid_kp` | float | 2.0 | PID Kp |
| `lift_pid_ki` | float | 0.5 | PID Ki |
| `lift_pid_kd` | float | 0.2 | PID Kd |
| `lift_angle_offset` | float | 0.0 | 编码器角度偏移（度） |

---

## 附录B：ROS接口总结

### B.1 订阅话题

| 话题名称 | 消息类型 | QoS | 频率 | 说明 |
|---------|---------|-----|------|------|
| [`/lift/cmd_position`](../需求分析/ROS接口定义文档.md) | `std_msgs/Float32` | RELIABLE | - | 目标高度指令 |

### B.2 发布话题

| 话题名称 | 消息类型 | QoS | 频率 | 说明 |
|---------|---------|-----|------|------|
| [`/lift/position`](../需求分析/ROS接口定义文档.md) | `std_msgs/Float32` | RELIABLE | 20Hz | 当前高度 |
| [`/lift/diagnostics`](../需求分析/ROS接口定义文档.md) | `diagnostic_msgs/DiagnosticStatus` | RELIABLE | 1Hz | 诊断信息 |

### B.3 服务

| 服务名称 | 服务类型 | 说明 |
|---------|---------|------|
| [`/lift/home`](../需求分析/ROS接口定义文档.md) | `std_srvs/Trigger` | 归零校准 |
| [`/lift/emergency_stop`](../需求分析/ROS接口定义文档.md) | `std_srvs/Trigger` | 紧急停止 |
| [`/lift/set_speed`](../需求分析/ROS接口定义文档.md) | `lift_interfaces/SetSpeed` | 设置升降速度 |

---

## 附录C：AS5600编码器技术参数

### C.1 硬件特性

| 参数 | 规格 | 说明 |
|------|------|------|
| **I2C地址** | 0x36 | 固定地址 |
| **分辨率** | 12位 | 4096步/圈 |
| **角度精度** | 0.0879°/LSB | 360°/4096 |
| **更新率** | 150 Hz | 最大采样频率 |
| **电源电压** | 3.3V / 5V | 兼容3.3V和5V |
| **工作温度** | -40°C ~ 125°C | 工业级 |

### C.2 寄存器地址表

| 寄存器 | 地址 | 长度 | 访问 | 说明 |
|--------|------|------|------|------|
| ZMCO | 0x00 | 1 | R | 编程次数 |
| ZPOS | 0x01 | 2 | R/W | 零点位置 |
| MPOS | 0x03 | 2 | R/W | 最大位置 |
| MANG | 0x05 | 2 | R/W | 最大角度 |
| CONF | 0x07 | 2 | R/W | 配置寄存器 |
| RAW_ANGLE | 0x0C | 2 | R | 原始角度 |
| ANGLE | 0x0E | 2 | R | 处理后角度 |
| STATUS | 0x0B | 1 | R | 状态寄存器 |
| AGC | 0x1A | 1 | R | 自动增益 |
| MAGNITUDE | 0x1B | 2 | R | 磁场强度 |

### C.3 状态寄存器位定义

```c
/**
 * @brief AS5600状态寄存器（0x0B）位定义
 */
#define AS5600_STATUS_MH    (1 << 3)  ///< 磁场过强
#define AS5600_STATUS_ML    (1 << 4)  ///< 磁场过弱
#define AS5600_STATUS_MD    (1 << 5)  ///< 磁铁检测到

// 磁铁状态判断
if (status & AS5600_STATUS_MD) {
    // 磁铁已检测到
    if (status & AS5600_STATUS_MH) {
        // 磁场过强
    } else if (status & AS5600_STATUS_ML) {
        // 磁场过弱
    } else {
        // 磁场正常
    }
} else {
    // 磁铁未检测到
}
```

---

## 附录D：代码规范参考

所有代码必须符合[`开发规范.md`](../开发架构文档/开发规范.md)：

- **函数命名**：`module_action_object()`格式，如[`servo_driver_set_angle()`](src/nodes/lift/servo_driver.c)
- **变量命名**：小写下划线，如`sprocket_diameter`, `angle_raw`
- **常量命名**：大写下划线，如`MAX_HEIGHT`, `PID_KP_DEFAULT`
- **错误处理**：统一使用`esp_err_t`返回值
- **日志输出**：使用`ESP_LOGI`/`ESP_LOGW`/`ESP_LOGE`，标签为模块名
- **线程安全**：使用互斥锁保护共享资源
- **注释规范**：使用Doxygen风格注释

---

## 附录E：测试要求

每个任务完成后必须通过以下测试：

1. **编译测试**
   - 无编译错误
   - 无编译警告（`-Wall -Wextra`）
   - 代码大小<800KB（Flash限制）

2. **单元测试**
   - 函数功能正确
   - 边界条件处理
   - 错误处理验证
   - 覆盖率>80%

3. **集成测试**
   - 模块间接口正常
   - 数据流正确
   - 性能达标
   - ROS通信正常

4. **硬件测试**
   - 实际运行稳定
   - 精度满足要求
   - 长时间运行无崩溃（>1小时）
   - 无内存泄漏

---

## 附录F：参考资源

- **ESP-IDF文档**: https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/
- **Micro-ROS文档**: https://micro.ros.org/docs/
- **ROS 2 Humble文档**: https://docs.ros.org/en/humble/
- **AS5600数据手册**: https://ams.com/as5600
- **伺服电机控制**: https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino
- **PID控制**: https://en.wikipedia.org/wiki/PID_controller
- **Adafruit QT Py ESP32-C3**: https://learn.adafruit.com/adafruit-qt-py-esp32-c3

---

**文档结束**

**编写人**：架构设计组  
**审核人**：技术负责人  
**版本**：v1.0.0  
**日期**：2025-10-23