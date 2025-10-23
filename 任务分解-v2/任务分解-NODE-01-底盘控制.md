
# 任务分解：NODE-01底盘控制节点

## 文档信息

| 项目 | 内容 |
|------|------|
| **文档标题** | NODE-01底盘控制节点任务分解文档 |
| **文档版本** | v1.0.0 |
| **编制日期** | 2025-10-23 |
| **适用范围** | NODE-01底盘控制节点 |
| **节点职责** | 差速移动底盘运动控制 |
| **硬件平台** | Adafruit QT Py ESP32-C3 |

---

## 目录

- [1. 节点概述](#1-节点概述)
- [2. 驱动层任务（motor_driver模块）](#2-驱动层任务motor_driver模块)
- [3. 控制算法层（chassis_control模块）](#3-控制算法层chassis_control模块)
- [4. 里程计层（odometry模块）](#4-里程计层odometry模块)
- [5. 节点应用层（chassis_node模块）](#5-节点应用层chassis_node模块)
- [6. 集成测试](#6-集成测试)
- [7. 依赖关系图](#7-依赖关系图)

---

## 1. 节点概述

### 1.1 节点职责

NODE-01底盘控制节点负责管理建筑喷涂施工机器人的差速移动底盘，实现精确的运动控制和位置跟踪。

**管理硬件**：
- 左侧驱动轮电机 + 编码器
- 右侧驱动轮电机 + 编码器
- 紧急停止开关

**主要功能**：
1. 接收ROS速度指令（`/chassis/cmd_vel`）
2. 执行电机PWM控制
3. 读取编码器反馈（通过I2C）
4. 计算里程计数据
5. 发布里程计（`/chassis/odom` @ 50Hz）
6. 实现PID速度闭环控制
7. 提供配置和诊断服务

### 1.2 硬件接口（Adafruit QT Py ESP32-C3）

| 接口类型 | 引脚分配 | 连接设备 | 说明 |
|---------|---------|---------|------|
| PWM输出 | GPIO0 | 左侧电机驱动器PWM | LEDC通道0 |
| PWM输出 | GPIO1 | 右侧电机驱动器PWM | LEDC通道1 |
| I2C总线 | GPIO8 (SDA), GPIO9 (SCL) | 电机驱动器I2C | 读取编码器 |
| GPIO输入 | GPIO10 | 紧急停止开关 | 上拉输入 |
| I2C显示 | GPIO5 (SCL), GPIO6 (SDA) | 128x64 OLED | 状态显示 |
| UART | GPIO20 (RX), GPIO21 (TX) | 调试串口 | 日志输出 |

### 1.3 性能指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| **速度控制精度** | ±5% | 实际速度与目标速度偏差 |
| **响应时间** | <100ms | 速度指令到电机响应 |
| **里程计精度** | ±2% | 1米直线运动误差<2cm |
| **发布频率** | 50 Hz | 里程计数据发布频率 |
| **PID控制周期** | 20ms | 50Hz控制循环 |

### 1.4 代码结构

```
src/nodes/chassis/
├── chassis_node.h          # 节点主接口
├── chassis_node.c          # 节点主逻辑
├── chassis_control.h       # 控制算法接口
├── chassis_control.c       # PID控制实现
├── motor_driver.h          # 电机驱动接口
├── motor_driver.c          # PWM控制和I2C编码器读取
├── odometry.h              # 里程计接口
├── odometry.c              # 里程计计算
└── README.md               # 节点说明
```

---

## 2. 驱动层任务（motor_driver模块）

### 2.1 任务1：PWM电机驱动初始化和控制

**任务ID**: TASK-CHASSIS-001  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: 无

**功能描述**:
- 初始化LEDC外设用于PWM输出
- 配置左右电机PWM通道
- 实现电机速度设置接口
- 实现电机急停功能

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/chassis/motor_driver.h`](src/nodes/chassis/motor_driver.h) - 电机驱动接口定义
   - [`src/nodes/chassis/motor_driver.c`](src/nodes/chassis/motor_driver.c) - 电机驱动实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化电机驱动
    * @return 
    *   - ESP_OK: 成功
    *   - ESP_FAIL: 初始化失败
    */
   esp_err_t motor_driver_init(void);
   
   /**
    * @brief 设置左侧电机速度
    * @param[in] speed 速度值（-100.0 ~ 100.0）
    *                  正值=前进，负值=后退，0=停止
    * @return ESP_OK: 成功
    */
   esp_err_t motor_driver_set_left_speed(float speed);
   
   /**
    * @brief 设置右侧电机速度
    * @param[in] speed 速度值（-100.0 ~ 100.0）
    * @return ESP_OK: 成功
    */
   esp_err_t motor_driver_set_right_speed(float speed);
   
   /**
    * @brief 设置两侧电机速度（原子操作）
    * @param[in] left_speed 左侧速度
    * @param[in] right_speed 右侧速度
    * @return ESP_OK: 成功
    */
   esp_err_t motor_driver_set_speed(float left_speed, float right_speed);
   
   /**
    * @brief 紧急停止所有电机
    * @return ESP_OK: 成功
    */
   esp_err_t motor_driver_emergency_stop(void);
   ```

3. **数据结构**
   ```c
   /**
    * @brief 电机配置结构体
    */
   typedef struct {
       uint8_t pwm_gpio_left;      ///< 左电机PWM引脚（GPIO0）
       uint8_t pwm_gpio_right;     ///< 右电机PWM引脚（GPIO1）
       uint32_t pwm_frequency;     ///< PWM频率（Hz，推荐20kHz）
       uint8_t pwm_resolution;     ///< PWM分辨率（位，推荐10bit）
       ledc_timer_t timer_num;     ///< LEDC定时器编号
       ledc_channel_t channel_left; ///< 左电机LEDC通道
       ledc_channel_t channel_right;///< 右电机LEDC通道
   } motor_config_t;
   
   /**
    * @brief 电机状态
    */
   typedef struct {
       float left_speed;           ///< 当前左侧速度（%）
       float right_speed;          ///< 当前右侧速度（%）
       bool is_stopped;            ///< 是否已停止
       uint32_t pwm_duty_left;     ///< 左侧PWM占空比
       uint32_t pwm_duty_right;    ///< 右侧PWM占空比
   } motor_state_t;
   ```

4. **实现要点**
   - 使用[`ledc_timer_config()`](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/peripherals/ledc.html)配置LEDC定时器
   - PWM频率设置为20kHz（超声频率，避免噪音）
   - PWM分辨率10位（1024级，0-1023）
   - 使用[`ledc_channel_config()`](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/peripherals/ledc.html)配置通道
   - 速度到占空比转换：`duty = (uint32_t)(abs(speed) * 1023 / 100.0f)`
   - 负速度通过反向引脚控制（如果支持H桥）
   - 急停时立即设置占空比为0
   - 线程安全：使用互斥锁保护速度设置

**验收标准**:
- [x] PWM输出波形正确（示波器验证）
- [x] 电机正反转正常
- [x] 速度设置响应迅速（<10ms）
- [x] 急停功能立即生效
- [x] 编译无警告

---

### 2.2 任务2：I2C编码器读取

**任务ID**: TASK-CHASSIS-002  
**优先级**: P0  
**估算时间**: 10小时  
**依赖任务**: 无

**功能描述**:
- 初始化I2C总线
- 配置电机驱动器I2C地址
- 读取左右编码器计数值
- 实现编码器计数重置

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化编码器读取
    * @return ESP_OK: 成功
    */
   esp_err_t motor_encoder_init(void);
   
   /**
    * @brief 读取左侧编码器计数
    * @param[out] count 编码器计数值
    * @return 
    *   - ESP_OK: 成功
    *   - ESP_ERR_TIMEOUT: I2C超时
    */
   esp_err_t motor_encoder_read_left(int32_t *count);
   
   /**
    * @brief 读取右侧编码器计数
    * @param[out] count 编码器计数值
    * @return ESP_OK: 成功
    */
   esp_err_t motor_encoder_read_right(int32_t *count);
   
   /**
    * @brief 读取两侧编码器计数（原子操作）
    * @param[out] left_count 左侧计数
    * @param[out] right_count 右侧计数
    * @return ESP_OK: 成功
    */
   esp_err_t motor_encoder_read_both(int32_t *left_count, int32_t *right_count);
   
   /**
    * @brief 重置编码器计数
    * @return ESP_OK: 成功
    */
   esp_err_t motor_encoder_reset(void);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 编码器配置
    */
   typedef struct {
       uint8_t i2c_addr_left;      ///< 左编码器I2C地址
       uint8_t i2c_addr_right;     ///< 右编码器I2C地址
       uint16_t ppr;               ///< 每转脉冲数（Pulses Per Revolution）
       uint32_t i2c_timeout_ms;    ///< I2C超时时间（毫秒）
   } encoder_config_t;
   
   /**
    * @brief 编码器状态
    */
   typedef struct {
       int32_t left_count;         ///< 左侧累计计数
       int32_t right_count;        ///< 右侧累计计数
       int32_t left_count_prev;    ///< 左侧上次计数
       int32_t right_count_prev;   ///< 右侧上次计数
       uint32_t last_read_time;    ///< 上次读取时间（毫秒）
   } encoder_state_t;
   ```

3. **实现要点**
   - 使用通用I2C总线模块（[`src/drivers/i2c_bus/i2c_bus.c`](src/drivers/i2c_bus/i2c_bus.c)）
   - 假设编码器接口模块地址：0x30（左），0x31（右）
   - 读取寄存器：0x00-0x03（32位计数器，大端序）
   - 使用[`i2c_master_write_read_device()`](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/peripherals/i2c.html)读取
   - I2C超时设置为100ms
   - 计数值为有符号32位整数（支持双向计数）
   - 失败时重试3次
   - 记录读取时间戳用于速度计算

**验收标准**:
- [x] I2C通信稳定（错误率<1%）
- [x] 编码器计数准确（与实际转动一致）
- [x] 读取延迟<10ms
- [x] 异常处理正确（总线错误、超时等）
- [x] 功能测试通过

---

### 2.3 任务3：电机方向控制和速度设置

**任务ID**: TASK-CHASSIS-003  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-CHASSIS-001]

**功能描述**:
- 实现电机正反转控制
- 实现速度平滑过渡
- 实现速度限幅和死区处理
- 实现电机状态查询

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 设置电机方向
    * @param[in] motor 电机选择（MOTOR_LEFT/MOTOR_RIGHT）
    * @param[in] direction 方向（MOTOR_FORWARD/MOTOR_BACKWARD）
    * @return ESP_OK: 成功
    */
   esp_err_t motor_driver_set_direction(motor_id_t motor, motor_direction_t direction);
   
   /**
    * @brief 设置速度（带平滑过渡）
    * @param[in] left_speed 左侧目标速度
    * @param[in] right_speed 右侧目标速度
    * @param[in] transition_ms 过渡时间（毫秒）
    * @return ESP_OK: 成功
    */
   esp_err_t motor_driver_set_speed_smooth(float left_speed, float right_speed, 
                                            uint32_t transition_ms);
   
   /**
    * @brief 获取电机状态
    * @param[out] state 电机状态
    * @return ESP_OK: 成功
    */
   esp_err_t motor_driver_get_state(motor_state_t *state);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 电机ID枚举
    */
   typedef enum {
       MOTOR_LEFT = 0,
       MOTOR_RIGHT = 1
   } motor_id_t;
   
   /**
    * @brief 电机方向枚举
    */
   typedef enum {
       MOTOR_FORWARD = 1,      ///< 正转（前进）
       MOTOR_BACKWARD = -1,    ///< 反转（后退）
       MOTOR_STOP = 0          ///< 停止
   } motor_direction_t;
   
   /**
    * @brief 电机控制参数
    */
   #define MOTOR_SPEED_MAX      100.0f    ///< 最大速度（%）
   #define MOTOR_SPEED_MIN      0.0f      ///< 最小速度（%）
   #define MOTOR_SPEED_DEADZONE 5.0f      ///< 死区（%，小于此值视为0）
   #define MOTOR_RAMP_RATE      500.0f    ///< 加速度（%/秒）
   ```

3. **实现要点**
   - 方向控制通过PWM占空比符号实现（正值=正转，负值=反转）
   - 速度限幅：`speed = constrain(speed, -100.0, 100.0)`
   - 死区处理：`if (abs(speed) < MOTOR_SPEED_DEADZONE) speed = 0`
   - 平滑过渡使用线性插值：`new_speed = current + (target - current) * dt / transition_time`
   - 使用FreeRTOS定时器实现平滑过渡
   - 状态查询返回当前速度、方向、PWM占空比

**验收标准**:
- [x] 方向切换正常
- [x] 速度平滑过渡无突变
- [x] 死区处理正确
- [x] 速度限幅生效
- [x] 状态查询准确

---

## 3. 控制算法层（chassis_control模块）

### 3.1 任务4：PID控制器实现

**任务ID**: TASK-CHASSIS-004  
**优先级**: P0  
**估算时间**: 12小时  
**依赖任务**: [TASK-CHASSIS-002]

**功能描述**:
- 实现标准PID控制器
- 支持PID参数配置
- 实现积分抗饱和（Anti-windup）
- 实现输出限幅

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/chassis/chassis_control.h`](src/nodes/chassis/chassis_control.h) - 控制算法接口
   - [`src/nodes/chassis/chassis_control.c`](src/nodes/chassis/chassis_control.c) - PID控制实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化PID控制器
    * @param[in] params PID参数
    * @return ESP_OK: 成功
    */
   esp_err_t chassis_pid_init(const pid_params_t *params);
   
   /**
    * @brief 执行PID控制（单次迭代）
    * @param[in] setpoint 目标值
    * @param[in] measured 实际测量值
    * @param[in] dt 时间步长（秒）
    * @param[out] output 控制输出
    * @return ESP_OK: 成功
    */
   esp_err_t chassis_pid_compute(float setpoint, float measured, 
                                  float dt, float *output);
   
   /**
    * @brief 重置PID控制器
    * @return ESP_OK: 成功
    */
   esp_err_t chassis_pid_reset(void);
   
   /**
    * @brief 设置PID参数
    * @param[in] params PID参数
    * @return ESP_OK: 成功
    */
   esp_err_t chassis_pid_set_params(const pid_params_t *params);
   ```

3. **数据结构**
   ```c
   /**
    * @brief PID参数
    */
   typedef struct {
       float kp;               ///< 比例系数
       float ki;               ///< 积分系数
       float kd;               ///< 微分系数
       float output_min;       ///< 输出下限（-100.0）
       float output_max;       ///< 输出上限（100.0）
       float integral_min;     ///< 积分下限（抗饱和）
       float integral_max;     ///< 积分上限（抗饱和）
   } pid_params_t;
   
   /**
    * @brief PID控制器状态
    */
   typedef struct {
       pid_params_t params;    ///< PID参数
       float error;            ///< 当前误差
       float error_prev;       ///< 上次误差
       float error_sum;        ///< 误差积分
       float output;           ///< 控制输出
       uint32_t sample_count;  ///< 采样计数
   } pid_controller_t;
   
   /**
    * @brief 默认PID参数（需要现场调试）
    */
   #define PID_KP_DEFAULT  1.0f
   #define PID_KI_DEFAULT  0.5f
   #define PID_KD_DEFAULT  0.1f
   ```

4. **实现要点**
   - PID算法：`output = Kp*e + Ki*Σe*dt + Kd*de/dt`
   - 比例项：`p_term = kp * error`
   - 积分项：`i_term += ki * error * dt`，限幅到[integral_min, integral_max]
   - 微分项：`d_term = kd * (error - error_prev) / dt`
   - 输出限幅：`output = constrain(p_term + i_term + d_term, output_min, output_max)`
   - 抗饱和：当输出饱和时停止积分累加
   - 初始化时清零所有状态变量
   - 线程安全：使用互斥锁保护PID状态

**验收标准**:
- [x] PID算法正确实现
- [x] 参数可在线调整
- [x] 
 积分抗饱和工作正常
- [x] 输出限幅生效
- [x] 单元测试通过（阶跃响应、正弦跟踪）

---

### 3.2 任务5：差速运动学逆解

**任务ID**: TASK-CHASSIS-005  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: 无

**功能描述**:
- 实现差速运动学逆解算法
- 将线速度和角速度转换为左右轮速
- 考虑轮距和轮径参数
- 实现速度约束检查

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 差速运动学逆解
    * @param[in] v_linear 线速度（m/s）
    * @param[in] v_angular 角速度（rad/s）
    * @param[out] v_left 左轮速度（m/s）
    * @param[out] v_right 右轮速度（m/s）
    * @return 
    *   - ESP_OK: 成功
    *   - ESP_ERR_INVALID_ARG: 参数超出范围
    */
   esp_err_t chassis_inverse_kinematics(float v_linear, float v_angular,
                                         float *v_left, float *v_right);
   
   /**
    * @brief 轮速转换为电机速度百分比
    * @param[in] wheel_velocity 轮速（m/s）
    * @param[out] motor_speed 电机速度（%）
    * @return ESP_OK: 成功
    */
   esp_err_t chassis_velocity_to_motor_speed(float wheel_velocity, 
                                              float *motor_speed);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 底盘机械参数
    */
   typedef struct {
       float wheel_base;       ///< 轮距（m）：两轮中心距离
       float wheel_diameter;   ///< 轮径（m）：驱动轮直径
       float max_velocity;     ///< 最大线速度（m/s）
       float max_omega;        ///< 最大角速度（rad/s）
   } chassis_params_t;
   
   /**
    * @brief 默认底盘参数（需根据实际硬件调整）
    */
   #define WHEEL_BASE_DEFAULT      0.3f    ///< 轮距30cm
   #define WHEEL_DIAMETER_DEFAULT  0.1f    ///< 轮径10cm
   #define MAX_VELOCITY_DEFAULT    1.0f    ///< 最大线速度1m/s
   #define MAX_OMEGA_DEFAULT       2.0f    ///< 最大角速度2rad/s
   ```

3. **运动学公式**
   ```c
   // 差速运动学逆解
   // v_left = v_linear - v_angular * wheel_base / 2.0
   // v_right = v_linear + v_angular * wheel_base / 2.0
   
   // 轮速到转速（RPM）
   // rpm = (wheel_velocity * 60.0) / (PI * wheel_diameter)
   
   // 转速到电机速度百分比（假设最大转速对应100%）
   // motor_speed = (rpm / max_rpm) * 100.0
   ```

4. **实现要点**
   - 使用上述公式计算左右轮速
   - 速度约束检查：确保`|v_left|, |v_right| <= max_velocity`
   - 如果超出限制，等比例缩放：`scale = max_velocity / max(|v_left|, |v_right|)`
   - 轮径和轮距从配置文件读取（NVS或sdkconfig）
   - 考虑电机减速比（如果有）
   - 单元测试：直行、原地旋转、弧线运动

**验收标准**:
- [x] 运动学计算正确（数学验证）
- [x] 速度约束生效
- [x] 单元测试覆盖主要运动模式
- [x] 边界条件处理正确
- [x] 编译无警告

---

### 3.3 任务6：速度闭环控制

**任务ID**: TASK-CHASSIS-006  
**优先级**: P0  
**估算时间**: 10小时  
**依赖任务**: [TASK-CHASSIS-004, TASK-CHASSIS-005]

**功能描述**:
- 实现左右轮独立PID速度闭环
- 整合编码器反馈
- 计算实际轮速
- 执行PID控制输出到电机

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化速度闭环控制
    * @return ESP_OK: 成功
    */
   esp_err_t chassis_velocity_control_init(void);
   
   /**
    * @brief 设置目标速度
    * @param[in] v_linear 目标线速度（m/s）
    * @param[in] v_angular 目标角速度（rad/s）
    * @return ESP_OK: 成功
    */
   esp_err_t chassis_set_velocity_target(float v_linear, float v_angular);
   
   /**
    * @brief 速度控制循环（每20ms调用一次）
    * @return ESP_OK: 成功
    */
   esp_err_t chassis_velocity_control_step(void);
   
   /**
    * @brief 获取实际速度
    * @param[out] v_linear 实际线速度（m/s）
    * @param[out] v_angular 实际角速度（rad/s）
    * @return ESP_OK: 成功
    */
   esp_err_t chassis_get_actual_velocity(float *v_linear, float *v_angular);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 速度控制状态
    */
   typedef struct {
       pid_controller_t pid_left;      ///< 左轮PID控制器
       pid_controller_t pid_right;     ///< 右轮PID控制器
       float target_v_left;            ///< 左轮目标速度（m/s）
       float target_v_right;           ///< 右轮目标速度（m/s）
       float actual_v_left;            ///< 左轮实际速度（m/s）
       float actual_v_right;           ///< 右轮实际速度（m/s）
       int32_t encoder_left_prev;      ///< 左轮上次编码器值
       int32_t encoder_right_prev;     ///< 右轮上次编码器值
       uint32_t last_update_time;      ///< 上次更新时间（毫秒）
   } velocity_control_t;
   ```

3. **控制流程**
   ```
   1. 读取编码器值（motor_encoder_read_both）
   2. 计算编码器增量（delta = current - previous）
   3. 计算实际轮速：
      wheel_velocity = (delta * PI * wheel_diameter) / (encoder_ppr * dt)
   4. 执行PID控制：
      pid_output_left = chassis_pid_compute(target_v_left, actual_v_left, dt)
      pid_output_right = chassis_pid_compute(target_v_right, actual_v_right, dt)
   5. 输出到电机：
      motor_driver_set_speed(pid_output_left, pid_output_right)
   6. 记录当前编码器值和时间戳
   ```

4. **实现要点**
   - 创建FreeRTOS任务，20ms周期执行（50Hz）
   - 使用`vTaskDelayUntil()`确保精确周期
   - 左右轮使用独立PID控制器
   - 编码器增量溢出处理（INT32_MAX边界）
   - 速度滤波：使用低通滤波器平滑编码器噪声
   - 线程安全：使用互斥锁保护目标速度和实际速度
   - 任务优先级设为10（较高优先级）

**验收标准**:
- [x] 速度跟踪精度±5%
- [x] 阶跃响应时间<100ms
- [x] 稳态误差<2%
- [x] 左右轮速度一致性好
- [x] 控制稳定无振荡

---

## 4. 里程计层（odometry模块）

### 4.1 任务7：编码器增量计算

**任务ID**: TASK-CHASSIS-007  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-CHASSIS-002]

**功能描述**:
- 计算编码器增量
- 处理编码器溢出
- 转换为轮子位移
- 实现增量累加

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/chassis/odometry.h`](src/nodes/chassis/odometry.h) - 里程计接口
   - [`src/nodes/chassis/odometry.c`](src/nodes/chassis/odometry.c) - 里程计实现

2. **关键函数**
   ```c
   /**
    * @brief 初始化里程计
    * @param[in] params 底盘参数
    * @return ESP_OK: 成功
    */
   esp_err_t odometry_init(const chassis_params_t *params);
   
   /**
    * @brief 计算编码器增量并更新位移
    * @param[in] encoder_left 左侧编码器当前值
    * @param[in] encoder_right 右侧编码器当前值
    * @param[out] delta_left 左轮位移增量（m）
    * @param[out] delta_right 右轮位移增量（m）
    * @return ESP_OK: 成功
    */
   esp_err_t odometry_compute_delta(int32_t encoder_left, int32_t encoder_right,
                                     float *delta_left, float *delta_right);
   
   /**
    * @brief 重置里程计
    * @return ESP_OK: 成功
    */
   esp_err_t odometry_reset(void);
   ```

3. **数据结构**
   ```c
   /**
    * @brief 里程计状态
    */
   typedef struct {
       int32_t encoder_left_prev;      ///< 上次左侧编码器值
       int32_t encoder_right_prev;     ///< 上次右侧编码器值
       float distance_left;            ///< 左轮累计距离（m）
       float distance_right;           ///< 右轮累计距离（m）
       float wheel_circumference;      ///< 轮子周长（m）
       uint16_t encoder_ppr;           ///< 编码器分辨率（PPR）
   } odometry_state_t;
   ```

4. **实现要点**
   - 编码器增量计算：`delta = current - previous`
   - 溢出处理：检测INT32_MAX边界跳变，但通常不会溢出
   - 位移转换：`distance = (delta / encoder_ppr) * wheel_circumference`
   - 轮子周长：`circumference = PI * wheel_diameter`
   - 累加总位移：`total_distance += abs(distance)`
   - 保存当前编码器值作为下次的previous值

**验收标准**:
- [x] 增量计算准确
- [x] 位移转换正确
- [x] 溢出处理无错误
- [x] 累计距离与实际一致（误差<2%）
- [x] 单元测试通过

---

### 4.2 任务8：差速运动学正解

**任务ID**: TASK-CHASSIS-008  
**优先级**: P0  
**估算时间**: 10小时  
**依赖任务**: [TASK-CHASSIS-007]

**功能描述**:
- 实现差速运动学正解算法
- 根据左右轮位移计算位姿变化
- 更新机器人位置（x, y）
- 更新机器人姿态角（theta）

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 差速运动学正解
    * @param[in] delta_left 左轮位移（m）
    * @param[in] delta_right 右轮位移（m）
    * @param[in,out] pose 位姿（输入当前值，输出更新值）
    * @return ESP_OK: 成功
    */
   esp_err_t odometry_forward_kinematics(float delta_left, float delta_right,
                                          pose_2d_t *pose);
   
   /**
    * @brief 更新里程计（完整流程）
    * @param[in] encoder_left 左侧编码器值
    * @param[in] encoder_right 右侧编码器值
    * @return ESP_OK: 成功
    */
   esp_err_t odometry_update(int32_t encoder_left, int32_t encoder_right);
   
   /**
    * @brief 获取当前位姿
    * @param[out] pose 位姿
    * @return ESP_OK: 成功
    */
   esp_err_t odometry_get_pose(pose_2d_t *pose);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 2D位姿
    */
   typedef struct {
       float x;                ///< X坐标（m）
       float y;                ///< Y坐标（m）
       float theta;            ///< 姿态角（rad，逆时针为正）
   } pose_2d_t;
   ```

3. **运动学公式**
   ```c
   // 差速运动学正解
   // 1. 计算中心位移和姿态变化
   float dist_center = (delta_left + delta_right) / 2.0f;
   float delta_theta = (delta_right - delta_left) / wheel_base;
   
   // 2. 更新位姿（假设小角度近似）
   pose.x += dist_center * cosf(pose.theta);
   pose.y += dist_center * sinf(pose.theta);
   pose.theta += delta_theta;
   
   // 3. 角度归一化到[-PI, PI]
   pose.theta = atan2f(sinf(pose.theta), cosf(pose.theta));
   ```

4. **实现要点**
   - 使用小角度近似（误差<1%当dt<0.1s）
   - 精确解（大角度）：使用圆弧积分
   - 角度归一化使用`atan2f()`避免跳变
   - 使用双精度浮点数（double）提高精度（可选）
   - 线程安全：使用互斥锁保护位姿
   - 提供位姿重置接口（`odometry_reset_pose()`）

**验收标准**:
- [x] 直线运动位姿计算正确
- [x] 旋转运动位姿计算正确
- [x] 弧线运动位姿计算正确
- [x] 误差累积可接受（1m运动<2cm误差）
- [x] 单元测试通过

---

### 4.3 任务9：里程计数据融合和发布

**任务ID**: TASK-CHASSIS-009  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: [TASK-CHASSIS-008, TASK-CHASSIS-006]

**功能描述**:
- 融合位姿和速度信息
- 构造ROS Odometry消息
- 以50Hz频率发布里程计
- 计算协方差矩阵

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 发布里程计消息
    * @return ESP_OK: 成功
    */
   esp_err_t odometry_publish(void);
   
   /**
    * @brief 设置里程计协方差
    * @param[in] pose_covariance 位姿协方差（6x6矩阵）
    * @param[in] twist_covariance 速度协方差（6x6矩阵）
    * @return ESP_OK: 成功
    */
   esp_err_t odometry_set_covariance(const float *pose_covariance,
                                      const float *twist_covariance);
   ```

2. **数据结构**
   ```c
   /**
    * @brief 完整里程计数据
    */
   typedef struct {
       pose_2d_t pose;             ///< 位姿
       float v_linear;             ///< 线速度（m/s）
       float v_angular;            ///< 角速度（rad/s）
       float pose_covariance[36];  ///< 位姿协方差（6x6）
       float twist_covariance[36]; ///< 速度协方差（6x6）
       uint64_t timestamp_us;      ///< 时间戳（微秒）
   } odometry_data_t;
   ```

3. **ROS消息构造**
   ```c
   // 使用nav_msgs/msg/Odometry消息类型
   nav_msgs__msg__Odometry odom_msg;
   
   // Header
   odom_msg.header.stamp.sec = timestamp_us / 1000000;
   odom_msg.header.stamp.nanosec = (timestamp_us % 1000000) * 1000;
   odom_msg.header.frame_id = "odom";
   odom_msg.child_frame_id = "base_link";
   
   // Pose
   odom_msg.pose.pose.position.x = pose.x;
   odom_msg.pose.pose.position.y = pose.y;
   odom_msg.pose.pose.position.z = 0.0;
   
   // Orientation (从theta转换为四元数)
   odom_msg.pose.pose.orientation.z = sinf(pose.theta / 2.0f);
   odom_msg.pose.pose.orientation.w = cosf(pose.theta / 2.0f);
   
   // Twist
   odom_msg.twist.twist.linear.x = v_linear;
   odom_msg.twist.twist.angular.z = v_angular;
   
   // Covariance
   memcpy(odom_msg.pose.covariance, pose_covariance, 36 * sizeof(float));
   memcpy(odom_msg.twist.covariance, twist_covariance, 36 * sizeof(float));
   ```

4. **实现要点**
   - 创建ROS发布者（话题：`/chassis/odom`）
   - 使用FreeRTOS任务，20ms周期发布（50Hz）
   - 与速度控制任务同步（使用同一时间戳）
   - 协方差矩阵默认值：对角线非零（如0.01），其余为0
   - 时间戳使用`esp_timer_get_time()`获取微秒精度
   - 欧拉角到四元数转换（仅绕Z轴旋转）
   - 线程安全：读取位姿和速度时加锁

**验收标准**:
- [x] ROS消息格式正确
- [x] 发布频率稳定50Hz（误差<1%）
- [x] 时间戳准确
- [x] rviz可视化正常
- [x] 数据与实际运动一致

---

## 5. 节点应用层（chassis_node模块）

### 5.1 任务10：节点初始化和主循环

**任务ID**: TASK-CHASSIS-010  
**优先级**: P0  
**估算时间**: 8小时  
**依赖任务**: [TASK-CHASSIS-006, TASK-CHASSIS-009]

**功能描述**:
- 初始化所有模块
- 创建ROS节点
- 启动控制任务
- 实现主循环

**技术实现细节**:

1. **文件结构**
   - [`src/nodes/chassis/chassis_node.h`](src/nodes/chassis/chassis_node.h) - 节点主接口
   - [`src/nodes/chassis/chassis_node.c`](src/nodes/chassis/chassis_node.c) - 节点主逻辑

2. **关键函数**
   ```c
   /**
    * @brief 初始化底盘节点
    * @return ESP_OK: 成功
    */
   esp_err_t chassis_node_init(void);
   
   /**
    * @brief 底盘节点主函数（不返回）
    */
   void chassis_node_main(void);
   
   /**
    * @brief 停止底盘节点
    * @return ESP_OK: 成功
    */
   esp_err_t chassis_node_stop(void);
   ```

3. **初始化流程**
   ```c
   esp_err_t chassis_node_init(void) {
       ESP_LOGI(TAG, "Initializing chassis node...");
       
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
       
       // 4. 初始化底盘专用模块
       ESP_ERROR_CHECK(motor_driver_init());
       ESP_ERROR_CHECK(motor_encoder_init());
       ESP_ERROR_CHECK(chassis_pid_init(&g_pid_params));
       ESP_ERROR_CHECK(chassis_velocity_control_init());
       ESP_ERROR_CHECK(odometry_init(&g_chassis_params));
       
       // 5. 创建ROS接口
       ESP_ERROR_CHECK(chassis_create_ros_interfaces());
       
       ESP_LOGI(TAG, "Chassis node initialized successfully");
       return ESP_OK;
   }
   ```

4. **主循环**
   ```c
   void chassis_node_main(void) {
       chassis_node_init();
       
       // 启动控制和里程计任务
       xTaskCreate(velocity_control_task, "vel_ctrl", 4096, NULL, 10, NULL);
       xTaskCreate(odometry_publish_task, "odom_pub", 4096, NULL, 10, NULL);
       xTaskCreate(emergency_stop_task, "e_stop", 2048, NULL, 15, NULL);
       
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

### 5.2 任务11：ROS话题订阅（/chassis/cmd_vel）

**任务ID**: TASK-CHASSIS-011  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-CHASSIS-010]

**功能描述**:
- 订阅速度指令话题
- 解析Twist消息
- 更新目标速度
- 实现超时保护

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 创建cmd_vel订阅者
    * @return ESP_OK: 成功
    */
   esp_err_t chassis_create_cmd_vel_subscriber(void);
   
   /**
    * @brief cmd_vel回调函数
    * @param[in] msg Twist消息
    * @param[in] user_data 用户数据
    */
   void chassis_cmd_vel_callback(const void *msg, void *user_data);
   ```

2. **实现要点**
   ```c
   void chassis_cmd_vel_callback(const void *msg, void *user_data) {
       const geometry_msgs__msg__Twist *twist = 
           (const geometry_msgs__msg__Twist *)msg;
       
       // 提取线速度和角速度
       float v_linear = twist->linear.x;
       float v_angular = twist->angular.z;
       
       // 速度限幅
       v_linear = constrain(v_linear, -MAX_VELOCITY, MAX_VELOCITY);
       v_angular = constrain(v_angular, -MAX_OMEGA, MAX_OMEGA);
       
       // 更新目标速度
       chassis_set_velocity_target(v_linear, v_angular);
       
       // 更新超时计数器
       g_cmd_vel_timeout = 0;
       
       ESP_LOGD(TAG, "cmd_vel: v=%.2f, w=%.2f", v_linear, v_angular);
   }
   
   // 创建订阅者
   esp_err_t chassis_create_cmd_vel_subscriber(void) {
       return ros_comm_create_subscription(
           &g_cmd_vel_sub,
           "/chassis/cmd_vel",
           ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
           &QOS_CONTROL_CMD,
           chassis_cmd_vel_callback
       );
   }
   ```

3. **超时保护**
   ```c
   // 在控制任务中检查超时
   #define CMD_VEL_TIMEOUT_MS  500  // 500ms无指令则停止
   
   void velocity_control_task(void *pvParameters) {
       while (1) {
           g_cmd_vel_timeout += 20;  // 每20ms增加
           
           if (g_cmd_vel_timeout > CMD_VEL_TIMEOUT_MS) {
               // 超时，停止电机
               chassis_set_velocity_target(0.0f, 0.0f);
               ESP_LOGW(TAG, "cmd_vel timeout, stopping");
           }
           
           chassis_velocity_control_step();
           vTaskDelay(pdMS_TO_TICKS(20));
       }
   }
   ```

**验收标准**:
- [x] 成功订阅话题
- [x] 回调函数正确执行
- [x] 速度限幅生效
- [x] 超时保护正常
- [x] 响应延迟<50ms

---

### 5.3 任务12：ROS话题发布（/chassis/odom）

**任务ID**: TASK-CHASSIS-012  
**优先级**: P0  
**估算时间**: 4小时  
**依赖任务**: [TASK-CHASSIS-009, TASK-CHASSIS-010]

**功能描述**:
- 创建里程计发布者
- 定期发布里程计消息
- 保证发布频率50Hz

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 创建odom发布者
    * @return ESP_OK: 成功
    */
   esp_err_t chassis_create_odom_publisher(void);
   ```

2. **实现要点**
   ```c
   esp_err_t chassis_create_odom_publisher(void) {
       return ros_comm_create_publisher(
           &g_odom_pub,
           "/chassis/odom",
           ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
           &QOS_SENSOR_DATA
       );
   }
   
   // 在里程计任务中发布
   void odometry_publish_task(void *pvParameters) {
       TickType_t last_wake_time = xTaskGetTickCount();
       const TickType_t frequency = pdMS_TO_TICKS(20);  // 50Hz
       
       while (1) {
           odometry_publish();
           vTaskDelayUntil(&last_wake_time, frequency);
       }
   }
   ```

**验收标准**:
- [x] 发布者创建成功
- [x] 消息正常发布
- [x] 频率稳定50Hz
- [x] `ros2 topic hz`验证通过
- [x] rviz显示正常

---

### 5.4 任务13：ROS服务实现

**任务ID**: TASK-CHASSIS-013  
**优先级**: P1  
**估算时间**: 10小时  
**依赖任务**: [TASK-CHASSIS-010]

**功能描述**:
- 实现`/chassis/set_pid`服务
- 实现`/chassis/reset_odom`服务
- 实现`/chassis/enable`服务
- 提供配置查询服务

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 创建所有服务
    * @return ESP_OK: 成功
    */
   esp_err_t chassis_create_services(void);
   
   /**
    * @brief set_pid服务回调
    */
   void chassis_set_pid_service_callback(const void *request, 
                                          void *response, void *user_data);
   
   /**
    * @brief reset_odom服务回调
    */
   void chassis_reset_odom_service_callback(const void *request,
                                             void *response, void *user_data);
   
   /**
    * @brief enable服务回调
    */
   void chassis_enable_service_callback(const void *request,
                                         void *response, void *user_data);
   ```

2. **服务定义**（需要自定义srv文件）
   ```
   # chassis_interfaces/srv/SetPID.srv
   float32 kp
   float32 ki
   float32 kd
   ---
   bool success
   string message
   
   # chassis_interfaces/srv/ResetOdom.srv
   ---
   bool success
   string message
   
   # chassis_interfaces/srv/Enable.srv
   bool enable
   ---
   bool success
   string message
   ```

3. **实现要点**
   - 使用`ros_comm_create_service()`创建服务
   - 服务回调中更新PID参数并保存到NVS
   - reset_odom重置位姿为(0, 0, 0)
   - enable控制是否响应cmd_vel
   - 所有服务返回成功/失败状态和消息
   - 参数验证：PID参数范围检查

**验收标准**:
- [x] 服务正常响应
- [x] PID参数在线调整生效
- [x] 里程计重置正确
- [x] 使能/禁用功能正常
- [x] `ros2 service call`测试通过

---

### 5.5 任务14：紧急停止处理

**任务ID**: TASK-CHASSIS-014  
**优先级**: P0  
**估算时间**: 6小时  
**依赖任务**: [TASK-CHASSIS-001, TASK-CHASSIS-010]

**功能描述**:
- 监测GPIO10紧急停止开关
- 实现中断处理
- 立即停止电机
- 发布紧急停止状态

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 初始化紧急停止
    * @return ESP_OK: 成功
    */
   esp_err_t chassis_emergency_stop_init(void);
   
   /**
    * @brief GPIO中断处理函数
    * @param[in] arg 参数
    */
   void IRAM_ATTR emergency_stop_isr_handler(void *arg);
   
   /**
    * @brief 紧急停止任务
    * @param[in] pvParameters 参数
    */
   void emergency_stop_task(void *pvParameters);
   ```

2. **实现要点**
   ```c
   esp_err_t chassis_emergency_stop_init(void) {
       // 配置GPIO10为输入，上拉
       gpio_config_t io_conf = {
           .pin_bit_mask = (1ULL << GPIO_EMERGENCY_STOP),
           .mode = GPIO_MODE_INPUT,
           .pull_up_en = GPIO_PULLUP_ENABLE,
           .pull_down_en = GPIO_PULLDOWN_DISABLE,
           .intr_type = GPIO_INTR_NEGEDGE  // 下降沿触发
       };
       gpio_config(&io_conf);
       
       // 安装GPIO ISR服务
       gpio_install_isr_service(0);
       gpio_isr_handler_add(GPIO_EMERGENCY_STOP, 
                            emergency_stop_isr_handler, NULL);
       
       return ESP_OK;
   }
   
   void IRAM_ATTR emergency_stop_isr_handler(void *arg) {
       BaseType_t xHigherPriorityTaskWoken = pdFALSE;
       
       // 通知任务处理
       xTaskNotifyFromISR(g_emergency_stop_task_handle, 
                          1, eSetValueWithOverwrite,
                          &xHigherPriorityTaskWoken);
       
       portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
   }
   
   void emergency_stop_task(void *pvParameters) {
       uint32_t notification_value;
       
       while (1) {
           // 等待通知
           if (xTaskNotifyWait(0, 0, &notification_value, portMAX_DELAY)) {
               ESP_LOGE(TAG, "EMERGENCY STOP TRIGGERED!");
               
               // 立即停止电机
               motor_driver_emergency_stop();
               
               // 设置标志
               g_emergency_stop_active = true;
               
               // 发布状态到ROS（可选）
               // chassis_publish_emergency_stop_status();
               
               // 等待按钮释放
               while (gpio_get_level(GPIO_EMERGENCY_STOP) == 0) {
                   vTaskDelay(pdMS_TO_TICKS(100));
               }
               
               ESP_LOGW(TAG, "Emergency stop released");
               g_emergency_stop_active = false;
           }
       }
   }
   ```

3. **安全策略**
   - 紧急停止触发后，禁止响应cmd_vel
   - 需要手动复位（按钮释放后再使能）
   - 优先级最高（15）确保及时响应
   - ISR中仅设置标志，实际处理在任务中
   - 防抖动：连续检测100ms确认状态

**验收标准**:
- [x] 按钮按下立即停止（<10ms）
- [x] 中断处理稳定无误
- [x] 防抖动机制有效
- [x] 状态正确发布
- [x] 压力测试通过

---

### 5.6 任务15：OLED状态显示

**任务ID**: TASK-CHASSIS-015  
**优先级**: P1  
**估算时间**: 6小时  
**依赖任务**: [TASK-CHASSIS-010]

**功能描述**:
- 显示当前速度
- 显示累计里程
- 显示节点状态
- 显示错误信息

**技术实现细节**:

1. **关键函数**
   ```c
   /**
    * @brief 更新OLED显示
    * @return ESP_OK: 成功
    */
   esp_err_t chassis_update_oled_display(void);
   ```

2. **显示布局**
   ```
   ┌────────────────────────┐
   │ CHASSIS NODE-01        │  ← 标题
   ├────────────────────────┤
   │ WiFi:✓ ROS:✓           │  ← 连接状态
   │ V:0.50m/s W:0.20rad/s  │  ← 当前速度
   │ Odom: 12.5m            │  ← 累计里程
   │ IP: 192.168.1.101      │  ← IP地址
   └────────────────────────┘
   ```

3. **实现要点**
   ```c
   void oled_display_update_task(void *pvParameters) {
       char line_buf[32];
       
       while (1) {
           // 获取数据
           float v_linear, v_angular;
           chassis_get_actual_velocity(&v_linear, &v_angular);
           
           float total_distance = odometry_get_total_distance();
           
           // 更新显示
           snprintf(line_buf, sizeof(line_buf), "V:%.2fm/s W:%.2frad/s", 
                    v_linear, v_angular);
           oled_ui_show_custom_text(2, line_buf);
           
           snprintf(line_buf, sizeof(line_buf), "Odom: %.1fm", total_distance);
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

## 6. 集成测试

### 6.1 任务16：单元测试

**任务ID**: TASK-CHASSIS-016  
**优先级**: P1  
**估算时间**: 12小时  
**依赖任务**: [TASK-CHASSIS-001~015]

**功能描述**:
- PID控制器单元测试
- 运动学算法单元测试
- 里程计算法单元测试
- 覆盖率>80%

**技术实现细节**:

1. **测试文件**
   - [`test/unit/test_chassis_control.c`](test/unit/test_chassis_control.c)
   - [`test/unit/test_odometry.c`](test/unit/test_odometry.c)
   - [`test/unit/test_kinematics.c`](test/unit/test_kinematics.c)

2. **测试用例**
   ```c
   // test_chassis_control.c
   void test_pid_proportional(void);
   void test_pid_integral(void);
   void test_pid_derivative(void);
   void test_pid_anti_windup(void);
   void test_pid_output_limit(void);
   
   // test_odometry.c
   void test_encoder_delta(void);
   void test_forward_kinematics_straight(void);
   void test_forward_kinematics_rotation(void);
   void test_forward_kinematics_arc(void);
   void test_odometry_reset(void);
   
   // test_kinematics.c
   void test_inverse_kinematics_straight(void);
   void test_inverse_kinematics_rotation(void);
   void test_inverse_kinematics_arc(void);
   void test_velocity_constraints(void);
   ```

3. **实现要点**
   - 使用Unity测试框架
   - 每个函数至少3个测试用例（正常、边界、异常）
   - 使用Mock模拟硬件（编码器、电机）
   - 浮点数比较使用容差（`TEST_ASSERT_FLOAT_WITHIN`）
   - 自动化测试脚本

**验收标准**:
- [x] 所有测试用例通过
- [x] 代码覆盖率>80%
- [x] 无内存泄漏
- [x] 测试报告生成
- [x] CI集成

---

### 6.2 任务17：硬件在环测试

**任务ID**: TASK-CHASSIS-017  
**优先级**: P0  
**估算时间**: 16小时  
**依赖任务**: [TASK-CHASSIS-001~015]

**功能描述**:
- 实际电机运行测试
- 编码器精度测试
- 闭环控制性能测试
- 里程计精度测试

**技术实现细节**:

1. **测试场景**
   - 直线运动1米（前进/后退）
   - 原地旋转360度（顺时针/逆时针）
   - S形轨迹跟踪
   - 速度阶跃响应
   - 长时间连续运行（1小时）

2. **测试指标**
   ```
   ┌──────────────────┬──────────┬──────────┐
   │ 测试项           │ 目标值   │ 实测值   │
   ├──────────────────┼──────────┼──────────┤
   │ 直线1m误差       │ <2cm     │          │
   │ 旋转360°误差     │ <5°      │          │
   │ 速度跟踪误差     │ <5%      │          │
   │ 阶跃响应时间     │ <100ms   │          │
   │ 稳态误差         │ <2%      │          │
   │ 里程计发布频率   │ 50±1Hz   │          │
   │ CPU占用          │ <60%     │          │
   │ 内存占用         │ <150KB   │          │
   └──────────────────┴──────────┴──────────┘
   ```

3. **测试工具**
   - 卷尺：测量实际距离
   - 量角器：测量旋转角度
   - ROS工具：`ros2 topic hz`, `ros2 topic echo`
   - Rviz：可视化里程计轨迹
   - `top`命令：监控系统资源
   - 示波器：检查PWM波形

4. **测试步骤**
   ```bash
   # 1. 启动ROS Agent
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   
   # 2. 烧录并启动节点
   pio run -e chassis_node -t upload
   
   # 3. 监控里程计话题
   ros2 topic echo /chassis/odom
   
   # 4. 发送速度指令测试
   ros2 topic pub /chassis/cmd_vel geometry_msgs/Twist \
       "{linear: {x: 0.5}, angular: {z: 0.0}}"
   
   # 5. 记录测试结果
   ```

**验收标准**:
- [x] 所有测试场景通过
- [x] 性能指标达标
- [x] 无系统崩溃
- [x] 无内存泄漏（>1小时运行）
- [x] 测试报告完整

---

## 7. 依赖关系图

### 7.1 任务依赖关系（DAG图）

```
初始化顺序（从上到下）：

1. 驱动层（并行）
   ├─→ TASK-001: PWM电机驱动
   ├─→ TASK-002: I2C编码器读取
   └─→ TASK-003: 电机方向控制 [依赖001]

2. 控制算法层（依赖驱动层）
   ├─→ TASK-004: PID控制器 [依赖002]
   ├─→ TASK-005: 差速运动学逆解
   └─→ TASK-006: 速度闭环控制 [依赖004,005]

3. 里程计层（依赖驱动层和控制层）
   ├─→ TASK-007: 编码器增量计算 [依赖002]
   ├─→ TASK-008: 差速运动学正解 [依赖007]
   └─→ TASK-009: 里程计发布 [依赖008,006]

4. 节点应用层（依赖所有底层）
   ├─→ TASK-010: 节点初始化 [依赖006,009]
   ├─→ TASK-011: cmd_vel订阅 [依赖010]
   ├─→ TASK-012: odom发布 [依赖009,010]
   ├─→ TASK-013: ROS服务 [依赖010]
   ├─→ TASK-014: 紧急停止 [依赖001,010]
   └─→ TASK-015: OLED显示 [依赖010]

5. 测试层（依赖全部）
   ├─→ TASK-016: 单元测试 [依赖001-015]
   └─→ TASK-017: 硬件测试 [依赖001-015]
```

### 7.2 任务依赖矩阵

| 任务ID | 任务名称 | 依赖任务 | 可并行任务 | 估算时间 |
|--------|---------|---------|-----------|---------|
| TASK-CHASSIS-001 | PWM电机驱动 | 无 | 002, 005 | 8h |
| TASK-CHASSIS-002 | I2C编码器读取 | 无 | 001, 005 | 10h |
| TASK-CHASSIS-003 | 电机方向控制 | 001 | 002, 004, 005 | 6h |
| TASK-CHASSIS-004 | PID控制器 | 002 | 005 | 12h |
| TASK-CHASSIS-005 | 差速运动学逆解 | 无 | 001, 002, 004 | 8h |
| TASK-CHASSIS-006 | 速度闭环控制 | 004, 005 | 007 | 10h |
| TASK-CHASSIS-007 | 编码器增量计算 | 002 | 005, 006 | 6h |
| TASK-CHASSIS-008 | 差速运动学正解 | 007 | 006 | 10h |
| TASK-CHASSIS-009 | 里程计发布 | 008, 006 | - | 8h |
| TASK-CHASSIS-010 | 节点初始化 | 006, 009 | 016 | 8h |
| TASK-CHASSIS-011 | cmd_vel订阅 | 010 | 012, 013, 014, 015 | 6h |
| TASK-CHASSIS-012 | odom发布 | 009, 010 | 011, 013, 014, 015 | 4h |
| TASK-CHASSIS-013 | ROS服务 | 010 | 011, 012, 014, 015 | 10h |
| TASK-CHASSIS-014 | 紧急停止 | 001, 010 | 011, 012, 013, 015 | 6h |
| TASK-CHASSIS-015 | OLED显示 | 010 | 011, 012, 013, 014 | 6h |
| TASK-CHASSIS-016 | 单元测试 | 001-015 | 017 | 12h |
| TASK-CHASSIS-017 | 硬件测试 | 001-015 | 016 | 16h |

### 7.3 开发里程碑

**阶段1：驱动层（1周）**
- 完成电机驱动（TASK-001~003）
- 完成编码器读取（TASK-002）
- 目标：电机能够响应速度指令，编码器能够读取

**阶段2：控制算法层（2周）**
- 完成PID控制器（TASK-004）
- 完成运动学解算（TASK-005）
- 完成速度闭环（TASK-006）
- 目标：实现精确的速度控制

**阶段3：里程计层（1.5周）**
- 完成里程计算法（TASK-007~008）
- 完成ROS发布（TASK-009）
- 目标：准确的位姿估计和发布

**阶段4：节点应用层（2周）**
- 完成节点框架（TASK-010）
- 完成ROS接口（TASK-011~013）
- 完成安全和显示（TASK-014~015）
- 目标：完整的ROS节点功能

**阶段5：测试和调优（2周）**
- 完成单元测试（TASK-016）
- 完成硬件测试（TASK-017）
- 目标：稳定可靠的系统

**总计：8.5周（约2个月）**

### 7.4 关键路径

```
Critical Path（关键路径）:
TASK-002 → TASK-004 → TASK-006 → TASK-009 → TASK-010 → TASK-017
(10h)    (12h)       (10h)       (8h)        (8h)        (16h)
总计：64小时（约8个工作日）
```

---

## 附录A：配置参数说明

### A.1 Kconfig配置项

```kconfig
menu "Chassis Node Configuration"
    
    config CHASSIS_WHEEL_BASE
        int "Wheel Base (mm)"
        range 100 1000
        default 300
        help
            Distance between left and right wheels in millimeters
    
    config CHASSIS_WHEEL_DIAMETER
        int "Wheel Diameter (mm)"
        range 50 300
        default 100
        help
            Diameter of drive wheels in millimeters
    
    config CHASSIS_ENCODER_PPR
        int "Encoder PPR"
        range 100 10000
        default 1024
        help
            Encoder pulses per revolution
    
    config CHASSIS_MAX_VELOCITY
        int "Max Velocity (mm/s)"
        range 100 2000
        default 1000
        help
            Maximum linear velocity in mm/s
    
    config CHASSIS_MAX_OMEGA
        int "Max Omega (mrad/s)"
        range 500 5000
        default 2000
        help
            Maximum angular velocity in milliradians/s
    
    config CHASSIS_PID_KP
        int "PID Kp (x1000)"
        range 0 10000
        default 1000
        help
            PID proportional gain (multiplied by 1000)
    
    config CHASSIS_PID_KI
        int "PID Ki (x1000)"
        range 0 10000
        default 500
        help
            PID integral gain (multiplied by 1000)
    
    config CHASSIS_PID_KD
        int "PID Kd (x1000)"
        range 0 10000
        default 100
        help
            PID derivative gain (multiplied by 1000)

endmenu
```

### A.2 NVS配置键

| 配置键 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `chassis_wheel_base` | float | 0.3 | 轮距（m） |
| `chassis_wheel_dia` | float | 0.1 | 轮径（m） |
| `chassis_encoder_ppr` | uint16 | 1024 | 编码器PPR |
| `chassis_max_vel` | float | 1.0 | 最大线速度（m/s） |
| `chassis_max_omega` | float | 2.0 | 最大角速度（rad/s） |
| `chassis_pid_kp` | float | 1.0 | PID Kp |
| `chassis_pid_ki` | float | 0.5 | PID Ki |
| `chassis_pid_kd` | float | 0.1 | PID Kd |

---

## 附录B：ROS接口总结

### B.1 订阅话题

| 话题名称 | 消息类型 | QoS | 频率 | 说明 |
|---------|---------|-----|------|------|
| `/chassis/cmd_vel` | `geometry_msgs/Twist` | RELIABLE | - | 速度指令 |

### B.2 发布话题

| 话题名称 | 消息类型 | QoS | 频率 | 说明 |
|---------|---------|-----|------|------|
| `/chassis/odom` | `nav_msgs/Odometry` | RELIABLE | 50Hz | 里程计 |
| `/chassis/diagnostics` | `diagnostic_msgs/DiagnosticStatus` | RELIABLE | 1Hz | 诊断信息 |

### B.3 服务

| 服务名称 | 服务类型 | 说明 |
|---------|---------|------|
| `/chassis/set_pid` | `chassis_interfaces/SetPID` | 设置PID参数 |
| `/chassis/reset_odom` | `chassis_interfaces/ResetOdom` | 重置里程计 |
| `/chassis/enable` | `chassis_interfaces/Enable` | 启用/禁用底盘 |

---

## 附录C：代码规范参考

所有代码必须符合[`开发规范.md`](../开发架构文档/开发规范.md)：

- **函数命名**：`module_action_object()`格式，如`motor_driver_set_speed()`
- **变量命名**：小写下划线，如`wheel_base`, `encoder_left`
- **常量命名**：大写下划线，如`MAX_VELOCITY`, `PID_KP_DEFAULT`
- **错误处理**：统一使用`esp_err_t`返回值
- **日志输出**：使用`ESP_LOGI`/`ESP_LOGW`/`ESP_LOGE`，标签为模块名
- **线程安全**：使用互斥锁保护共享资源
- **注释规范**：使用Doxygen风格注释

---

## 附录D：测试要求

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

## 附录E：参考资源

- **ESP-IDF文档**: https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/
- **Micro-ROS文档**: https://micro.ros.org/docs/
- **ROS 2 Humble文档**: https://docs.ros.org/en/humble/
- **差速运动学**: https://en.wikipedia.org/wiki/Differential_wheeled_robot
- **PID控制**: https://en.wikipedia.org/wiki/PID_controller
- **Adafruit QT Py ESP32-C3**: https://learn.adafruit.com/adafruit-qt-py-esp32-c3

---

**文档结束**

**编写人**：架构设计组  
**审核人**：技术负责人  
**版本**：v1.0.0  
**日期**：2025-10-23