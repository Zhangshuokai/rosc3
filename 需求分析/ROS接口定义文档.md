# 建筑喷涂施工机器人ROS接口定义文档

## 文档信息

| 项目 | 内容 |
|------|------|
| **文档标题** | 建筑喷涂施工机器人ROS接口定义文档 |
| **文档版本** | v2.0.0 |
| **编制日期** | 2025-10-23 |
| **ROS版本** | ROS 2 Humble |
| **DDS实现** | rmw_microxrcedds |
| **系统架构** | 分布式多节点（7个ESP32-C3节点） |

---

## 目录

- [1. 系统架构概述](#1-系统架构概述)
- [2. 底盘控制节点接口](#2-底盘控制节点接口)
- [3. 升降机构节点接口](#3-升降机构节点接口)
- [4. 环境监测节点接口](#4-环境监测节点接口)
- [5. 喷涂监控节点接口](#5-喷涂监控节点接口)
- [6. 距离测量节点接口](#6-距离测量节点接口)
- [7. IMU姿态节点接口](#7-imu姿态节点接口)
- [8. 电源管理节点接口](#8-电源管理节点接口)
- [9. QoS配置规范](#9-qos配置规范)
- [10. 命名规范](#10-命名规范)
- [11. 使用示例](#11-使用示例)

---

## 1. 系统架构概述

### 1.1 分布式节点拓扑

```
        ROS 2 Agent (上位机工控机)
                |
         WiFi Network (192.168.1.x)
                |
    ┌───────────┼───────────┐
    │           │           │
NODE-01      NODE-02     NODE-03
底盘控制     升降机构    环境监测
    │           │           │
NODE-04      NODE-05     NODE-06
喷涂监控     距离测量    IMU姿态
    │           │           │
NODE-07      [扩展]      [扩展]
电源管理     激光雷达    视觉识别
```

### 1.2 节点命名规范

**节点名称格式**：`<function>_node_<instance>`

| 节点编号 | 节点名称 | ROS节点ID | 主要功能 |
|---------|---------|-----------|---------|
| NODE-01 | 底盘控制节点 | `chassis_node_01` | 差速轮控制、里程计 |
| NODE-02 | 升降机构节点 | `lift_node_01` | 垂直升降控制 |
| NODE-03 | 环境监测节点 | `environment_node_01` | 温湿度气压监测 |
| NODE-04 | 喷涂监控节点 | `spray_node_01` | 压力流量料位监测 |
| NODE-05 | 距离测量节点 | `range_node_01` | 四向墙面距离测量 |
| NODE-06 | IMU姿态节点 | `imu_node_01` | 姿态和加速度测量 |
| NODE-07 | 电源管理节点 | `power_node_01` | 电池状态监测 |

### 1.3 通信特点

- **分布式架构**：每个节点独立运行，通过ROS 2网络通信
- **单一职责**：每个节点只管理一类传感器或执行器
- **故障隔离**：单节点故障不影响其他节点
- **负载均衡**：多节点分担计算和通信压力

---

## 2. 底盘控制节点接口

### 2.1 节点信息

**节点名称**：`chassis_node_01`

**功能描述**：管理差速移动底盘，包括电机控制和里程计反馈

**管理硬件**：
- 左侧驱动轮电机 + 编码器
- 右侧驱动轮电机 + 编码器

### 2.2 订阅话题

#### 2.2.1 速度指令话题

**话题名称**：`/chassis/cmd_vel`

**消息类型**：[`geometry_msgs/msg/Twist`](https://docs.ros.org/en/humble/p/geometry_msgs/msg/Twist.html)

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(1)
- Durability: VOLATILE

**消息结构**：
```yaml
geometry_msgs/Vector3 linear
  float64 x        # 线速度 (m/s): -1.0 ~ 1.0
  float64 y: 0.0   # 差速底盘不使用
  float64 z: 0.0   # 差速底盘不使用

geometry_msgs/Vector3 angular
  float64 x: 0.0   # 差速底盘不使用
  float64 y: 0.0   # 差速底盘不使用
  float64 z        # 角速度 (rad/s): -2.0 ~ 2.0
```

**使用示例**：
```bash
# 前进 0.5 m/s
ros2 topic pub /chassis/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 原地左转 0.5 rad/s
ros2 topic pub /chassis/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

---

### 2.3 发布话题

#### 2.3.1 里程计话题

**话题名称**：`/chassis/odom`

**消息类型**：[`nav_msgs/msg/Odometry`](https://docs.ros.org/en/humble/p/nav_msgs/msg/Odometry.html)

**发布频率**：50 Hz

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(10)
- Durability: VOLATILE

**消息结构**：
```yaml
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id: "odom"

string child_frame_id: "base_link"

geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x        # X坐标 (m)
      float64 y        # Y坐标 (m)
      float64 z: 0.0   # Z坐标恒为0（平面运动）
    geometry_msgs/Quaternion orientation
      float64 x, y, z, w  # 姿态四元数
  float64[36] covariance  # 位置和姿态协方差

geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x        # 线速度 (m/s)
    geometry_msgs/Vector3 angular
      float64 z        # 角速度 (rad/s)
  float64[36] covariance  # 速度协方差
```

**坐标系定义**：
- `odom`：里程计坐标系（起点为原点）
- `base_link`：机器人基座坐标系

---

#### 2.3.2 诊断信息话题

**话题名称**：`/chassis/diagnostics`

**消息类型**：[`diagnostic_msgs/msg/DiagnosticStatus`](https://docs.ros.org/en/humble/p/diagnostic_msgs/msg/DiagnosticStatus.html)

**发布频率**：1 Hz

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(10)
- Durability: VOLATILE

**消息结构**：
```yaml
byte level                # 诊断级别
  # OK=0, WARN=1, ERROR=2, STALE=3

string name: "Chassis Control Node"
string message           # 状态描述
string hardware_id       # 节点MAC地址

diagnostic_msgs/KeyValue[] values
  # 示例键值对：
  # - key: "Left Motor Status", value: "OK"
  # - key: "Right Motor Status", value: "OK"
  # - key: "Left Encoder", value: "1234 pulses"
  # - key: "Right Encoder", value: "1230 pulses"
  # - key: "CPU Usage", value: "45%"
  # - key: "Free Heap", value: "280 KB"
  # - key: "WiFi RSSI", value: "-65 dBm"
```

---

### 2.4 提供服务

#### 2.4.1 PID参数设置服务

**服务名称**：`/chassis/set_pid`

**服务类型**：自定义 `SetPID.srv`

**服务定义**：
```yaml
# 请求
string motor      # "left" 或 "right"
float32 kp       # 比例系数
float32 ki       # 积分系数
float32 kd       # 微分系数
---
# 响应
bool success
string message
```

#### 2.4.2 里程计重置服务

**服务名称**：`/chassis/reset_odom`

**服务类型**：[`std_srvs/srv/Trigger`](https://docs.ros.org/en/humble/p/std_srvs/srv/Trigger.html)

**服务定义**：
```yaml
# 请求
# 无字段
---
# 响应
bool success
string message: "Odometry reset to (0, 0, 0)"
```

#### 2.4.3 底盘使能服务

**服务名称**：`/chassis/enable`

**服务类型**：[`std_srvs/srv/SetBool`](https://docs.ros.org/en/humble/p/std_srvs/srv/SetBool.html)

**服务定义**：
```yaml
# 请求
bool data    # true=启用, false=禁用
---
# 响应
bool success
string message
```

---

## 3. 升降机构节点接口

### 3.1 节点信息

**节点名称**：`lift_node_01`

**功能描述**：管理垂直升降机构，控制作业高度

**管理硬件**：
- 伺服电机（升降驱动）
- 位置传感器（编码器）
- 上下限位开关

### 3.2 订阅话题

#### 3.2.1 位置指令话题

**话题名称**：`/lift/cmd_position`

**消息类型**：[`std_msgs/msg/Float32`](https://docs.ros.org/en/humble/p/std_msgs/msg/Float32.html)

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(1)

**消息结构**：
```yaml
float32 data    # 目标高度 (m): 0.0 ~ 3.0
```

**使用示例**：
```bash
# 升至2米高度
ros2 topic pub /lift/cmd_position std_msgs/msg/Float32 "{data: 2.0}"
```

---

### 3.3 发布话题

#### 3.3.1 当前位置话题

**话题名称**：`/lift/position`

**消息类型**：`std_msgs/msg/Float32`

**发布频率**：20 Hz

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(5)

**消息结构**：
```yaml
float32 data    # 当前高度 (m): 0.0 ~ 3.0
```

#### 3.3.2 诊断信息话题

**话题名称**：`/lift/diagnostics`

**消息类型**：`diagnostic_msgs/msg/DiagnosticStatus`

**发布频率**：1 Hz

**诊断内容示例**：
```yaml
values:
  - key: "Servo Motor Status", value: "OK"
  - key: "Current Position", value: "1.50 m"
  - key: "Target Position", value: "1.50 m"
  - key: "Position Error", value: "0.002 m"
  - key: "Upper Limit", value: "Not Triggered"
  - key: "Lower Limit", value: "Not Triggered"
```

---

### 3.4 提供服务

#### 3.4.1 归零服务

**服务名称**：`/lift/home`

**服务类型**：`std_srvs/srv/Trigger`

**功能**：驱动升降机构回到零点（最低位置）

#### 3.4.2 紧急停止服务

**服务名称**：`/lift/emergency_stop`

**服务类型**：`std_srvs/srv/Trigger`

**功能**：立即停止升降运动

#### 3.4.3 速度设置服务

**服务名称**：`/lift/set_speed`

**服务类型**：自定义 `SetSpeed.srv`

**服务定义**：
```yaml
# 请求
float32 speed    # 升降速度 (m/s): 0.1 ~ 0.5
---
# 响应
bool success
string message
float32 actual_speed
```

---

## 4. 环境监测节点接口

### 4.1 节点信息

**节点名称**：`environment_node_01`

**功能描述**：监测施工现场环境参数

**管理硬件**：
- 温湿度传感器（BME280或SHT3x）
- 气压传感器（BME280集成）

### 4.2 发布话题

#### 4.2.1 温度话题

**话题名称**：`/environment/temperature`

**消息类型**：[`sensor_msgs/msg/Temperature`](https://docs.ros.org/en/humble/p/sensor_msgs/msg/Temperature.html)

**发布频率**：0.5 Hz（每2秒）

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(5)

**消息结构**：
```yaml
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id: "environment_sensor"

float64 temperature    # 温度 (°C): -20 ~ 60
float64 variance: 0.01 # 方差（±0.1°C）
```

**施工适宜性判断**：
- 适宜：5°C ~ 35°C
- 警告：0°C ~ 5°C 或 35°C ~ 40°C
- 不适宜：<0°C 或 >40°C

---

#### 4.2.2 湿度话题

**话题名称**：`/environment/humidity`

**消息类型**：[`sensor_msgs/msg/RelativeHumidity`](https://docs.ros.org/en/humble/p/sensor_msgs/msg/RelativeHumidity.html)

**发布频率**：0.5 Hz

**消息结构**：
```yaml
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id: "environment_sensor"

float64 relative_humidity  # 相对湿度 (0.0-1.0): 0-100%
float64 variance: 0.0004   # 方差（±2%）
```

**施工适宜性判断**：
- 适宜：40% ~ 85%
- 警告：30% ~ 40% 或 85% ~ 90%
- 不适宜：<30% 或 >90%

---

#### 4.2.3 气压话题

**话题名称**：`/environment/pressure`

**消息类型**：[`sensor_msgs/msg/FluidPressure`](https://docs.ros.org/en/humble/p/sensor_msgs/msg/FluidPressure.html)

**发布频率**：0.5 Hz

**消息结构**：
```yaml
std_msgs/Header header

float64 fluid_pressure    # 气压 (Pa): 80000 ~ 110000
float64 variance: 100.0   # 方差
```

---

### 4.3 提供服务

#### 4.3.1 报警阈值设置服务

**服务名称**：`/environment/set_alarm`

**服务类型**：自定义 `SetEnvironmentAlarm.srv`

**服务定义**：
```yaml
# 请求
float32 temp_min     # 最低温度 (°C)
float32 temp_max     # 最高温度 (°C)
float32 humidity_min # 最低湿度 (%)
float32 humidity_max # 最高湿度 (%)
---
# 响应
bool success
string message
```

---

## 5. 喷涂监控节点接口

### 5.1 节点信息

**节点名称**：`spray_node_01`

**功能描述**：监控喷涂作业参数

**管理硬件**：
- 喷涂压力传感器
- 流量传感器
- 料位传感器

### 5.2 发布话题

#### 5.2.1 喷涂压力话题

**话题名称**：`/spray/pressure`

**消息类型**：`sensor_msgs/msg/FluidPressure`

**发布频率**：10 Hz

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(10)

**消息结构**：
```yaml
std_msgs/Header header

float64 fluid_pressure    # 喷涂压力 (Pa): 0 ~ 1,000,000 (0-10 bar)
float64 variance
```

**压力范围**：
- 正常：300,000 ~ 800,000 Pa (3-8 bar)
- 警告：200,000 ~ 300,000 Pa 或 800,000 ~ 900,000 Pa
- 异常：<200,000 Pa 或 >900,000 Pa

---

#### 5.2.2 流量话题

**话题名称**：`/spray/flow_rate`

**消息类型**：`std_msgs/msg/Float32`

**发布频率**：10 Hz

**消息结构**：
```yaml
float32 data    # 流量 (L/min): 0.0 ~ 5.0
```

**流量范围**：
- 正常：0.5 ~ 3.0 L/min
- 警告：0.2 ~ 0.5 L/min（可能堵塞）
- 异常：<0.2 L/min（堵塞）或 >3.5 L/min（异常）

---

#### 5.2.3 料位话题

**话题名称**：`/spray/material_level`

**消息类型**：`std_msgs/msg/Float32`

**发布频率**：1 Hz

**消息结构**：
```yaml
float32 data    # 料位百分比 (%): 0.0 ~ 100.0
```

**料位管理**：
- 正常：>20%
- 警告：10% ~ 20%（提示准备加料）
- 低位：<10%（需要立即加料）

---

### 5.3 提供服务

#### 5.3.1 流量计数器重置服务

**服务名称**：`/spray/reset_counter`

**服务类型**：`std_srvs/srv/Trigger`

**功能**：重置累计流量计数器

#### 5.3.2 传感器校准服务

**服务名称**：`/spray/calibrate`

**服务类型**：自定义 `CalibrateSpraySensor.srv`

**服务定义**：
```yaml
# 请求
string sensor_name    # "pressure", "flow", "level"
float32 reference_value
---
# 响应
bool success
string message
float32 calibration_factor
```

---

## 6. 距离测量节点接口

### 6.1 节点信息

**节点名称**：`range_node_01`

**功能描述**：测量机器人与墙面的距离

**管理硬件**：
- 前方距离传感器（VL53L0X或超声波）
- 左侧距离传感器
- 右侧距离传感器
- 后方距离传感器

### 6.2 发布话题

#### 6.2.1 前方距离话题

**话题名称**：`/range/front`

**消息类型**：[`sensor_msgs/msg/Range`](https://docs.ros.org/en/humble/p/sensor_msgs/msg/Range.html)

**发布频率**：10 Hz

**QoS配置**：
- Reliability: BEST_EFFORT
- History: KEEP_LAST(5)

**消息结构**：
```yaml
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id: "range_front"

uint8 radiation_type    # ULTRASOUND=0, INFRARED=1
float32 field_of_view   # 视场角 (弧度)
float32 min_range       # 最小测距 (m): 0.02
float32 max_range       # 最大测距 (m): 2.0
float32 range           # 当前距离 (m)
  # -inf: 小于min_range
  # +inf: 大于max_range
```

**其他方向话题**：
- `/range/left` - 左侧距离
- `/range/right` - 右侧距离
- `/range/rear` - 后方距离（可选）

---

### 6.3 提供服务

#### 6.3.1 偏移校准服务

**服务名称**：`/range/calibrate_offset`

**服务类型**：自定义 `CalibrateRangeOffset.srv`

**服务定义**：
```yaml
# 请求
string sensor_name    # "front", "left", "right", "rear"
float32 offset        # 偏移量 (m)
---
# 响应
bool success
string message
```

---

## 7. IMU姿态节点接口

### 7.1 节点信息

**节点名称**：`imu_node_01`

**功能描述**：测量机器人姿态和加速度

**管理硬件**：
- IMU传感器（MPU6050或ICM20948）

### 7.2 发布话题

#### 7.2.1 IMU数据话题

**话题名称**：`/imu/data`

**消息类型**：[`sensor_msgs/msg/Imu`](https://docs.ros.org/en/humble/p/sensor_msgs/msg/Imu.html)

**发布频率**：50 Hz

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(10)

**消息结构**：
```yaml
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id: "imu_link"

geometry_msgs/Quaternion orientation
  float64 x, y, z, w    # 姿态四元数（若有融合算法）
  # 无融合时：x=0, y=0, z=0, w=1

float64[9] orientation_covariance
  # 第一个元素为-1.0表示orientation无效

geometry_msgs/Vector3 angular_velocity
  float64 x    # 绕X轴角速度 (rad/s)
  float64 y    # 绕Y轴角速度 (rad/s)
  float64 z    # 绕Z轴角速度 (rad/s)

float64[9] angular_velocity_covariance

geometry_msgs/Vector3 linear_acceleration
  float64 x    # X轴加速度 (m/s²)
  float64 y    # Y轴加速度 (m/s²)
  float64 z    # Z轴加速度 (m/s²，包含重力)

float64[9] linear_acceleration_covariance
```

**坐标系定义**（右手坐标系）：
- X轴：机器人前进方向
- Y轴：机器人左侧方向
- Z轴：机器人向上方向

**倾斜监测**：
- Roll/Pitch角 >5°：警告（可能倾倒）
- Roll/Pitch角 >10°：紧急停止

---

### 7.3 提供服务

#### 7.3.1 零点校准服务

**服务名称**：`/imu/calibrate`

**服务类型**：自定义 `CalibrateIMU.srv`

**服务定义**：
```yaml
# 请求
string calibration_type    # "gyro_offset", "accel_offset", "full"
int32 sample_count: 100    # 采样次数
---
# 响应
bool success
string message
float64[] calibration_params
  # 陀螺仪：[gx_offset, gy_offset, gz_offset]
  # 加速度：[ax_offset, ay_offset, az_offset]
float64 calibration_error
```

#### 7.3.2 姿态重置服务

**服务名称**：`/imu/reset_orientation`

**服务类型**：`std_srvs/srv/Trigger`

**功能**：重置姿态角为零（当前方向设为参考方向）

---

## 8. 电源管理节点接口

### 8.1 节点信息

**节点名称**：`power_node_01`

**功能描述**：监测电池状态和电源管理

**管理硬件**：
- 电池电压监测
- 电池电流监测
- 充电状态检测

### 8.2 发布话题

#### 8.2.1 电池状态话题

**话题名称**：`/battery/state`

**消息类型**：[`sensor_msgs/msg/BatteryState`](https://docs.ros.org/en/humble/p/sensor_msgs/msg/BatteryState.html)

**发布频率**：1 Hz

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(5)

**消息结构**：
```yaml
std_msgs/Header header

float32 voltage              # 电压 (V): 40.0 ~ 54.6（48V锂电池）
float32 temperature          # 电池温度 (°C)，NaN=未测量
float32 current              # 电流 (A)，负值=放电
float32 charge               # 剩余电量 (Ah)
float32 capacity: 20.0       # 总容量 (Ah)
float32 design_capacity: 20.0
float32 percentage           # 电量百分比 (0.0-1.0)

uint8 power_supply_status
  # UNKNOWN=0, CHARGING=1, DISCHARGING=2, 
  # NOT_CHARGING=3, FULL=4

uint8 power_supply_health
  # UNKNOWN=0, GOOD=1, OVERHEAT=2, DEAD=3,
  # OVERVOLTAGE=4, UNSPEC_FAILURE=5, COLD=6

uint8 power_supply_technology: 3  # LIPO=3（锂聚合物）

bool present: true           # 电池存在
string location: "Main Battery"
string serial_number         # 电池序列号
```

**电源管理策略**：

| 电量范围 | 系统状态 | 动作 |
|---------|---------|------|
| **>50%** | 正常工作 | 无限制 |
| **20-50%** | 低电量预警 | 发布警告，提示充电 |
| **10-20%** | 低电量保护 | 降低功耗，限制高功率动作 |
| **<10%** | 极低电量 | 自动返回充电站/停止作业 |

---

### 8.3 提供服务

#### 8.3.1 电量校准服务

**服务名称**：`/battery/calibrate`

**服务类型**：自定义 `CalibrateBattery.srv`

**服务定义**：
```yaml
# 请求
float32 actual_capacity    # 实际容量 (Ah)
---
# 响应
bool success
string message
```

---

## 9. QoS配置规范

### 9.1 预定义QoS配置文件

| 配置文件名称 | Reliability | History | Durability | 适用场景 |
|------------|-------------|---------|------------|---------|
| **CONTROL_CMD** | RELIABLE | KEEP_LAST(1) | VOLATILE | 控制指令 |
| **SENSOR_DATA** | RELIABLE | KEEP_LAST(10) | VOLATILE | 传感器数据 |
| **BEST_EFFORT_SENSOR** | BEST_EFFORT | KEEP_LAST(5) | VOLATILE | 高频非关键数据 |
| **DIAGNOSTICS** | RELIABLE | KEEP_LAST(10) | VOLATILE | 诊断信息 |
| **SERVICES** | RELIABLE | KEEP_LAST(1) | VOLATILE | 服务调用 |

### 9.2 QoS参数说明

**Reliability（可靠性）**：
- `RELIABLE`：保证消息送达，适用于关键数据（控制指令、状态反馈）
- `BEST_EFFORT`：尽力而为，适用于高频非关键数据（距离传感器）

**History（历史记录）**：
- `KEEP_LAST(n)`：保留最后n条消息
- `KEEP_ALL`：保留所有消息（不推荐，内存受限）

**Durability（持久性）**：
- `VOLATILE`：订阅者只接收订阅后的消息（推荐）
- `TRANSIENT_LOCAL`：新订阅者接收历史消息

---

## 10. 命名规范

### 10.1 话题命名规范

**格式**：`/<node_function>/<data_type>`

**示例**：
- `/chassis/cmd_vel` - 底盘速度指令
- `/chassis/odom` - 底盘里程计
- `/lift/position` - 升降位置
- `/environment/temperature` - 环境温度

**规则**：
- 使用小写字母
- 使用下划线分隔单词
- 避免使用驼峰命名
- 保持简洁明了

### 10.2 服务命名规范

**格式**：`/<node_function>/<action>`

**示例**：
- `/chassis/reset_odom` - 重置里程计
- `/lift/home` - 升降归零
- `/imu/calibrate` - IMU校准

### 10.3 坐标系命名规范

**标准坐标系ID**：

| 坐标系ID | 说明 | 父坐标系 |
|---------|------|---------|
| `map` | 全局地图坐标系 | - |
| `odom` | 里程计坐标系 | map |
| `base_link` | 机器人基座坐标系 | odom |
| `imu_link` | IMU传感器坐标系 | base_link |
| `range_front` | 前方距离传感器坐标系 | base_link |
| `range_left` | 左侧距离传感器坐标系 | base_link |
| `range_right` | 右侧距离传感器坐标系 | base_link |

**遵循REP 105标准**：[ROS坐标系约定](https://www.ros.org/reps/rep-0105.html)

---

## 11. 使用示例

### 11.1 启动Micro-ROS Agent

```bash
# 在上位机启动Agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v
```

### 11.2 控制底盘运动

```bash
# 前进0.5m/s
ros2 topic pub /chassis/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 左转0.3rad/s
ros2 topic pub /chassis/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# 停止
ros2 topic pub /chassis/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 11.3 控制升降高度

```bash
# 升至1.5米
ros2 topic pub /lift/cmd_position std_msgs/msg/Float32 "{data: 1.5}"

# 归零
ros2 service call /lift/home std_srvs/srv/Trigger
```

### 11.4 订阅传感器数据

```bash
# 订阅里程计
ros2 topic echo /chassis/odom

# 订阅环境温度
ros2 topic echo /environment/temperature

# 订阅喷涂压力
ros2 topic echo /spray/pressure

# 订阅前方距离
ros2 topic echo /range/front

# 订阅IMU数据
ros2 topic echo /imu/data

# 订阅电池状态
ros2 topic echo /battery/state
```

### 11.5 调用服务

```bash
# 重置里程计
ros2 service call /chassis/reset_odom std_srvs/srv/Trigger

# 校准IMU
ros2 service call /imu/calibrate CalibrateIMU \
  "{calibration_type: 'gyro_offset', sample_count: 100}"

# 设置环境报警阈值
ros2 service call /environment/set_alarm SetEnvironmentAlarm \
  "{temp_min: 5.0, temp_max: 35.0, humidity_min: 40.0, humidity_max: 85.0}"
```

### 11.6 监控系统状态

```bash
# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看所有服务
ros2 service list

# 查看话题频率
ros2 topic hz /chassis/odom

# 查看话题信息
ros2 topic info /chassis/odom

# 查看节点信息
ros2 node info /chassis_node_01
```

---

## 附录：消息类型依赖

本系统使用以下ROS 2消息包：

| 消息包 | 版本 | 用途 |
|--------|------|------|
| `std_msgs` | humble | 基础消息类型 |
| `sensor_msgs` | humble | 传感器消息 |
| `diagnostic_msgs` | humble | 诊断消息 |
| `geometry_msgs` | humble | 几何消息 |
| `nav_msgs` | humble | 导航消息 |
| `std_srvs` | humble | 标准服务 |
| `builtin_interfaces` | humble | 内置接口 |

---

**文档结束**