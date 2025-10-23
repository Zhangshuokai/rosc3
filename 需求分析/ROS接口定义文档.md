# ROS接口定义文档

## 文档信息

| 项目 | 内容 |
|------|------|
| **文档标题** | ROS接口定义文档 |
| **文档版本** | v1.0.0 |
| **编制日期** | 2025-10-23 |
| **ROS版本** | ROS 2 Humble |
| **DDS实现** | rmw_microxrcedds |

---

## 目录

- [1. 话题定义](#1-话题定义)
  - [1.1 传感器数据话题](#11-传感器数据话题)
  - [1.2 系统状态话题](#12-系统状态话题)
  - [1.3 控制指令话题](#13-控制指令话题)
- [2. 服务定义](#2-服务定义)
  - [2.1 配置管理服务](#21-配置管理服务)
  - [2.2 传感器控制服务](#22-传感器控制服务)
  - [2.3 系统管理服务](#23-系统管理服务)
- [3. 自定义消息类型](#3-自定义消息类型)
- [4. QoS配置详情](#4-qos配置详情)
- [5. 命名规范](#5-命名规范)

---

## 1. 话题定义

### 1.1 传感器数据话题

#### 1.1.1 IMU数据话题

**话题名称**：`/esp32c3/imu/data`

**消息类型**：`sensor_msgs/msg/Imu`

**发布频率**：50 Hz（可通过Menuconfig配置1-100 Hz）

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(10)
- Durability: VOLATILE

**消息结构**：
```yaml
std_msgs/Header header
  uint32 seq                          # 序列号，自动递增
  builtin_interfaces/Time stamp       # 时间戳（ROS时间或系统时间）
  string frame_id                     # 坐标系ID："imu_link"

geometry_msgs/Quaternion orientation  # 姿态四元数（若有融合算法）
  float64 x: 0.0                      # 默认值（无融合时）
  float64 y: 0.0
  float64 z: 0.0
  float64 w: 1.0

float64[9] orientation_covariance     # 姿态协方差矩阵
  # 若未知则全部设为0，第一个元素设为-1表示未知
  # [0]: -1.0 表示orientation数据无效

geometry_msgs/Vector3 angular_velocity  # 角速度（rad/s）
  float64 x                             # 绕X轴角速度
  float64 y                             # 绕Y轴角速度
  float64 z                             # 绕Z轴角速度

float64[9] angular_velocity_covariance
  # 对角线元素为方差，非对角线为协方差

geometry_msgs/Vector3 linear_acceleration  # 线性加速度（m/s²）
  float64 x                                # X轴加速度
  float64 y                                # Y轴加速度
  float64 z                                # Z轴加速度（包含重力）

float64[9] linear_acceleration_covariance
```

**数据示例**：
```python
# Python示例
imu_msg = Imu()
imu_msg.header.stamp = self.get_clock().now().to_msg()
imu_msg.header.frame_id = "imu_link"
imu_msg.angular_velocity.x = 0.01  # rad/s
imu_msg.angular_velocity.y = 0.02
imu_msg.angular_velocity.z = 0.00
imu_msg.linear_acceleration.x = 0.05  # m/s²
imu_msg.linear_acceleration.y = 0.10
imu_msg.linear_acceleration.z = 9.81  # 重力
```

**坐标系定义**：
- X轴：前进方向
- Y轴：左侧方向
- Z轴：向上方向（右手坐标系）

---

#### 1.1.2 温度数据话题

**话题名称**：`/esp32c3/temperature`

**消息类型**：`sensor_msgs/msg/Temperature`

**发布频率**：1 Hz（可配置0.1-10 Hz）

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(5)
- Durability: VOLATILE

**消息结构**：
```yaml
std_msgs/Header header
  uint32 seq
  builtin_interfaces/Time stamp
  string frame_id: "temperature_sensor"

float64 temperature        # 温度（摄氏度）
float64 variance          # 方差（0.0表示未知）
```

**数据范围**：-40.0°C ~ 85.0°C

**精度要求**：±0.1°C

---

#### 1.1.3 湿度数据话题

**话题名称**：`/esp32c3/humidity`

**消息类型**：`sensor_msgs/msg/RelativeHumidity`

**发布频率**：1 Hz（可配置0.1-10 Hz）

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(5)

**消息结构**：
```yaml
std_msgs/Header header
  uint32 seq
  builtin_interfaces/Time stamp
  string frame_id: "humidity_sensor"

float64 relative_humidity  # 相对湿度（0.0-1.0，0-100%）
float64 variance          # 方差
```

**数据范围**：0.0 ~ 1.0（对应0% ~ 100% RH）

**精度要求**：±0.02（±2% RH）

---

#### 1.1.4 光照强度话题

**话题名称**：`/esp32c3/illuminance`

**消息类型**：`sensor_msgs/msg/Illuminance`

**发布频率**：1 Hz（可配置0.1-10 Hz）

**QoS配置**：
- Reliability: BEST_EFFORT
- History: KEEP_LAST(1)

**消息结构**：
```yaml
std_msgs/Header header
  uint32 seq
  builtin_interfaces/Time stamp
  string frame_id: "light_sensor"

float64 illuminance       # 光照强度（勒克斯 lx）
float64 variance
```

**数据范围**：0.0 ~ 65535.0 lx

---

#### 1.1.5 气压数据话题

**话题名称**：`/esp32c3/pressure`

**消息类型**：`sensor_msgs/msg/FluidPressure`

**发布频率**：1 Hz（可配置0.1-10 Hz）

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(5)

**消息结构**：
```yaml
std_msgs/Header header
  uint32 seq
  builtin_interfaces/Time stamp
  string frame_id: "pressure_sensor"

float64 fluid_pressure    # 气压（帕斯卡 Pa）
float64 variance
```

**数据范围**：30000.0 ~ 110000.0 Pa（300 ~ 1100 hPa）

**单位转换**：1 hPa = 100 Pa

---

#### 1.1.6 距离测量话题

**话题名称**：`/esp32c3/range`

**消息类型**：`sensor_msgs/msg/Range`

**发布频率**：10 Hz（可配置1-50 Hz）

**QoS配置**：
- Reliability: BEST_EFFORT
- History: KEEP_LAST(1)

**消息结构**：
```yaml
std_msgs/Header header
  uint32 seq
  builtin_interfaces/Time stamp
  string frame_id: "range_sensor"

uint8 radiation_type      # 辐射类型
  # ULTRASOUND=0, INFRARED=1
  
float32 field_of_view     # 视场角（弧度）
  # 例如：0.1 rad ≈ 5.7度
  
float32 min_range         # 最小测距（米）
  # 例如：0.02 (20mm)
  
float32 max_range         # 最大测距（米）
  # 例如：2.0 (2000mm)
  
float32 range             # 当前距离（米）
  # -inf: 小于min_range
  # +inf: 大于max_range
  # 其他: 实际距离值
```

**传感器类型示例**：
- VL53L0X: radiation_type=INFRARED, max_range=2.0
- HC-SR04: radiation_type=ULTRASOUND, max_range=4.0

---

### 1.2 系统状态话题

#### 1.2.1 诊断信息话题

**话题名称**：`/esp32c3/diagnostics`

**消息类型**：`diagnostic_msgs/msg/DiagnosticStatus`

**发布频率**：0.5 Hz（每2秒）

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(10)

**消息结构**：
```yaml
byte level                # 诊断级别
  # OK=0, WARN=1, ERROR=2, STALE=3

string name              # 组件名称
  # 例如："ESP32-C3 Node", "IMU Sensor", "WiFi"

string message           # 状态描述
  # 例如："All systems operational"

string hardware_id       # 硬件ID
  # 例如："ESP32-C3:AB:CD:EF:01:23:45"

diagnostic_msgs/KeyValue[] values  # 键值对数组
  # 例如：
  # - key: "CPU Usage", value: "45%"
  # - key: "Free Heap", value: "150 KB"
  # - key: "WiFi RSSI", value: "-65 dBm"
  # - key: "Uptime", value: "12345 s"
```

**诊断级别说明**：
- **OK (0)**：所有系统正常
- **WARN (1)**：有警告但不影响运行（如WiFi信号弱）
- **ERROR (2)**：有错误需要关注（如传感器失败）
- **STALE (3)**：数据过期或通信中断

**发布内容示例**：
```python
diag_msg = DiagnosticStatus()
diag_msg.level = DiagnosticStatus.OK
diag_msg.name = "ESP32-C3 Node"
diag_msg.message = "All systems operational"
diag_msg.hardware_id = "ESP32-C3:AB:CD:EF:01:23:45"
diag_msg.values = [
    KeyValue(key="CPU Usage", value="45%"),
    KeyValue(key="Free Heap", value="150 KB"),
    KeyValue(key="WiFi RSSI", value="-65 dBm"),
    KeyValue(key="Temperature", value="45.2 C"),
    KeyValue(key="Uptime", value="12345 s"),
    KeyValue(key="IMU Status", value="OK"),
    KeyValue(key="Sensor Count", value="3")
]
```

---

#### 1.2.2 电池状态话题（可选）

**话题名称**：`/esp32c3/battery`

**消息类型**：`sensor_msgs/msg/BatteryState`

**发布频率**：0.1 Hz（每10秒）

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(5)

**消息结构**：
```yaml
std_msgs/Header header

float32 voltage              # 电压（V）
float32 temperature          # 温度（°C，NaN=未知）
float32 current             # 电流（A，负值=放电）
float32 charge              # 剩余电量（Ah）
float32 capacity            # 总容量（Ah）
float32 design_capacity     # 设计容量（Ah）
float32 percentage          # 电量百分比（0.0-1.0）

uint8 power_supply_status   # 电源状态
  # UNKNOWN=0, CHARGING=1, DISCHARGING=2, 
  # NOT_CHARGING=3, FULL=4

uint8 power_supply_health   # 电池健康状态
  # UNKNOWN=0, GOOD=1, OVERHEAT=2, DEAD=3,
  # OVERVOLTAGE=4, UNSPEC_FAILURE=5, COLD=6,
  # WATCHDOG_TIMER_EXPIRE=7, SAFETY_TIMER_EXPIRE=8

uint8 power_supply_technology  # 电池技术
  # UNKNOWN=0, NIMH=1, LION=2, LIPO=3, LIFE=4, NICD=5, LIMN=6

bool present                # 电池是否存在
string location            # 电池位置
string serial_number       # 序列号
```

---

### 1.3 控制指令话题

#### 1.3.1 LED控制话题

**话题名称**：`/esp32c3/cmd_led`

**消息类型**：`std_msgs/msg/ColorRGBA`

**订阅频率**：实时响应

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(1)

**消息结构**：
```yaml
float32 r  # 红色分量（0.0-1.0）
float32 g  # 绿色分量（0.0-1.0）
float32 b  # 蓝色分量（0.0-1.0）
float32 a  # 亮度（0.0-1.0）
```

**命令示例**：
```bash
# 设置LED为红色，全亮度
ros2 topic pub /esp32c3/cmd_led std_msgs/msg/ColorRGBA \
  "{r: 1.0, g: 0.0, b: 0.0, a: 1.0}"

# 设置LED为绿色，50%亮度
ros2 topic pub /esp32c3/cmd_led std_msgs/msg/ColorRGBA \
  "{r: 0.0, g: 1.0, b: 0.0, a: 0.5}"
```

---

#### 1.3.2 睡眠模式控制话题

**话题名称**：`/esp32c3/cmd_sleep`

**消息类型**：`std_msgs/msg/Bool`

**订阅频率**：实时响应

**消息结构**：
```yaml
bool data  # true=进入睡眠, false=退出睡眠
```

**命令示例**：
```bash
# 进入睡眠模式
ros2 topic pub /esp32c3/cmd_sleep std_msgs/msg/Bool "{data: true}"

# 退出睡眠模式
ros2 topic pub /esp32c3/cmd_sleep std_msgs/msg/Bool "{data: false}"
```

**睡眠模式行为**：
- 进入睡眠：停止传感器采样，降低WiFi功率，保持ROS连接
- 退出睡眠：恢复正常采样和发布

---

#### 1.3.3 紧急停止话题

**话题名称**：`/esp32c3/cmd_emergency_stop`

**消息类型**：`std_msgs/msg/Empty`

**订阅频率**：实时响应

**QoS配置**：
- Reliability: RELIABLE
- History: KEEP_LAST(1)

**消息结构**：
```yaml
# 无数据字段
```

**命令示例**：
```bash
# 紧急停止
ros2 topic pub /esp32c3/cmd_emergency_stop std_msgs/msg/Empty "{}"
```

**紧急停止行为**：
- 立即停止所有传感器采样
- 停止所有话题发布
- 保持ROS连接以接收恢复指令
- 发布紧急停止状态到诊断话题

---

## 2. 服务定义

### 2.1 配置管理服务

#### 2.1.1 设置采样频率服务

**服务名称**：`/esp32c3/set_sampling_rate`

**服务类型**：自定义 `SetSamplingRate.srv`

**超时时间**：2秒

**服务定义**：
```yaml
# 请求（Request）
string sensor_name      # 传感器名称
  # 可选值："imu", "temperature", "humidity", "light", 
  #         "pressure", "range", "all"
  
float32 frequency_hz    # 目标频率（Hz）
  # 范围：0.1 ~ 1000.0
  
---

# 响应（Response）
bool success           # 是否成功
string message         # 详细信息
  # 例如："Sampling rate updated successfully"
  #      "Invalid frequency: out of range"
  
float32 actual_frequency  # 实际设置的频率（Hz）
  # 可能与请求不同（受硬件限制）
```

**调用示例**：
```bash
# 设置IMU采样频率为100Hz
ros2 service call /esp32c3/set_sampling_rate \
  example_interfaces/srv/SetSamplingRate \
  "{sensor_name: 'imu', frequency_hz: 100.0}"
```

**响应示例**：
```yaml
success: true
message: "IMU sampling rate updated successfully"
actual_frequency: 100.0
```

**错误码**：
- `success=false, message="Invalid sensor name"`：传感器名称不存在
- `success=false, message="Frequency out of range"`：频率超出范围
- `success=false, message="Sensor not initialized"`：传感器未初始化

---

#### 2.1.2 传感器校准服务

**服务名称**：`/esp32c3/calibrate_sensor`

**服务类型**：自定义 `CalibrateSensor.srv`

**超时时间**：10秒

**服务定义**：
```yaml
# 请求
string sensor_name      # 传感器名称："imu", "temperature"等
string calibration_type # 校准类型
  # "zero_offset"    - 零点校准
  # "scale_factor"   - 比例因子校准
  # "full"          - 完整校准
  
int32 sample_count     # 采样次数（默认100）
  
---

# 响应
bool success
string message
float64[] calibration_params  # 校准参数数组
  # IMU零点校准：[accel_x_offset, accel_y_offset, accel_z_offset,
  #               gyro_x_offset, gyro_y_offset, gyro_z_offset]
  # 温度校准：[offset, scale]
  
float64 calibration_error    # 校准误差
```

**调用示例**：
```bash
# IMU零点校准
ros2 service call /esp32c3/calibrate_sensor \
  CalibrateSensor \
  "{sensor_name: 'imu', calibration_type: 'zero_offset', sample_count: 100}"
```

**校准流程**：
1. 请求收到后开始采样
2. 采集指定数量的样本
3. 计算偏移量或比例因子
4. 保存参数到NVS
5. 返回校准结果

---

#### 2.1.3 获取配置服务

**服务名称**：`/esp32c3/get_config`

**服务类型**：自定义 `GetConfig.srv`

**超时时间**：1秒

**服务定义**：
```yaml
# 请求
string config_section   # 配置节（可选）
  # ""           - 获取全部配置
  # "wifi"       - 仅WiFi配置
  # "ros"        - 仅ROS配置
  # "sensors"    - 仅传感器配置
  
---

# 响应
bool success
string message
string config_json     # JSON格式配置字符串
```

**响应JSON格式**：
```json
{
  "wifi": {
    "ssid": "MyNetwork",
    "encryption": "WPA2_PSK",
    "connected": true,
    "rssi": -65,
    "ip_address": "192.168.1.100"
  },
  "ros": {
    "agent_ip": "192.168.1.10",
    "agent_port": 8888,
    "node_name": "esp32c3_node",
    "namespace": "",
    "domain_id": 0,
    "connected": true
  },
  "sensors": {
    "imu": {
      "enabled": true,
      "sampling_rate": 50.0,
      "status": "OK"
    },
    "temperature": {
      "enabled": true,
      "sampling_rate": 1.0,
      "status": "OK"
    }
  },
  "system": {
    "uptime": 12345,
    "cpu_usage": 45,
    "free_heap": 153600,
    "chip_temperature": 45.2
  }
}
```

---

#### 2.1.4 更新WiFi配置服务

**服务名称**：`/esp32c3/set_wifi`

**服务类型**：自定义 `SetWiFi.srv`

**超时时间**：30秒

**服务定义**：
```yaml
# 请求
string ssid            # WiFi SSID
string password        # WiFi密码
string encryption      # 加密方式："WPA2_PSK", "WPA3_PSK", "OPEN"
bool auto_reconnect   # 是否自动重连
bool save_to_nvs      # 是否保存到NVS

---

# 响应
bool success
string message
bool connected        # 是否已连接
string ip_address     # 分配的IP地址
int32 rssi           # 信号强度（dBm）
```

**调用示例**：
```bash
ros2 service call /esp32c3/set_wifi SetWiFi \
  "{ssid: 'NewNetwork', password: 'newpass123', \
    encryption: 'WPA2_PSK', auto_reconnect: true, save_to_nvs: true}"
```

---

### 2.2 传感器控制服务

#### 2.2.1 启用/禁用传感器服务

**服务名称**：`/esp32c3/enable_sensor`

**服务类型**：自定义 `EnableSensor.srv`

**服务定义**：
```yaml
# 请求
string sensor_name    # 传感器名称
bool enable          # true=启用, false=禁用

---

# 响应
bool success
string message
bool is_enabled      # 当前启用状态
```

---

#### 2.2.2 单次采样服务

**服务名称**：`/esp32c3/sample_once`

**服务类型**：自定义 `SampleOnce.srv`

**服务定义**：
```yaml
# 请求
string sensor_name    # 传感器名称

---

# 响应
bool success
string message
float64[] data       # 传感器数据数组
  # IMU: [ax, ay, az, gx, gy, gz]
  # Temperature: [temperature]
  # Humidity: [humidity]
  
builtin_interfaces/Time timestamp  # 采样时间戳
```

---

### 2.3 系统管理服务

#### 2.3.1 系统重启服务

**服务名称**：`/esp32c3/reset_node`

**服务类型**：`std_srvs/srv/Trigger`

**超时时间**：5秒

**服务定义**：
```yaml
# 请求
# 无字段

---

# 响应
bool success
string message
  # 例如："System will restart in 3 seconds"
```

**调用示例**：
```bash
ros2 service call /esp32c3/reset_node std_srvs/srv/Trigger
```

**重启流程**：
1. 停止所有传感器采样
2. 保存关键数据到NVS
3. 断开ROS连接
4. 3秒后重启系统

---

#### 2.3.2 恢复出厂设置服务

**服务名称**：`/esp32c3/factory_reset`

**服务类型**：`std_srvs/srv/Trigger`

**超时时间**：10秒

**服务定义**：
```yaml
# 请求
# 无字段

---

# 响应
bool success
string message
```

**恢复内容**：
- 清除所有NVS配置
- 恢复默认WiFi设置
- 清除传感器校准数据
- 重启系统

---

## 3. 自定义消息类型

### 3.1 传感器数组消息（可选）

**文件路径**：`msg/SensorArray.msg`

**用途**：批量发布多个传感器数据

**消息定义**：
```yaml
std_msgs/Header header

SensorData[] sensors    # 传感器数据数组
```

**SensorData子消息**：
```yaml
# msg/SensorData.msg
string sensor_name      # 传感器名称
string sensor_type      # 传感器类型
uint8 status           # 状态：0=OK, 1=WARN, 2=ERROR
float64[] values       # 数据值数组
builtin_interfaces/Time timestamp
```

---

## 4. QoS配置详情

### 4.1 预定义QoS配置文件

| 配置文件名称 | Reliability | History | Durability | 适用场景 |
|------------|-------------|---------|------------|---------|
| **SENSOR_DATA** | RELIABLE | KEEP_LAST(10) | VOLATILE | 传感器数据 |
| **BEST_EFFORT_SENSOR** | BEST_EFFORT | KEEP_LAST(1) | VOLATILE | 高频非关键数据 |
| **DIAGNOSTICS** | RELIABLE | KEEP_LAST(10) | VOLATILE | 诊断信息 |
| **COMMANDS** | RELIABLE | KEEP_LAST(1) | VOLATILE | 控制指令 |
| **SERVICES** | RELIABLE | KEEP_LAST(1) | VOLATILE | 服务调用 |

### 4.2 QoS参数说明

**Reliability（可靠性）**：
- `RELIABLE`：保证消息送达，适用于关键数据
- `BEST_EFFORT`：尽力而为，适用于高频非关键数据

**History（历史记录）**：
- `KEEP_LAST(n)`：保留最后n条消息
- `KEEP_ALL`：保留所有消息（不推荐，内存受限）

**Durability（持久性）**：
- `VOLATILE`：订阅者只接收订阅后的消息
- `TRANSIENT_LOCAL`：新订阅者接收历史消息

---

## 5. 命名规范

### 5.1 话题命名规范

**格式**：`/<namespace>/<node_name>/<topic_type>/<topic_name>`

**示例**：
- `/robot1/esp32c3/sensor/imu`
- `/robot1/esp32c3/diagnostic/status`
- `/robot1/esp32c3/cmd/led`

**规则**：
- 使用小写字母
- 使用下划线分隔单词
- 避免使用驼峰命名
- 命名空间可选但推荐使用

### 5.2 服务命名规范

**格式**：`/<namespace>/<node_name>/<service_name>`

**示例**：
- `/robot1/esp32c3/set_sampling_rate`
- `/robot1/esp32c3/calibrate_sensor`

### 5.3 坐标系命名规范

**标准坐标系ID**：
- `imu_link`：IMU传感器坐标系
- `base_link`：机器人基座坐标系
- `temperature_sensor`：温度传感器
- `humidity_sensor`：湿度传感器

**遵循REP 105标准**：[ROS坐标系约定](https://www.ros.org/reps/rep-0105.html)

---

## 6. 使用示例

### 6.1 启动Micro-ROS Agent

```bash
# 在上位机启动Agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v
```

### 6.2 订阅传感器数据

```bash
# 订阅IMU数据
ros2 topic echo /esp32c3/imu/data

# 订阅温度数据
ros2 topic echo /esp32c3/temperature

# 订阅诊断信息
ros2 topic echo /esp32c3/diagnostics
```

### 6.3 调用服务

```bash
# 设置IMU采样频率为100Hz
ros2 service call /esp32c3/set_sampling_rate \
  SetSamplingRate "{sensor_name: 'imu', frequency_hz: 100.0}"

# 校准IMU
ros2 service call /esp32c3/calibrate_sensor \
  CalibrateSensor "{sensor_name: 'imu', calibration_type: 'zero_offset', sample_count: 100}"

# 获取配置
ros2 service call /esp32c3/get_config \
  GetConfig "{config_section: ''}"
```

### 6.4 发送控制指令

```bash
# 设置LED为蓝色
ros2 topic pub /esp32c3/cmd_led std_msgs/msg/ColorRGBA \
  "{r: 0.0, g: 0.0, b: 1.0, a: 1.0}"

# 进入睡眠模式
ros2 topic pub /esp32c3/cmd_sleep std_msgs/msg/Bool "{data: true}"
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
| `std_srvs` | humble | 标准服务 |
| `builtin_interfaces` | humble | 内置接口 |

---

**文档结束**