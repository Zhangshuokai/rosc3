# Menuconfig配置项说明表

## 文档信息

| 项目 | 内容 |
|------|------|
| **文档标题** | Menuconfig配置项说明表 |
| **文档版本** | v1.0.0 |
| **编制日期** | 2025-10-23 |
| **适用系统** | ESP-IDF v5.1+ |
| **配置文件** | `Kconfig.projbuild` |

---

## 目录

- [1. WiFi配置](#1-wifi配置)
- [2. Micro-ROS配置](#2-micro-ros配置)
- [3. 传感器配置](#3-传感器配置)
- [4. 系统配置](#4-系统配置)
- [5. 调试与日志配置](#5-调试与日志配置)
- [6. 高级配置](#6-高级配置)
- [7. 配置文件示例](#7-配置文件示例)

---

## 1. WiFi配置

### 1.1 基本连接配置

#### 1.1.1 WiFi SSID

**配置路径**：`Component config → WiFi Configuration → WiFi SSID`

**配置项名称**：`CONFIG_WIFI_SSID`

**类型**：String

**默认值**：`""`（空字符串）

**取值范围**：1-32字符

**必填性**：✅ 必填

**说明**：WiFi网络名称（SSID），用于连接无线路由器。

**约束规则**：
- 长度必须在1-32个字符之间
- 支持ASCII字符
- 编译时检查非空
- 特殊字符需要转义

**配置示例**：
```kconfig
CONFIG_WIFI_SSID="MyHomeNetwork"
```

**错误处理**：
- 如果为空，编译时报错：`Error: WiFi SSID cannot be empty`
- 如果超长，编译时报错：`Error: WiFi SSID exceeds 32 characters`

---

#### 1.1.2 WiFi密码

**配置路径**：`Component config → WiFi Configuration → WiFi Password`

**配置项名称**：`CONFIG_WIFI_PASSWORD`

**类型**：String

**默认值**：`""`（空字符串）

**取值范围**：
- WPA2/WPA3模式：8-64字符
- OPEN模式：允许为空

**必填性**：⚠️ 条件必填（取决于加密方式）

**说明**：WiFi密码，用于WPA2/WPA3加密网络认证。

**约束规则**：
- WPA2/WPA3模式下必须8-64字符
- OPEN模式下必须为空
- 密码明文存储在sdkconfig中（⚠️安全警告）
- 运行时可通过NVS加密存储

**安全警告**：
```
⚠️ WARNING: WiFi password is stored in plain text in sdkconfig file.
   For production, use NVS encrypted storage or Flash encryption.
```

**配置示例**：
```kconfig
CONFIG_WIFI_PASSWORD="MySecurePass123"
```

---

#### 1.1.3 WiFi加密方式

**配置路径**：`Component config → WiFi Configuration → WiFi Encryption`

**配置项名称**：`CONFIG_WIFI_ENCRYPTION`

**类型**：Choice（单选）

**默认值**：`WPA2_PSK`

**可选值**：
- `WPA2_PSK` - WPA2个人模式（推荐）
- `WPA3_PSK` - WPA3个人模式（高安全性）
- `WPA_PSK` - WPA个人模式（不推荐，兼容旧设备）
- `OPEN` - 开放网络（仅测试用）

**说明**：WiFi网络的加密方式，影响网络安全性。

**选择建议**：

| 加密方式 | 安全等级 | 推荐场景 | 备注 |
|---------|---------|---------|------|
| WPA3_PSK | ⭐⭐⭐⭐⭐ | 高安全要求 | 需路由器支持 |
| WPA2_PSK | ⭐⭐⭐⭐ | 通用场景 | 最常用 |
| WPA_PSK | ⭐⭐ | 兼容旧设备 | 不推荐 |
| OPEN | ❌ | 仅开发测试 | 禁止生产环境 |

**配置示例**：
```kconfig
CONFIG_WIFI_ENCRYPTION=WPA2_PSK
```

---

#### 1.1.4 WiFi最大重试次数

**配置路径**：`Component config → WiFi Configuration → Maximum Retry Count`

**配置项名称**：`CONFIG_WIFI_MAX_RETRY`

**类型**：Integer

**默认值**：`5`

**取值范围**：1-20

**说明**：WiFi连接失败后的最大重试次数。

**行为说明**：
- 连接失败后自动重试
- 超过最大次数后停止重试
- 可通过ROS服务触发手动重连

**配置示例**：
```kconfig
CONFIG_WIFI_MAX_RETRY=5
```

**推荐值**：
- 开发环境：3-5次
- 生产环境：5-10次

---

#### 1.1.5 WiFi连接超时时间

**配置路径**：`Component config → WiFi Configuration → Connection Timeout (seconds)`

**配置项名称**：`CONFIG_WIFI_CONNECT_TIMEOUT`

**类型**：Integer

**默认值**：`30`

**取值范围**：5-120秒

**说明**：单次WiFi连接尝试的超时时间。

**配置示例**：
```kconfig
CONFIG_WIFI_CONNECT_TIMEOUT=30
```

---

### 1.2 高级WiFi配置

#### 1.2.1 启用静态IP

**配置路径**：`Component config → WiFi Configuration → Enable Static IP`

**配置项名称**：`CONFIG_WIFI_STATIC_IP_ENABLE`

**类型**：Bool

**默认值**：`false`

**说明**：启用静态IP配置，禁用DHCP。

**依赖配置**：
- `CONFIG_WIFI_STATIC_IP_ADDR`：静态IP地址
- `CONFIG_WIFI_STATIC_NETMASK`：子网掩码
- `CONFIG_WIFI_STATIC_GATEWAY`：网关地址

**配置示例**：
```kconfig
CONFIG_WIFI_STATIC_IP_ENABLE=y
CONFIG_WIFI_STATIC_IP_ADDR="192.168.1.100"
CONFIG_WIFI_STATIC_NETMASK="255.255.255.0"
CONFIG_WIFI_STATIC_GATEWAY="192.168.1.1"
```

---

#### 1.2.2 WiFi省电模式

**配置路径**：`Component config → WiFi Configuration → Power Save Mode`

**配置项名称**：`CONFIG_WIFI_POWER_SAVE_MODE`

**类型**：Choice

**默认值**：`NONE`

**可选值**：
- `NONE` - 不省电（性能最佳）
- `MIN_MODEM` - 最小调制解调器省电
- `MAX_MODEM` - 最大调制解调器省电

**说明**：WiFi省电模式，影响功耗和响应时间。

**性能对比**：

| 模式 | 功耗 | 延迟 | 吞吐量 | 适用场景 |
|------|------|------|--------|---------|
| NONE | 高 | 低 | 高 | 实时通信 |
| MIN_MODEM | 中 | 中 | 中 | 平衡模式 |
| MAX_MODEM | 低 | 高 | 低 | 电池供电 |

---

## 2. Micro-ROS配置

### 2.1 Agent连接配置

#### 2.1.1 Micro-ROS Agent IP地址

**配置路径**：`Component config → Micro-ROS Configuration → Agent IP Address`

**配置项名称**：`CONFIG_MICRO_ROS_AGENT_IP`

**类型**：String

**默认值**：`"192.168.1.100"`

**格式要求**：IPv4地址格式（xxx.xxx.xxx.xxx）

**必填性**：✅ 必填

**说明**：运行Micro-ROS Agent的上位机IP地址。

**约束规则**：
- 必须是有效的IPv4地址
- 编译时进行格式校验
- 运行时自动验证可达性

**配置示例**：
```kconfig
CONFIG_MICRO_ROS_AGENT_IP="192.168.1.10"
```

**验证正则**：
```
^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$
```

---

#### 2.1.2 Micro-ROS Agent端口

**配置路径**：`Component config → Micro-ROS Configuration → Agent Port`

**配置项名称**：`CONFIG_MICRO_ROS_AGENT_PORT`

**类型**：Integer

**默认值**：`8888`

**取值范围**：1024-65535

**说明**：Micro-ROS Agent监听的UDP端口号。

**约束规则**：
- 必须在有效端口范围内（1024-65535）
- 避免使用已知服务端口

**常用端口**：
- `8888`：默认端口（推荐）
- `9999`：备用端口

**配置示例**：
```kconfig
CONFIG_MICRO_ROS_AGENT_PORT=8888
```

---

#### 2.1.3 ROS节点名称

**配置路径**：`Component config → Micro-ROS Configuration → ROS Node Name`

**配置项名称**：`CONFIG_MICRO_ROS_NODE_NAME`

**类型**：String

**默认值**：`"esp32c3_node"`

**格式要求**：字母、数字、下划线，不能以数字开头

**说明**：ROS 2图中的节点名称。

**约束规则**：
- 仅允许字母、数字、下划线
- 不能以数字开头
- 长度1-255字符
- 命名唯一性由用户保证

**配置示例**：
```kconfig
CONFIG_MICRO_ROS_NODE_NAME="esp32c3_sensor_node"
```

**命名建议**：
- 描述性命名：`temperature_sensor_01`
- 包含设备类型：`esp32c3_imu_node`
- 避免特殊字符：使用下划线代替空格

---

#### 2.1.4 ROS命名空间

**配置路径**：`Component config → Micro-ROS Configuration → ROS Namespace`

**配置项名称**：`CONFIG_MICRO_ROS_NAMESPACE`

**类型**：String

**默认值**：`""`（空，表示全局命名空间）

**格式要求**：以`/`开头的路径格式

**说明**：ROS话题和服务的命名空间前缀。

**配置示例**：
```kconfig
CONFIG_MICRO_ROS_NAMESPACE="/robot1"
# 话题变为：/robot1/esp32c3/imu/data
```

**使用场景**：
- 多机器人系统：`/robot1`, `/robot2`
- 分层系统：`/floor1/room1`
- 测试隔离：`/test`

---

#### 2.1.5 ROS Domain ID

**配置路径**：`Component config → Micro-ROS Configuration → ROS Domain ID`

**配置项名称**：`CONFIG_MICRO_ROS_DOMAIN_ID`

**类型**：Integer

**默认值**：`0`

**取值范围**：0-232

**说明**：ROS 2的域ID，用于网络隔离。

**使用场景**：
- 同一网络多个ROS系统隔离
- 避免不同项目间干扰
- 测试与生产环境隔离

**配置示例**：
```kconfig
CONFIG_MICRO_ROS_DOMAIN_ID=0    # 默认域
CONFIG_MICRO_ROS_DOMAIN_ID=42   # 自定义域
```

---

### 2.2 Micro-ROS传输配置

#### 2.2.1 传输协议

**配置路径**：`Component config → Micro-ROS Configuration → Transport Protocol`

**配置项名称**：`CONFIG_MICRO_ROS_TRANSPORT`

**类型**：Choice

**默认值**：`UDP`

**可选值**：
- `UDP` - UDP over WiFi（推荐）
- `TCP` - TCP over WiFi（可选）
- `SERIAL` - 串口通信（调试用）

**配置示例**：
```kconfig
CONFIG_MICRO_ROS_TRANSPORT=UDP
```

---

#### 2.2.2 连接超时时间

**配置路径**：`Component config → Micro-ROS Configuration → Connection Timeout (ms)`

**配置项名称**：`CONFIG_MICRO_ROS_CONNECT_TIMEOUT`

**类型**：Integer

**默认值**：`5000`（5秒）

**取值范围**：1000-30000毫秒

**说明**：与Micro-ROS Agent建立连接的超时时间。

---

#### 2.2.3 心跳间隔

**配置路径**：`Component config → Micro-ROS Configuration → Heartbeat Interval (ms)`

**配置项名称**：`CONFIG_MICRO_ROS_HEARTBEAT_INTERVAL`

**类型**：Integer

**默认值**：`1000`（1秒）

**取值范围**：100-10000毫秒

**说明**：向Agent发送心跳的间隔时间。

---

## 3. 传感器配置

### 3.1 IMU传感器配置

#### 3.1.1 启用IMU

**配置路径**：`Component config → Sensor Configuration → Enable IMU`

**配置项名称**：`CONFIG_SENSOR_IMU_ENABLE`

**类型**：Bool

**默认值**：`true`

**说明**：启用IMU传感器（MPU6050/ICM20948等）。

---

#### 3.1.2 IMU采样频率

**配置路径**：`Component config → Sensor Configuration → IMU → Sampling Rate (Hz)`

**配置项名称**：`CONFIG_SENSOR_IMU_SAMPLING_RATE`

**类型**：Integer

**默认值**：`50`

**取值范围**：1-100 Hz

**说明**：IMU数据采样频率。

**性能影响**：

| 频率 | CPU占用 | 功耗 | 适用场景 |
|------|---------|------|---------|
| 10 Hz | 低 | 低 | 静态监测 |
| 50 Hz | 中 | 中 | 通用应用 |
| 100 Hz | 高 | 高 | 高速运动 |

**配置示例**：
```kconfig
CONFIG_SENSOR_IMU_SAMPLING_RATE=50
```

---

#### 3.1.3 IMU I2C地址

**配置路径**：`Component config → Sensor Configuration → IMU → I2C Address`

**配置项名称**：`CONFIG_SENSOR_IMU_I2C_ADDR`

**类型**：Hex

**默认值**：`0x68`

**可选值**：
- `0x68` - AD0引脚接地（默认）
- `0x69` - AD0引脚接VCC

**配置示例**：
```kconfig
CONFIG_SENSOR_IMU_I2C_ADDR=0x68
```

---

#### 3.1.4 IMU加速度量程

**配置路径**：`Component config → Sensor Configuration → IMU → Accelerometer Range`

**配置项名称**：`CONFIG_SENSOR_IMU_ACCEL_RANGE`

**类型**：Choice

**默认值**：`8G`

**可选值**：
- `2G` - ±2g（高精度）
- `4G` - ±4g（平衡）
- `8G` - ±8g（推荐）
- `16G` - ±16g（高冲击）

**配置示例**：
```kconfig
CONFIG_SENSOR_IMU_ACCEL_RANGE=8G
```

---

#### 3.1.5 IMU陀螺仪量程

**配置路径**：`Component config → Sensor Configuration → IMU → Gyroscope Range`

**配置项名称**：`CONFIG_SENSOR_IMU_GYRO_RANGE`

**类型**：Choice

**默认值**：`1000DPS`

**可选值**：
- `250DPS` - ±250°/s（高精度）
- `500DPS` - ±500°/s（平衡）
- `1000DPS` - ±1000°/s（推荐）
- `2000DPS` - ±2000°/s（高速旋转）

---

### 3.2 温湿度传感器配置

#### 3.2.1 启用温湿度传感器

**配置路径**：`Component config → Sensor Configuration → Enable Temperature/Humidity Sensor`

**配置项名称**：`CONFIG_SENSOR_TEMP_HUM_ENABLE`

**类型**：Bool

**默认值**：`true`

---

#### 3.2.2 传感器类型

**配置路径**：`Component config → Sensor Configuration → Temperature/Humidity → Sensor Type`

**配置项名称**：`CONFIG_SENSOR_TEMP_HUM_TYPE`

**类型**：Choice

**默认值**：`BME280`

**可选值**：
- `BME280` - 温湿度气压传感器（推荐）
- `SHT3X` - 高精度温湿度传感器
- `DHT22` - 经济型温湿度传感器

**配置示例**：
```kconfig
CONFIG_SENSOR_TEMP_HUM_TYPE=BME280
```

---

#### 3.2.3 采样频率

**配置路径**：`Component config → Sensor Configuration → Temperature/Humidity → Sampling Rate (Hz)`

**配置项名称**：`CONFIG_SENSOR_TEMP_HUM_SAMPLING_RATE`

**类型**：Float

**默认值**：`0.5`（每2秒）

**取值范围**：0.1-10.0 Hz

**推荐值**：
- 室内监测：0.1-0.5 Hz
- 实时监控：1-2 Hz
- 快速响应：5-10 Hz

---

### 3.3 其他传感器配置

类似地，还可以配置：
- 光照传感器（BH1750/VEML7700）
- 气压传感器（BMP280/MS5611）
- 距离传感器（VL53L0X/HC-SR04）
- ADC模拟传感器

---

## 4. 系统配置

### 4.1 串口配置

#### 4.1.1 控制台UART端口

**配置路径**：`Component config → Console Configuration → Console UART Number`

**配置项名称**：`CONFIG_CONSOLE_UART_NUM`

**类型**：Choice

**默认值**：`UART0`

**可选值**：
- `UART0` - 默认调试串口
- `UART1` - 备用串口

---

#### 4.1.2 控制台波特率

**配置路径**：`Component config → Console Configuration → Console Baud Rate`

**配置项名称**：`CONFIG_CONSOLE_BAUD_RATE`

**类型**：Choice

**默认值**：`115200`

**可选值**：
- `9600` - 低速（兼容性好）
- `115200` - 标准速度（推荐）
- `921600` - 高速（调试用）

**配置示例**：
```kconfig
CONFIG_CONSOLE_BAUD_RATE=115200
```

---

#### 4.1.3 启用USB CDC

**配置路径**：`Component config → Console Configuration → Enable USB Serial/JTAG Console`

**配置项名称**：`CONFIG_CONSOLE_USB_CDC_ENABLE`

**类型**：Bool

**默认值**：`true`

**说明**：启用USB CDC串口，用于USB直连调试。

---

### 4.2 内存配置

#### 4.2.1 主任务堆栈大小

**配置路径**：`Component config → System Configuration → Main Task Stack Size`

**配置项名称**：`CONFIG_MAIN_TASK_STACK_SIZE`

**类型**：Integer

**默认值**：`4096`（字节）

**取值范围**：2048-16384字节

**说明**：主任务的堆栈大小。

**推荐值**：
- 简单应用：2048-4096字节
- 中等应用：4096-8192字节
- 复杂应用：8192-16384字节

---

#### 4.2.2 LVGL缓冲区大小

**配置路径**：`Component config → LVGL Configuration → Buffer Size`

**配置项名称**：`CONFIG_LVGL_BUFFER_SIZE`

**类型**：Integer

**默认值**：`8192`（像素）

**说明**：LVGL图形库的缓冲区大小。

---

### 4.3 看门狗配置

#### 4.3.1 启用任务看门狗

**配置路径**：`Component config → System Configuration → Enable Task Watchdog Timer`

**配置项名称**：`CONFIG_TASK_WDT_ENABLE`

**类型**：Bool

**默认值**：`true`

**说明**：启用任务看门狗，监控任务是否响应。

---

#### 4.3.2 看门狗超时时间

**配置路径**：`Component config → System Configuration → Task Watchdog Timeout (seconds)`

**配置项名称**：`CONFIG_TASK_WDT_TIMEOUT_S`

**类型**：Integer

**默认值**：`5`

**取值范围**：1-60秒

**说明**：任务未喂狗的超时时间，超时后系统重启。

---

## 5. 调试与日志配置

### 5.1 日志级别配置

#### 5.1.1 默认日志级别

**配置路径**：`Component config → Log Output → Default Log Level`

**配置项名称**：`CONFIG_LOG_DEFAULT_LEVEL`

**类型**：Choice

**默认值**：`INFO`

**可选值**：
- `NONE` - 无日志输出
- `ERROR` - 仅错误
- `WARN` - 警告及以上
- `INFO` - 信息及以上（推荐）
- `DEBUG` - 调试及以上
- `VERBOSE` - 所有日志

**日志级别说明**：

| 级别 | 用途 | 输出量 | 适用场景 |
|------|------|--------|---------|
| NONE | 禁用日志 | 无 | 生产环境（不推荐） |
| ERROR | 仅错误 | 极少 | 生产环境 |
| WARN | 警告 | 少 | 生产环境 |
| INFO | 信息 | 中 | 开发/生产 |
| DEBUG | 调试 | 多 | 开发调试 |
| VERBOSE | 详细 | 极多 | 深度调试 |

**配置示例**：
```kconfig
CONFIG_LOG_DEFAULT_LEVEL=INFO  # 生产环境
CONFIG_LOG_DEFAULT_LEVEL=DEBUG # 开发环境
```

---

#### 5.1.2 组件日志级别覆盖

可为每个组件单独设置日志级别：

**WiFi日志级别**：
```kconfig
CONFIG_LOG_DEFAULT_LEVEL_WIFI=INFO
```

**Micro-ROS日志级别**：
```kconfig
CONFIG_LOG_DEFAULT_LEVEL_MICRO_ROS=DEBUG
```

---

### 5.2 日志输出配置

#### 5.2.1 日志包含时间戳

**配置路径**：`Component config → Log Output → Include Timestamp`

**配置项名称**：`CONFIG_LOG_TIMESTAMP_ENABLE`

**类型**：Bool

**默认值**：`true`

**说明**：日志输出包含时间戳（毫秒级）。

---

#### 5.2.2 日志包含颜色

**配置路径**：`Component config → Log Output → Use ANSI Color Codes`

**配置项名称**：`CONFIG_LOG_COLORS`

**类型**：Bool

**默认值**：`true`

**说明**：使用ANSI颜色代码区分不同日志级别。

---

## 6. 高级配置

### 6.1 安全配置

#### 6.1.1 启用Flash加密

**配置路径**：`Security Features → Enable Flash Encryption`

**配置项名称**：`CONFIG_SECURE_FLASH_ENCRYPTION_MODE_DEVELOPMENT`

**类型**：Bool

**默认值**：`false`

**说明**：启用Flash加密，保护固件和数据。

**⚠️ 警告**：
- 启用后无法轻易恢复
- 需要专用密钥烧录
- 仅生产环境使用

---

#### 6.1.2 启用安全启动

**配置路径**：`Security Features → Enable Secure Boot v2`

**配置项名称**：`CONFIG_SECURE_BOOT_V2_ENABLED`

**类型**：Bool

**默认值**：`false`

**说明**：启用安全启动，验证固件签名。

---

### 6.2 性能优化配置

#### 6.2.1 CPU频率

**配置路径**：`Component config → ESP32C3-Specific → CPU Frequency`

**配置项名称**：`CONFIG_ESP32C3_DEFAULT_CPU_FREQ_MHZ`

**类型**：Choice

**默认值**：`160`

**可选值**：
- `80` - 低功耗
- `160` - 平衡（推荐）

**配置示例**：
```kconfig
CONFIG_ESP32C3_DEFAULT_CPU_FREQ_MHZ=160
```

---

#### 6.2.2 WiFi静态RX/TX缓冲区

**配置路径**：`Component config → Wi-Fi → WiFi RX Buffer Number`

**配置项名称**：`CONFIG_ESP32_WIFI_STATIC_RX_BUFFER_NUM`

**类型**：Integer

**默认值**：`10`

**取值范围**：6-20

**说明**：WiFi接收缓冲区数量，影响吞吐量。

---

## 7. 配置文件示例

### 7.1 基础配置示例

```kconfig
# WiFi配置
CONFIG_WIFI_SSID="MyHomeNetwork"
CONFIG_WIFI_PASSWORD="MyPassword123"
CONFIG_WIFI_ENCRYPTION=WPA2_PSK
CONFIG_WIFI_MAX_RETRY=5
CONFIG_WIFI_CONNECT_TIMEOUT=30

# Micro-ROS配置
CONFIG_MICRO_ROS_AGENT_IP="192.168.1.10"
CONFIG_MICRO_ROS_AGENT_PORT=8888
CONFIG_MICRO_ROS_NODE_NAME="esp32c3_node"
CONFIG_MICRO_ROS_NAMESPACE=""
CONFIG_MICRO_ROS_DOMAIN_ID=0

# 传感器配置
CONFIG_SENSOR_IMU_ENABLE=y
CONFIG_SENSOR_IMU_SAMPLING_RATE=50
CONFIG_SENSOR_TEMP_HUM_ENABLE=y
CONFIG_SENSOR_TEMP_HUM_SAMPLING_RATE=1

# 日志配置
CONFIG_LOG_DEFAULT_LEVEL=INFO
CONFIG_LOG_TIMESTAMP_ENABLE=y
CONFIG_LOG_COLORS=y

# 系统配置
CONFIG_CONSOLE_BAUD_RATE=115200
CONFIG_TASK_WDT_ENABLE=y
CONFIG_TASK_WDT_TIMEOUT_S=5
```

---

### 7.2 高性能配置示例

```kconfig
# 高性能WiFi
CONFIG_WIFI_POWER_SAVE_MODE=NONE
CONFIG_ESP32_WIFI_STATIC_RX_BUFFER_NUM=16

# 高频率采样
CONFIG_SENSOR_IMU_SAMPLING_RATE=100
CONFIG_SENSOR_TEMP_HUM_SAMPLING_RATE=10

# 高CPU频率
CONFIG_ESP32C3_DEFAULT_CPU_FREQ_MHZ=160

# 详细日志
CONFIG_LOG_DEFAULT_LEVEL=DEBUG
```

---

### 7.3 低功耗配置示例

```kconfig
# WiFi省电
CONFIG_WIFI_POWER_SAVE_MODE=MAX_MODEM

# 低采样频率
CONFIG_SENSOR_IMU_SAMPLING_RATE=10
CONFIG_SENSOR_TEMP_HUM_SAMPLING_RATE=0.1

# 低CPU频率
CONFIG_ESP32C3_DEFAULT_CPU_FREQ_MHZ=80

# 简化日志
CONFIG_LOG_DEFAULT_LEVEL=WARN
```

---

## 8. 配置修改方法

### 8.1 图形化配置（推荐）

**PlatformIO环境**：
```bash
# 打开Menuconfig
pio run -t menuconfig
```

**ESP-IDF环境**：
```bash
# 打开Menuconfig
idf.py menuconfig
```

### 8.2 直接编辑sdkconfig文件

**文件位置**：项目根目录下的`sdkconfig`文件

**编辑方式**：
```bash
# 使用文本编辑器
vi sdkconfig
# 或
code sdkconfig
```

**⚠️ 注意**：
- 手动编辑后需运行`pio run`重新生成
- 某些配置互相依赖，建议使用Menuconfig

### 8.3 使用sdkconfig.defaults

**文件位置**：项目根目录下的`sdkconfig.defaults`

**说明**：
- 定义默认配置
- 版本控制友好
- 团队共享配置

**示例**：
```kconfig
# sdkconfig.defaults
CONFIG_WIFI_SSID="DefaultNetwork"
CONFIG_LOG_DEFAULT_LEVEL=INFO
```

---

## 9. 配置验证清单

### 9.1 必填配置检查

- [ ] WiFi SSID已配置且非空
- [ ] WiFi密码符合加密方式要求
- [ ] Micro-ROS Agent IP格式正确
- [ ] Micro-ROS Agent端口在有效范围
- [ ] ROS节点名称符合命名规范

### 9.2 安全配置检查

- [ ] 生产环境禁用OPEN WiFi
- [ ] 密码长度≥8字符（WPA2/WPA3）
- [ ] 考虑启用Flash加密
- [ ] 日志级别不要设为VERBOSE

### 9.3 性能配置检查

- [ ] 采样频率与CPU能力匹配
- [ ] WiFi缓冲区配置合理
- [ ] 堆栈大小足够

---

## 附录：配置项快速索引

| 配置项名称 | 默认值 | 页面 |
|-----------|--------|------|
| `CONFIG_WIFI_SSID` | "" | 1.1.1 |
| `CONFIG_WIFI_PASSWORD` | "" | 1.1.2 |
| `CONFIG_WIFI_ENCRYPTION` | WPA2_PSK | 1.1.3 |
| `CONFIG_MICRO_ROS_AGENT_IP` | "192.168.1.100" | 2.1.1 |
| `CONFIG_MICRO_ROS_AGENT_PORT` | 8888 | 2.1.2 |
| `CONFIG_MICRO_ROS_NODE_NAME` | "esp32c3_node" | 2.1.3 |
| `CONFIG_SENSOR_IMU_SAMPLING_RATE` | 50 | 3.1.2 |
| `CONFIG_LOG_DEFAULT_LEVEL` | INFO | 5.1.1 |
| `CONFIG_CONSOLE_BAUD_RATE` | 115200 | 4.1.2 |

---

**文档结束**