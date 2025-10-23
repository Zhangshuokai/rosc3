# ESP32-C3 SSD1306 OLED 项目

基于PlatformIO和ESP-IDF框架的ESP32-C3单色OLED显示项目，使用LVGL 8.3.11图形库。

## 项目特性

✨ **完整的驱动支持**
- SSD1306 OLED驱动（128x64分辨率，I2C接口）
- LVGL 8.3.11图形库集成
- 优化的单色显示配置（1bpp）

🔧 **关键修复**
- Gap偏移修复（0,0），避免显示乱码
- 针对ESP32-C3优化的内存配置
- 完整的错误处理和日志记录

📦 **开箱即用**
- PlatformIO项目配置
- 自动管理ESP-IDF组件依赖
- 详细的代码注释和文档

## 硬件要求

- ESP32-C3开发板（如ESP32-C3-DevKitM-1）
- SSD1306 OLED显示屏（128x64，I2C接口）
- USB数据线

## 快速开始

### 1. 克隆项目

```bash
cd your-workspace
# 项目已在当前目录
```

### 2. 硬件连接

按照[硬件连接说明](docs/HARDWARE_CONNECTION.md)连接ESP32-C3和OLED：

| ESP32-C3 | OLED |
|----------|------|
| GPIO5    | SDA  |
| GPIO6    | SCL  |
| 3.3V     | VCC  |
| GND      | GND  |

### 3. 构建和上传

```bash
# 构建项目（首次会自动下载依赖，需要一些时间）
pio run

# 上传到开发板
pio run --target upload

# 上传并打开串口监视器
pio run --target upload --target monitor
```

### 4. 查看效果

上传成功后，OLED屏幕应该显示滚动文本："Hello ESP32-C3! Hello LVGL!"

串口输出示例：
```
I (318) main: === ESP32-C3 SSD1306 OLED初始化开始 ===
I (318) main: [1/5] 初始化I2C总线...
I (328) main:   ✓ I2C总线初始化成功 (SDA: GPIO5, SCL: GPIO6)
I (338) main: [2/5] 安装LCD面板IO接口...
I (348) main:   ✓ LCD IO接口安装成功 (地址: 0x3C)
I (358) main: [3/5] 安装SSD1306面板驱动...
I (368) main:   ✓ SSD1306驱动安装成功
I (378) main:   ⚠ 应用显示修复：Gap偏移 = (0, 0)
I (388) main:   ✓ 面板已激活 (分辨率: 128x64)
I (398) main: [4/5] 初始化LVGL图形库...
I (448) main:   ✓ LVGL初始化成功
I (448) main: [5/5] 创建UI界面...
I (458) main:   ✓ UI创建成功
I (458) main: === 初始化完成，系统运行中 ===
```

## 项目结构

```
rosc3/
├── platformio.ini              # PlatformIO配置
├── sdkconfig.defaults          # ESP-IDF SDK配置
├── README.md                   # 本文件
├── .gitignore                  # Git忽略规则
│
├── include/
│   └── lv_conf.h              # LVGL配置文件
│
├── src/
│   ├── main.c                 # 主程序
│   ├── lvgl_demo_ui.c         # UI演示代码
│   ├── lvgl_demo_ui.h         # UI头文件
│   ├── idf_component.yml      # ESP-IDF组件依赖
│   └── CMakeLists.txt         # 构建配置
│
└── docs/
    ├── PLATFORMIO_ARCHITECTURE_DESIGN.md  # 完整架构文档
    └── HARDWARE_CONNECTION.md             # 硬件连接说明
```

## 配置说明

### 修改I2C引脚

在[`src/main.c`](src/main.c)中修改：

```c
#define I2C_MASTER_SCL_IO       GPIO_NUM_6    /* 修改为你的SCL引脚 */
#define I2C_MASTER_SDA_IO       GPIO_NUM_5    /* 修改为你的SDA引脚 */
```

### 修改I2C地址

如果OLED使用不同的I2C地址（通常是0x3D），修改：

```c
#define OLED_I2C_ADDRESS        0x3C    /* 修改为0x3D或其他地址 */
```

### 自定义UI

编辑[`src/lvgl_demo_ui.c`](src/lvgl_demo_ui.c)来创建自己的UI界面。

## 文档

- 📖 [完整架构设计文档](docs/PLATFORMIO_ARCHITECTURE_DESIGN.md) - 详细的项目架构、配置说明和开发指南
- 🔌 [硬件连接说明](docs/HARDWARE_CONNECTION.md) - 引脚连接和常见硬件问题排查

## 常见问题

### 1. 显示乱码或花屏

✅ **已修复**：代码中已包含Gap偏移修复（设置为0,0）

### 2. 编译错误：找不到头文件

这是正常的IntelliSense错误。执行`pio run`后会自动下载所需组件并解决。

### 3. 上传失败

- 检查USB连接
- 确认开发板型号正确（在platformio.ini中）
- 尝试按住BOOT按钮再上传

### 4. OLED无显示

- 检查硬件连接（参考[硬件连接说明](docs/HARDWARE_CONNECTION.md)）
- 确认I2C地址正确（0x3C或0x3D）
- 查看串口输出是否有错误

## 技术栈

- **平台**: PlatformIO
- **框架**: ESP-IDF 5.3.x
- **芯片**: ESP32-C3
- **显示**: SSD1306 OLED (128x64, I2C)
- **图形库**: LVGL 8.3.11
- **组件依赖**:
  - lvgl/lvgl ~8.3.0
  - esp_lcd_sh1107 ^1
  - esp_lvgl_port ^1

## 开发

### 构建命令

```bash
# 仅编译
pio run

# 清理构建
pio run --target clean

# 完全清理（包括下载的组件）
pio run --target fullclean

# 打开串口监视器
pio device monitor
```

### VSCode集成

项目已配置PlatformIO IDE，可直接使用VSCode的PlatformIO插件进行开发。

## 许可证

本项目基于参考代码开发，遵循相关开源许可证。

## 参考资料

- [ESP-IDF文档](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [LVGL文档](https://docs.lvgl.io/8.3/)
- [PlatformIO文档](https://docs.platformio.org/)
- [SSD1306数据手册](https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf)

## 贡献

欢迎提交Issue和Pull Request！

---

**注意**：首次构建会下载ESP-IDF框架和组件依赖，可能需要5-10分钟，请耐心等待。