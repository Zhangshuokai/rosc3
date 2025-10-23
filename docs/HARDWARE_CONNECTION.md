# 硬件连接说明

## ESP32-C3 与 SSD1306 OLED连接

### 引脚连接表

| ESP32-C3引脚 | OLED引脚 | 说明          |
|--------------|----------|---------------|
| GPIO5        | SDA      | I2C数据线     |
| GPIO6        | SCL      | I2C时钟线     |
| 3.3V         | VCC      | 电源正极      |
| GND          | GND      | 电源负极      |

### 连接示意图

```
ESP32-C3                    SSD1306 OLED
+----------------+          +----------------+
|                |          |                |
|    GPIO5 (SDA) +----------+ SDA            |
|                |          |                |
|    GPIO6 (SCL) +----------+ SCL            |
|                |          |                |
|          3.3V  +----------+ VCC            |
|                |          |                |
|           GND  +----------+ GND            |
|                |          |                |
+----------------+          +----------------+
```

### 注意事项

1. **引脚可修改**: 可在[`src/main.c`](../src/main.c)中修改I2C引脚定义：
   ```c
   #define I2C_MASTER_SCL_IO       GPIO_NUM_6    /* 修改SCL引脚 */
   #define I2C_MASTER_SDA_IO       GPIO_NUM_5    /* 修改SDA引脚 */
   ```

2. **上拉电阻**: ESP32-C3内部上拉已启用，通常无需外接上拉电阻。如果通信不稳定，可以尝试添加4.7kΩ上拉电阻到SDA和SCL线。

3. **电源**: 确保OLED使用**3.3V**供电。部分模块支持5V，但ESP32-C3的GPIO是3.3V电平，建议统一使用3.3V。

4. **I2C地址**: 默认地址为**0x3C**。部分OLED模块可能使用0x3D，可通过模块背面的焊盘配置。如需修改地址，在[`src/main.c`](../src/main.c)中修改：
   ```c
   #define OLED_I2C_ADDRESS        0x3C    /* 修改为0x3D如果需要 */
   ```

5. **线缆长度**: 建议I2C连线长度不超过20cm，过长可能导致通信不稳定。

## 常见硬件问题排查

### 1. 显示无反应

**可能原因**:
- 电源未连接或电压不足
- I2C引脚连接错误
- I2C地址不匹配

**排查步骤**:
1. 用万用表测量OLED模块的VCC和GND之间电压，应为3.3V
2. 检查SDA和SCL连接是否正确
3. 尝试修改I2C地址为0x3D
4. 查看串口日志，确认初始化步骤是否成功

### 2. 显示乱码或花屏

**可能原因**:
- Gap偏移设置不正确（已在代码中修复）
- I2C通信不稳定

**解决方案**:
1. 代码中已包含关键修复：`esp_lcd_panel_set_gap(panel_handle, 0, 0)`
2. 如果仍有问题，尝试降低I2C频率：
   ```c
   #define I2C_MASTER_FREQ_HZ      100000    /* 降低到100kHz */
   ```
3. 添加外部上拉电阻（4.7kΩ）

### 3. I2C通信失败

**可能原因**:
- 线缆过长或接触不良
- 上拉电阻不足

**解决方案**:
1. 缩短I2C连线长度
2. 检查连接是否牢固
3. 添加外部上拉电阻
4. 降低I2C通信速度

## 硬件测试步骤

### 1. 基础电源测试

```bash
# 上传程序后，通过串口监视器查看初始化日志
pio device monitor
```

期望看到：
```
I (318) main: === ESP32-C3 SSD1306 OLED初始化开始 ===
I (318) main: [1/5] 初始化I2C总线...
I (328) main:   ✓ I2C总线初始化成功 (SDA: GPIO5, SCL: GPIO6)
...
```

### 2. I2C设备扫描（可选）

如果不确定OLED的I2C地址，可以使用I2C扫描工具：

```c
// 临时添加到main.c中测试
#include "driver/i2c_master.h"

void scan_i2c_devices(i2c_master_bus_handle_t bus) {
    for (uint8_t addr = 0x00; addr < 0x7F; addr++) {
        // 尝试与设备通信
        // 如果成功，打印地址
    }
}
```

## 支持的开发板

本项目已在以下开发板上测试：

- ✅ ESP32-C3-DevKitM-1
- ✅ Adafruit QT Py ESP32-C3
- ✅ Seeed Studio XIAO ESP32-C3

其他ESP32-C3开发板理论上也支持，只需根据实际引脚调整配置。

## 参考资料

- [ESP32-C3技术规格书](https://www.espressif.com/sites/default/files/documentation/esp32-c3_datasheet_en.pdf)
- [SSD1306数据手册](https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf)
- [ESP-IDF I2C驱动文档](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/i2c.html)

## 问题反馈

如遇到硬件连接问题，请在项目中提交Issue，并提供：
1. 开发板型号
2. OLED模块型号
3. 连接方式（引脚定义）
4. 串口日志输出