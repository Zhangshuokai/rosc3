# OLED显示模块

## 概述

OLED显示模块提供了基于LVGL图形库的SSD1306 OLED显示屏驱动实现，支持128x64单色显示。

## 硬件配置

- **显示器**: SSD1306 128x64 单色OLED
- **接口**: I2C (地址 0x3C)
- **I2C引脚**:
  - SDA: GPIO21 (可配置)
  - SCL: GPIO22 (可配置)
- **I2C频率**: 400kHz

## 文件结构

```
src/common/oled/
├── oled_display.h              # OLED显示接口定义
├── oled_display.c              # OLED显示实现
├── oled_display_example.c      # 使用示例
└── README.md                   # 本文档
```

## API接口

### 初始化函数

```c
esp_err_t oled_display_init(void);
```

初始化OLED显示模块，包括I2C总线、LCD面板、LVGL库和定时器任务。

**返回值**:
- `ESP_OK`: 初始化成功
- `ESP_ERR_NO_MEM`: 内存不足
- `ESP_FAIL`: 初始化失败

### 显示控制函数

```c
esp_err_t oled_display_clear(void);
```

清空显示内容，将屏幕清空为黑色。

```c
esp_err_t oled_display_refresh(void);
```

手动刷新显示（通常由定时器自动调用）。

```c
esp_err_t oled_display_set_backlight(bool on);
```

设置显示开关状态（SSD1306通过显示开关控制）。

```c
lv_disp_t* oled_display_get_disp(void);
```

获取LVGL显示对象指针，用于创建UI元素。

## 使用示例

### 基本使用

```c
#include "oled_display.h"
#include "lvgl.h"

void app_main(void)
{
    // 1. 初始化OLED显示
    esp_err_t ret = oled_display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OLED初始化失败");
        return;
    }

    // 2. 获取显示对象
    lv_disp_t *disp = oled_display_get_disp();
    if (!disp) {
        ESP_LOGE(TAG, "无法获取显示对象");
        return;
    }

    // 3. 创建UI元素
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "Hello OLED!");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    // 4. LVGL定时器任务会自动刷新显示
}
```

### 创建滚动文本

```c
lv_obj_t *label = lv_label_create(scr);
lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
lv_label_set_text(label, "这是一段滚动文本...");
lv_obj_set_width(label, OLED_WIDTH);
lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
```

### 显示控制

```c
// 清空显示
oled_display_clear();

// 关闭显示
oled_display_set_backlight(false);

// 等待1秒
vTaskDelay(pdMS_TO_TICKS(1000));

// 开启显示
oled_display_set_backlight(true);
```

## 技术特性

### 内存管理

- **显示缓冲区**: 1024字节 (128 × 64 ÷ 8)
- **缓冲模式**: 单缓冲（节省内存）
- **LVGL内存池**: 16KB (可在lv_conf.h调整)
- **总内存占用**: < 30KB

### 刷新性能

- **刷新周期**: 10ms
- **刷新率**: > 100fps (理论值)
- **实际帧率**: > 10fps (满足流畅显示要求)

### LVGL配置

- **颜色深度**: 1bpp (单色)
- **主题**: 单色主题 (lv_theme_mono)
- **默认字体**: Montserrat 12号
- **支持组件**: Label, Button, Image等基础组件

## 依赖配置

### idf_component.yml

```yaml
dependencies:
  lvgl/lvgl: "~8.3.0"
  esp_lcd_sh1107: "^1"  # 包含SSD1306驱动
  esp_lvgl_port: "^1"
```

### CMakeLists.txt

```cmake
idf_component_register(
    SRCS
        "common/oled/oled_display.c"
    INCLUDE_DIRS
        "common/oled"
    REQUIRES
        driver
        esp_lcd
        lvgl
)
```

## 引脚配置

可以通过Kconfig或代码定义自定义I2C引脚：

```c
#define OLED_I2C_SDA_PIN    GPIO_NUM_21  // 自定义SDA引脚
#define OLED_I2C_SCL_PIN    GPIO_NUM_22  // 自定义SCL引脚
```

## 故障排除

### 显示乱码

- 检查I2C引脚连接是否正确
- 验证I2C地址是否为0x3C
- 确保gap偏移设置为(0, 0)

### 显示无内容

- 检查OLED电源供电是否正常
- 验证I2C总线初始化是否成功
- 确认显示已开启（调用oled_display_set_backlight(true)）

### 内存不足

- 减少LVGL内存池大小（lv_conf.h中的LV_MEM_SIZE）
- 禁用不需要的LVGL组件
- 使用单缓冲模式（已默认启用）

## 相关文档

- [LVGL官方文档](https://docs.lvgl.io/8.3/)
- [ESP-IDF LCD组件](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/peripherals/lcd.html)
- [SSD1306规格书](https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf)

## 版本历史

- **v1.0** (2025-10-23): 初始版本
  - 实现基本的SSD1306驱动
  - 支持LVGL 8.3.x
  - 单色显示（1bpp）
  - 单缓冲模式