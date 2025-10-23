/**
 * @file main.c
 * @brief ESP32-C3 SSD1306 OLED主程序
 * 
 * 功能：
 * - 初始化I2C总线
 * - 配置SSD1306 OLED面板
 * - 初始化LVGL图形库
 * - 显示UI内容
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "lvgl_demo_ui.h"

static const char *TAG = "main";

/*******************************************************************************
 * 硬件配置参数
 ******************************************************************************/

/* I2C配置 */
#define I2C_BUS_PORT            0                   /* I2C端口号 */
#define I2C_MASTER_SCL_IO       GPIO_NUM_6          /* SCL引脚（根据实际硬件修改） */
#define I2C_MASTER_SDA_IO       GPIO_NUM_5          /* SDA引脚（根据实际硬件修改） */
#define I2C_MASTER_FREQ_HZ      400000              /* I2C时钟频率：400kHz */

/* OLED配置 */
#define OLED_I2C_ADDRESS        0x3C                /* SSD1306 I2C地址 */
#define OLED_PIXEL_CLOCK_HZ     (400 * 1000)        /* 像素时钟：400kHz */
#define OLED_CMD_BITS           8                   /* 命令位宽 */
#define OLED_PARAM_BITS         8                   /* 参数位宽 */

/* 显示参数 */
#define LCD_H_RES               128                 /* 水平分辨率 */
#define LCD_V_RES               64                  /* 垂直分辨率 */
#define LCD_BITS_PER_PIXEL      1                   /* 位深度：1位单色 */

/*******************************************************************************
 * 主程序入口
 ******************************************************************************/

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-C3 SSD1306 OLED初始化开始 ===");
    
    /* -------------------------------------------------------------------------
     * 步骤1: 初始化I2C主机总线
     * ------------------------------------------------------------------------- */
    ESP_LOGI(TAG, "[1/5] 初始化I2C总线...");
    
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,          /* 使用默认时钟源 */
        .glitch_ignore_cnt = 7,                     /* 毛刺过滤计数 */
        .i2c_port = I2C_BUS_PORT,                   /* I2C端口 */
        .sda_io_num = I2C_MASTER_SDA_IO,            /* SDA引脚 */
        .scl_io_num = I2C_MASTER_SCL_IO,            /* SCL引脚 */
        .flags.enable_internal_pullup = true,       /* 启用内部上拉 */
    };
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));
    ESP_LOGI(TAG, "  ✓ I2C总线初始化成功 (SDA: GPIO%d, SCL: GPIO%d)", 
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    
    /* -------------------------------------------------------------------------
     * 步骤2: 安装LCD面板IO接口（I2C）
     * ------------------------------------------------------------------------- */
    ESP_LOGI(TAG, "[2/5] 安装LCD面板IO接口...");
    
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = OLED_I2C_ADDRESS,               /* I2C设备地址 */
        .scl_speed_hz = OLED_PIXEL_CLOCK_HZ,        /* SCL速度 */
        .control_phase_bytes = 1,                   /* 控制阶段字节数（SSD1306需要） */
        .lcd_cmd_bits = OLED_CMD_BITS,              /* 命令位宽 */
        .lcd_param_bits = OLED_PARAM_BITS,          /* 参数位宽 */
        .dc_bit_offset = 6,                         /* D/C位偏移（SSD1306协议） */
    };
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));
    ESP_LOGI(TAG, "  ✓ LCD IO接口安装成功 (地址: 0x%02X)", OLED_I2C_ADDRESS);
    
    /* -------------------------------------------------------------------------
     * 步骤3: 安装SSD1306面板驱动
     * ------------------------------------------------------------------------- */
    ESP_LOGI(TAG, "[3/5] 安装SSD1306面板驱动...");
    
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = LCD_BITS_PER_PIXEL,       /* 位深度 */
        .reset_gpio_num = -1,                       /* 不使用硬件复位 */
    };
    
    /* SSD1306特定配置 */
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = LCD_V_RES,                        /* 屏幕高度 */
    };
    panel_config.vendor_config = &ssd1306_config;
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
    ESP_LOGI(TAG, "  ✓ SSD1306驱动安装成功");
    
    /* 复位、初始化面板 */
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    
    /* 【关键修复】设置Gap偏移为(0,0)，避免显示乱码 */
    ESP_LOGI(TAG, "  ⚠ 应用显示修复：Gap偏移 = (0, 0)");
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 0));
    
    /* 打开显示 */
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_LOGI(TAG, "  ✓ 面板已激活 (分辨率: %dx%d)", LCD_H_RES, LCD_V_RES);
    
    /* -------------------------------------------------------------------------
     * 步骤4: 初始化LVGL库
     * ------------------------------------------------------------------------- */
    ESP_LOGI(TAG, "[4/5] 初始化LVGL图形库...");
    
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);
    
    /* 配置LVGL显示设备 */
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = LCD_H_RES * LCD_V_RES,       /* 完整帧缓冲 */
        .double_buffer = true,                      /* 双缓冲（减少闪烁） */
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = true,                         /* 单色模式 */
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);
    
    if (disp == NULL) {
        ESP_LOGE(TAG, "  ✗ LVGL显示添加失败！");
        return;
    }
    
    ESP_LOGI(TAG, "  ✓ LVGL初始化成功");
    ESP_LOGI(TAG, "    - 缓冲区大小: %d像素 (%d字节)", 
             LCD_H_RES * LCD_V_RES, 
             (LCD_H_RES * LCD_V_RES) / 8);
    ESP_LOGI(TAG, "    - 颜色模式: 单色 (1bpp)");
    ESP_LOGI(TAG, "    - 双缓冲: 已启用");
    
    /* 设置屏幕旋转（可选） */
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);
    
    /* -------------------------------------------------------------------------
     * 步骤5: 创建并显示UI
     * ------------------------------------------------------------------------- */
    ESP_LOGI(TAG, "[5/5] 创建UI界面...");
    
    /* 加锁访问LVGL（线程安全） */
    if (lvgl_port_lock(0)) {
        lvgl_demo_ui(disp);
        lvgl_port_unlock();
        ESP_LOGI(TAG, "  ✓ UI创建成功");
    } else {
        ESP_LOGE(TAG, "  ✗ 无法获取LVGL锁");
    }
    
    ESP_LOGI(TAG, "=== 初始化完成，系统运行中 ===");
}