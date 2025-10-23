/**
 * @file oled_display.h
 * @brief OLED显示模块接口定义
 * @details 提供SSD1306 OLED显示的初始化和控制接口
 * 
 * 硬件配置:
 * - 显示器: SSD1306 128x64 单色OLED
 * - 接口: I2C (地址 0x3C)
 * - 颜色深度: 1bpp (单色)
 * 
 * @version 1.0
 * @date 2025-10-23
 */

#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "esp_err.h"
#include "lvgl.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*====================
   配置参数
 *====================*/

/** @brief OLED显示宽度（像素） */
#define OLED_WIDTH      128

/** @brief OLED显示高度（像素） */
#define OLED_HEIGHT     64

/** @brief OLED I2C地址 */
#define OLED_I2C_ADDR   0x3C

/** @brief OLED颜色深度（1bpp单色） */
#define OLED_COLOR_DEPTH LV_COLOR_DEPTH_1

/** @brief LVGL时钟周期（毫秒） */
#define LVGL_TICK_PERIOD_MS  10

/** @brief LVGL显示缓冲区大小（字节）
 *  单缓冲模式：128 * 64 / 8 = 1024字节
 */
#define LVGL_BUFFER_SIZE     (OLED_WIDTH * OLED_HEIGHT / 8)

/** @brief I2C时钟频率（Hz） */
#define OLED_I2C_FREQ_HZ    400000

/*====================
   GPIO引脚定义
 *====================*/

#ifndef OLED_I2C_SDA_PIN
    /** @brief I2C SDA引脚（默认GPIO21） */
    #define OLED_I2C_SDA_PIN    GPIO_NUM_21
#endif

#ifndef OLED_I2C_SCL_PIN
    /** @brief I2C SCL引脚（默认GPIO22） */
    #define OLED_I2C_SCL_PIN    GPIO_NUM_22
#endif

/*====================
   函数声明
 *====================*/

/**
 * @brief 初始化OLED显示模块
 * @details 执行以下操作：
 *          1. 初始化I2C总线
 *          2. 初始化SSD1306驱动
 *          3. 初始化LVGL图形库
 *          4. 创建显示缓冲区和显示驱动
 *          5. 启动LVGL定时器任务
 * 
 * @return 
 *     - ESP_OK: 初始化成功
 *     - ESP_ERR_NO_MEM: 内存不足
 *     - ESP_FAIL: 初始化失败
 * 
 * @note 该函数应在系统初始化阶段调用一次
 * @note 初始化失败时会自动释放已分配的资源
 */
esp_err_t oled_display_init(void);

/**
 * @brief 清空OLED显示内容
 * @details 将显示屏清空为黑色（关闭所有像素）
 * 
 * @return 
 *     - ESP_OK: 清空成功
 *     - ESP_FAIL: 显示未初始化或清空失败
 */
esp_err_t oled_display_clear(void);

/**
 * @brief 刷新OLED显示
 * @details 调用LVGL任务处理器，更新显示内容
 *          通常由定时器自动调用，也可手动调用强制刷新
 * 
 * @return 
 *     - ESP_OK: 刷新成功
 *     - ESP_FAIL: 显示未初始化或刷新失败
 * 
 * @note 建议刷新周期 >= 10ms，以保持流畅显示
 */
esp_err_t oled_display_refresh(void);

/**
 * @brief 设置OLED背光状态
 * @details SSD1306通常没有独立背光控制，该函数控制显示开关
 * 
 * @param on true=开启显示, false=关闭显示
 * 
 * @return 
 *     - ESP_OK: 设置成功
 *     - ESP_ERR_NOT_SUPPORTED: 不支持背光控制
 *     - ESP_FAIL: 显示未初始化或设置失败
 */
esp_err_t oled_display_set_backlight(bool on);

/**
 * @brief 获取LVGL显示对象指针
 * @details 用于在应用层创建UI元素
 * 
 * @return LVGL显示对象指针，如果未初始化则返回NULL
 */
lv_disp_t* oled_display_get_disp(void);

#ifdef __cplusplus
}
#endif

#endif /* OLED_DISPLAY_H */