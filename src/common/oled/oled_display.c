/**
 * @file oled_display.c
 * @brief OLED显示模块实现
 * @details 基于LVGL和ESP-LCD实现SSD1306 OLED显示控制
 * 
 * @version 1.0
 * @date 2025-10-23
 */

#include "oled_display.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

/*====================
   日志标签
 *====================*/
static const char *TAG = "OLED_DISP";

/*====================
   静态变量
 *====================*/

/** @brief LVGL显示对象 */
static lv_disp_t *s_disp = NULL;

/** @brief LVGL显示缓冲区 */
static lv_disp_draw_buf_t s_disp_buf;

/** @brief 显示缓冲区数据 */
static lv_color_t *s_buf1 = NULL;

/** @brief LCD面板句柄 */
static esp_lcd_panel_handle_t s_panel_handle = NULL;

/** @brief LCD IO句柄 */
static esp_lcd_panel_io_handle_t s_io_handle = NULL;

/** @brief LVGL定时器任务句柄 */
static TaskHandle_t s_lvgl_task_handle = NULL;

/** @brief 显示初始化标志 */
static bool s_display_initialized = false;

/*====================
   内部函数声明
 *====================*/

/**
 * @brief LVGL显示刷新回调函数
 * @param disp_drv 显示驱动对象
 * @param area 刷新区域
 * @param color_map 颜色数据
 */
static void lvgl_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_map);

/**
 * @brief LVGL定时器任务
 * @param arg 任务参数（未使用）
 */
static void lvgl_timer_task(void *arg);

/**
 * @brief 初始化I2C总线
 * @return ESP_OK: 成功, ESP_FAIL: 失败
 */
static esp_err_t init_i2c_bus(void);

/**
 * @brief 初始化LCD面板
 * @return ESP_OK: 成功, ESP_FAIL: 失败
 */
static esp_err_t init_lcd_panel(void);

/*====================
   内部函数实现
 *====================*/

static esp_err_t init_i2c_bus(void)
{
    ESP_LOGI(TAG, "初始化I2C总线 (SDA: GPIO%d, SCL: GPIO%d, 频率: %d Hz)",
             OLED_I2C_SDA_PIN, OLED_I2C_SCL_PIN, OLED_I2C_FREQ_HZ);

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = OLED_I2C_SDA_PIN,
        .scl_io_num = OLED_I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = OLED_I2C_FREQ_HZ,
    };

    ESP_RETURN_ON_ERROR(i2c_param_config(I2C_NUM_0, &i2c_conf), TAG, "I2C参数配置失败");
    ESP_RETURN_ON_ERROR(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0), TAG, "I2C驱动安装失败");

    ESP_LOGI(TAG, "I2C总线初始化成功");
    return ESP_OK;
}

static esp_err_t init_lcd_panel(void)
{
    ESP_LOGI(TAG, "初始化LCD面板 (SSD1306 %dx%d @ 0x%02X)", OLED_WIDTH, OLED_HEIGHT, OLED_I2C_ADDR);

    // 创建I2C面板IO
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = OLED_I2C_ADDR,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };

    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_NUM_0, &io_config, &s_io_handle),
        TAG, "创建LCD IO失败"
    );

    // 创建SSD1306面板
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1,
    };

    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_ssd1306(s_io_handle, &panel_config, &s_panel_handle),
        TAG, "创建SSD1306面板失败"
    );

    // 复位并初始化面板
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel_handle), TAG, "LCD面板复位失败");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel_handle), TAG, "LCD面板初始化失败");
    
    // 设置显示方向（不镜像）
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(s_panel_handle, false, false), TAG, "设置显示方向失败");
    
    // 开启显示
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel_handle, true), TAG, "开启显示失败");

    ESP_LOGI(TAG, "LCD面板初始化成功");
    return ESP_OK;
}

static void lvgl_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_map)
{
    if (!s_panel_handle) {
        ESP_LOGE(TAG, "刷新回调：面板句柄无效");
        lv_disp_flush_ready(disp_drv);
        return;
    }

    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    // 将数据发送到LCD
    esp_lcd_panel_draw_bitmap(s_panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);

    // 通知LVGL刷新完成
    lv_disp_flush_ready(disp_drv);
}

static void lvgl_timer_task(void *arg)
{
    ESP_LOGI(TAG, "LVGL定时器任务启动");

    while (1) {
        // 处理LVGL任务
        lv_timer_handler();
        
        // 延迟，控制刷新率
        vTaskDelay(pdMS_TO_TICKS(LVGL_TICK_PERIOD_MS));
    }
}

/*====================
   公共函数实现
 *====================*/

esp_err_t oled_display_init(void)
{
    if (s_display_initialized) {
        ESP_LOGW(TAG, "OLED显示已初始化");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "开始初始化OLED显示模块");

    esp_err_t ret;

    // 1. 初始化I2C总线
    ret = init_i2c_bus();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C总线初始化失败");
        return ret;
    }

    // 2. 初始化LCD面板
    ret = init_lcd_panel();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD面板初始化失败");
        goto err_cleanup_i2c;
    }

    // 3. 初始化LVGL
    ESP_LOGI(TAG, "初始化LVGL (版本: %d.%d.%d)", 
             lv_version_major(), lv_version_minor(), lv_version_patch());
    lv_init();

    // 4. 分配显示缓冲区
    s_buf1 = heap_caps_malloc(LVGL_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    if (!s_buf1) {
        ESP_LOGE(TAG, "分配显示缓冲区失败 (需要 %d 字节)", LVGL_BUFFER_SIZE * sizeof(lv_color_t));
        ret = ESP_ERR_NO_MEM;
        goto err_cleanup_panel;
    }
    ESP_LOGI(TAG, "分配显示缓冲区: %d 字节", LVGL_BUFFER_SIZE * sizeof(lv_color_t));

    // 5. 初始化显示缓冲区（单缓冲模式）
    lv_disp_draw_buf_init(&s_disp_buf, s_buf1, NULL, LVGL_BUFFER_SIZE);

    // 6. 注册显示驱动
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = OLED_WIDTH;
    disp_drv.ver_res = OLED_HEIGHT;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &s_disp_buf;
    disp_drv.rounder_cb = NULL;  // 单色屏幕不需要rounder
    disp_drv.set_px_cb = NULL;   // 使用默认的像素设置函数

    s_disp = lv_disp_drv_register(&disp_drv);
    if (!s_disp) {
        ESP_LOGE(TAG, "注册显示驱动失败");
        ret = ESP_FAIL;
        goto err_cleanup_buffer;
    }
    ESP_LOGI(TAG, "注册LVGL显示驱动成功");

    // 7. 设置单色主题
    lv_theme_t *theme = lv_theme_mono_init(s_disp, false, &lv_font_montserrat_12);
    if (theme) {
        lv_disp_set_theme(s_disp, theme);
        ESP_LOGI(TAG, "设置单色主题成功");
    }

    // 8. 创建LVGL定时器任务
    BaseType_t task_ret = xTaskCreate(
        lvgl_timer_task,
        "lvgl_timer",
        4096,
        NULL,
        5,
        &s_lvgl_task_handle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "创建LVGL定时器任务失败");
        ret = ESP_FAIL;
        goto err_cleanup_display;
    }

    s_display_initialized = true;
    ESP_LOGI(TAG, "OLED显示模块初始化完成");
    ESP_LOGI(TAG, "显示分辨率: %dx%d, 缓冲区: %d 字节, 刷新周期: %d ms",
             OLED_WIDTH, OLED_HEIGHT, LVGL_BUFFER_SIZE, LVGL_TICK_PERIOD_MS);

    return ESP_OK;

err_cleanup_display:
    s_disp = NULL;

err_cleanup_buffer:
    if (s_buf1) {
        heap_caps_free(s_buf1);
        s_buf1 = NULL;
    }

err_cleanup_panel:
    if (s_panel_handle) {
        esp_lcd_panel_del(s_panel_handle);
        s_panel_handle = NULL;
    }
    if (s_io_handle) {
        esp_lcd_panel_io_del(s_io_handle);
        s_io_handle = NULL;
    }

err_cleanup_i2c:
    i2c_driver_delete(I2C_NUM_0);

    ESP_LOGE(TAG, "OLED显示模块初始化失败");
    return ret;
}

esp_err_t oled_display_clear(void)
{
    if (!s_display_initialized || !s_disp) {
        ESP_LOGE(TAG, "显示未初始化");
        return ESP_FAIL;
    }

    // 获取当前活动屏幕
    lv_obj_t *scr = lv_disp_get_scr_act(s_disp);
    if (scr) {
        // 清空屏幕上的所有对象
        lv_obj_clean(scr);
        ESP_LOGI(TAG, "清空显示完成");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "清空显示失败：无法获取活动屏幕");
    return ESP_FAIL;
}

esp_err_t oled_display_refresh(void)
{
    if (!s_display_initialized) {
        ESP_LOGE(TAG, "显示未初始化");
        return ESP_FAIL;
    }

    // 手动调用LVGL任务处理器
    lv_timer_handler();
    return ESP_OK;
}

esp_err_t oled_display_set_backlight(bool on)
{
    if (!s_display_initialized || !s_panel_handle) {
        ESP_LOGE(TAG, "显示未初始化");
        return ESP_FAIL;
    }

    // SSD1306通过显示开关控制"背光"
    esp_err_t ret = esp_lcd_panel_disp_on_off(s_panel_handle, on);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "显示%s", on ? "开启" : "关闭");
    } else {
        ESP_LOGE(TAG, "设置显示状态失败");
    }

    return ret;
}

lv_disp_t* oled_display_get_disp(void)
{
    if (!s_display_initialized) {
        ESP_LOGW(TAG, "显示未初始化，返回NULL");
        return NULL;
    }
    return s_disp;
}