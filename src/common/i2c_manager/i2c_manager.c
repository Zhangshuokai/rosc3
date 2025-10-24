/**
 * @file i2c_manager.c
 * @brief I2C 总线管理器实现
 */

#include "i2c_manager.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

/*******************************************************************************
 * 日志标签
 ******************************************************************************/
static const char *TAG = "I2C_MGR";

/*******************************************************************************
 * 内部数据结构
 ******************************************************************************/

/**
 * @brief I2C 管理器上下文
 */
typedef struct {
    i2c_master_bus_handle_t bus_handle;  ///< I2C 总线句柄
    bool initialized;                     ///< 初始化标志
    SemaphoreHandle_t mutex;              ///< 互斥锁
    i2c_device_info_t devices[I2C_MAX_DEVICES]; ///< 已注册设备列表
    size_t device_count;                  ///< 已注册设备数量
} i2c_manager_context_t;

/*******************************************************************************
 * 静态变量
 ******************************************************************************/
static i2c_manager_context_t s_i2c_ctx = {
    .bus_handle = NULL,
    .initialized = false,
    .mutex = NULL,
    .device_count = 0
};

/*******************************************************************************
 * 内部函数声明
 ******************************************************************************/
static esp_err_t validate_config(const i2c_manager_config_t *config);

/*******************************************************************************
 * 公共函数实现
 ******************************************************************************/

esp_err_t i2c_manager_init_default(void)
{
    i2c_manager_config_t default_config = {
        .scl_pin = I2C_MANAGER_SCL_PIN,
        .sda_pin = I2C_MANAGER_SDA_PIN,
        .freq_hz = I2C_MANAGER_FREQ_HZ,
        .enable_pullup = true,
        .glitch_ignore_cnt = 7
    };
    
    return i2c_manager_init(&default_config);
}

esp_err_t i2c_manager_init(const i2c_manager_config_t *config)
{
    // 参数检查
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "配置为空");
    ESP_RETURN_ON_FALSE(!s_i2c_ctx.initialized, ESP_ERR_INVALID_STATE, TAG, "I2C管理器已初始化");
    
    // 验证配置
    ESP_RETURN_ON_ERROR(validate_config(config), TAG, "配置参数无效");
    
    ESP_LOGI(TAG, "初始化 I2C 总线管理器");
    ESP_LOGI(TAG, "  SCL: GPIO%d, SDA: GPIO%d", config->scl_pin, config->sda_pin);
    ESP_LOGI(TAG, "  频率: %lu Hz, 上拉: %s", 
             config->freq_hz, config->enable_pullup ? "启用" : "禁用");
    
    // 创建互斥锁
    s_i2c_ctx.mutex = xSemaphoreCreateMutex();
    if (s_i2c_ctx.mutex == NULL) {
        ESP_LOGE(TAG, "创建互斥锁失败");
        return ESP_ERR_NO_MEM;
    }
    
    // 配置 I2C 总线
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = config->glitch_ignore_cnt,
        .i2c_port = I2C_MANAGER_PORT,
        .sda_io_num = config->sda_pin,
        .scl_io_num = config->scl_pin,
        .flags.enable_internal_pullup = config->enable_pullup,
    };
    
    // 创建 I2C 总线
    esp_err_t ret = i2c_new_master_bus(&bus_config, &s_i2c_ctx.bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "创建 I2C 总线失败: %s", esp_err_to_name(ret));
        vSemaphoreDelete(s_i2c_ctx.mutex);
        s_i2c_ctx.mutex = NULL;
        return ret;
    }
    
    // 初始化设备列表
    memset(s_i2c_ctx.devices, 0, sizeof(s_i2c_ctx.devices));
    s_i2c_ctx.device_count = 0;
    
    s_i2c_ctx.initialized = true;
    ESP_LOGI(TAG, "I2C 总线管理器初始化成功");
    
    return ESP_OK;
}

i2c_master_bus_handle_t i2c_manager_get_bus_handle(void)
{
    if (!s_i2c_ctx.initialized) {
        ESP_LOGW(TAG, "I2C 管理器未初始化");
        return NULL;
    }
    
    return s_i2c_ctx.bus_handle;
}

esp_err_t i2c_manager_register_device(uint8_t addr, const char *name)
{
    ESP_RETURN_ON_FALSE(s_i2c_ctx.initialized, ESP_ERR_INVALID_STATE, TAG, "I2C管理器未初始化");
    ESP_RETURN_ON_FALSE(addr < 0x80, ESP_ERR_INVALID_ARG, TAG, "设备地址无效: 0x%02X", addr);
    ESP_RETURN_ON_FALSE(name != NULL, ESP_ERR_INVALID_ARG, TAG, "设备名称为空");
    
    // 加锁
    if (xSemaphoreTake(s_i2c_ctx.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "获取互斥锁失败");
        return ESP_ERR_TIMEOUT;
    }
    
    // 检查设备是否已注册
    for (size_t i = 0; i < s_i2c_ctx.device_count; i++) {
        if (s_i2c_ctx.devices[i].addr == addr) {
            xSemaphoreGive(s_i2c_ctx.mutex);
            ESP_LOGW(TAG, "设备 0x%02X (%s) 已注册", addr, name);
            return ESP_OK;
        }
    }
    
    // 检查设备槽是否已满
    if (s_i2c_ctx.device_count >= I2C_MAX_DEVICES) {
        xSemaphoreGive(s_i2c_ctx.mutex);
        ESP_LOGE(TAG, "设备槽已满，无法注册更多设备");
        return ESP_ERR_NO_MEM;
    }
    
    // 注册设备
    s_i2c_ctx.devices[s_i2c_ctx.device_count].addr = addr;
    s_i2c_ctx.devices[s_i2c_ctx.device_count].name = name;
    s_i2c_ctx.devices[s_i2c_ctx.device_count].registered = true;
    s_i2c_ctx.device_count++;
    
    xSemaphoreGive(s_i2c_ctx.mutex);
    
    ESP_LOGI(TAG, "注册 I2C 设备: 0x%02X (%s)", addr, name);
    
    return ESP_OK;
}

bool i2c_manager_is_device_registered(uint8_t addr)
{
    if (!s_i2c_ctx.initialized) {
        return false;
    }
    
    if (xSemaphoreTake(s_i2c_ctx.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }
    
    bool found = false;
    for (size_t i = 0; i < s_i2c_ctx.device_count; i++) {
        if (s_i2c_ctx.devices[i].addr == addr && s_i2c_ctx.devices[i].registered) {
            found = true;
            break;
        }
    }
    
    xSemaphoreGive(s_i2c_ctx.mutex);
    
    return found;
}

size_t i2c_manager_get_devices(i2c_device_info_t *devices, size_t max_count)
{
    if (!s_i2c_ctx.initialized || devices == NULL || max_count == 0) {
        return 0;
    }
    
    if (xSemaphoreTake(s_i2c_ctx.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return 0;
    }
    
    size_t count = s_i2c_ctx.device_count < max_count ? s_i2c_ctx.device_count : max_count;
    memcpy(devices, s_i2c_ctx.devices, count * sizeof(i2c_device_info_t));
    
    xSemaphoreGive(s_i2c_ctx.mutex);
    
    return count;
}

esp_err_t i2c_manager_deinit(void)
{
    if (!s_i2c_ctx.initialized) {
        ESP_LOGW(TAG, "I2C 管理器未初始化，无需反初始化");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "反初始化 I2C 总线管理器");
    
    // 删除 I2C 总线
    if (s_i2c_ctx.bus_handle != NULL) {
        esp_err_t ret = i2c_del_master_bus(s_i2c_ctx.bus_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "删除 I2C 总线失败: %s", esp_err_to_name(ret));
        }
        s_i2c_ctx.bus_handle = NULL;
    }
    
    // 删除互斥锁
    if (s_i2c_ctx.mutex != NULL) {
        vSemaphoreDelete(s_i2c_ctx.mutex);
        s_i2c_ctx.mutex = NULL;
    }
    
    // 清空设备列表
    memset(s_i2c_ctx.devices, 0, sizeof(s_i2c_ctx.devices));
    s_i2c_ctx.device_count = 0;
    
    s_i2c_ctx.initialized = false;
    
    ESP_LOGI(TAG, "I2C 总线管理器已反初始化");
    
    return ESP_OK;
}

/*******************************************************************************
 * 内部函数实现
 ******************************************************************************/

static esp_err_t validate_config(const i2c_manager_config_t *config)
{
    // 验证引脚
    if (config->scl_pin >= GPIO_NUM_MAX || config->sda_pin >= GPIO_NUM_MAX) {
        ESP_LOGE(TAG, "GPIO 引脚号无效");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 验证频率（10kHz ~ 1MHz）
    if (config->freq_hz < 10000 || config->freq_hz > 1000000) {
        ESP_LOGE(TAG, "I2C 频率无效: %lu Hz (范围: 10kHz ~ 1MHz)", config->freq_hz);
        return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}