/**
 * @file i2c_manager.h
 * @brief 统一的 I2C 总线管理器
 * @details 管理 ESP32-C3 的 I2C 总线，提供统一的初始化和设备访问接口
 * 
 * 特性：
 * - 支持 ESP-IDF 5.x 新驱动 API
 * - 单例模式管理 I2C 总线
 * - 自动设备注册与查询
 * - 线程安全的总线访问
 * 
 * @version 1.0
 * @date 2025-10-24
 */

#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "hal/gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * 配置参数
 ******************************************************************************/

/** @brief I2C 端口号 */
#define I2C_MANAGER_PORT            I2C_NUM_0

/** @brief I2C SCL 引脚（GPIO6） */
#define I2C_MANAGER_SCL_PIN         GPIO_NUM_6

/** @brief I2C SDA 引脚（GPIO5） */
#define I2C_MANAGER_SDA_PIN         GPIO_NUM_5

/** @brief I2C 时钟频率（400kHz） */
#define I2C_MANAGER_FREQ_HZ         400000

/** @brief 最大设备注册数量 */
#define I2C_MAX_DEVICES             8

/*******************************************************************************
 * 数据结构
 ******************************************************************************/

/**
 * @brief I2C 总线配置
 */
typedef struct {
    gpio_num_t scl_pin;        ///< SCL 引脚
    gpio_num_t sda_pin;        ///< SDA 引脚
    uint32_t freq_hz;          ///< 时钟频率 (Hz)
    bool enable_pullup;        ///< 是否启用内部上拉
    uint8_t glitch_ignore_cnt; ///< 毛刺过滤计数
} i2c_manager_config_t;

/**
 * @brief I2C 设备信息
 */
typedef struct {
    uint8_t addr;              ///< I2C 设备地址
    const char *name;          ///< 设备名称
    bool registered;           ///< 是否已注册
} i2c_device_info_t;

/*******************************************************************************
 * 公共函数
 ******************************************************************************/

/**
 * @brief 初始化 I2C 管理器（使用默认配置）
 * 
 * 默认配置：
 * - SCL: GPIO6
 * - SDA: GPIO5
 * - 频率: 400kHz
 * - 启用内部上拉
 * 
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_STATE: 已初始化
 *   - ESP_FAIL: 初始化失败
 */
esp_err_t i2c_manager_init_default(void);

/**
 * @brief 初始化 I2C 管理器（自定义配置）
 * 
 * @param config I2C 总线配置
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效
 *   - ESP_ERR_INVALID_STATE: 已初始化
 *   - ESP_FAIL: 初始化失败
 */
esp_err_t i2c_manager_init(const i2c_manager_config_t *config);

/**
 * @brief 获取 I2C 总线句柄
 * 
 * @note 调用此函数前必须先初始化 I2C 管理器
 * 
 * @return I2C 总线句柄，如果未初始化则返回 NULL
 */
i2c_master_bus_handle_t i2c_manager_get_bus_handle(void);

/**
 * @brief 注册 I2C 设备
 * 
 * @param addr 设备 I2C 地址
 * @param name 设备名称（用于调试）
 * @return 
 *   - ESP_OK: 成功
 *   - ESP_ERR_INVALID_ARG: 参数无效
 *   - ESP_ERR_NO_MEM: 设备槽已满
 */
esp_err_t i2c_manager_register_device(uint8_t addr, const char *name);

/**
 * @brief 查询设备是否已注册
 * 
 * @param addr 设备 I2C 地址
 * @return true=已注册, false=未注册
 */
bool i2c_manager_is_device_registered(uint8_t addr);

/**
 * @brief 获取已注册的设备列表
 * 
 * @param devices 输出设备信息数组
 * @param max_count 数组最大容量
 * @return 实际设备数量
 */
size_t i2c_manager_get_devices(i2c_device_info_t *devices, size_t max_count);

/**
 * @brief 反初始化 I2C 管理器
 * 
 * @return ESP_OK: 成功
 */
esp_err_t i2c_manager_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* I2C_MANAGER_H */