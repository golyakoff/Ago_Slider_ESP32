#ifndef INA219_H
#define INA219_H

#include "esp_err.h"
#include "driver/i2c_master.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback type for INA219 measurement data.
 * @param voltage_volts Bus voltage in volts
 * @param current_amps Current in amperes
 * @param power_watts Power in watts
 */
typedef void (*ina219_data_cb_t)(float voltage_volts, float current_amps, float power_watts);

/**
 * @brief Initialize INA219 sensor with given I2C device handle.
 * @param dev_handle I2C device handle from i2c_manager
 * @param data_cb Callback to receive measurement data (may be NULL)
 * @return ESP_OK on success, or error code
 */
esp_err_t ina219_init(i2c_master_dev_handle_t dev_handle, ina219_data_cb_t data_cb);

/**
 * @brief Start periodic measurements (default every 2 seconds).
 * @return ESP_OK on success, or error code
 */
esp_err_t ina219_start(void);

/**
 * @brief Stop periodic measurements.
 * @return ESP_OK on success, or error code
 */
esp_err_t ina219_stop(void);

/**
 * @brief Manually trigger a single measurement (data will be sent via callback).
 * @return ESP_OK on success, or error code
 */
esp_err_t ina219_measure_once(void);

#ifdef __cplusplus
}
#endif

#endif // INA219_H
