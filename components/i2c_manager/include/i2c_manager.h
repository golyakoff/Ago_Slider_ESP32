#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize I2C bus
 * @param port I2C port number (I2C_NUM_0 or I2C_NUM_1)
 * @param sda_pin GPIO pin for SDA
 * @param scl_pin GPIO pin for SCL
  * @return i2c_bus_handle_t or NULL on failure
 */
esp_err_t i2c_manager_init(i2c_port_t port, gpio_num_t scl_pin, gpio_num_t sda_pin);

/**
 * @brief Get I2C master bus handle
 * @param[out] i2c_bus_handle Pointer to store the I2C master bus handle
 * @return esp_err_t Error code
 */
esp_err_t i2c_manager_get_bus(i2c_master_bus_handle_t *i2c_bus_handle);

/**
 * @brief Create a device on the I2C bus
 * @param addr 7-bit I2C device address
 * @param freq_hz I2C clock frequency in Hz
 * @param[out] device_handle Pointer to store the I2C device handle
 * @return esp_err_t Error code
 */
esp_err_t i2c_manager_add_device(uint16_t addr, uint32_t freq_hz, i2c_master_dev_handle_t *device_handle);

#ifdef __cplusplus
}
#endif

#endif /* I2C_MANAGER_H */
