#ifndef NVS_CONFIG_H
#define NVS_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Use the structure from app_config
typedef app_config_t nvs_config_t;   // alias

// Default configuration (filled with sensible values)
extern const app_config_t NVS_CONFIG_DEFAULT;

/**
 * @brief Initialise NVS and load configuration.
 *        If no configuration exists, default is written and loaded.
 * @return ESP_OK on success
 */
esp_err_t nvs_config_init(void);

/**
 * @brief Get pointer to current configuration (read-only).
 * @return pointer to app_config_t structure (NULL if not initialized)
 */
const app_config_t* nvs_config_get(void);

/**
 * @brief Reset configuration to factory defaults and save.
 * @return ESP_OK on success
 */
esp_err_t nvs_config_reset(void);

/**
 * @brief Save current configuration (as stored in internal buffer) to flash.
 *        Usually you don't need this – configuration setters call it automatically.
 * @return ESP_OK on success
 */
esp_err_t nvs_config_save(void);

// -----------------------------------------------------------------------------
// Convenience setters (they modify internal config and save to flash)
// -----------------------------------------------------------------------------

/**
 * @brief Set microsteps for all axes and save to flash.
 * @param x  microstep value for X axis (0=256,1=1,2=2,4,8,16,32,64)
 * @param c  microstep value for C axis
 * @param b  microstep value for B axis
 * @return ESP_OK on success
 */
esp_err_t nvs_config_set_microsteps(uint8_t x, uint8_t c, uint8_t b);

/**
 * @brief Set run current (mA) for all axes and save to flash.
 * @param x  run current for X axis (mA)
 * @param c  run current for C axis (mA)
 * @param b  run current for B axis (mA)
 * @return ESP_OK on success
 */
esp_err_t nvs_config_set_run_current(uint16_t x, uint16_t c, uint16_t b);

/**
 * @brief Set hold current (mA) for all axes and save to flash.
 * @param x  hold current for X axis (mA)
 * @param c  hold current for C axis (mA)
 * @param b  hold current for B axis (mA)
 * @return ESP_OK on success
 */
esp_err_t nvs_config_set_hold_current(uint16_t x, uint16_t c, uint16_t b);

/**
 * @brief Set axis units and save to flash.
 * @param x_deg  True if unit for X axis is degrees, False for mm.
 * @param c_deg  True if unit for C axis is degrees, False for mm.
 * @param b_deg  True if unit for B axis is degrees, False for mm.
 * @return ESP_OK on success
 */
esp_err_t nvs_config_set_axis_unit(bool x_deg, bool c_deg, bool b_deg);

/**
 * @brief Set units per step (mm/step or deg/step) and save to flash.
 * @param x  value for X axis
 * @param c  value for C axis
 * @param b  value for B axis
 * @return ESP_OK on success
 */
esp_err_t nvs_config_set_units_per_step(float x, float c, float b);

/**
 * @brief Set axis speeds (steps per second) and save to flash.
 * @param x  speed for X axis
 * @param c  speed for C axis
 * @param b  speed for B axis
 * @return ESP_OK on success
 */
esp_err_t nvs_config_set_axis_speed(uint16_t x, uint16_t c, uint16_t b);

/**
 * @brief Set axis accelerations (steps per second) and save to flash.
 * @param x  speed for X axis
 * @param c  speed for C axis
 * @param b  speed for B axis
 * @return ESP_OK on success
 */
esp_err_t nvs_config_set_axis_accel(uint16_t x, uint16_t c, uint16_t b);

/**
 * @brief Set virtual limit enable flags and save to flash.
 * @param x_en  enable virtual limit for X axis
 * @param c_en  enable for C axis
 * @param b_en  enable for B axis
 * @return ESP_OK on success
 */
esp_err_t nvs_config_set_virtual_limit(bool x_en, bool c_en, bool b_en);

/**
 * @brief Set stealthchop enable flags and save to flash.
 * @param x_en  enable stealthchop for X axis
 * @param c_en  enable for C axis
 * @param b_en  enable for B axis
 * @return ESP_OK on success
 */
esp_err_t nvs_config_set_stealthchop(bool x_en, bool c_en, bool b_en);

/**
 * @brief Set invert direction flags and save to flash.
 * @param x_inv  invert direction for X axis
 * @param c_inv  invert for C axis
 * @param b_inv  invert for B axis
 * @return ESP_OK on success
 */
esp_err_t nvs_config_set_invert_dir(bool x_inv, bool c_inv, bool b_inv);

#ifdef __cplusplus
}
#endif

#endif // NVS_CONFIG_H