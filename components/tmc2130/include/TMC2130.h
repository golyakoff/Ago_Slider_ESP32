#ifndef TMC2130_H
#define TMC2130_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "tmc2130_trinamic.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TMC2130_BUS_NUM_DRIVERS
#define TMC2130_BUS_NUM_DRIVERS 3
#endif

#define TMC2130_DRIVER_INVALID 0xFF

typedef struct {
    int cs_pin;                     // Chip Select
    int step_pin;                   // STEP
    int dir_pin;                    // DIR        
    uint8_t microsteps;             // 1,2,4,8,16,32,64,128,0=256   (default 256)
    uint16_t run_current_ma;        // run current, mA              (default = 1000)
    uint16_t hold_current_ma;       // hold current, mA             (default = 600)
    bool stealthchop;               // enable StealthChop?          (default = false)
    bool invert_dir;                // invert shaft rotation?       (default = false)
} tmc2130_driver_config_t;

typedef struct {
    spi_host_device_t spi_host;     // SPI2_HOST (HSPI) / SPI3_HOST (VSPI)
    uint32_t spi_clock_hz;          // 1-10 MHz (suggested 1 MHz)
    int sclk_pin;                   // SPI clock
    int mosi_pin;                   // SPI master out slave in
    int miso_pin;                   // SPI master in slave out    
    int en_pin;                     // shared ENABLE for all drivers, active LOW
    tmc2130_driver_config_t drivers[TMC2130_BUS_NUM_DRIVERS];
} tmc2130_config_t;

/**
 * @brief Init TMC2130 SPI bus and all drivers.
 * @param config TMC2130 SPI bus and driver configuration.
 * @return ESP_OK for success, otherwise error code.
 */
esp_err_t tmc2130_init(const tmc2130_config_t *config);

/**
 * @brief Enable/disable all motor drivers via shared EN pin.
 * @param enable true - enable, false - disable.
 * @return ESP_OK if success, ESP_ERR_INVALID_STATE if bus isn't initialized yet.
 */
esp_err_t tmc2130_enable(bool enable);

/**
 * @brief Move a specific driver by a number of steps (relative movement).
 * @param driver_id  Driver index (0..TMC2130_BUS_NUM_DRIVERS-1)
 * @param steps number of steps (positive = forward, negative = backward)
 * @param delay_us delay between steps in microseconds (for speed control) – deprecated? Use separate speed setting.
 *                 Actually this is a simple blocking move, not recommended for real applications.
 *                 Better to use external stepper controller. This function is kept for compatibility.
 */
void tmc2130_move_steps(uint8_t driver_id, int32_t steps, uint32_t delay_us);

/**
 * @brief Read driver status register (DRV_STATUS) for diagnostics.
 * @param driver_id  Driver index (0..TMC2130_BUS_NUM_DRIVERS-1)
 * @param value pointer to store 32‑bit status
 * @return ESP_OK on success
 */
esp_err_t tmc2130_read_status(uint8_t driver_id, uint32_t *value);

/**
 * @brief Check if driver indicates an error (over temperature, short circuit, etc.)
 * @param driver_id  Driver index (0..TMC2130_BUS_NUM_DRIVERS-1)
 * @return true if error present, false otherwise
 */
bool tmc2130_is_error(uint8_t driver_id);

/**
 * @brief Log the driver status register (DRV_STATUS) for diagnostics.
 *
 * This function prints a formatted log message containing:
 *   - Raw status value (hex)
 *   - StallGuard result (SG) value
 *   - Actual current scale (CS) value
 *   - Human‑readable flags (if any)
 *
 * The function silently returns if the status is zero (no active flags).
 *
 * @param driver_id  Driver index (0..TMC2130_BUS_NUM_DRIVERS-1)
 * @param status     32‑bit DRV_STATUS register value (from tmc2130_read_status())
 *
 * @note The following flags are recognised:
 *       SG  – StallGuard threshold reached
 *       OT! – Over temperature shutdown
 *       OTPW – Over temperature warning
 *       S2GA – Short to GND on phase A
 *       S2GB – Short to GND on phase B
 *       OLA – Open load on phase A
 *       OLB – Open load on phase B
 *       STST – Standstill (motor not moving)
 */
void tmc2130_drv_status(uint8_t driver_id, uint32_t status);

// =============================================================================
// Runtime configuration setters (update driver settings on the fly)
// =============================================================================

/**
 * @brief Set microsteps for a specific driver.
 * @param driver_id  driver index (0 .. TMC2130_BUS_NUM_DRIVERS-1)
 * @param microsteps microstep value: 0=256, 1=1, 2=2, 4, 8, 16, 32, 64, 128
 * @return ESP_OK on success
 */
esp_err_t tmc2130_set_microsteps(uint8_t driver_id, uint8_t microsteps);

/**
 * @brief Set run current (RMS) for a specific driver.
 * @param driver_id driver index
 * @param ma current in milliamperes (max depends on sense resistors, typical up to 2000)
 * @return ESP_OK on success
 */
esp_err_t tmc2130_set_run_current(uint8_t driver_id, uint16_t ma);

/**
 * @brief Set hold current (RMS) for a specific driver.
 * @param driver_id driver index
 * @param ma current in milliamperes
 * @return ESP_OK on success
 */
esp_err_t tmc2130_set_hold_current(uint8_t driver_id, uint16_t ma);

/**
 * @brief Enable or disable StealthChop mode for a specific driver.
 * @param driver_id driver index
 * @param enable true to enable StealthChop, false to use SpreadCycle
 * @return ESP_OK on success
 */
esp_err_t tmc2130_set_stealthchop(uint8_t driver_id, bool enable);

/**
 * @brief Invert direction of rotation for a specific driver.
 * @param driver_id driver index
 * @param invert true to invert direction, false for normal
 * @return ESP_OK on success
 */
esp_err_t tmc2130_set_invert_dir(uint8_t driver_id, bool invert);

#ifdef __cplusplus
}
#endif

#endif /* TMC2130_H */