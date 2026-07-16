#ifndef PCA9555_H
#define PCA9555_H

#include <esp_err.h>
#include <esp_log.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

// Register addresses
#define PCA9555_REG_INPUT_0    0x00
#define PCA9555_REG_INPUT_1    0x01
#define PCA9555_REG_OUTPUT_0   0x02
#define PCA9555_REG_OUTPUT_1   0x03
#define PCA9555_REG_POLARITY_0 0x04
#define PCA9555_REG_POLARITY_1 0x05
#define PCA9555_REG_CONFIG_0   0x06
#define PCA9555_REG_CONFIG_1   0x07

// Pin direction constants
#define PCA9555_DIR_IN  0
#define PCA9555_DIR_OUT 1

// Pin polarity constants
#define PCA9555_POL_NORMAL   0
#define PCA9555_POL_INVERTED 1

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Interrupt handler callback type
 * @param pin Pin number that changed (0-15)
 * @param value Current value of the pin (true = HIGH, false = LOW)
 */
typedef void (*pca9555_intr_t)(uint8_t pin, bool value);

/**
 */
typedef struct PCA9555 {
    i2c_master_dev_handle_t device;             // I2C device handle
    int                     pin_interrupt;      // GPIO pin for interrupt (or -1 if not used)
    uint16_t                pin_state;          // Cached pin states
    uint8_t                 reg_direction[2];   // Direction registers (1 = input, 0 = output)
    uint8_t                 reg_polarity[2];    // Polarity registers (1 = inverted)
    uint8_t                 reg_output[2];      // Output latch registers
    pca9555_intr_t          intr_handler[16];   // Interrupt handlers per pin
    TaskHandle_t            intr_task_handle;   // Task handle for interrupt processing
    SemaphoreHandle_t       intr_trigger;       // Binary semaphore for interrupt trigger
    SemaphoreHandle_t       mux;                // Mutex for thread safety
} PCA9555;

/**
 * @brief Initialize PCA9555 device
 * @param device Pointer to PCA9555 structure to initialize
 * @param i2c_dev I2C device handle from i2c_manager
 * @param pin_interrupt GPIO pin for interrupt (or -1 if not used)
 * @return ESP_OK on success, or error code
 */
esp_err_t pca9555_init(PCA9555 *device, i2c_master_dev_handle_t i2c_dev, int pin_interrupt);

/**
 * @brief Destroy PCA9555 device and free resources
 * @param device Pointer to PCA9555 structure
 * @return ESP_OK on success, or error code
 */
esp_err_t pca9555_destroy(PCA9555* device);

/**
 * @brief Set GPIO pin direction
 * @param device Pointer to PCA9555 structure
 * @param pin Pin number (0-15)
 * @param direction PCA9555_DIR_IN or PCA9555_DIR_OUT
 * @return ESP_OK on success, or error code
 */
esp_err_t pca9555_set_gpio_direction(PCA9555* device, uint8_t pin, bool direction);

/**
 * @brief Get GPIO pin direction
 * @param device Pointer to PCA9555 structure
 * @param pin Pin number (0-15)
 * @param direction Pointer to store direction value
 * @return ESP_OK on success, or error code
 */
esp_err_t pca9555_get_gpio_direction(PCA9555* device, uint8_t pin, bool* direction);

/**
 * @brief Set GPIO pin polarity (input inversion)
 * @param device Pointer to PCA9555 structure
 * @param pin Pin number (0-15)
 * @param polarity PCA9555_POL_NORMAL or PCA9555_POL_INVERTED
 * @return ESP_OK on success, or error code
 */
esp_err_t pca9555_set_gpio_polarity(PCA9555* device, uint8_t pin, bool polarity);

/**
 * @brief Get GPIO pin polarity
 * @param device Pointer to PCA9555 structure
 * @param pin Pin number (0-15)
 * @param polarity Pointer to store polarity value
 * @return ESP_OK on success, or error code
 */
esp_err_t pca9555_get_gpio_polarity(PCA9555* device, uint8_t pin, bool* polarity);

/**
 * @brief Set GPIO output value
 * @param device Pointer to PCA9555 structure
 * @param pin Pin number (0-15)
 * @param value true = HIGH, false = LOW
 * @return ESP_OK on success, or error code
 */
esp_err_t pca9555_set_gpio_value(PCA9555* device, uint8_t pin, bool value);

/**
 * @brief Get GPIO value (input or output)
 * @param device Pointer to PCA9555 structure
 * @param pin Pin number (0-15)
 * @param value Pointer to store the pin value
 * @return ESP_OK on success, or error code
 */
esp_err_t pca9555_get_gpio_value(PCA9555* device, uint8_t pin, bool* value);

/**
 * @brief Set interrupt handler for a specific pin
 * @param device Pointer to PCA9555 structure
 * @param pin Pin number (0-15)
 * @param handler Callback function called when pin changes (or NULL to disable)
 * @return ESP_OK on success, or error code
 */
esp_err_t pca9555_set_interrupt_handler(PCA9555 *device, uint8_t pin, pca9555_intr_t handler);

#ifdef __cplusplus
}
#endif

#endif /* PCA9555_H */