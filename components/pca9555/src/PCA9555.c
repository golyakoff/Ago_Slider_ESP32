#include "pca9555.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

static const char *TAG = "PCA9555";

// Helper: I2C read wrapper
static esp_err_t i2c_read_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, -1);
}

// Helper: I2C write wrapper
static esp_err_t i2c_write_reg_n(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(buf + 1, data, len);
    return i2c_master_transmit(dev_handle, buf, len + 1, -1);
}

// Helper: Write single byte
static esp_err_t i2c_write_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(dev_handle, buf, 2, -1);
}

// Interrupt task
static void pca9555_intr_task(void *arg)
{
    PCA9555 *device = (PCA9555 *)arg;
    uint8_t data[2] = {0, 0};

    while (1) {
        if (xSemaphoreTake(device->intr_trigger, portMAX_DELAY)) {
            esp_err_t res = i2c_read_reg(device->device, PCA9555_REG_INPUT_0, data, 2);
            if (res != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read input state");
                continue;
            }

            uint16_t current_state = data[0] | (data[1] << 8);

            // Check for pin changes
            for (uint8_t pin = 0; pin < 16; pin++) {
                if ((current_state & (1 << pin)) != (device->pin_state & (1 << pin))) {
                    bool value = (current_state & (1 << pin)) != 0;
                    xSemaphoreTake(device->mux, portMAX_DELAY);
                    pca9555_intr_t handler = device->intr_handler[pin];
                    xSemaphoreGive(device->mux);

                    if (handler != NULL) {
                        handler(pin, value);
                    }
                }
            }

            vTaskDelay(10 / portTICK_PERIOD_MS);
            device->pin_state = current_state;
        }
    }
}

// ISR handler for interrupt pin
static void IRAM_ATTR pca9555_intr_handler(void *arg)
{
    PCA9555 *device = (PCA9555 *)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(device->intr_trigger, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Initialize PCA9555
esp_err_t pca9555_init(PCA9555 *device, i2c_master_dev_handle_t i2c_dev, int pin_interrupt)
{
    if (device == NULL || i2c_dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t res;
    device->device = i2c_dev;
    device->pin_interrupt = pin_interrupt;
    device->pin_state = 0;

    // Default register values
    device->reg_direction[0] = 0xFF; // All inputs
    device->reg_direction[1] = 0xFF;
    device->reg_polarity[0]  = 0x00; // Normal polarity
    device->reg_polarity[1]  = 0x00;
    device->reg_output[0]    = 0x00; // Outputs LOW
    device->reg_output[1]    = 0x00;

    // Write configuration registers
    res = i2c_write_reg_n(device->device, PCA9555_REG_CONFIG_0, device->reg_direction, 2);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write direction registers, error code: 0x%02X.", res);
        return res;
    }

    res = i2c_write_reg_n(device->device, PCA9555_REG_POLARITY_0, device->reg_polarity, 2);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write polarity registers");
        return res;
    }

    res = i2c_write_reg_n(device->device, PCA9555_REG_OUTPUT_0, device->reg_output, 2);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write output registers");
        return res;
    }

    // Create mutex
    device->mux = xSemaphoreCreateMutex();
    if (device->mux == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Create interrupt trigger semaphore
    device->intr_trigger = xSemaphoreCreateBinary();
    if (device->intr_trigger == NULL) {
        vSemaphoreDelete(device->mux);
        return ESP_ERR_NO_MEM;
    }

    // Clear interrupt handlers
    for (uint8_t pin = 0; pin < 16; pin++) {
        device->intr_handler[pin] = NULL;
    }

    device->intr_task_handle = NULL;

    // Setup interrupt pin if specified
    if (device->pin_interrupt >= 0) {
        // Configure GPIO as input with pull-up
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << device->pin_interrupt),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE,
        };

        res = gpio_config(&io_conf);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure interrupt pin GPIO%d", device->pin_interrupt);
            vSemaphoreDelete(device->mux);
            vSemaphoreDelete(device->intr_trigger);
            return res;
        }

        // Add ISR handler
        res = gpio_isr_handler_add(device->pin_interrupt, pca9555_intr_handler, (void *)device);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add ISR handler for GPIO%d", device->pin_interrupt);
            vSemaphoreDelete(device->mux);
            vSemaphoreDelete(device->intr_trigger);
            return res;
        }

        // Create interrupt processing task
        xTaskCreate(pca9555_intr_task, "PCA9555_intr", 4096, (void *)device, 10, &device->intr_task_handle);

        // Give initial semaphore to read initial state
        xSemaphoreGive(device->intr_trigger);
    }

    ESP_LOGI(TAG, "PCA9555 successfully initialized.");
    return ESP_OK;
}

// Destroy PCA9555
esp_err_t pca9555_destroy(PCA9555 *device)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (device->pin_interrupt >= 0) {
        // Delete interrupt task
        if (device->intr_task_handle != NULL) {
            vTaskDelete(device->intr_task_handle);
        }

        // Remove ISR handler
        gpio_isr_handler_remove(device->pin_interrupt);

        // Reconfigure GPIO to default state
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << device->pin_interrupt),
            .pull_down_en = 0,
            .pull_up_en = 0,
        };
        gpio_config(&io_conf);
    }

    // Delete semaphores
    vSemaphoreDelete(device->mux);
    vSemaphoreDelete(device->intr_trigger);

    ESP_LOGI(TAG, "PCA9555 destroyed");
    return ESP_OK;
}

// Set GPIO direction
esp_err_t pca9555_set_gpio_direction(PCA9555 *device, uint8_t pin, bool direction)
{
    if (device == NULL || pin > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit = pin % 8;

    if (direction == PCA9555_DIR_OUT) {
        device->reg_direction[port] &= ~(1 << bit);
    } else {
        device->reg_direction[port] |= (1 << bit);
    }

    return i2c_write_reg_n(device->device, PCA9555_REG_CONFIG_0, device->reg_direction, 2);
}

// Get GPIO direction
esp_err_t pca9555_get_gpio_direction(PCA9555 *device, uint8_t pin, bool *direction)
{
    if (device == NULL || direction == NULL || pin > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit = pin % 8;

    *direction = ((device->reg_direction[port] >> bit) & 1) ? PCA9555_DIR_IN : PCA9555_DIR_OUT;
    return ESP_OK;
}

// Set GPIO polarity
esp_err_t pca9555_set_gpio_polarity(PCA9555 *device, uint8_t pin, bool polarity)
{
    if (device == NULL || pin > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit = pin % 8;

    if (polarity == PCA9555_POL_INVERTED) {
        device->reg_polarity[port] |= (1 << bit);
    } else {
        device->reg_polarity[port] &= ~(1 << bit);
    }

    return i2c_write_reg_n(device->device, PCA9555_REG_POLARITY_0, device->reg_polarity, 2);
}

// Get GPIO polarity
esp_err_t pca9555_get_gpio_polarity(PCA9555 *device, uint8_t pin, bool *polarity)
{
    if (device == NULL || polarity == NULL || pin > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit = pin % 8;

    *polarity = ((device->reg_polarity[port] >> bit) & 1) ? PCA9555_POL_INVERTED : PCA9555_POL_NORMAL;
    return ESP_OK;
}

// Set GPIO output value
esp_err_t pca9555_set_gpio_value(PCA9555 *device, uint8_t pin, bool value)
{
    if (device == NULL || pin > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit = pin % 8;

    // Check if pin is configured as output
    bool direction;
    esp_err_t res = pca9555_get_gpio_direction(device, pin, &direction);
    if (res != ESP_OK) {
        return res;
    }

    if (direction != PCA9555_DIR_OUT) {
        ESP_LOGW(TAG, "Pin %d is not configured as output", pin);
        return ESP_ERR_INVALID_STATE;
    }

    if (value) {
        device->reg_output[port] |= (1 << bit);
    } else {
        device->reg_output[port] &= ~(1 << bit);
    }

    // Write only the affected port
    return i2c_write_reg(device->device, port ? PCA9555_REG_OUTPUT_1 : PCA9555_REG_OUTPUT_0,
                         device->reg_output[port]);
}

// Get GPIO value
esp_err_t pca9555_get_gpio_value(PCA9555 *device, uint8_t pin, bool *value)
{
    if (device == NULL || value == NULL || pin > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t port = (pin >= 8) ? 1 : 0;
    uint8_t bit = pin % 8;

    bool direction;
    esp_err_t res = pca9555_get_gpio_direction(device, pin, &direction);
    if (res != ESP_OK) {
        return res;
    }

    uint8_t reg;
    if (direction == PCA9555_DIR_IN) {
        reg = port ? PCA9555_REG_INPUT_1 : PCA9555_REG_INPUT_0;
    } else {
        reg = port ? PCA9555_REG_OUTPUT_1 : PCA9555_REG_OUTPUT_0;
    }

    uint8_t reg_value;
    res = i2c_read_reg(device->device, reg, &reg_value, 1);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X", reg);
        return res;
    }

    *value = (reg_value >> bit) & 1;
    return ESP_OK;
}

// Set interrupt handler for a pin
esp_err_t pca9555_set_interrupt_handler(PCA9555 *device, uint8_t pin, pca9555_intr_t handler)
{
    if (device == NULL || pin > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(device->mux, portMAX_DELAY);
    device->intr_handler[pin] = handler;
    xSemaphoreGive(device->mux);

    return ESP_OK;
}
