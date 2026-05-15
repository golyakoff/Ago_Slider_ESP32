#include "i2c_manager.h"
#include "driver/i2c_master.h"

static i2c_master_bus_handle_t bus_handle = NULL;

esp_err_t i2c_manager_init( 
    i2c_port_t port,
    gpio_num_t scl_pin,
    gpio_num_t sda_pin
) {
    if (bus_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = port,
        .scl_io_num = scl_pin,
        .sda_io_num = sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
        .intr_priority = 0
    };

    return i2c_new_master_bus(&i2c_mst_config, &bus_handle);
}

esp_err_t i2c_manager_get_bus(i2c_master_bus_handle_t *i2c_bus_handle) {
    if (bus_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    *i2c_bus_handle = bus_handle;
    return ESP_OK;
}

esp_err_t i2c_manager_add_device(uint16_t addr, uint32_t freq_hz, i2c_master_dev_handle_t *device_handle) {
    if (bus_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = freq_hz,
    };

    return i2c_master_bus_add_device(bus_handle, &dev_cfg, device_handle);
}
