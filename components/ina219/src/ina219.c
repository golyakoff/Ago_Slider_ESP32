#include "ina219.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

static const char *TAG = "INA219";

// INA219 registers
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNT_VOLTAGE 0x01
#define INA219_REG_BUS_VOLTAGE   0x02
#define INA219_REG_POWER         0x03
#define INA219_REG_CURRENT       0x04
#define INA219_REG_CALIBRATION   0x05

// Calibration constants for expected max 32V, 2A, 0.1 Ohm shunt
#define INA219_CURRENT_LSB       0.0001f   // 100 uA per bit
#define INA219_CAL_VALUE         2048      // Calculated for 100uA LSB, 0.2 Ohm
//#define INA219_CAL_VALUE         4096      // Calculated for 100uA LSB, 0.1 Ohm

// Default measurement interval (ms)
#define MEASURE_INTERVAL_MS      2000

static i2c_master_dev_handle_t s_dev_handle = NULL;
static ina219_data_cb_t s_data_cb = NULL;
static bool s_is_running = false;

static float last_voltage = -1.0f;
static float last_current = -1.0f;
static float last_power = -1.0f;

#define VOLTAGE_THRESHOLD 0.1f   // 0.01 V
#define CURRENT_THRESHOLD 0.02f   // 0.01 A
#define POWER_THRESHOLD   0.2f   // 0.01 W

static esp_err_t write_reg(uint8_t reg, uint16_t value)
{
    uint8_t buffer[3] = {reg, (value >> 8) & 0xFF, value & 0xFF};
    return i2c_master_transmit(s_dev_handle, buffer, sizeof(buffer), -1);
}

static esp_err_t read_reg(uint8_t reg, uint16_t *value)
{
    uint8_t buffer[2];
    esp_err_t err = i2c_master_transmit_receive(s_dev_handle, &reg, 1, buffer, 2, -1);
    if (err == ESP_OK) {
        *value = (buffer[0] << 8) | buffer[1];
    }
    return err;
}

static void read_and_send(void)
{
    uint16_t bus_raw, current_raw;

    if (read_reg(INA219_REG_BUS_VOLTAGE, &bus_raw) != ESP_OK) return;
    if (read_reg(INA219_REG_CURRENT, &current_raw) != ESP_OK) return;

    // Convert raw values (bus: 4mV/bit, current: depends on calibration)
    float voltage = (bus_raw >> 3) * 0.004f;      // 4mV per bit
    float current = (int16_t)current_raw * INA219_CURRENT_LSB;
    float power = voltage * current;

    bool changed = false;
    if (fabsf(voltage - last_voltage) >= VOLTAGE_THRESHOLD) changed = true;
    else if (fabsf(current - last_current) >= CURRENT_THRESHOLD) changed = true;
    else if (fabsf(power - last_power) >= POWER_THRESHOLD) changed = true;

    if (changed) {
        last_voltage = voltage;
        last_current = current;
        last_power = power;

        if (s_data_cb) {
            s_data_cb(voltage, current, power);
        }

        ESP_LOGI(TAG, "Voltage: %.1f V, Current: %.2f A, Power: %.1f W", voltage, current, power);
    }
}

static void measure_task(void *arg)
{
    while (s_is_running) {
        read_and_send();
        vTaskDelay(pdMS_TO_TICKS(MEASURE_INTERVAL_MS));
    }
    vTaskDelete(NULL);
}

esp_err_t ina219_init(i2c_master_dev_handle_t dev_handle, ina219_data_cb_t data_cb)
{
    if (dev_handle == NULL) return ESP_ERR_INVALID_ARG;

    s_dev_handle = dev_handle;
    s_data_cb = data_cb;

    // 1. Reset and configure the sensor for ±32V, ±1.6A range with 0.2 Ohm shunt
    write_reg(INA219_REG_CONFIG, 0x399F);    // reset
    vTaskDelay(pdMS_TO_TICKS(100));

    // 2. Set up calibration register
    write_reg(INA219_REG_CALIBRATION, INA219_CAL_VALUE);

    ESP_LOGI(TAG, "Sensor initialized on I2C device");
    return ESP_OK;
}

esp_err_t ina219_start(void)
{
    if (s_is_running) return ESP_ERR_INVALID_STATE;

    s_is_running = true;
    xTaskCreate(measure_task, "ina219_task", 4096, NULL, 1, NULL);
    ESP_LOGI(TAG, "Periodic measurements started");
    return ESP_OK;
}

esp_err_t ina219_stop(void)
{
    if (!s_is_running) return ESP_ERR_INVALID_STATE;

    s_is_running = false;
    vTaskDelay(pdMS_TO_TICKS(100)); // wait for task to exit
    ESP_LOGI(TAG, "Measurements stopped");
    return ESP_OK;
}

esp_err_t ina219_measure_once(void)
{
    if (s_dev_handle == NULL) return ESP_ERR_INVALID_STATE;

    read_and_send();
    return ESP_OK;
}
