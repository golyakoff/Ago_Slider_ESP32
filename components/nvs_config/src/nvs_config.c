#include "nvs_config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "NVS_CONFIG";
static app_config_t s_config;
static bool s_initialized = false;

// Default values (use app_config pack functions to fill)
const app_config_t NVS_CONFIG_DEFAULT = {
    .microsteps = {16, 16, 16},
    .run_current = {900 & 0xFF, (900 >> 8) & 0xFF,  // X: 900 mA
                    700 & 0xFF, (700 >> 8) & 0xFF,  // C: 700 mA
                    900 & 0xFF, (900 >> 8) & 0xFF}, // B: 900 mA
    .hold_current = {450 & 0xFF, (450 >> 8) & 0xFF, // X: 450 mA
                     350 & 0xFF, (350 >> 8) & 0xFF, // C: 350 mA
                     450 & 0xFF, (450 >> 8) & 0xFF},// B: 450 mA
    .axis_unit = 0x06,                              // X=mm, C=deg, B=deg
    .units_per_step = {0x9A, 0x8F, 0x44, 0x3E,      // X axis: 0.19195 mm/step
                       0x9A, 0x99, 0x59, 0x3E,      // C axis: 0.2125 deg/step
                       0xE7, 0xD7, 0x95, 0x3E},     // B axis: 0.292682927 deg/step
    .axis_speed = {1000 & 0xFF, (1000 >> 8) & 0xFF, // X: 1000 steps/sec
                   1000 & 0xFF, (1000 >> 8) & 0xFF, // C: 1000 steps/sec
                   1000 & 0xFF, (1000 >> 8) & 0xFF},// B: 1000 steps/sec
    .axis_accel = {2000 & 0xFF, (2000 >> 8) & 0xFF, // X: 2000 steps/sec²
                   2000 & 0xFF, (2000 >> 8) & 0xFF, // C: 2000 steps/sec²
                   2000 & 0xFF, (2000 >> 8) & 0xFF},// B: 2000 steps/sec²
    .virtual_limit = 0x05,                          // X and B enabled
    .stealthchop = 0x07,                            // all axes enabled
    .invert_dir = 0x00,                             // not invert all
};
static esp_err_t save_to_nvs(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("config", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(nvs, "app_config", &s_config, sizeof(s_config));
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGE(TAG, "Failed to set blob: %s", esp_err_to_name(err));
    }
    nvs_close(nvs);
    return err;
}

esp_err_t nvs_config_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized.");
        return ESP_OK;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open("config", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    size_t size = sizeof(s_config);
    err = nvs_get_blob(nvs, "app_config", &s_config, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "No saved config, using defaults");
        memcpy(&s_config, &NVS_CONFIG_DEFAULT, sizeof(s_config));
        err = nvs_set_blob(nvs, "app_config", &s_config, sizeof(s_config));
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write default config: %s", esp_err_to_name(err));
        }
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read config: %s", esp_err_to_name(err));
        memcpy(&s_config, &NVS_CONFIG_DEFAULT, sizeof(s_config));
        nvs_set_blob(nvs, "app_config", &s_config, sizeof(s_config));
        nvs_commit(nvs);
    } else {
        if (size != sizeof(s_config)) {
            ESP_LOGW(TAG, "Size mismatch, using defaults");
            memcpy(&s_config, &NVS_CONFIG_DEFAULT, sizeof(s_config));
            nvs_set_blob(nvs, "app_config", &s_config, sizeof(s_config));
            nvs_commit(nvs);
        } else {
            ESP_LOGI(TAG, "Configuration loaded from NVS");
            // optionally unpack and log
        }
    }

    nvs_close(nvs);
    s_initialized = true;
    return ESP_OK;
}

const app_config_t* nvs_config_get(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized, call nvs_config_init() first.");
        return NULL;
    }
    return &s_config;
}

esp_err_t nvs_config_save(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    return save_to_nvs();
}

esp_err_t nvs_config_reset(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    memcpy(&s_config, &NVS_CONFIG_DEFAULT, sizeof(s_config));
    return save_to_nvs();
}

// -----------------------------------------------------------------------------
// Convenience setters (update s_config and save)
// -----------------------------------------------------------------------------
esp_err_t nvs_config_set_microsteps(uint8_t x, uint8_t c, uint8_t b)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    app_config_pack_microsteps(&s_config, x, c, b);
    return save_to_nvs();
}

esp_err_t nvs_config_set_run_current(uint16_t x, uint16_t c, uint16_t b)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    app_config_pack_run_current(&s_config, x, c, b);
    return save_to_nvs();
}

esp_err_t nvs_config_set_hold_current(uint16_t x, uint16_t c, uint16_t b)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    app_config_pack_hold_current(&s_config, x, c, b);
    return save_to_nvs();
}

esp_err_t nvs_config_set_axis_unit(bool x_deg, bool c_deg, bool b_deg)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    app_config_pack_axis_unit(&s_config, x_deg, c_deg, b_deg);
    return save_to_nvs();
}

esp_err_t nvs_config_set_units_per_step(float x, float c, float b)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    app_config_pack_units_per_step(&s_config, x, c, b);
    return save_to_nvs();
}

esp_err_t nvs_config_set_axis_speed(uint16_t x, uint16_t c, uint16_t b)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    app_config_pack_axis_speed(&s_config, x, c, b);
    return save_to_nvs();
}

esp_err_t nvs_config_set_axis_accel(uint16_t x, uint16_t c, uint16_t b)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    app_config_pack_axis_accel(&s_config, x, c, b);
    return save_to_nvs();
}

esp_err_t nvs_config_set_virtual_limit(bool x_en, bool c_en, bool b_en)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    app_config_pack_virtual_limit(&s_config, x_en, c_en, b_en);
    return save_to_nvs();
}

esp_err_t nvs_config_set_stealthchop(bool x_en, bool c_en, bool b_en)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    app_config_pack_stealthchop(&s_config, x_en, c_en, b_en);
    return save_to_nvs();
}

esp_err_t nvs_config_set_invert_dir(bool x_inv, bool c_inv, bool b_inv)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    app_config_pack_invert_dir(&s_config, x_inv, c_inv, b_inv);
    return save_to_nvs();
}
