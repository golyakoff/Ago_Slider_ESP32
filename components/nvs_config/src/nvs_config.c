#include "nvs_config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "NVS_CONFIG";
static app_config_t s_config;
static bool s_initialized = false;

// Schema version of the stored settings. It lives in its own NVS key rather than
// inside app_config_t on purpose: the loader below treats any change of
// sizeof(app_config_t) as a corrupt blob and falls back to defaults, so growing
// the struct would silently wipe every user's configuration on upgrade.
// Devices written before versioning existed have no key at all and read as 0.
#define CONFIG_SCHEMA_KEY     "cfg_ver"
#define CONFIG_SCHEMA_VERSION 1

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
    // Chosen to match what the hardware actually runs at: with 1/16 microstepping these
    // are ~48 mm/s on X, ~53 deg/s on C and ~73 deg/s on B. The previous 1000 dated from
    // the 10^7 speed bug, under which it meant a barely usable 1.2 mm/s on X.
    .axis_speed = {4000 & 0xFF, (4000 >> 8) & 0xFF, // X: 4000 steps/sec
                   4000 & 0xFF, (4000 >> 8) & 0xFF, // C: 4000 steps/sec
                   4000 & 0xFF, (4000 >> 8) & 0xFF},// B: 4000 steps/sec
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

// v0 -> v1: axis speeds were stored while setSpeedInUs() was fed 1e7/speed instead
// of 1e6/speed, so an axis really ran at a tenth of its setting and users tuned the
// numbers ten times too high to compensate. Scale the stored values down so the
// corrected formula reproduces the speed the hardware was actually doing.
static void migrate_v0_to_v1(app_config_t *cfg)
{
    uint16_t x, c, b;
    app_config_unpack_axis_speed(cfg, &x, &c, &b);
    // Never let a rounded-down value reach 0: speed_sps_to_us() reads 0 as "invalid"
    // and substitutes 100 pulses/s, which would speed such an axis up, not slow it.
    uint16_t nx = (x >= 10) ? (x / 10) : 1;
    uint16_t nc = (c >= 10) ? (c / 10) : 1;
    uint16_t nb = (b >= 10) ? (b / 10) : 1;
    app_config_pack_axis_speed(cfg, nx, nc, nb);
    ESP_LOGW(TAG, "Schema v0->v1: axis speeds rescaled X %u->%u, C %u->%u, B %u->%u",
             x, nx, c, nc, b, nb);
}

// Applies every migration between the stored schema version and the current one.
// Returns true when the configuration was changed and needs saving.
static bool migrate_config(uint8_t from_version, app_config_t *cfg)
{
    bool changed = false;
    if (from_version < 1) {
        migrate_v0_to_v1(cfg);
        changed = true;
    }
    return changed;
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

    bool needs_version_stamp = true;   // only a config loaded at the current version doesn't
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
            // Settings saved by an older firmware may need rewriting before use.
            uint8_t stored_version = 0;   // absent key = the original, unversioned schema
            if (nvs_get_u8(nvs, CONFIG_SCHEMA_KEY, &stored_version) != ESP_OK) {
                stored_version = 0;
            }
            if (stored_version < CONFIG_SCHEMA_VERSION &&
                migrate_config(stored_version, &s_config)) {
                nvs_set_blob(nvs, "app_config", &s_config, sizeof(s_config));
            }
            needs_version_stamp = (stored_version != CONFIG_SCHEMA_VERSION);
        }
    }

    // Stamp the version on the paths that changed it: a migrated configuration, and a
    // freshly defaulted one, which is already in the new format and must not be
    // migrated on the next boot. Writing it unconditionally would touch flash on
    // every single boot.
    if (needs_version_stamp) {
        nvs_set_u8(nvs, CONFIG_SCHEMA_KEY, CONFIG_SCHEMA_VERSION);
        nvs_commit(nvs);
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
