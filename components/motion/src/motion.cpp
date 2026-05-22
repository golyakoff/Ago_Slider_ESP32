#include "motion.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tmc2130.h"
#include "pca9555.h"               // for pca9555_get_gpio_value
#include "FastAccelStepper.h"      // for FastAccelStepper class definition

static const char *TAG = "MOTION";


static FastAccelStepper* s_steppers[3] = {nullptr, nullptr, nullptr};
static PCA9555* s_pca = nullptr;
static uint8_t s_limit_pins[3] = {0, 0, 0};
static motion_home_status_cb_t s_home_cb = nullptr;

// Homing state (explicit initialization to avoid warnings)
static struct {
    bool active;
    bool requested[3];
    bool homed[3];
    uint32_t start_time_ms;
} s_homing = {
    .active = false,
    .requested = {false, false, false},
    .homed = {false, false, false},
    .start_time_ms = 0
};

// Configuration storage
static bool s_axis_deg[3] = {false, false, false};   // true = degrees, false = mm
static float s_units_per_step[3] = {0.0f, 0.0f, 0.0f};
static bool s_virtual_limit_enabled[3] = {false, false, false};

#define HOMING_TIMEOUT_MS   30000

// -----------------------------------------------------------------------------
// Private methods
// -----------------------------------------------------------------------------

static void stop_all_axes(void) {
    for (int i = 0; i < 3; i++) {
        if (s_steppers[i]) s_steppers[i]->forceStop();
    }
}

/**
 * @brief Background task that continuously monitors limit switches.
 *        If a virtual limit is enabled for an axis and its limit switch
 *        becomes active, the axis is stopped immediately using forceStop().
 *        Runs every 50 ms.
 */
static void monitor_limits_task(void *arg) {
    while (1) {
        // Read current limit switch states
        bool x_lim = false, c_lim = false, b_lim = false;
        motion_get_limits(&x_lim, &c_lim, &b_lim);

        // Stop axis if virtual limit enabled and limit active
        if (s_virtual_limit_enabled[0] && x_lim && s_steppers[0]) {
            s_steppers[0]->forceStop();
            ESP_LOGI(TAG, "Virtual limit X triggered, axis stopped");
        }
        if (s_virtual_limit_enabled[1] && c_lim && s_steppers[1]) {
            s_steppers[1]->forceStop();
            ESP_LOGI(TAG, "Virtual limit C triggered, axis stopped");
        }
        if (s_virtual_limit_enabled[2] && b_lim && s_steppers[2]) {
            s_steppers[2]->forceStop();
            ESP_LOGI(TAG, "Virtual limit B triggered, axis stopped");
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Task that monitors limit switches during homing.
 *        Stops each axis when its limit switch is triggered.
 *        Sends BLE notifications on progress and checks timeout.
 *        Runs every 50 ms.
 */
static void homing_task(void *arg) {
    while (1) {
        if (!s_homing.active) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // read limit switches (active LOW → invert)
        bool raw[3];
        for (int i = 0; i < 3; i++) {
            bool val;
            if (s_pca && pca9555_get_gpio_value(s_pca, s_limit_pins[i], &val) == ESP_OK) {
                raw[i] = !val;   // limit reached = true
            } else {
                raw[i] = false;
            }
        }

        // stop axes that hit limits
        for (int i = 0; i < 3; i++) {
            if (s_homing.requested[i] && !s_homing.homed[i] && raw[i]) {
                if (s_steppers[i]) s_steppers[i]->forceStop();
                s_homing.homed[i] = true;
                ESP_LOGI(TAG, "Axis %c homed", "XCB"[i]);
            }
        }

        // send progress via callback
        if (s_home_cb) {
            s_home_cb(s_homing.requested[0], s_homing.requested[1], s_homing.requested[2],
                      s_homing.homed[0], s_homing.homed[1], s_homing.homed[2]);
        }

        // check if all requested axes are done
        bool all_done = true;
        for (int i = 0; i < 3; i++) {
            if (s_homing.requested[i] && !s_homing.homed[i]) {
                all_done = false;
                break;
            }
        }
        if (all_done) {
            ESP_LOGI(TAG, "Homing completed successfully");
            s_homing.active = false;
            stop_all_axes();
            // final status update
            if (s_home_cb) {
                s_home_cb(s_homing.requested[0], s_homing.requested[1], s_homing.requested[2],
                          s_homing.homed[0], s_homing.homed[1], s_homing.homed[2]);
            }
            continue;
        }

        // timeout
        uint32_t now = esp_log_timestamp();
        if ((now - s_homing.start_time_ms) > HOMING_TIMEOUT_MS) {
            ESP_LOGE(TAG, "Homing timeout! Stopping all axes.");
            stop_all_axes();
            s_homing.active = false;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void motion_init(FastAccelStepper* stepper_x,
                 FastAccelStepper* stepper_c,
                 FastAccelStepper* stepper_b,
                 PCA9555* pca9555_dev,
                 uint8_t pin_limit_x,
                 uint8_t pin_limit_c,
                 uint8_t pin_limit_b,
                 motion_home_status_cb_t home_status_cb)
{
    s_steppers[0] = stepper_x;
    s_steppers[1] = stepper_c;
    s_steppers[2] = stepper_b;
    s_pca = pca9555_dev;
    s_limit_pins[0] = pin_limit_x;
    s_limit_pins[1] = pin_limit_c;
    s_limit_pins[2] = pin_limit_b;
    s_home_cb = home_status_cb;

    xTaskCreate(homing_task, "homing_monitor", 4096, nullptr, 1, nullptr);
    xTaskCreate(monitor_limits_task, "limit_monitor", 2048, nullptr, 1, nullptr);
    ESP_LOGI(TAG, "Motion controller initialised");
}

// -----------------------------------------------------------------------------
// Configuration setters
// -----------------------------------------------------------------------------

void motion_set_axis_unit(bool x_deg, bool c_deg, bool b_deg)
{
    s_axis_deg[0] = x_deg;
    s_axis_deg[1] = c_deg;
    s_axis_deg[2] = b_deg;
    ESP_LOGI(TAG, "Axis unit: X=%s, C=%s, B=%s",
             x_deg ? "deg" : "mm",
             c_deg ? "deg" : "mm",
             b_deg ? "deg" : "mm");
}

void motion_set_units_per_step(float x, float c, float b)
{
    s_units_per_step[0] = x;
    s_units_per_step[1] = c;
    s_units_per_step[2] = b;
    ESP_LOGI(TAG, "Units per step: X=%f, C=%f, B=%f", x, c, b);
}

void motion_set_virtual_limit(bool x_en, bool c_en, bool b_en)
{
    s_virtual_limit_enabled[0] = x_en;
    s_virtual_limit_enabled[1] = c_en;
    s_virtual_limit_enabled[2] = b_en;
    ESP_LOGI(TAG, "Virtual limit: X=%d, C=%d, B=%d", x_en, c_en, b_en);
}

// -----------------------------------------------------------------------------
// Commands
// -----------------------------------------------------------------------------

void motion_start_homing(bool home_x, bool home_c, bool home_b)
{
    if (s_homing.active) {
        ESP_LOGW(TAG, "Homing already in progress, ignoring new command");
        return;
    }

    stop_all_axes();

    s_homing.active = true;
    s_homing.requested[0] = home_x;
    s_homing.requested[1] = home_c;
    s_homing.requested[2] = home_b;
    s_homing.homed[0] = false;
    s_homing.homed[1] = false;
    s_homing.homed[2] = false;
    s_homing.start_time_ms = esp_log_timestamp();

    for (int i = 0; i < 3; i++) {
        if (s_homing.requested[i] && s_steppers[i]) {
            s_steppers[i]->move(-2000000);
        }
    }

    if (s_home_cb) {
        s_home_cb(home_x, home_c, home_b, false, false, false);
    }
    ESP_LOGI(TAG, "Homing started: X=%d C=%d B=%d", home_x, home_c, home_b);
}

void motion_move_relative(int32_t x, int32_t c, int32_t b)
{
    if (s_homing.active) {
        ESP_LOGW(TAG, "Cannot move while homing is active");
        return;
    }

    if (s_steppers[0] && x != 0) s_steppers[0]->move(x);
    if (s_steppers[1] && c != 0) s_steppers[1]->move(c);
    if (s_steppers[2] && b != 0) s_steppers[2]->move(b);
    ESP_LOGI(TAG, "Relative move: X=%ld C=%ld B=%ld", x, c, b);
}

void motion_set_enable(bool enable)
{
    esp_err_t ret = tmc2130_enable(enable);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Motors %s", enable ? "ENABLED" : "DISABLED");
    } else {
        ESP_LOGE(TAG, "Failed to %s motors: %s", enable ? "enable" : "disable", esp_err_to_name(ret));
    }
}

void motion_get_limits(bool *x_limited, bool *c_limited, bool *b_limited)
{
    bool raw[3];
    for (int i = 0; i < 3; i++) {
        bool val;
        if (s_pca && pca9555_get_gpio_value(s_pca, s_limit_pins[i], &val) == ESP_OK) {
            raw[i] = !val;
        } else {
            raw[i] = false;
        }
    }

    if (x_limited) *x_limited = raw[0];
    if (c_limited) *c_limited = raw[1];
    if (b_limited) *b_limited = raw[2];
}

void motion_emergency_stop(void)
{
    stop_all_axes();
    s_homing.active = false;
    ESP_LOGW(TAG, "Emergency stop");
}