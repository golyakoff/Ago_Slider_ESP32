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

static void calib_task(void *arg);

// The X carriage needs well over 30 s to cross the full rail at homing speed
#define HOMING_TIMEOUT_MS   90000

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
 *        If a virtual limit is enabled for an axis, its limit switch is
 *        active and the axis is moving toward the switch (negative
 *        direction), the axis is stopped using forceStop(). Moving away
 *        from the switch stays allowed, otherwise a homed axis could
 *        never leave its endstop. Runs every 50 ms.
 */
static void monitor_limits_task(void *arg) {
    while (1) {
        bool lim[3] = {false, false, false};
        motion_get_limits(&lim[0], &lim[1], &lim[2]);

        for (int i = 0; i < 3; i++) {
            if (!s_virtual_limit_enabled[i] || !lim[i] || !s_steppers[i]) continue;
            if (s_steppers[i]->getCurrentSpeedInMilliHz() >= 0) continue;
            // The calibration state machine drives into the endstops on purpose
            if (motion_is_calibrating()) continue;
            s_steppers[i]->forceStop();
            ESP_LOGI(TAG, "Virtual limit %c triggered, axis stopped", "XCB"[i]);
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

        // stop axes that hit limits; a request is fulfilled (and cleared) the
        // moment its endstop is reached, so `requested` always means "still on
        // its way" — an axis that keeps the bit after homing ends never made it
        for (int i = 0; i < 3; i++) {
            if (s_homing.requested[i] && !s_homing.homed[i] && raw[i]) {
                // Atomic stop + zero: the homed switch is the position anchor
                if (s_steppers[i]) s_steppers[i]->forceStopAndNewPosition(0);
                s_homing.homed[i] = true;
                s_homing.requested[i] = false;
                ESP_LOGI(TAG, "Axis %c homed, position zeroed", "XCB"[i]);
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
    xTaskCreate(calib_task, "calib_monitor", 4096, nullptr, 2, nullptr);
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

// -----------------------------------------------------------------------------
// Hardware calibration state machine. Endstop events (motion_on_limit_pin_event,
// called from the PCA9555 interrupt task) stop the seeks with ~1 ms latency and
// advance the phase; the calibration task issues the next move once the stepper
// is idle and watches the per-phase timeout. Events during retreat/park phases
// are ignored by design — the retreats cross the sensor again on purpose.
// -----------------------------------------------------------------------------
#define CALIB_SEEK_STEPS        500000000  // "drive until the endstop answers"
#define CALIB_SLOW_FACTOR       5          // slow re-seek at 1/5 of the axis speed
#define CALIB_SLOW_SEEK_RETREATS 4         // slow seek bound, in retreat distances
#define CALIB_PHASE_TIMEOUT_MS  60000

static struct {
    volatile motion_calib_phase_t phase;
    uint8_t axis;
    int32_t park_offset;
    int32_t retreat;
    int32_t span;
    bool move_issued;
    uint32_t phase_start_ms;
    uint32_t saved_speed_us;
} s_calib = {
    .phase = MOTION_CALIB_IDLE,
};

static motion_calib_status_cb_t s_calib_cb = NULL;
static portMUX_TYPE s_calib_mux = portMUX_INITIALIZER_UNLOCKED;

void motion_set_calib_status_cb(motion_calib_status_cb_t cb)
{
    s_calib_cb = cb;
}

bool motion_is_calibrating(void)
{
    motion_calib_phase_t ph = s_calib.phase;
    return ph != MOTION_CALIB_IDLE && ph != MOTION_CALIB_DONE && ph != MOTION_CALIB_FAILED;
}

// Sets the phase and notifies; the next move (if any) is issued by calib_task
static void calib_set_phase(motion_calib_phase_t phase)
{
    s_calib.phase = phase;
    s_calib.move_issued = false;
    s_calib.phase_start_ms = esp_log_timestamp();
    ESP_LOGI(TAG, "Calibration axis %c: phase %d", "XCB"[s_calib.axis], (int)phase);
    if (s_calib_cb) s_calib_cb(s_calib.axis, (uint8_t)phase, s_calib.span);
}

static void calib_finish(bool ok)
{
    FastAccelStepper *st = s_steppers[s_calib.axis];
    if (st) {
        if (!ok) st->forceStop();
        st->setSpeedInUs(s_calib.saved_speed_us);
    }
    calib_set_phase(ok ? MOTION_CALIB_DONE : MOTION_CALIB_FAILED);
}

void motion_start_calibration(uint8_t axis, int32_t park_offset_steps, int32_t retreat_steps)
{
    if (axis > 2 || !s_steppers[axis]) {
        ESP_LOGW(TAG, "Calibration: invalid axis %u", axis);
        return;
    }
    if (motion_is_calibrating() || s_homing.active) {
        ESP_LOGW(TAG, "Calibration: busy, ignoring");
        return;
    }
    s_calib.axis = axis;
    s_calib.park_offset = park_offset_steps;
    s_calib.retreat = (retreat_steps > 0) ? retreat_steps : 1000;
    s_calib.span = 0;
    s_calib.saved_speed_us = s_steppers[axis]->getSpeedInUs();
    calib_set_phase(MOTION_CALIB_SEEK_MIN_FAST);
}

void motion_abort_calibration(void)
{
    if (!motion_is_calibrating()) return;
    ESP_LOGW(TAG, "Calibration aborted");
    calib_finish(false);
}

void motion_on_limit_pin_event(uint8_t pin, bool raw)
{
    if (raw) return; // endstops are active-low; only the activation edge matters
    int axis = -1;
    for (int i = 0; i < 3; i++) {
        if (s_limit_pins[i] == pin) axis = i;
    }
    if (axis < 0 || axis != s_calib.axis) return;

    portENTER_CRITICAL(&s_calib_mux);
    motion_calib_phase_t phase = s_calib.phase;
    bool acting = s_calib.move_issued;
    portEXIT_CRITICAL(&s_calib_mux);
    if (!acting) return;

    FastAccelStepper *st = s_steppers[axis];
    switch (phase) {
        case MOTION_CALIB_SEEK_MIN_FAST:
            st->forceStop();
            calib_set_phase(MOTION_CALIB_RETREAT_MIN);
            break;
        case MOTION_CALIB_SEEK_MIN_SLOW:
            // The min trigger is the position anchor
            st->forceStopAndNewPosition(0);
            calib_set_phase(MOTION_CALIB_SEEK_MAX_FAST);
            break;
        case MOTION_CALIB_SEEK_MAX_FAST:
            st->forceStop();
            calib_set_phase(MOTION_CALIB_RETREAT_MAX);
            break;
        case MOTION_CALIB_SEEK_MAX_SLOW:
            st->forceStop();
            s_calib.span = st->getCurrentPosition();
            calib_set_phase(MOTION_CALIB_PARK);
            break;
        default:
            break; // retreats and parking cross the sensor on purpose — ignore
    }
}

static void calib_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (!motion_is_calibrating()) continue;

        FastAccelStepper *st = s_steppers[s_calib.axis];
        if (!st) continue;

        if (esp_log_timestamp() - s_calib.phase_start_ms > CALIB_PHASE_TIMEOUT_MS) {
            ESP_LOGE(TAG, "Calibration timeout in phase %d", (int)s_calib.phase);
            calib_finish(false);
            continue;
        }
        if (st->isRunning()) continue;

        if (!s_calib.move_issued) {
            // Stepper idle and the current phase's move not sent yet — issue it
            switch (s_calib.phase) {
                case MOTION_CALIB_SEEK_MIN_FAST:
                    st->setSpeedInUs(s_calib.saved_speed_us);
                    st->move(-CALIB_SEEK_STEPS);
                    break;
                case MOTION_CALIB_RETREAT_MIN:
                    st->move(s_calib.retreat);
                    break;
                case MOTION_CALIB_SEEK_MIN_SLOW:
                    st->setSpeedInUs(s_calib.saved_speed_us * CALIB_SLOW_FACTOR);
                    st->move(-CALIB_SLOW_SEEK_RETREATS * s_calib.retreat);
                    break;
                case MOTION_CALIB_SEEK_MAX_FAST:
                    st->setSpeedInUs(s_calib.saved_speed_us);
                    st->move(CALIB_SEEK_STEPS);
                    break;
                case MOTION_CALIB_RETREAT_MAX:
                    st->move(-s_calib.retreat);
                    break;
                case MOTION_CALIB_SEEK_MAX_SLOW:
                    st->setSpeedInUs(s_calib.saved_speed_us * CALIB_SLOW_FACTOR);
                    st->move(CALIB_SLOW_SEEK_RETREATS * s_calib.retreat);
                    break;
                case MOTION_CALIB_PARK:
                    st->setSpeedInUs(s_calib.saved_speed_us);
                    st->moveTo(s_calib.park_offset);
                    break;
                default:
                    break;
            }
            portENTER_CRITICAL(&s_calib_mux);
            s_calib.move_issued = true;
            portEXIT_CRITICAL(&s_calib_mux);
            continue;
        }

        // Stepper idle with the move already issued: the move ran to completion
        switch (s_calib.phase) {
            case MOTION_CALIB_RETREAT_MIN:
                calib_set_phase(MOTION_CALIB_SEEK_MIN_SLOW);
                break;
            case MOTION_CALIB_RETREAT_MAX:
                calib_set_phase(MOTION_CALIB_SEEK_MAX_SLOW);
                break;
            case MOTION_CALIB_PARK:
                ESP_LOGI(TAG, "Calibration axis %c done: span=%ld steps",
                         "XCB"[s_calib.axis], (long)s_calib.span);
                calib_finish(true);
                break;
            case MOTION_CALIB_SEEK_MIN_SLOW:
            case MOTION_CALIB_SEEK_MAX_SLOW:
                // The bounded slow seek ended without the sensor answering
                ESP_LOGE(TAG, "Calibration: endstop silent during slow seek");
                calib_finish(false);
                break;
            default:
                // A fast seek can only end via the endstop event or the timeout
                break;
        }
    }
}

void motion_get_positions(int32_t *x, int32_t *c, int32_t *b)
{
    if (x) *x = s_steppers[0] ? s_steppers[0]->getCurrentPosition() : 0;
    if (c) *c = s_steppers[1] ? s_steppers[1]->getCurrentPosition() : 0;
    if (b) *b = s_steppers[2] ? s_steppers[2]->getCurrentPosition() : 0;
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