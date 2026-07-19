#include "motion_scenario.h"
#include "motion.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SCENARIO";

// Every movement pattern the device knows. Adding one is a line here plus a file
// implementing motion_scenario_def_t.
static const motion_scenario_def_t *scenario_registry[] = {
    NULL,   // filled in motion_scenario_init(), since the defs come from other units
};
#define SCENARIO_COUNT (sizeof(scenario_registry) / sizeof(scenario_registry[0]))

static struct {
    volatile motion_scenario_state_t state;
    volatile motion_scenario_reason_t reason;
    const motion_scenario_def_t *active;
    uint8_t last_id;
    uint32_t start_ms;
    uint32_t elapsed_ms;
    uint32_t total_ms;
} s_run = {
    .state = MOTION_SCENARIO_IDLE,
    .reason = MOTION_SCENARIO_REASON_NONE,
    .active = NULL,
    .last_id = MOTION_SCENARIO_ID_NONE,
};

static motion_scenario_status_cb_t s_status_cb = NULL;

void motion_scenario_set_status_cb(motion_scenario_status_cb_t cb)
{
    s_status_cb = cb;
}

bool motion_scenario_is_running(void)
{
    return s_run.state == MOTION_SCENARIO_RUNNING;
}

void motion_scenario_get_status(uint8_t *scenario_id, uint8_t *state, uint8_t *reason,
                                uint32_t *elapsed_ms, uint32_t *total_ms)
{
    if (scenario_id) *scenario_id = s_run.last_id;
    if (state) *state = (uint8_t)s_run.state;
    if (reason) *reason = (uint8_t)s_run.reason;
    if (elapsed_ms) *elapsed_ms = s_run.elapsed_ms;
    if (total_ms) *total_ms = s_run.total_ms;
}

static void notify(void)
{
    if (s_status_cb) {
        s_status_cb(s_run.last_id, (uint8_t)s_run.state, (uint8_t)s_run.reason,
                    s_run.elapsed_ms, s_run.total_ms);
    }
}

/** Ends a run whatever the cause; the scenario's own stop() is called exactly once. */
static void finish(motion_scenario_state_t state, motion_scenario_reason_t reason)
{
    if (s_run.state != MOTION_SCENARIO_RUNNING) return;
    if (s_run.active && s_run.active->stop) s_run.active->stop();
    motion_stop_all_keep_position();
    s_run.state = state;
    s_run.reason = reason;
    s_run.active = NULL;
    ESP_LOGI(TAG, "Run finished: state=%d reason=%d", (int)state, (int)reason);
    notify();
}

void motion_scenario_stop(void)
{
    finish(MOTION_SCENARIO_ABORTED, MOTION_SCENARIO_REASON_USER_STOP);
}

void motion_scenario_abort(motion_scenario_reason_t reason)
{
    finish(MOTION_SCENARIO_ABORTED, reason);
}

/** Reports a refusal the same way as any other outcome, so a client is never left guessing. */
static void refuse(uint8_t scenario_id, motion_scenario_reason_t reason)
{
    s_run.last_id = scenario_id;
    s_run.state = MOTION_SCENARIO_FAILED;
    s_run.reason = reason;
    s_run.elapsed_ms = 0;
    s_run.total_ms = 0;
    ESP_LOGW(TAG, "Refused scenario %u: reason=%d", scenario_id, (int)reason);
    notify();
}

void motion_scenario_start(uint8_t scenario_id, const uint8_t *payload, size_t len)
{
    if (s_run.state == MOTION_SCENARIO_RUNNING) {
        refuse(scenario_id, MOTION_SCENARIO_REASON_BUSY);
        return;
    }
    if (motion_is_busy()) {
        refuse(scenario_id, MOTION_SCENARIO_REASON_BUSY);
        return;
    }

    const motion_scenario_def_t *def = NULL;
    for (size_t i = 0; i < SCENARIO_COUNT; i++) {
        if (scenario_registry[i] && scenario_registry[i]->id == scenario_id) {
            def = scenario_registry[i];
            break;
        }
    }
    if (!def) {
        refuse(scenario_id, MOTION_SCENARIO_REASON_UNKNOWN_ID);
        return;
    }

    uint32_t total_ms = 0;
    motion_scenario_reason_t reason = def->start(payload, len, &total_ms);
    if (reason != MOTION_SCENARIO_REASON_NONE) {
        refuse(scenario_id, reason);
        return;
    }

    s_run.active = def;
    s_run.last_id = scenario_id;
    s_run.total_ms = total_ms;
    s_run.elapsed_ms = 0;
    s_run.start_ms = esp_log_timestamp();
    s_run.state = MOTION_SCENARIO_RUNNING;
    s_run.reason = MOTION_SCENARIO_REASON_NONE;
    ESP_LOGI(TAG, "Started '%s' for %lu ms", def->name, (unsigned long)total_ms);
    notify();
}

static void scenario_task(void *arg)
{
    uint32_t last_notify = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(MOTION_SCENARIO_TICK_MS));
        if (s_run.state != MOTION_SCENARIO_RUNNING) continue;

        uint32_t now = esp_log_timestamp();
        s_run.elapsed_ms = now - s_run.start_ms;

        // Common abort rules, enforced for every scenario so none has to remember them.
        // Losing holding torque is the important one: with the motors off the rig can be
        // moved by hand, and every coordinate the scenario is steering by becomes fiction.
        bool vx = false, vc = false, vb = false;
        motion_get_home_valid(&vx, &vc, &vb);
        if (!vx && !vc && !vb) {
            finish(MOTION_SCENARIO_ABORTED, MOTION_SCENARIO_REASON_MOTORS_OFF);
            continue;
        }
        if (motion_is_busy()) {
            finish(MOTION_SCENARIO_ABORTED, MOTION_SCENARIO_REASON_BUSY);
            continue;
        }

        if (s_run.active && s_run.active->tick && s_run.active->tick(s_run.elapsed_ms)) {
            s_run.elapsed_ms = s_run.total_ms;
            finish(MOTION_SCENARIO_DONE, MOTION_SCENARIO_REASON_NONE);
            continue;
        }

        // A watching client gets a steady progress feed; one that was away for the whole run
        // still gets the outcome, because the characteristic is readable too
        if (now - last_notify >= 1000) {
            last_notify = now;
            notify();
        }
    }
}

void motion_scenario_init(void)
{
    scenario_registry[0] = motion_scenario_focus_def();
    xTaskCreate(scenario_task, "scenario", 4096, NULL, 2, NULL);
    ESP_LOGI(TAG, "Scenario runner ready, %u pattern(s)", (unsigned)SCENARIO_COUNT);
}
