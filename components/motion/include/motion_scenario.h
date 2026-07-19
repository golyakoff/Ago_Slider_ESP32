#ifndef MOTION_SCENARIO_H
#define MOTION_SCENARIO_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Scenario framework.
//
// A scenario is a timed movement pattern that runs ENTIRELY ON THE DEVICE. That is the whole
// point of the design: a pass can last an hour, and the phone must be free to disconnect,
// close the app or reboot without disturbing it. A client reconnects at any time, reads the
// status characteristic and sees exactly where the run got to.
//
// Every scenario shares one contract, so the transport, the status reporting and the abort
// rules are written once:
//   * start  — validate the payload and arm the run;
//   * tick   — steer the axes, called on a fixed cadence, says when it is finished;
//   * stop   — release whatever the scenario grabbed.
// The runner owns everything that is common: the state machine, elapsed/total timing, the
// notification cadence and the conditions under which ANY scenario must abort (holding torque
// lost, an endstop reached, homing or calibration taking the axes). A new movement pattern is
// therefore a new file implementing this struct plus one entry in the registry — the BLE
// format, the app-side status handling and the safety rules do not change.
// -----------------------------------------------------------------------------

/** Identifiers travel over BLE, so they are fixed once assigned and never reused. */
typedef enum {
    MOTION_SCENARIO_ID_NONE  = 0,
    MOTION_SCENARIO_ID_FOCUS = 1,   // timed X pass keeping a subject centred via C
} motion_scenario_id_t;

typedef enum {
    MOTION_SCENARIO_IDLE    = 0,
    MOTION_SCENARIO_RUNNING = 1,
    MOTION_SCENARIO_DONE    = 2,
    MOTION_SCENARIO_ABORTED = 3,
    MOTION_SCENARIO_FAILED  = 4,
} motion_scenario_state_t;

typedef enum {
    MOTION_SCENARIO_REASON_NONE       = 0,
    MOTION_SCENARIO_REASON_USER_STOP  = 1,
    MOTION_SCENARIO_REASON_LIMIT      = 2,
    MOTION_SCENARIO_REASON_MOTORS_OFF = 3,
    MOTION_SCENARIO_REASON_NOT_HOMED  = 4,
    MOTION_SCENARIO_REASON_BUSY       = 5,
    MOTION_SCENARIO_REASON_BAD_PARAMS = 6,
    MOTION_SCENARIO_REASON_UNKNOWN_ID = 7,
} motion_scenario_reason_t;

/**
 * The contract every scenario implements.
 *
 * `start` gets the raw BLE payload — each scenario owns its own parameter layout, which is
 * what lets new patterns be added without touching the framing. It returns
 * MOTION_SCENARIO_REASON_NONE to begin, or the reason it refused. It must also report the
 * intended total duration through `total_ms` so the runner can drive a progress display for
 * scenarios it knows nothing about.
 *
 * `tick` is called every MOTION_SCENARIO_TICK_MS while running and returns true when the run
 * has reached its natural end. It does NOT have to police the common abort conditions.
 *
 * `stop` is called exactly once for every started run, whatever ended it.
 */
typedef struct {
    uint8_t id;
    const char *name;
    motion_scenario_reason_t (*start)(const uint8_t *payload, size_t len, uint32_t *total_ms);
    bool (*tick)(uint32_t elapsed_ms);
    void (*stop)(void);
} motion_scenario_def_t;

#define MOTION_SCENARIO_TICK_MS 50

/** Status callback; the payload is identical for every scenario. */
typedef void (*motion_scenario_status_cb_t)(uint8_t scenario_id, uint8_t state, uint8_t reason,
                                            uint32_t elapsed_ms, uint32_t total_ms);

void motion_scenario_init(void);
void motion_scenario_set_status_cb(motion_scenario_status_cb_t cb);

/**
 * @brief Start the scenario with the given id, handing it the rest of the write payload.
 *        Refusals are reported through the status callback, never by silence.
 */
void motion_scenario_start(uint8_t scenario_id, const uint8_t *payload, size_t len);

/** @brief Stop at the user's request, keeping the step counts intact. */
void motion_scenario_stop(void);

/**
 * @brief Abort a running scenario for a reason outside its control. Called by motion when
 *        holding torque is lost, an endstop is reached, or homing/calibration takes over.
 */
void motion_scenario_abort(motion_scenario_reason_t reason);

void motion_scenario_get_status(uint8_t *scenario_id, uint8_t *state, uint8_t *reason,
                                uint32_t *elapsed_ms, uint32_t *total_ms);

bool motion_scenario_is_running(void);

// -----------------------------------------------------------------------------
// Registry — one line per movement pattern
// -----------------------------------------------------------------------------
const motion_scenario_def_t *motion_scenario_focus_def(void);

#ifdef __cplusplus
}
#endif

#endif // MOTION_SCENARIO_H
