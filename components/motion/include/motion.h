#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <stdint.h>

// Forward declaration of FastAccelStepper (C++ class)
#ifdef __cplusplus
class FastAccelStepper;
#else
typedef struct FastAccelStepper FastAccelStepper;
#endif

// PCA9555 struct forward declaration
typedef struct PCA9555 PCA9555;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback to notify about homing progress.
 * @param x_requested true if X axis was requested to home
 * @param c_requested true if C axis was requested
 * @param b_requested true if B axis was requested
 * @param x_homed true if X axis has homed
 * @param c_homed true if C axis has homed
 * @param b_homed true if B axis has homed
 */
typedef void (*motion_home_status_cb_t)(
    bool x_requested,
    bool c_requested,
    bool b_requested,
    bool x_homed,
    bool c_homed,
    bool b_homed);

/**
 * @brief Initialise motion controller.
 * @param stepper_x pointer to FastAccelStepper for X axis
 * @param stepper_c pointer to FastAccelStepper for C axis
 * @param stepper_b pointer to FastAccelStepper for B axis
 * @param pca9555_dev pointer to initialised PCA9555 device
 * @param pin_limit_x PCA9555 pin number for X limit switch (active LOW)
 * @param pin_limit_c PCA9555 pin number for C limit switch
 * @param pin_limit_b PCA9555 pin number for B limit switch
 * @param home_status_cb callback for home progress (may be NULL)
 */
void motion_init(
    FastAccelStepper* stepper_x,
    FastAccelStepper* stepper_c,
    FastAccelStepper* stepper_b,
    PCA9555* pca9555_dev,
    uint8_t pin_limit_x,
    uint8_t pin_limit_c,
    uint8_t pin_limit_b,
    motion_home_status_cb_t home_status_cb);

// -----------------------------------------------------------------------------
// Configuration setters
// -----------------------------------------------------------------------------

/**
 * @brief Set axis units (millimeters or degrees) for each axis.
 * @param x_deg  true = degrees, false = millimeters
 * @param c_deg  true = degrees, false = millimeters
 * @param b_deg  true = degrees, false = millimeters
 */
void motion_set_axis_unit(bool x_deg, bool c_deg, bool b_deg);

/**
 * @brief Set units per step (mm/step or deg/step) for each axis.
 * @param x  value for X axis
 * @param c  value for C axis
 * @param b  value for B axis
 */
void motion_set_units_per_step(float x, float c, float b);

/**
 * @brief Set virtual limit enable flags.
 * @param x_en  true = enable virtual limit for X axis
 * @param c_en  true = enable virtual limit for C axis
 * @param b_en  true = enable virtual limit for B axis
 */
void motion_set_virtual_limit(bool x_en, bool c_en, bool b_en);

// -----------------------------------------------------------------------------
// Commands
// -----------------------------------------------------------------------------

/**
 * @brief Start homing for selected axes.
 * @param home_x true to home X
 * @param home_c true to home C
 * @param home_b true to home B
 */
void motion_start_homing(bool home_x, bool home_c, bool home_b);

// -----------------------------------------------------------------------------
// Hardware calibration: measures an axis's endstop-to-endstop span entirely
// on-device, reacting to endstop events with ~1 ms latency (the sensors emit
// millisecond blinks that a BLE-driven loop cannot catch in time).
// Sequence: fast seek to the min endstop -> retreat -> slow re-seek (position
// zeroed at the trigger) -> fast seek to the max endstop -> retreat -> slow
// re-seek (span captured) -> park at the given offset.
// -----------------------------------------------------------------------------

/** Calibration phases reported via the status callback. */
typedef enum {
    MOTION_CALIB_IDLE = 0,
    MOTION_CALIB_SEEK_MIN_FAST,
    MOTION_CALIB_RETREAT_MIN,
    MOTION_CALIB_SEEK_MIN_SLOW,
    MOTION_CALIB_SEEK_MAX_FAST,
    MOTION_CALIB_RETREAT_MAX,
    MOTION_CALIB_SEEK_MAX_SLOW,
    MOTION_CALIB_PARK,
    MOTION_CALIB_DONE,
    MOTION_CALIB_FAILED,
} motion_calib_phase_t;

/**
 * @brief Calibration progress callback.
 * @param axis        0=X, 1=C, 2=B
 * @param phase       current motion_calib_phase_t
 * @param span_steps  measured min-to-max span in STEP pulses (valid from PARK on)
 */
typedef void (*motion_calib_status_cb_t)(uint8_t axis, uint8_t phase, int32_t span_steps);

/** @brief Register the calibration status callback (may be NULL). */
void motion_set_calib_status_cb(motion_calib_status_cb_t cb);

/**
 * @brief Start hardware calibration of one axis.
 * @param axis               0=X, 1=C, 2=B
 * @param park_offset_steps  where to park when done, in STEP pulses from the min trigger
 * @param retreat_steps      how far to back off an endstop before the slow re-seek
 */
void motion_start_calibration(uint8_t axis, int32_t park_offset_steps, int32_t retreat_steps);

/** @brief Abort a running calibration (force-stops the axis). */
void motion_abort_calibration(void);

/** @brief True while a calibration sequence is running. */
bool motion_is_calibrating(void);

/**
 * @brief Feed an endstop pin event (from the PCA9555 interrupt path) into the
 *        calibration state machine. Safe to call for any pin; non-endstop pins
 *        and inactive phases are ignored.
 * @param pin  PCA9555 pin number
 * @param raw  raw pin level (endstops are active-low)
 */
void motion_on_limit_pin_event(uint8_t pin, bool raw);

/**
 * @brief Move axes by relative distances (steps).
 * @param x steps (positive = forward, negative = backward)
 * @param c steps
 * @param b steps
 */
void motion_move_relative(int32_t x, int32_t c, int32_t b);

/**
 * @brief Enable or disable motors (through tmc2130).
 * @param enable true = enable, false = disable
 */
void motion_set_enable(bool enable);

/**
 * @brief Read current limit switch states.
 * @param x_limited pointer to bool for X (may be NULL)
 * @param c_limited pointer to bool for C (may be NULL)
 * @param b_limited pointer to bool for B (may be NULL)
 */
void motion_get_limits(bool *x_limited, bool *c_limited, bool *b_limited);

/**
 * @brief Read current positions in STEP pulses, as counted by the step generator.
 *        An axis's position is reset to 0 the moment it completes homing.
 * @param x pointer for X position (may be NULL)
 * @param c pointer for C position (may be NULL)
 * @param b pointer for B position (may be NULL)
 */
void motion_get_positions(int32_t *x, int32_t *c, int32_t *b);

/**
 * @brief Read whether each axis's position still means something: true from the moment the
 *        axis is zeroed at its endstop until the drivers are disabled or the board reboots.
 *        With the motors off the carriage can be moved by hand, so the step count is no
 *        longer a measurement.
 * @param x pointer for X validity (may be NULL)
 * @param c pointer for C validity (may be NULL)
 * @param b pointer for B validity (may be NULL)
 */
void motion_get_home_valid(bool *x, bool *c, bool *b);

/**
 * @brief Drop the home reference on every axis. Call when holding torque is lost.
 */
void motion_invalidate_home(void);

/**
 * @brief Emergency stop all motion.
 */
void motion_emergency_stop(void);

#ifdef __cplusplus
}
#endif

#endif // MOTION_H
