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
 * @brief Emergency stop all motion.
 */
void motion_emergency_stop(void);

#ifdef __cplusplus
}
#endif

#endif // MOTION_H
