#ifndef __BLE_H__
#define __BLE_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Callback types for system events
// ============================================================================

/**
 * @brief Callback when BLE advertising starts successfully
 */
typedef void (*ble_start_cb_t)(void);

/**
 * @brief Callback when a client connects
 */
typedef void (*ble_connect_cb_t)(void);

/**
 * @brief Callback when a client disconnects
 */
typedef void (*ble_disconnect_cb_t)(void);

// ============================================================================
// Callback types for application events
// ============================================================================

/**
 * @brief Callback for MOTOR ENABLE command
 * @param enable  true = motors enabled, false = disabled
 */
typedef void (*ble_mot_en_cb_t)(bool enable);

/**
 * @brief Callback for HOME command
 * @param home_x  true to home X axis
 * @param home_c  true to home C axis
 * @param home_b  true to home B axis
 */
typedef void (*ble_home_cmd_cb_t)(bool home_x, bool home_c, bool home_b);

/**
 * @brief Callback for MOVE command (relative distance)
 * @param x  steps (or mm/deg * 10) for X axis
 * @param c  steps (or mm/deg * 10) for C axis
 * @param b  steps (or mm/deg * 10) for B axis
 */
typedef void (*ble_move_cb_t)(int32_t x, int32_t c, int32_t b);

/**
 * @brief Callback for LIMIT characteristic read request.
 *        Should return current limit switches status as a byte (bits 0-2: X, C, B).
 * @brief Callback for LIMIT characteristic read request.
 *        Should fill current limit switches status for X, C, B axes.
 *        true = limit reached, false = not reached.
 */
typedef void (*ble_limit_read_cb_t)(bool *x, bool *c, bool *b);

/**
 * @brief Callback for POSITION characteristic read request.
 *        Should fill the current commanded positions in STEP pulses for X, C, B axes.
 */
typedef void (*ble_position_read_cb_t)(int32_t *x, int32_t *c, int32_t *b, uint8_t *valid_mask);

/**
 * @brief Callback for CALIBRATE characteristic write.
 * @param axis           0=X, 1=C, 2=B; 0xFF = abort the running calibration
 * @param park_offset    park position in STEP pulses from the min trigger
 * @param retreat_steps  endstop back-off distance in STEP pulses
 */
typedef void (*ble_calibrate_cb_t)(uint8_t axis, int32_t park_offset, int32_t retreat_steps);

// ============================================================================
// Callback types for configuration
// ============================================================================

/**
 * @brief Callback for Microsteps configuration for three axis in format 0=256,1=1,2=2,4,8,16,32,64.
 * @param x_microsteps  Microsteps for X axis
 * @param c_microsteps  Microsteps for C axis
 * @param b_microsteps  Microsteps for B axis
 */
typedef void (*ble_microsteps_cb_t)(uint8_t x_microsteps, uint8_t c_microsteps, uint8_t b_microsteps);

/**
 * @brief Callback for Run current configuration for three axis in mA.
 * @param x_run_current  Run current for X axis driver
 * @param c_run_current  Run current for C axis driver
 * @param b_run_current  Run current for B axis driver
 */
typedef void (*ble_run_current_cb_t)(uint16_t x_run_current, uint16_t c_run_current, uint16_t b_run_current);

/**
 * @brief Callback for Hold current configuration for three axis in mA.
 * @param x_hold_current  Hold current for X axis driver
 * @param c_hold_current  Hold current for C axis driver
 * @param b_hold_current  Hold current for B axis driver
 */
typedef void (*ble_hold_current_cb_t)(uint16_t x_hold_current, uint16_t c_hold_current, uint16_t b_hold_current);

/**
 * @brief Callback for Unit configuration for three axis: AXIS_UNIT_MM or AXIS_UNIT_DEG.
 * @param x_deg  True if unit for X axis is degrees, False for mm.
 * @param c_deg  True if unit for C axis is degrees, False for mm.
 * @param b_deg  True if unit for B axis is degrees, False for mm.
 */
typedef void (*ble_axis_unit_cb_t)(bool x_deg, bool c_deg, bool b_deg);

/**
 * @brief Callback for Units per step for three axis (depends on Unit).
 * @param x  mm/step or deg/step for X axis
 * @param c  mm/step or deg/step for C axis
 * @param b  mm/step or deg/step for B axis
 */
typedef void (*ble_units_per_step_cb_t)(float x, float c, float b);

/**
 * @brief Callback for Axis speed for three axis.
 * @param x  steps per second for X axis
 * @param c  steps per second for C axis
 * @param b  steps per second for B axis
 */
typedef void (*ble_axis_speed_cb_t)(uint16_t x, uint16_t c, uint16_t b);

/**
 * @brief Callback for Axis acceleration for three axis.
 * @param x  Steps per second for X axis
 * @param c  Steps per second for C axis
 * @param b  Steps per second for B axis
 */
typedef void (*ble_axis_accel_cb_t)(uint16_t x, uint16_t c, uint16_t b);

/**
 * @brief Callback for Virtual limit enable (1 byte, 3 bits)
 * @param x_en  True = stop when limit is reached on X axis, otherwise False.
 * @param c_en  True = stop when limit is reached on C axis, otherwise False.
 * @param b_en  True = stop when limit is reached on B axis, otherwise False.
 */
typedef void (*ble_virtual_limit_cb_t)(bool x_en, bool c_en, bool b_en);

/**
 * @brief Callback for Stealthchop mode enable (1 byte, 3 bits)
 * @param x_stealthchop  True = enable stealthchop on X axis, otherwise False.
 * @param c_stealthchop  True = enable stealthchop on C axis, otherwise False.
 * @param b_stealthchop  True = enable stealthchop on B axis, otherwise False.
 */
typedef void (*ble_stealthchop_cb_t)(bool x_stealthchop, bool c_stealthchop, bool b_stealthchop);

/**
 * @brief Callback for Invert direction on axis (1 byte, 3 bits)
 * @param x_invert_dir  True = stop when limit is reached on X axis, otherwise False.
 * @param c_invert_dir  True = stop when limit is reached on C axis, otherwise False.
 * @param b_invert_dir  True = stop when limit is reached on B axis, otherwise False.
 */
typedef void (*ble_invert_dir_cb_t)(bool x_invert_dir, bool c_invert_dir, bool b_invert_dir);

/**
 * @brief Callback for reading firmware version (characteristic VERSION)
 * @return Pointer to null-terminated version string (e.g., "1.0.0").
 *         The string must remain valid for the lifetime of the BLE stack.
 */
typedef const char* (*ble_version_read_cb_t)(void);

/**
 * @brief Callback for OTA control command (characteristic OTA_CONTROL)
 * @param data Pointer to received command bytes.
 * @param len  Length of the data buffer (at least 1 byte).
 *
 * Command format:
 *   - 0x01 (start) followed by 4 bytes: total firmware size (uint32_t, little‑endian)
 *   - 0x02 (end)   no extra data
 *   - 0x03 (abort) no extra data
 */
typedef void (*ble_ota_control_cb_t)(const uint8_t* data, size_t len);

/**
 * @brief Callback for OTA firmware data (characteristic OTA_DATA)
 * @param data Pointer to raw firmware chunk.
 * @param len  Number of bytes to write.
 *
 * This callback is called each time the client writes to the OTA_DATA characteristic.
 * The implementation should forward the data to the OTA update module.
 */
typedef void (*ble_ota_data_cb_t)(const uint8_t* data, size_t len);

// ============================================================================
// BLE initialization
// ============================================================================

/**
 * @brief Initialize BLE server with GATT attribute table.
 *        Starts BLE stack, creates services/characteristics, and begins advertising.
 *
 * @param device_name           BLE device name (max 32 chars)
 * @param on_start_cb           Called when advertising starts (optional, may be NULL)
 * @param on_connect_cb         Called when client connects (optional)
 * @param on_disconnect_cb      Called when client disconnects (optional)
 * @param mot_en_cb             Called when MOT_EN characteristic is written (1 byte: 0/1)
 * @param home_cmd_cb           Called when HOME characteristic is written (byte with bits 4-6)
 * @param move_cb               Called when MOVE characteristic is written (6 bytes: X,C,B int16)
 * @param limit_read_cb         Called when LIMIT characteristic is read; should fill X,C,B limit flags
 * @param position_read_cb      Called when POSITION characteristic is read; should fill X,C,B positions (int32 steps)
 * @param microsteps_cb         Called when MICROSTEPS characteristic is written (3 bytes)
 * @param run_current_cb        Called when RUN_CURRENT characteristic is written (3x uint16 mA)
 * @param hold_current_cb       Called when HOLD_CURRENT characteristic is written (3x uint16 mA)
 * @param axis_unit_cb          Called when AXIS_UNIT characteristic is written (1 byte, bits per axis)
 * @param units_per_step_cb     Called when UNITS_PER_STEP characteristic is written (12 bytes, 3 floats)
 * @param axis_speed_cb         Called when AXIS_SPEED characteristic is written (6 bytes, 3 uint16)
 * @param axis_accel_cb          Called when AXIS_ACCEL characteristic is written (6 bytes, 3 uint16)
 * @param virtual_limit_cb      Called when VIRTUAL_LIMIT characteristic is written (1 byte, bits)
 * @param stealthchop_cb        Called when STEALTHCHOP characteristic is written (1 byte, bits)
 * @param invert_dir_cb         Called when INVERT_DIR characteristic is written (1 byte, bits)
 */
void ble_init(
    const char *device_name,
    ble_start_cb_t on_start_cb,
    ble_connect_cb_t on_connect_cb,
    ble_disconnect_cb_t on_disconnect_cb,
    ble_version_read_cb_t version_read_cb,
    ble_ota_control_cb_t ota_control_cb,
    ble_ota_data_cb_t ota_data_cb,
    ble_mot_en_cb_t mot_en_cb,
    ble_home_cmd_cb_t home_cmd_cb,
    ble_move_cb_t move_cb,
    ble_limit_read_cb_t limit_read_cb,
    ble_position_read_cb_t position_read_cb,
    ble_calibrate_cb_t calibrate_cb,
    ble_microsteps_cb_t microsteps_cb,
    ble_run_current_cb_t run_current_cb,
    ble_hold_current_cb_t hold_current_cb,
    ble_axis_unit_cb_t axis_unit_cb,
    ble_units_per_step_cb_t units_per_step_cb,
    ble_axis_speed_cb_t axis_speed_cb,
    ble_axis_accel_cb_t axis_accel_cb,
    ble_virtual_limit_cb_t virtual_limit_cb,
    ble_stealthchop_cb_t stealthchop_cb,
    ble_invert_dir_cb_t invert_dir_cb);

// ============================================================================
// Initial configuration setters (call before ble_init)
// ============================================================================

/**
 * @brief Set microsteps buffer before BLE initialisation.
 * @param x  microstep value for X axis (0=256,1=1,2=2,4,8,16,32,64)
 * @param c  for C axis
 * @param b  for B axis
 */
void ble_set_microsteps_value(uint8_t x, uint8_t c, uint8_t b);

/**
 * @brief Set run current buffer before BLE initialisation.
 * @param x  run current in mA for X axis (uint16_t)
 * @param c  run current in mA for C axis
 * @param b  run current in mA for B axis
 */
void ble_set_run_current_value(uint16_t x, uint16_t c, uint16_t b);

/**
 * @brief Set hold current buffer before BLE initialisation.
 * @param x  hold current in mA for X axis (uint16_t)
 * @param c  hold current in mA for C axis
 * @param b  hold current in mA for B axis
 */
void ble_set_hold_current_value(uint16_t x, uint16_t c, uint16_t b);

/**
 * @brief Set axis unit before BLE initialisation.
 * @param x_deg  True if unit for X axis is degrees, False for mm.
 * @param c_deg  True if unit for C axis is degrees, False for mm.
 * @param b_deg  True if unit for B axis is degrees, False for mm.
 */
void ble_set_axis_unit_value(bool x_deg, bool c_deg, bool b_deg);

/**
 * @brief Set units per step (floats) before BLE initialisation.
 * @param x  mm/step or deg/step for X axis
 * @param c  for C axis
 * @param b  for B axis
 */
void ble_set_units_per_step_value(float x, float c, float b);

/**
 * @brief Set axis speed (steps/sec) before BLE initialisation.
 * @param x  speed for X axis
 * @param c  for C axis
 * @param b  for B axis
 */
void ble_set_axis_speed_value(uint16_t x, uint16_t c, uint16_t b);

/**
 * @brief Set axis acceleration (steps/sec) before BLE initialisation.
 * @param x  speed for X axis
 * @param c  for C axis
 * @param b  for B axis
 */
void ble_set_axis_accel_value(uint16_t x, uint16_t c, uint16_t b);

/**
 * @brief Set virtual limit enable flags before BLE initialisation.
 * @param x_en  enable for X axis
 * @param c_en  enable for C axis
 * @param b_en  enable for B axis
 */
void ble_set_virtual_limit_value(bool x_en, bool c_en, bool b_en);

// ============================================================================
// Sending notifications / updating characteristic values
// ============================================================================

/**
 * @brief Send limit switch status (3 axes) via notification (characteristic LIMIT)
 * @param x_limited  true if X axis limit reached
 * @param c_limited  true if C axis limit reached
 * @param b_limited  true if B axis limit reached
 */
void ble_set_limit(bool x_limited, bool c_limited, bool b_limited);

/**
 * @brief Update the POSITION characteristic (3x int32 little-endian, STEP pulses) and send
 *        a notification if the client has subscribed. An axis's position is reset to 0 by
 *        the motion module when the axis completes homing.
 * @param x  current position of X axis in steps
 * @param c  current position of C axis in steps
 * @param b  current position of B axis in steps
 */
void ble_set_position(int32_t x, int32_t c, int32_t b, uint8_t valid_mask);

/**
 * @brief Publish hardware-calibration progress (characteristic CALIBRATE, notify).
 * @param axis        0=X, 1=C, 2=B
 * @param phase       motion_calib_phase_t value
 * @param span_steps  measured endstop-to-endstop span in STEP pulses (valid from PARK on)
 */
void ble_set_calib_status(uint8_t axis, uint8_t phase, int32_t span_steps);

/**
 * @brief Send homing progress update (notification on HOME characteristic)
 * @param x_requested  true if X axis homing was requested (still pending unless homed)
 * @param c_requested  true if C axis homing was requested
 * @param b_requested  true if B axis homing was requested
 * @param x_homed      true if X axis has already homed
 * @param c_homed      true if C axis has already homed
 * @param b_homed      true if B axis has already homed
 *
 * The function will set the high nibble (bits 4-6) to the axes that were requested but not yet homed,
 * and the low nibble (bits 0-2) to the axes that are already homed.
 * A notification is sent if the client has subscribed to it.
 */
void ble_set_home_status(
    bool x_requested,
    bool c_requested,
    bool b_requested,
    bool x_homed,
    bool c_homed,
    bool b_homed);

/**
 * @brief Update the MOT_EN characteristic value (without sending notification).
 *        Useful if motor enable state is changed by other means (e.g., hardware button).
 * @param enable  new state (1 = enabled, 0 = disabled)
 */
void ble_set_mot_en_state(uint8_t enable);

/**
 * @brief Update the battery level characteristic value and send notification if enabled.
 *        Battery level is specified as a percentage (0–100). Internally it is mapped linearly
 *        to the characteristic value 0–255, where 0% → 0, 100% → 255.
 *        The current voltage range (18.0V – 21.5V) is mapped to 0–100% by the caller,
 *        or you can use a simple linear transformation if the battery voltage is known.
 * @param percent  battery level in percent (0–100)
 */
void ble_set_battery_level(uint8_t percent);

/**
 * @brief Update the power info characteristic (binary float values) and send notification if enabled.
 *        The characteristic value consists of three little‑endian floats:
 *        voltage (volts), current (amperes), power (watts).
 *        A notification is sent if the client has subscribed.
 * @param voltage  bus voltage in volts
 * @param current  current in amperes
 * @param power    power in watts (voltage * current)
 */
void ble_set_power_info(float voltage, float current, float power);

/**
 * @brief Update the power info string characteristic and send notification if enabled.
 *        The string is formatted as "XX.XXV Y.YYYA Z.ZZW" (e.g., "21.48V 0.082A 1.76W").
 *        A notification is sent if the client has subscribed.
 * @param str  null‑terminated string, up to 40 characters
 */
void ble_set_power_info_string(const char* str);

/**
 * @brief Update the firmware version characteristic and send a notification.
 *
 * Reads the current firmware version via the version read callback
 * (set in ble_init) and updates the VERSION characteristic value.
 * If the client has enabled notifications, a notification is sent.
 *
 * This function should be called after the BLE stack is initialised and
 * the client has connected, typically once, to push the version to the client.
 */
void ble_update_firmware_version(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLE_H__ */