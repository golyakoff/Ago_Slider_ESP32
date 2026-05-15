#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Configuration structure (raw storage)
// -----------------------------------------------------------------------------
#pragma pack(push, 1)
typedef struct {
    uint8_t microsteps[3];          // 0=256,1=1,2=2,4,8,16,32,64
    uint8_t run_current[6];         // 3 uint16 (run current in mA) 
    uint8_t hold_current[6];        // 3 uint16 (hold current in mA)
    uint8_t axis_unit;              // 3 bits: 0=mm,1=deg per axis (bit0=X,bit1=C,bit2=B)
    uint8_t units_per_step[12];     // 3 floats (little-endian)
    uint8_t axis_speed[6];          // 3 uint16 (step/sec)
    uint8_t axis_accel[6];          // 3 uint16 (step/sec²)
    uint8_t virtual_limit;          // 3 bits: enable virtual limit per axis (bit0=X,bit1=C,bit2=B)
    uint8_t stealthchop;            // 3 bits: enable stealthchop (bit0=X,bit1=C,bit2=B)
    uint8_t invert_dir;             // 3 bits: invert axis direction (bit0=X,bit1=C,bit2=B)
} app_config_t;
#pragma pack(pop)

// -----------------------------------------------------------------------------
// Pack/Unpack utilities (convert raw bytes to structured values)
// -----------------------------------------------------------------------------

/**
 * @brief Unpack microsteps from raw array into separate variables.
 */
void app_config_unpack_microsteps(const app_config_t *cfg, uint8_t *x, uint8_t *c, uint8_t *b);

/**
 * @brief Pack microsteps into raw config structure.
 */
void app_config_pack_microsteps(app_config_t *cfg, uint8_t x, uint8_t c, uint8_t b);

/**
 * @brief Unpack run current (mA) from raw array into three uint16 values.
 */
void app_config_unpack_run_current(const app_config_t *cfg, uint16_t *x, uint16_t *c, uint16_t *b);

/**
 * @brief Pack run current values into raw config structure (little-endian).
 */
void app_config_pack_run_current(app_config_t *cfg, uint16_t x, uint16_t c, uint16_t b);

/**
 * @brief Unpack hold current (mA) from raw array into three uint16 values.
 */
void app_config_unpack_hold_current(const app_config_t *cfg, uint16_t *x, uint16_t *c, uint16_t *b);

/**
 * @brief Pack hold current values into raw config structure (little-endian).
 */
void app_config_pack_hold_current(app_config_t *cfg, uint16_t x, uint16_t c, uint16_t b);

/**
 * @brief Unpack axis unit from raw byte into three app_cfg_axis_unit_t.
 */
void app_config_unpack_axis_unit(const app_config_t *cfg, bool *x_deg, bool *c_deg, bool *b_deg);

/**
 * @brief Pack axis unit into raw config structure.
 */
void app_config_pack_axis_unit(app_config_t *cfg, bool x_deg, bool c_deg, bool b_deg);

/**
 * @brief Unpack units per step (floats).
 */
void app_config_unpack_units_per_step(const app_config_t *cfg, float *x, float *c, float *b);

/**
 * @brief Pack units per step.
 */
void app_config_pack_units_per_step(app_config_t *cfg, float x, float c, float b);

/**
 * @brief Unpack axis speeds (uint16_t).
 */
void app_config_unpack_axis_speed(const app_config_t *cfg, uint16_t *x, uint16_t *c, uint16_t *b);

/**
 * @brief Pack axis speeds.
 */
void app_config_pack_axis_speed(app_config_t *cfg, uint16_t x, uint16_t c, uint16_t b);

/**
 * @brief Unpack axis accelerations.
 */
void app_config_unpack_axis_accel(const app_config_t *cfg, uint16_t *x, uint16_t *c, uint16_t *b);

/**
 * @brief Pack axis accelerations.
 */
void app_config_pack_axis_accel(app_config_t *cfg, uint16_t x, uint16_t c, uint16_t b);

/**
 * @brief Unpack virtual limit into booleans.
 */
void app_config_unpack_virtual_limit(const app_config_t *cfg, bool *x_en, bool *c_en, bool *b_en);

/**
 * @brief Pack virtual limit.
 */
void app_config_pack_virtual_limit(app_config_t *cfg, bool x_en, bool c_en, bool b_en);

/**
 * @brief Unpack stealthchop enable flags from raw byte into booleans.
 */
void app_config_unpack_stealthchop(const app_config_t *cfg, bool *x_en, bool *c_en, bool *b_en);

/**
 * @brief Pack stealthchop enable flags into raw config structure.
 */
void app_config_pack_stealthchop(app_config_t *cfg, bool x_en, bool c_en, bool b_en);

/**
 * @brief Unpack invert direction flags from raw byte into booleans.
 */
void app_config_unpack_invert_dir(const app_config_t *cfg, bool *x_inv, bool *c_inv, bool *b_inv);

/**
 * @brief Pack invert direction flags into raw config structure.
 */
void app_config_pack_invert_dir(app_config_t *cfg, bool x_inv, bool c_inv, bool b_inv);


#ifdef __cplusplus
}
#endif

#endif // APP_CONFIG_H
