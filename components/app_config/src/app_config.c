#include "app_config.h"
#include <string.h>

void app_config_unpack_microsteps(const app_config_t *cfg, uint8_t *x, uint8_t *c, uint8_t *b)
{
    if (x) *x = cfg->microsteps[0];
    if (c) *c = cfg->microsteps[1];
    if (b) *b = cfg->microsteps[2];
}

void app_config_pack_microsteps(app_config_t *cfg, uint8_t x, uint8_t c, uint8_t b)
{
    cfg->microsteps[0] = x;
    cfg->microsteps[1] = c;
    cfg->microsteps[2] = b;
}

void app_config_unpack_run_current(const app_config_t *cfg, uint16_t *x, uint16_t *c, uint16_t *b)
{
    if (x) *x = cfg->run_current[0] | (cfg->run_current[1] << 8);
    if (c) *c = cfg->run_current[2] | (cfg->run_current[3] << 8);
    if (b) *b = cfg->run_current[4] | (cfg->run_current[5] << 8);
}

void app_config_pack_run_current(app_config_t *cfg, uint16_t x, uint16_t c, uint16_t b)
{
    cfg->run_current[0] = x & 0xFF;
    cfg->run_current[1] = (x >> 8) & 0xFF;
    cfg->run_current[2] = c & 0xFF;
    cfg->run_current[3] = (c >> 8) & 0xFF;
    cfg->run_current[4] = b & 0xFF;
    cfg->run_current[5] = (b >> 8) & 0xFF;
}

void app_config_unpack_hold_current(const app_config_t *cfg, uint16_t *x, uint16_t *c, uint16_t *b)
{
    if (x) *x = cfg->hold_current[0] | (cfg->hold_current[1] << 8);
    if (c) *c = cfg->hold_current[2] | (cfg->hold_current[3] << 8);
    if (b) *b = cfg->hold_current[4] | (cfg->hold_current[5] << 8);
}

void app_config_pack_hold_current(app_config_t *cfg, uint16_t x, uint16_t c, uint16_t b)
{
    cfg->hold_current[0] = x & 0xFF;
    cfg->hold_current[1] = (x >> 8) & 0xFF;
    cfg->hold_current[2] = c & 0xFF;
    cfg->hold_current[3] = (c >> 8) & 0xFF;
    cfg->hold_current[4] = b & 0xFF;
    cfg->hold_current[5] = (b >> 8) & 0xFF;
}

void app_config_unpack_axis_unit(const app_config_t *cfg, bool *x_deg, bool *c_deg, bool *b_deg)
{
    if (x_deg) *x_deg = (cfg->axis_unit & 0x01) != 0;
    if (c_deg) *c_deg = (cfg->axis_unit & 0x02) != 0;
    if (b_deg) *b_deg = (cfg->axis_unit & 0x04) != 0;
}

void app_config_pack_axis_unit(app_config_t *cfg, bool x_deg, bool c_deg, bool b_deg)
{
    cfg->axis_unit = (x_deg ? 0x01 : 0) |
                     (c_deg ? 0x02 : 0) |
                     (b_deg ? 0x04 : 0);
}

void app_config_unpack_units_per_step(const app_config_t *cfg, float *x, float *c, float *b)
{
    if (x) memcpy(x, cfg->units_per_step, 4);
    if (c) memcpy(c, cfg->units_per_step + 4, 4);
    if (b) memcpy(b, cfg->units_per_step + 8, 4);
}

void app_config_pack_units_per_step(app_config_t *cfg, float x, float c, float b)
{
    memcpy(cfg->units_per_step, &x, 4);
    memcpy(cfg->units_per_step + 4, &c, 4);
    memcpy(cfg->units_per_step + 8, &b, 4);
}

void app_config_unpack_axis_speed(const app_config_t *cfg, uint16_t *x, uint16_t *c, uint16_t *b)
{
    if (x) *x = cfg->axis_speed[0] | (cfg->axis_speed[1] << 8);
    if (c) *c = cfg->axis_speed[2] | (cfg->axis_speed[3] << 8);
    if (b) *b = cfg->axis_speed[4] | (cfg->axis_speed[5] << 8);
}

void app_config_pack_axis_speed(app_config_t *cfg, uint16_t x, uint16_t c, uint16_t b)
{
    cfg->axis_speed[0] = x & 0xFF;
    cfg->axis_speed[1] = (x >> 8) & 0xFF;
    cfg->axis_speed[2] = c & 0xFF;
    cfg->axis_speed[3] = (c >> 8) & 0xFF;
    cfg->axis_speed[4] = b & 0xFF;
    cfg->axis_speed[5] = (b >> 8) & 0xFF;
}

void app_config_unpack_axis_accel(const app_config_t *cfg, uint16_t *x, uint16_t *c, uint16_t *b)
{
    if (x) *x = cfg->axis_accel[0] | (cfg->axis_accel[1] << 8);
    if (c) *c = cfg->axis_accel[2] | (cfg->axis_accel[3] << 8);
    if (b) *b = cfg->axis_accel[4] | (cfg->axis_accel[5] << 8);
}

void app_config_pack_axis_accel(app_config_t *cfg, uint16_t x, uint16_t c, uint16_t b)
{
    cfg->axis_accel[0] = x & 0xFF;
    cfg->axis_accel[1] = (x >> 8) & 0xFF;
    cfg->axis_accel[2] = c & 0xFF;
    cfg->axis_accel[3] = (c >> 8) & 0xFF;
    cfg->axis_accel[4] = b & 0xFF;
    cfg->axis_accel[5] = (b >> 8) & 0xFF;
}

void app_config_unpack_virtual_limit(const app_config_t *cfg, bool *x_en, bool *c_en, bool *b_en)
{
    if (x_en) *x_en = (cfg->virtual_limit & 0x01) != 0;
    if (c_en) *c_en = (cfg->virtual_limit & 0x02) != 0;
    if (b_en) *b_en = (cfg->virtual_limit & 0x04) != 0;
}

void app_config_pack_virtual_limit(app_config_t *cfg, bool x_en, bool c_en, bool b_en)
{
    cfg->virtual_limit = (x_en ? 0x01 : 0) | (c_en ? 0x02 : 0) | (b_en ? 0x04 : 0);
}

void app_config_unpack_stealthchop(const app_config_t *cfg, bool *x_en, bool *c_en, bool *b_en)
{
    if (x_en) *x_en = (cfg->stealthchop & 0x01) != 0;
    if (c_en) *c_en = (cfg->stealthchop & 0x02) != 0;
    if (b_en) *b_en = (cfg->stealthchop & 0x04) != 0;
}

void app_config_pack_stealthchop(app_config_t *cfg, bool x_en, bool c_en, bool b_en)
{
    cfg->stealthchop = (x_en ? 0x01 : 0) |
                       (c_en ? 0x02 : 0) |
                       (b_en ? 0x04 : 0);
}

void app_config_unpack_invert_dir(const app_config_t *cfg, bool *x_inv, bool *c_inv, bool *b_inv)
{
    if (x_inv) *x_inv = (cfg->invert_dir & 0x01) != 0;
    if (c_inv) *c_inv = (cfg->invert_dir & 0x02) != 0;
    if (b_inv) *b_inv = (cfg->invert_dir & 0x04) != 0;
}

void app_config_pack_invert_dir(app_config_t *cfg, bool x_inv, bool c_inv, bool b_inv)
{
    cfg->invert_dir = (x_inv ? 0x01 : 0) |
                      (c_inv ? 0x02 : 0) |
                      (b_inv ? 0x04 : 0);
}

void app_config_unpack_continuous(const app_config_t *cfg, bool *x, bool *c, bool *b)
{
    if (x) *x = (cfg->continuous & 0x01) != 0;
    if (c) *c = (cfg->continuous & 0x02) != 0;
    if (b) *b = (cfg->continuous & 0x04) != 0;
}

void app_config_pack_continuous(app_config_t *cfg, bool x, bool c, bool b)
{
    cfg->continuous = (uint8_t)((x ? 0x01 : 0) | (c ? 0x02 : 0) | (b ? 0x04 : 0));
}
