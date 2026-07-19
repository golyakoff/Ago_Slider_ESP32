#include "motion_scenario.h"
#include "motion.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "SCEN_FOCUS";

// -----------------------------------------------------------------------------
// "Focus on subject": a timed pass along X that keeps a subject centred by turning C, with an
// optional linear B tilt.
//
// X and B are linear in time, so each is issued once as a single trapezoid — inherently
// smooth. C is the one axis that cannot be. For a subject at perpendicular distance D whose
// projection on the rail is x_s, the angle is atan((x_s - x) / D): the angular rate peaks
// opposite the subject and falls away towards the ends. Turning C at a constant rate would be
// right only at the two ends of the pass and would let the subject drift out of frame in
// between — around 0.8 degrees for a subject beside one end at a metre, a visible slide.
//
// C is therefore driven by VELOCITY, not by chasing a position. Repeatedly calling moveTo()
// with a fresh target would have the ramp generator planning a stop at every update, so the
// axis would brake and re-accelerate on every tick: a judder the camera would record. Instead
// C is set running continuously and only its speed is updated, which is what a curve is.
// Two terms make up that speed:
//   * feed-forward — the exact instantaneous rate omega = v_x * D / (D^2 + (x_s - x)^2),
//     evaluated from the axis's REAL current speed and position, so it needs no assumption
//     about where the ramp has got to and falls to zero by itself when X stops;
//   * a slow correction that bleeds off any accumulated angle error over seconds, far too
//     gently to be seen.
// The direction never reverses: atan is monotonic in x, so a monotonic X pass gives a
// monotonic C sweep even when the subject is opposite the middle of the travel.
//
// Nothing here uses the stored speed and acceleration settings — those belong to homing and
// to the manual moves on the Service tab. A scenario runs only on figures it computed itself.
//
// Payload after the shared duration field (18 bytes, little-endian):
//   0..3   int32  subject projection on the rail, X STEP pulses
//   4..7   uint32 perpendicular distance, X STEP pulses; 0 = infinity, C is left alone
//   8..11  int32  signed X travel, STEP pulses
//   12..15 int32  signed B travel, STEP pulses
//   16     int8   C tracking direction, +1 or -1, from the client's two-point aiming
//   17     uint8  reserved, must be 0
// -----------------------------------------------------------------------------

#define FOCUS_PAYLOAD_LEN     18
#define FOCUS_RAMP_FRACTION   0.05f
#define FOCUS_RAMP_MIN_S      0.2f
#define FOCUS_RAMP_MAX_S      2.0f
// Time constant over which a residual angle error is corrected. Long on purpose: the
// correction must never be fast enough to see as a twitch.
#define FOCUS_CORRECTION_TAU_S 3.0f
// Ceiling on C's commanded speed, as a multiple of the peak feed-forward rate
#define FOCUS_C_SPEED_CEILING  3.0f

static struct {
    int32_t subject_pos;
    uint32_t distance;
    int32_t x_travel;
    int32_t b_travel;
    int8_t c_sign;
    int32_t x_start;
    int32_t c_start;
    float theta_start;
    float c_pulses_per_rad;
    uint32_t duration_ms;
    bool track_c;
    bool c_forward;
    uint32_t c_speed_ceiling_mhz;
} s_focus;

/** Angle from the rail's perpendicular to the subject when the carriage sits at x. */
static inline float focus_theta(int32_t x)
{
    return atan2f((float)(s_focus.subject_pos - x), (float)s_focus.distance);
}

/** Where C should be standing, in pulses, for the carriage to be aimed at the subject. */
static inline int32_t focus_c_target(int32_t x)
{
    float dtheta = focus_theta(x) - s_focus.theta_start;
    return s_focus.c_start + (int32_t)(s_focus.c_sign * dtheta * s_focus.c_pulses_per_rad);
}

static motion_scenario_reason_t focus_start(const uint8_t *payload, size_t len, uint32_t *total_ms)
{
    // The shared duration field sits in front of the pattern's own parameters
    if (len < FOCUS_PAYLOAD_LEN + 4) return MOTION_SCENARIO_REASON_BAD_PARAMS;

    uint32_t duration_ms;
    memcpy(&duration_ms, &payload[0], 4);
    const uint8_t *p = &payload[4];

    memcpy(&s_focus.subject_pos, &p[0], 4);
    memcpy(&s_focus.distance, &p[4], 4);
    memcpy(&s_focus.x_travel, &p[8], 4);
    memcpy(&s_focus.b_travel, &p[12], 4);
    s_focus.c_sign = ((int8_t)p[16] < 0) ? -1 : 1;
    s_focus.duration_ms = duration_ms;

    if (duration_ms == 0 || (s_focus.x_travel == 0 && s_focus.b_travel == 0)) {
        return MOTION_SCENARIO_REASON_BAD_PARAMS;
    }

    s_focus.track_c = s_focus.distance > 0 && s_focus.x_travel != 0;

    // Tracking is only meaningful while the axes it steers by are still anchored
    bool vx = false, vc = false, vb = false;
    motion_get_home_valid(&vx, &vc, &vb);
    if (!vx || (s_focus.track_c && !vc) || (s_focus.b_travel != 0 && !vb)) {
        return MOTION_SCENARIO_REASON_NOT_HOMED;
    }

    int32_t x_now = 0, c_now = 0, b_now = 0;
    motion_get_positions(&x_now, &c_now, &b_now);
    s_focus.x_start = x_now;
    s_focus.c_start = c_now;
    s_focus.theta_start = s_focus.track_c ? focus_theta(x_now) : 0.0f;

    float c_units_per_pulse = motion_units_per_pulse(1);
    s_focus.c_pulses_per_rad = (c_units_per_pulse > 0.0f)
        ? (57.2957795f / c_units_per_pulse) : 0.0f;
    if (s_focus.track_c && s_focus.c_pulses_per_rad <= 0.0f) {
        return MOTION_SCENARIO_REASON_BAD_PARAMS;
    }

    float total_s = duration_ms / 1000.0f;

    // One ramp time shared by the axes keeps their velocity ratio constant while they
    // accelerate, so the framing holds through the ramps and not only at cruise. A trapezoid
    // of height v over T with ramps t_r covers v*(T - t_r), which is the speed that lands
    // each axis on its target at exactly T.
    float ramp_s = total_s * FOCUS_RAMP_FRACTION;
    if (ramp_s < FOCUS_RAMP_MIN_S) ramp_s = FOCUS_RAMP_MIN_S;
    if (ramp_s > FOCUS_RAMP_MAX_S) ramp_s = FOCUS_RAMP_MAX_S;
    if (ramp_s > total_s / 2.0f) ramp_s = total_s / 2.0f;
    float cruise_s = total_s - ramp_s;
    if (cruise_s < 0.1f) cruise_s = 0.1f;

    int32_t steps[3] = {s_focus.x_travel, 0, s_focus.b_travel};
    uint32_t speed[3] = {0, 0, 0};
    uint32_t accel[3] = {0, 0, 0};
    for (int i = 0; i < 3; i += 2) {
        if (steps[i] == 0) continue;
        float v = fabsf((float)steps[i]) / cruise_s;
        // Never hand down a zero: motion_move_relative_ex reads that as "use the configured
        // default", and a scenario must never inherit the settings meant for manual moves
        uint32_t mhz = (uint32_t)(v * 1000.0f);
        speed[i] = mhz > 0 ? mhz : 1;
        uint32_t a = (uint32_t)(v / ramp_s);
        accel[i] = a > 0 ? a : 1;
    }

    if (s_focus.track_c) {
        // atan is monotonic, so the sweep direction is fixed for the whole pass and can be
        // decided once, here, from the total angle change
        int32_t c_end_target = focus_c_target(x_now + s_focus.x_travel);
        s_focus.c_forward = (c_end_target >= c_now);

        // Peak rate happens at the closest approach, i.e. opposite the subject
        float v_x = fabsf((float)s_focus.x_travel) / cruise_s;
        float omega_peak = v_x / (float)s_focus.distance;
        float c_peak_pps = omega_peak * s_focus.c_pulses_per_rad;
        s_focus.c_speed_ceiling_mhz = (uint32_t)(c_peak_pps * 1000.0f * FOCUS_C_SPEED_CEILING) + 1;

        // Gentle enough that every later speed change is a slow glide, not a step
        uint32_t c_accel = (uint32_t)(c_peak_pps / ramp_s) + 1;
        motion_axis_run(1, s_focus.c_forward, c_accel);
        motion_axis_set_speed(1, 1);   // essentially still; the first tick sets the real rate
    }

    ESP_LOGI(TAG, "x_travel=%ld b_travel=%ld subject=%ld d=%lu sign=%d over %lu ms",
             (long)s_focus.x_travel, (long)s_focus.b_travel, (long)s_focus.subject_pos,
             (unsigned long)s_focus.distance, (int)s_focus.c_sign,
             (unsigned long)duration_ms);

    motion_move_relative_ex(steps, speed, accel);
    *total_ms = duration_ms;
    return MOTION_SCENARIO_REASON_NONE;
}

static bool focus_tick(uint32_t elapsed_ms)
{
    if (s_focus.track_c) {
        int32_t x_now = 0, c_now = 0;
        motion_get_positions(&x_now, &c_now, NULL);

        // Feed-forward from what X is REALLY doing, so the C rate follows X through its own
        // acceleration ramp and settles to zero by itself when X finishes
        float v_x = fabsf((float)motion_axis_current_speed_mhz(0)) / 1000.0f;   // pulses/s
        float dx = (float)(s_focus.subject_pos - x_now);
        float d = (float)s_focus.distance;
        float omega = v_x * d / (d * d + dx * dx);              // rad/s
        float c_pps = omega * s_focus.c_pulses_per_rad;

        // Bleed off any residual angle error slowly enough to be invisible. The target
        // already carries c_sign, so the only conversion still needed is into "ahead of /
        // behind the direction of travel" — applying c_sign a second time here would flip
        // the correction on one of the two mounting directions and drive the commanded
        // speed negative, where the clamp below would park the axis for the rest of the run.
        int32_t c_target = focus_c_target(x_now);
        float dir = s_focus.c_forward ? 1.0f : -1.0f;
        float error = (float)(c_target - c_now) * dir;
        c_pps += error / FOCUS_CORRECTION_TAU_S;

        if (c_pps < 0.0f) c_pps = 0.0f;                          // the sweep never reverses
        uint32_t c_mhz = (uint32_t)(c_pps * 1000.0f);
        if (c_mhz > s_focus.c_speed_ceiling_mhz) c_mhz = s_focus.c_speed_ceiling_mhz;
        if (c_mhz < 1) c_mhz = 1;
        motion_axis_set_speed(1, c_mhz);
    }

    // The pass is over when the clock has run out and the timed axes have actually landed
    bool done = elapsed_ms >= s_focus.duration_ms &&
                !motion_axis_is_running(0) && !motion_axis_is_running(2);
    if (done && s_focus.track_c) motion_axis_stop_smooth(1);
    return done;
}

static void focus_stop(void)
{
    if (s_focus.track_c) motion_axis_stop_smooth(1);
    ESP_LOGI(TAG, "Focus pass over");
}

static const motion_scenario_def_t s_focus_def = {
    .id = MOTION_SCENARIO_ID_FOCUS,
    .name = "focus",
    .start = focus_start,
    .tick = focus_tick,
    .stop = focus_stop,
};

const motion_scenario_def_t *motion_scenario_focus_def(void)
{
    return &s_focus_def;
}
