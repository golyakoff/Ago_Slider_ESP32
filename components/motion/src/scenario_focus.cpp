#include "motion_scenario.h"
#include "motion.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "SCEN_FOCUS";

// -----------------------------------------------------------------------------
// "Focus on subject": a timed pass along X that keeps a subject framed by turning C (pan) and
// B (tilt) at a fixed 3D point.
//
// X is linear in time and gets one trapezoid. Neither C nor B can be. The subject sits at a
// point in space; as the dolly travels, the angle to it is an arctangent, so both the pan and
// the tilt rate peak at the closest-approach point and fall off towards the ends. Interpolating
// either linearly between the endpoints would be right only at the ends and let the subject
// drift out of frame in between — for tilt this is the visible mid-pass vertical slide.
//
// So both C and B are driven by VELOCITY, not by chasing a position — a repeatedly updated
// moveTo() would have the ramp generator plan a stop at every tick, a judder the camera would
// record. Each axis runs continuously and only its speed is updated, which is what a curve is.
// The speed is the exact instantaneous rate (evaluated from X's REAL speed, so it follows X
// through its own ramps and settles to zero when X stops) plus a slow correction for residual
// error.
//
//   pan angle  = atan((x_s - x) / D)                 [horizontal]
//   r(x)       = sqrt(D^2 + (x_s - x)^2)             [ground distance to the subject]
//   tilt angle = atan(dz / r(x))                     [vertical]
//
// The difference between the two: the pan sweep is monotonic in x, but the tilt is symmetric
// about x_s and PEAKS there, so B reverses direction at closest approach. Its tracking must
// therefore run with a signed speed that passes through zero, where C's clamps to one way.
//
// Everything is measured relative to where the axes stand at the start, which is why the client
// never has to say which angle points square at the rail.
//
// Payload after the shared duration field (18 bytes, little-endian):
//   0..3   int32  subject projection on the rail, X STEP pulses
//   4..7   uint32 perpendicular distance, X STEP pulses; 0 = infinity, C is left alone
//   8..11  int32  signed X travel, STEP pulses
//   12..15 int32  vertical offset of the subject, in X STEP-equivalent pulses (so it shares a
//                  scale with r); signed, 0 = no tilt tracking, B is left alone
//   16     int8   C tracking direction, +1 or -1, from the client's three-point aiming
//   17     uint8  reserved, must be 0
// -----------------------------------------------------------------------------

#define FOCUS_PAYLOAD_LEN     18
#define FOCUS_RAMP_FRACTION   0.05f
#define FOCUS_RAMP_MIN_S      0.2f
#define FOCUS_RAMP_MAX_S      2.0f
// Time constant over which a residual angle error is corrected. Long during the body of the
// pass so the correction is never fast enough to see as a twitch; it tightens towards the end
// (see the tick) so the small residual left by X's deceleration is closed before X stops,
// rather than being frozen in by the stop.
#define FOCUS_CORRECTION_TAU_S     3.0f
#define FOCUS_CORRECTION_TAU_MIN_S 1.0f
// How quickly a tracked axis reaches its commanded speed. Short on purpose: the tracking is
// closed-loop on X's REAL speed, so the commanded speed already follows X through its ramps —
// a brisk acceleration only tightens how closely the axis holds that command, which is what
// keeps the actual speed from lagging the command as X decelerates and stops.
#define FOCUS_TRACK_ACCEL_TIME_S   0.3f
// Ceiling on a tracked axis's commanded speed, as a multiple of its peak feed-forward rate
#define FOCUS_SPEED_CEILING   3.0f

static struct {
    int32_t subject_pos;
    uint32_t distance;
    int32_t x_travel;
    float vert_offset;         // X-step-equivalent pulses, signed; 0 = no tilt tracking
    int8_t c_sign;
    int32_t x_start;
    // C (pan)
    bool track_c;
    int32_t c_start;
    float theta_start;
    float c_pulses_per_rad;
    bool c_forward;
    uint32_t c_speed_ceiling_mhz;
    // B (tilt)
    bool track_b;
    int32_t b_start;
    float b_tilt_start;
    float b_pulses_per_rad;
    bool b_forward;
    uint32_t b_accel;
    uint32_t b_speed_ceiling_mhz;
    // shared
    uint32_t duration_ms;
    float v_x_cruise;          // X pulses/s at cruise, for the feed-forward ceilings
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

/** Ground distance from the carriage at x to the subject, in X pulses. */
static inline float focus_r(int32_t x)
{
    float dx = (float)(s_focus.subject_pos - x);
    return sqrtf((float)s_focus.distance * (float)s_focus.distance + dx * dx);
}

/** Tilt angle (radians) to the subject when the carriage sits at x. */
static inline float focus_tilt(int32_t x)
{
    return atan2f(s_focus.vert_offset, focus_r(x));
}

/** Where B should be standing, in pulses, for the camera to be aimed at the subject's height. */
static inline int32_t focus_b_target(int32_t x)
{
    return s_focus.b_start +
        (int32_t)((focus_tilt(x) - s_focus.b_tilt_start) * s_focus.b_pulses_per_rad);
}

/** d(tilt)/dx in radians per pulse — signed, zero at closest approach. */
static inline float focus_dtilt_dx(int32_t x)
{
    float r = focus_r(x);
    float dz = s_focus.vert_offset;
    return dz * (float)(s_focus.subject_pos - x) / (r * (r * r + dz * dz));
}

static motion_scenario_reason_t focus_start(const uint8_t *payload, size_t len, uint32_t *total_ms)
{
    // The shared duration field sits in front of the pattern's own parameters
    if (len < FOCUS_PAYLOAD_LEN + 4) return MOTION_SCENARIO_REASON_BAD_PARAMS;

    uint32_t duration_ms;
    memcpy(&duration_ms, &payload[0], 4);
    const uint8_t *p = &payload[4];

    int32_t vert_offset_i = 0;
    memcpy(&s_focus.subject_pos, &p[0], 4);
    memcpy(&s_focus.distance, &p[4], 4);
    memcpy(&s_focus.x_travel, &p[8], 4);
    memcpy(&vert_offset_i, &p[12], 4);
    s_focus.vert_offset = (float)vert_offset_i;
    s_focus.c_sign = ((int8_t)p[16] < 0) ? -1 : 1;
    s_focus.duration_ms = duration_ms;

    // A pass is a travel along X; C and B both track that travel, so without it there is nothing
    if (duration_ms == 0 || s_focus.x_travel == 0) {
        return MOTION_SCENARIO_REASON_BAD_PARAMS;
    }

    s_focus.track_c = s_focus.distance > 0;
    s_focus.track_b = vert_offset_i != 0;

    // Tracking is only meaningful while the axes it steers by are still anchored
    bool vx = false, vc = false, vb = false;
    motion_get_home_valid(&vx, &vc, &vb);
    if (!vx || (s_focus.track_c && !vc) || (s_focus.track_b && !vb)) {
        return MOTION_SCENARIO_REASON_NOT_HOMED;
    }

    int32_t x_now = 0, c_now = 0, b_now = 0;
    motion_get_positions(&x_now, &c_now, &b_now);
    s_focus.x_start = x_now;
    s_focus.c_start = c_now;
    s_focus.b_start = b_now;
    s_focus.theta_start = s_focus.track_c ? focus_theta(x_now) : 0.0f;
    s_focus.b_tilt_start = s_focus.track_b ? focus_tilt(x_now) : 0.0f;

    float c_units_per_pulse = motion_units_per_pulse(1);
    s_focus.c_pulses_per_rad = (c_units_per_pulse > 0.0f) ? (57.2957795f / c_units_per_pulse) : 0.0f;
    float b_units_per_pulse = motion_units_per_pulse(2);
    s_focus.b_pulses_per_rad = (b_units_per_pulse > 0.0f) ? (57.2957795f / b_units_per_pulse) : 0.0f;
    if (s_focus.track_c && s_focus.c_pulses_per_rad <= 0.0f) return MOTION_SCENARIO_REASON_BAD_PARAMS;
    if (s_focus.track_b && s_focus.b_pulses_per_rad <= 0.0f) return MOTION_SCENARIO_REASON_BAD_PARAMS;

    float total_s = duration_ms / 1000.0f;

    // One ramp time shared by the axes keeps their velocity ratio constant while they
    // accelerate, so the framing holds through the ramps and not only at cruise. A trapezoid
    // of height v over T with ramps t_r covers v*(T - t_r), which is the speed that lands
    // the axis on its target at exactly T.
    float ramp_s = total_s * FOCUS_RAMP_FRACTION;
    if (ramp_s < FOCUS_RAMP_MIN_S) ramp_s = FOCUS_RAMP_MIN_S;
    if (ramp_s > FOCUS_RAMP_MAX_S) ramp_s = FOCUS_RAMP_MAX_S;
    if (ramp_s > total_s / 2.0f) ramp_s = total_s / 2.0f;
    float cruise_s = total_s - ramp_s;
    if (cruise_s < 0.1f) cruise_s = 0.1f;
    s_focus.v_x_cruise = fabsf((float)s_focus.x_travel) / cruise_s;

    // Only X is a timed trapezoid; C and B run continuously and are steered by speed
    int32_t steps[3] = {s_focus.x_travel, 0, 0};
    uint32_t speed[3] = {0, 0, 0};
    uint32_t accel[3] = {0, 0, 0};
    {
        float v = s_focus.v_x_cruise;
        uint32_t mhz = (uint32_t)(v * 1000.0f);
        speed[0] = mhz > 0 ? mhz : 1;   // 0 would be read as "use the configured default"
        uint32_t a = (uint32_t)(v / ramp_s);
        accel[0] = a > 0 ? a : 1;
    }

    if (s_focus.track_c) {
        // atan is monotonic, so the C sweep direction is fixed for the whole pass
        int32_t c_end_target = focus_c_target(x_now + s_focus.x_travel);
        s_focus.c_forward = (c_end_target >= c_now);
        float omega_peak = s_focus.v_x_cruise / (float)s_focus.distance;   // rad/s at closest approach
        float c_peak_pps = omega_peak * s_focus.c_pulses_per_rad;
        s_focus.c_speed_ceiling_mhz = (uint32_t)(c_peak_pps * 1000.0f * FOCUS_SPEED_CEILING) + 1;
        uint32_t c_accel = (uint32_t)(c_peak_pps / FOCUS_TRACK_ACCEL_TIME_S) + 1;
        motion_axis_run(1, s_focus.c_forward, c_accel);
        motion_axis_set_speed(1, 1);   // essentially still; the first tick sets the real rate
    }

    if (s_focus.track_b) {
        // The tilt rate has no closed-form peak, so scan the pass for the largest |d tilt/dx|.
        // It is zero at closest approach (tilt at its extremum) and on the shoulders either
        // side, so a coarse scan finds it well enough for a speed cap.
        float b_peak_dtilt = 0.0f;
        for (int k = 0; k <= 10; k++) {
            int32_t x = x_now + (int32_t)((float)s_focus.x_travel * (float)k / 10.0f);
            float d = fabsf(focus_dtilt_dx(x));
            if (d > b_peak_dtilt) b_peak_dtilt = d;
        }
        float b_peak_pps = b_peak_dtilt * s_focus.v_x_cruise * s_focus.b_pulses_per_rad;
        s_focus.b_speed_ceiling_mhz = (uint32_t)(b_peak_pps * 1000.0f * FOCUS_SPEED_CEILING) + 1;
        s_focus.b_accel = (uint32_t)(b_peak_pps / FOCUS_TRACK_ACCEL_TIME_S) + 1;
        // Initial direction: sign of d tilt/dx times the sign of the X travel
        float dtilt0 = focus_dtilt_dx(x_now);
        s_focus.b_forward = (dtilt0 * (float)s_focus.x_travel) >= 0.0f;
        motion_axis_run(2, s_focus.b_forward, s_focus.b_accel);
        motion_axis_set_speed(2, 1);
    }

    ESP_LOGI(TAG, "x_travel=%ld vert=%ld subject=%ld d=%lu c_sign=%d track_c=%d track_b=%d over %lu ms",
             (long)s_focus.x_travel, (long)vert_offset_i, (long)s_focus.subject_pos,
             (unsigned long)s_focus.distance, (int)s_focus.c_sign,
             (int)s_focus.track_c, (int)s_focus.track_b, (unsigned long)duration_ms);

    motion_move_relative_ex(steps, speed, accel);
    *total_ms = duration_ms;
    return MOTION_SCENARIO_REASON_NONE;
}

static bool focus_tick(uint32_t elapsed_ms)
{
    int32_t x_now = 0, c_now = 0, b_now = 0;
    motion_get_positions(&x_now, &c_now, &b_now);
    float v_x = (float)motion_axis_current_speed_mhz(0) / 1000.0f;   // pulses/s, signed

    // The correction is gentle through the body of the pass but tightens as it ends: X's
    // deceleration leaves the tracked axes a little behind (their actual speed lags the
    // shrinking commanded speed), and a fixed 3 s time constant is too slow to close that gap
    // in the ~1 s of deceleration. Scaling the time constant down with the time remaining
    // closes it smoothly before X stops, so the residual is not frozen in by the stop.
    float remaining = s_focus.duration_ms > 0
        ? (float)(s_focus.duration_ms - (elapsed_ms < s_focus.duration_ms ? elapsed_ms : s_focus.duration_ms))
          / (float)s_focus.duration_ms
        : 0.0f;
    float tau = FOCUS_CORRECTION_TAU_S * remaining;
    if (tau < FOCUS_CORRECTION_TAU_MIN_S) tau = FOCUS_CORRECTION_TAU_MIN_S;

    if (s_focus.track_c) {
        // Feed-forward from what X is REALLY doing, so the C rate follows X through its own
        // acceleration ramp and settles to zero by itself when X finishes
        float dx = (float)(s_focus.subject_pos - x_now);
        float d = (float)s_focus.distance;
        float omega = fabsf(v_x) * d / (d * d + dx * dx);           // rad/s
        float c_pps = omega * s_focus.c_pulses_per_rad;

        // Bleed off any residual angle error slowly enough to be invisible. The target already
        // carries c_sign, so the only conversion still needed is into "ahead of / behind the
        // direction of travel".
        int32_t c_target = focus_c_target(x_now);
        float dir = s_focus.c_forward ? 1.0f : -1.0f;
        c_pps += (float)(c_target - c_now) * dir / tau;

        if (c_pps < 0.0f) c_pps = 0.0f;                             // the pan sweep never reverses
        uint32_t c_mhz = (uint32_t)(c_pps * 1000.0f);
        if (c_mhz > s_focus.c_speed_ceiling_mhz) c_mhz = s_focus.c_speed_ceiling_mhz;
        if (c_mhz < 1) c_mhz = 1;
        motion_axis_set_speed(1, c_mhz);
    }

    if (s_focus.track_b) {
        // Feed-forward straight off the analytic derivative, signed, so it naturally reverses
        // at closest approach where the tilt is at its extremum
        float dB_dt = focus_dtilt_dx(x_now) * v_x * s_focus.b_pulses_per_rad;   // pulses/s, signed
        float error = (float)(focus_b_target(x_now) - b_now);
        float b_pps = dB_dt + error / tau;

        // Unlike the pan, the tilt can go either way, so the direction is followed rather than
        // clamped: reverse the run only on a sign change, where the speed is near zero anyway
        bool want_forward = b_pps >= 0.0f;
        if (want_forward != s_focus.b_forward) {
            s_focus.b_forward = want_forward;
            motion_axis_run(2, want_forward, s_focus.b_accel);
        }
        uint32_t b_mhz = (uint32_t)(fabsf(b_pps) * 1000.0f);
        if (b_mhz > s_focus.b_speed_ceiling_mhz) b_mhz = s_focus.b_speed_ceiling_mhz;
        if (b_mhz < 1) b_mhz = 1;
        motion_axis_set_speed(2, b_mhz);
    }

    // The pass is over when the clock has run out and X, the one timed axis, has landed
    bool done = elapsed_ms >= s_focus.duration_ms && !motion_axis_is_running(0);
    if (done) {
        if (s_focus.track_c) motion_axis_stop_smooth(1);
        if (s_focus.track_b) motion_axis_stop_smooth(2);
    }
    return done;
}

static void focus_stop(void)
{
    if (s_focus.track_c) motion_axis_stop_smooth(1);
    if (s_focus.track_b) motion_axis_stop_smooth(2);
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
