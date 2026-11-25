/*
 * fixedpoint.c -- fixed-point unit conversion implementation
 *
 * Integer-only, zero dependencies: this file only includes "fixedpoint.h" (which only includes <stdint.h>).
 * No dynamic memory, no global mutable state, no recursion.
 */

#include "fixedpoint.h"

/* Pi is approximated as 355/113 (error 2.7e-7). On an 80 mm wheel diameter,
 * the circumference error is under 0.0001 mm -- three orders of magnitude below the wheel diameter tolerance.
 * A rational approximation keeps everything integer, with no library calls. */
#define FIXP_PI_NUM 355
#define FIXP_PI_DEN 113

#define FIXP_ONE_MILLION 1000000LL

/* Safety ceiling before multiplying: keeps the num side from overflowing int64 (see each function's comments). */
#define FIXP_MUL_GUARD 9000000000000LL /* 9e12 */

static int64_t fixp_div_round(int64_t num, int64_t den)
{
    if (den <= 0) {
        return 0;
    }
    if (num >= 0) {
        return (num + den / 2) / den;
    }
    return -((-num + den / 2) / den);
}

static int64_t fixp_clamp(int64_t v, int64_t lo, int64_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static int32_t fixp_to_i32(int64_t v)
{
    return (int32_t)fixp_clamp(v, -2147483648LL, 2147483647LL);
}

/* ------------------------------------------------------------------ */
/* Parameters                                                         */
/* ------------------------------------------------------------------ */

fixp_scale_t fixp_default_scale(void)
{
    fixp_scale_t s;
    s.counts_per_m      = FIXP_COUNTS_PER_M_NOMINAL;
    s.wheel_diameter_mm = FIXP_WHEEL_DIAMETER_MM_NOMINAL;
    s.period_ms         = FIXP_CONTROL_PERIOD_MS_NOMINAL;
    return s;
}

int fixp_scale_is_valid(const fixp_scale_t *s)
{
    if (s == 0) {
        return 0;
    }
    if (s->counts_per_m <= 0) {
        return 0;
    }
    if (s->wheel_diameter_mm <= 0) {
        return 0;
    }
    if (s->period_ms <= 0) {
        return 0;
    }
    return 1;
}

int32_t fixp_counts_per_m_from_params(int32_t counts_per_motor_rev,
                                      int32_t gear_ratio,
                                      int32_t wheel_diameter_mm)
{
    int64_t num;
    int64_t den;

    if (counts_per_motor_rev <= 0 || gear_ratio <= 0 || wheel_diameter_mm <= 0) {
        return 0;
    }
    /* counts/m = cpr x gear x 1000 / (pi x D)
     *          = cpr x gear x 1000 x 113 / (355 x D) */
    num = (int64_t)counts_per_motor_rev * (int64_t)gear_ratio * 1000LL
        * (int64_t)FIXP_PI_DEN;
    den = (int64_t)FIXP_PI_NUM * (int64_t)wheel_diameter_mm;
    if (den <= 0) {
        return 0;
    }
    return fixp_to_i32(fixp_div_round(num, den));
}

/* ------------------------------------------------------------------ */
/* Velocity conversion                                                */
/* ------------------------------------------------------------------ */

int32_t fixp_mm_s_to_counts(const fixp_scale_t *s, int32_t mm_s)
{
    int64_t num;
    int64_t den;

    if (s == 0 || s->counts_per_m <= 0 || s->period_ms <= 0) {
        return 0;
    }
    /* counts = mm_s x counts_per_m x period_ms / 1e6
     * |mm_s| <= 2^31, multiplier <= 2.1e5 => product <= 4.5e14, safe in int64 */
    num = (int64_t)mm_s * (int64_t)s->counts_per_m * (int64_t)s->period_ms;
    den = FIXP_ONE_MILLION;
    return fixp_to_i32(fixp_div_round(num, den));
}

int32_t fixp_counts_to_mm_s(const fixp_scale_t *s, int32_t counts)
{
    int64_t num;
    int64_t den;

    if (s == 0 || s->counts_per_m <= 0 || s->period_ms <= 0) {
        return 0;
    }
    /* mm_s = counts x 1e6 / (counts_per_m x period_ms) */
    num = (int64_t)counts * FIXP_ONE_MILLION;
    den = (int64_t)s->counts_per_m * (int64_t)s->period_ms;
    if (den <= 0) {
        return 0;
    }
    return fixp_to_i32(fixp_div_round(num, den));
}

int32_t fixp_counts_sum_to_mm_s(const fixp_scale_t *s, int32_t counts_sum,
                                int32_t periods)
{
    int64_t num;
    int64_t total_ms;
    int64_t den;

    if (s == 0 || s->counts_per_m <= 0 || s->period_ms <= 0) {
        return 0;
    }
    if (periods <= 0) {
        return 0;
    }
    /* counts_sum spans periods windows, so total time = period_ms x periods.
     * mm_s = counts_sum x 1e6 / (counts_per_m x period_ms x periods)
     * -- computed directly from the whole window instead of dividing by periods first, so no extra rounding step loses precision. */
    total_ms = (int64_t)s->period_ms * (int64_t)periods;
    if (total_ms <= 0) {
        return 0;
    }
    num = (int64_t)counts_sum * FIXP_ONE_MILLION;
    den = (int64_t)s->counts_per_m * total_ms;
    if (den <= 0) {
        return 0;
    }
    return fixp_to_i32(fixp_div_round(num, den));
}

/* ------------------------------------------------------------------ */
/* Distance conversion                                                */
/* ------------------------------------------------------------------ */

int64_t fixp_counts_to_mm(const fixp_scale_t *s, int64_t counts)
{
    int64_t c;

    if (s == 0 || s->counts_per_m <= 0) {
        return 0;
    }
    /* mm = counts x 1000 / counts_per_m; clamp counts first so the x1000 cannot overflow */
    c = fixp_clamp(counts, -FIXP_MUL_GUARD, FIXP_MUL_GUARD);
    return fixp_div_round(c * 1000LL, (int64_t)s->counts_per_m);
}

int64_t fixp_mm_to_counts(const fixp_scale_t *s, int64_t mm)
{
    int64_t m;

    if (s == 0 || s->counts_per_m <= 0) {
        return 0;
    }
    /* counts = mm x counts_per_m / 1000 */
    m = fixp_clamp(mm, -FIXP_MUL_GUARD, FIXP_MUL_GUARD);
    return fixp_div_round(m * (int64_t)s->counts_per_m, 1000LL);
}

/* ------------------------------------------------------------------ */
/* 1-D angular velocity conversion and self-check value               */
/* ------------------------------------------------------------------ */

int32_t fixp_wheel_mm_s_to_omega_mrad_s(int32_t wheel_mm_s, int32_t l_mm)
{
    if (l_mm <= 0) {
        return 0; /* no divide-by-zero */
    }
    /* omega[mrad/s] = v[mm/s] / L[mm] x 1000 */
    return fixp_to_i32(fixp_div_round((int64_t)wheel_mm_s * 1000LL, (int64_t)l_mm));
}

int64_t fixp_counts_per_wheel_turn(const fixp_scale_t *s)
{
    int64_t num;
    int64_t den;

    if (s == 0 || s->counts_per_m <= 0 || s->wheel_diameter_mm <= 0) {
        return 0;
    }
    /* counts per wheel turn = counts_per_m x pi x D / 1000
     *             = counts_per_m x 355 x D / (113 x 1000) */
    num = (int64_t)s->counts_per_m * (int64_t)FIXP_PI_NUM
        * (int64_t)s->wheel_diameter_mm;
    den = (int64_t)FIXP_PI_DEN * 1000LL;
    return fixp_div_round(num, den);
}
