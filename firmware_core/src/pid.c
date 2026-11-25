/*
 * pid.c -- velocity-loop PID implementation
 *
 * Integer-only, zero dependencies: this file only includes "pid.h" (which only includes <stdint.h>).
 * No dynamic memory, no global mutable state, no recursion, no floating point.
 *
 * The three-term formula and the three anti-windup measures are in the pid.h file
 * header (which explains why); this file only holds the implementation.
 */

#include "pid.h"

/* ------------------------------------------------------------------ */
/* Internal helpers                                                   */
/* ------------------------------------------------------------------ */

static int64_t iabs64(int64_t v)
{
    return (v < 0) ? -v : v;
}

static int64_t clamp_i64(int64_t v, int64_t lo, int64_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

/* ------------------------------------------------------------------ */
/* Defaults and state                                                 */
/* ------------------------------------------------------------------ */

pid_gains_t pid_default_gains(void)
{
    pid_gains_t g;
    g.kp_q16         = PID_Q16_ONE / 2;  /* 0.5  */
    g.ki_q16         = PID_Q16_ONE / 50; /* 0.02 */
    g.kd_q16         = 0;                /* derivative off by default: it amplifies encoder quantization noise */
    g.out_min        = -1000;
    g.out_max        = 1000;
    g.integ_sep_mm_s = 0;                /* no separation by default */
    g.integ_limit    = 1000;             /* integral term clamped separately, same order as output */
    return g;
}

void pid_reset(pid_state_t *st)
{
    if (st == 0) {
        return;
    }
    st->integ_raw = 0;
    st->prev_meas = 0;
    st->started   = (uint8_t)0;
}

int pid_gains_is_valid(const pid_gains_t *g)
{
    if (g == 0) {
        return 0;
    }
    if (g->out_min >= g->out_max) {
        return 0;
    }
    if (g->kp_q16 < 0 || g->ki_q16 < 0 || g->kd_q16 < 0) {
        return 0;
    }
    return 1;
}

/* ------------------------------------------------------------------ */
/* Main step                                                          */
/* ------------------------------------------------------------------ */

int32_t pid_step(const pid_gains_t *g, pid_state_t *st,
                 int32_t setpoint, int32_t meas, uint32_t *flags)
{
    int64_t e;
    int64_t p_term;
    int64_t d_term;
    int64_t i_term;
    int64_t out_raw;
    int64_t out;
    int64_t integ_bound = 0;
    int64_t integ_cand;
    int64_t out_lo;
    int64_t out_hi;
    int64_t i_limit_abs;
    uint8_t sep     = (uint8_t)0;
    uint8_t frozen  = (uint8_t)0;
    uint8_t clipped = (uint8_t)0; /* out_raw out of range and clipped (for anti-windup) */
    uint8_t sat     = (uint8_t)0; /* output sits on the clamp boundary (for the flag) */

    if (flags != 0) {
        *flags = 0u;
    }
    if (g == 0 || st == 0) {
        return 0;
    }

    out_lo = (int64_t)g->out_min;
    out_hi = (int64_t)g->out_max;
    e      = (int64_t)setpoint - (int64_t)meas;

    /* --- P --- */
    p_term = (int64_t)g->kp_q16 * e / (int64_t)PID_Q16_ONE;

    /* --- D: acts on the measurement, avoiding derivative spikes on setpoint steps --- */
    if (st->started != (uint8_t)0) {
        int64_t dmeas = (int64_t)meas - (int64_t)st->prev_meas;
        d_term = (int64_t)g->kd_q16 * (-dmeas) / (int64_t)PID_Q16_ONE;
    } else {
        d_term = 0;
    }

    /* --- integral separation test --- */
    if (g->integ_sep_mm_s > 0 && iabs64(e) > (int64_t)g->integ_sep_mm_s) {
        sep = (uint8_t)1;
    }

    /* --- integral accumulator bound, derived back from integ_limit --- */
    if (g->ki_q16 > 0) {
        if (g->integ_limit > 0) {
            i_limit_abs = (int64_t)g->integ_limit;
        } else if (out_hi > -out_lo) {
            i_limit_abs = out_hi;
        } else {
            i_limit_abs = -out_lo;
        }
        integ_bound = i_limit_abs * (int64_t)PID_Q16_ONE / (int64_t)g->ki_q16;
    }

    /* --- integral accumulator to commit --- */
    integ_cand = st->integ_raw;
    if (g->ki_q16 == 0) {
        /* integral off: keep the accumulator still, so a parked value cannot act later */
        sep = (uint8_t)0;
    } else if (sep == (uint8_t)0) {
        integ_cand = clamp_i64(st->integ_raw + e, -integ_bound, integ_bound);
    } else {
        /* no accumulation while separated, but clamp the old value back into range */
        integ_cand = clamp_i64(st->integ_raw, -integ_bound, integ_bound);
    }

    i_term  = (int64_t)g->ki_q16 * integ_cand / (int64_t)PID_Q16_ONE;
    out_raw = p_term + i_term + d_term;
    out     = clamp_i64(out_raw, out_lo, out_hi);
    clipped = (out != out_raw) ? (uint8_t)1 : (uint8_t)0;

    /* --- conditional integration (clamping anti-windup): roll back only when actually
           clipped AND the error still pushes deeper. Test "out_raw out of range", never
           "on the boundary", or a normal out_raw == out_max point counts as windup. --- */
    if (sep == (uint8_t)0 && clipped != (uint8_t)0) {
        uint8_t push_deeper = (uint8_t)0;
        if (e > 0 && out_raw > out_hi) {
            push_deeper = (uint8_t)1;
        } else if (e < 0 && out_raw < out_lo) {
            push_deeper = (uint8_t)1;
        }
        if (push_deeper != (uint8_t)0) {
            frozen     = (uint8_t)1;
            integ_cand = st->integ_raw;
            i_term     = (int64_t)g->ki_q16 * integ_cand / (int64_t)PID_Q16_ONE;
            out_raw    = p_term + i_term + d_term;
            out        = clamp_i64(out_raw, out_lo, out_hi);
        }
    }

    /* --- saturation flag: "output sits on the clamp boundary"; exact equality
           (out_raw == out_max) counts too -- more error cannot push it, which also
           matters for diagnostics (full PWM duty). Distinct from `clipped`: do not merge. --- */
    if (out >= out_hi || out <= out_lo) {
        sat = (uint8_t)1;
    }

    st->integ_raw = integ_cand;
    st->prev_meas = meas;
    st->started   = (uint8_t)1;

    if (flags != 0) {
        uint32_t f = 0u;
        if (sat != (uint8_t)0) {
            f |= PID_FLAG_SATURATED;
        }
        if (frozen != (uint8_t)0) {
            f |= PID_FLAG_INTEG_FROZEN;
        }
        if (sep != (uint8_t)0) {
            f |= PID_FLAG_INTEG_SEP;
        }
        *flags = f;
    }

    return (int32_t)out;
}
