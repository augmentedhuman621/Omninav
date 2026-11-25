/*
 * pid.h — velocity-loop PID (pure integer, Q16 gains)
 * ---------------------------------------------------------------------------
 *
 * Design constraints (consistent with the iron rules of this project's firmware_core)
 *   - Depends only on <stdint.h>. No HAL / ROS2 / third-party headers, no <stdio.h> / <math.h>.
 *   - All-integer, zero floating-point. Gains are Q16 (the stored integer = real value × 65536).
 *   - No dynamic memory, no global mutable state, no recursion.
 *   - All state lives in the caller-provided pid_state_t — four wheels are four instances,
 *     so module-level globals are neither needed nor allowed.
 *
 * Unit conventions (unified across modules, see STM32_README.md §5)
 *   setpoint / feedback : mm/s (int32_t)
 *   output              : "PWM units", symmetric range usually -1000 ~ +1000 (int32_t)
 *   error e = setpoint - meas, in mm/s
 *
 * Why Q16 gains instead of integer ratios
 *   Velocity-loop kp is typically 0.1 ~ 2.0. Integer ratios cannot express a value like 0.35;
 *   with Q16, 0.35 is 22938 and all arithmetic runs on int64 intermediates: exact, no overflow.
 *
 * How each of the three terms is handled (every choice is deliberate, not a default)
 *   P : p_term   = (kp_q16 * e) >> 16
 *   I : the accumulator stores the raw sum of errors Σe (unit mm/s·tick), not a value already
 *       multiplied by ki — this way changing ki does not poison the history, and a small ki
 *       is not lost to rounding. The integral term i_term = (ki_q16 * integ_raw) >> 16, clamped separately.
 *   D : ★ the derivative acts on the measurement, not on the error (derivative-on-measurement):
 *       d_term = (kd_q16 * (-(meas - prev_meas))) >> 16
 *       this way a step in the setpoint produces no derivative kick.
 *       The cost is slightly less lead during setpoint ramps; in a velocity loop that trade-off is standard.
 *
 * Anti-windup (three layers, all of them required)
 *   1) The integral term is clamped on its own (integ_limit) — stops the integral term from pinning the output.
 *   2) Integral separation: suspend integration while |e| > integ_sep_mm_s — during large errors (start-up,
 *      steps) integration should not be running anyway; this is also the cheapest anti-overshoot tool.
 *   3) Conditional integration (clamping): if this step's output is saturated and the error still pushes
 *      deeper into saturation, **roll back** this step's integral accumulation.
 *      — without layer 3, a command persistently beyond the physical range winds the integral up to the sky,
 *        and when the command reverses the robot must first "wait for the integral to unwind", i.e. it looks unresponsive.
 *
 * Saturation flags
 *   pid_step() reports what happened this step through flags, for logging / PWM-enable decisions upstream:
 *     PID_FLAG_SATURATED   the output is **pinned at the clamp boundary** (out >= out_max or
 *                          out <= out_min). The semantics are "at the rail", not
 *                          "clipped this step" — for the layer above, landing exactly on the boundary
 *                          (out_raw == out_max) and genuinely exceeding it both mean
 *                          "no larger error can push it further", which is exactly the information
 *                          needed for full-duty PWM and `A,<text>` reporting.
 *                          Note: anti-windup (layer 3 below) uses the stricter
 *                          "out_raw genuinely out of range" test; the two are not the same thing.
 *     PID_FLAG_INTEG_FROZEN the integral was rolled back this step (conditional integration fired)
 *     PID_FLAG_INTEG_SEP   integration was suspended this step by integral separation
 *   flags may be NULL.
 *
 * Overflow and value ranges (int64 intermediates throughout)
 *   |e| <= 2^20 (about 1 million mm/s, far beyond anything real)
 *   |integ_raw| is clamped to integ_bound, see pid.c
 *   |out| <= max(|out_min|, |out_max|)
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Q16 representation of 1.0 */
#define PID_Q16_ONE 65536

/* Flags reported by pid_step() */
#define PID_FLAG_SATURATED    0x01u
#define PID_FLAG_INTEG_FROZEN 0x02u
#define PID_FLAG_INTEG_SEP    0x04u

/* Gain and clamp configuration. Pure "data": calibration changes numbers only, never the algorithm. */
typedef struct {
    int32_t kp_q16;          /* proportional gain, Q16 */
    int32_t ki_q16;          /* integral gain, Q16; 0 = integration off */
    int32_t kd_q16;          /* derivative gain, Q16; 0 = derivative off */
    int32_t out_min;         /* output lower clamp (usually -1000). Must be < out_max */
    int32_t out_max;         /* output upper clamp (usually +1000) */
    int32_t integ_sep_mm_s;  /* integral separation: suspend integration while |e| is above it; <=0 = off */
    int32_t integ_limit;     /* absolute clamp on the integral term; <=0 = derive from out_max/-out_min */
} pid_gains_t;

/* Run-time state of one control channel. Four wheels = four independent instances. */
typedef struct {
    int64_t integ_raw;  /* Σe, unit mm/s·tick */
    int32_t prev_meas;  /* previous feedback value, used by the derivative term */
    uint8_t started;    /* 0 = first step not taken yet (derivative term treated as 0) */
} pid_state_t;

/* Default gains: kp = 0.5, ki = 1/50 = 0.02, kd = 0, output ±1000.
 * !! These are only a starting point to get the firmware running; they must be tuned on the real robot. */
pid_gains_t pid_default_gains(void);

/* Clear the state. st may be 0 (no-op). */
void pid_reset(pid_state_t *st);

/*
 * Parameter validity: out_min < out_max, all three gains non-negative, and
 * integ_sep_mm_s / integ_limit may be 0 or negative (meaning off).
 * Returns 1 if valid.
 */
int pid_gains_is_valid(const pid_gains_t *g);

/*
 * Run one step. Returns the clamped output.
 *   Returns 0 when g / st is 0 (no crash, nothing written).
 *   flags may be 0.
 *   setpoint / meas are in mm/s.
 *
 * This function is a pure state machine: same input sequence → same output sequence, independent of call time.
 */
int32_t pid_step(const pid_gains_t *g, pid_state_t *st,
                 int32_t setpoint, int32_t meas, uint32_t *flags);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
