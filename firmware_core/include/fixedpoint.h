/*
 * fixedpoint.h — fixed-point unit conversion (encoder counts <-> mm / mm per second)
 * ---------------------------------------------------------------------------
 *
 * Design constraints (consistent with the firmware_core iron rules)
 *   - Only includes <stdint.h>. All-integer, zero floating-point, no dynamic memory, no global mutable state.
 *
 * ★★ The **single source of truth for the encoder-resolution number** in this project is this file
 *    (contract §6: never write a second copy anywhere else).
 *    Anything else that needs this number must call fixp_default_scale(), never a hand-written constant.
 *
 * How these numbers were derived (the tests print the detailed arithmetic)
 *   Motor: 11 PPR Hall encoder, quadrature-decoded (quadrature A/B phases) to 4x:
 *     11 × 4 = 44 counts / motor revolution
 *   Gearbox 1:60 (the variant actually purchased, see 03_chassis_bom.md) →
 *     44 × 60 = 2640 counts / wheel revolution
 *   Wheel diameter 80 mm → circumference 251.3274 mm
 *     2640 ÷ 0.2513274 = 10504.2 → **the project nominal value is 10505** (+0.008%,
 *     far below the 7% gear-ratio ambiguity below, and both numbers need on-vehicle calibration anyway)
 *
 *   ⚠️ Ambiguity warning: the seller labels the same gearbox "1:56" based on no-load speed.
 *      If on-vehicle calibration confirms 1:56, then
 *        44 × 56 = 2464 counts / wheel revolution → 9806 counts / metre
 *      The two differ by 7%. So **on-vehicle calibration is mandatory**; the constants here are only nominal.
 *      To change it, edit only the FIXP_COUNTS_PER_M_NOMINAL line here.
 *
 * Unit conventions
 *   Linear velocity mm/s, angular velocity mrad/s, distance mm, control period ms.
 */

#ifndef FIXEDPOINT_H
#define FIXEDPOINT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Nominal value: 1:60 variant (actually purchased). ★ The one and only definition in the project. */
#define FIXP_COUNTS_PER_M_NOMINAL 10505

/* If on-vehicle calibration confirms the 1:56 gearbox, uncomment this and remove the line above (pick one, never both)
 * #define FIXP_COUNTS_PER_M_NOMINAL 9806
 */

#define FIXP_WHEEL_DIAMETER_MM_NOMINAL 80
#define FIXP_CONTROL_PERIOD_MS_NOMINAL 20  /* 50 Hz speed feedback inside a 100 Hz control loop */
#define FIXP_ENCODER_PPR_NOMINAL 11
#define FIXP_GEAR_RATIO_NOMINAL 60
#define FIXP_QUADRATURE_FACTOR 4

/* Conversion parameters. Pure data: calibration only changes numbers. */
typedef struct {
    int32_t counts_per_m;    /* encoder counts per metre, must be >= 1 */
    int32_t wheel_diameter_mm;
    int32_t period_ms;       /* speed-measurement window, ms, must be >= 1 */
} fixp_scale_t;

/* Default parameters: built from the nominal constants above. */
fixp_scale_t fixp_default_scale(void);

/* Parameter validity: all three fields must be >= 1. Returns 1 if valid. */
int fixp_scale_is_valid(const fixp_scale_t *s);

/*
 * Derive counts/m from counts per motor revolution (PPR × quadrature) and the gear ratio.
 *   counts_per_m = ppr_x_quadrature × gear_ratio × 1000 / (π × wheel diameter)
 * No library function is used here: pi is approximated by 355/113 (error 2.7e-7,
 * which is under 0.0001 mm across 251.327 mm).
 * Returns 0 if the arguments are invalid (zero denominator etc.); the caller should keep the old value.
 */
int32_t fixp_counts_per_m_from_params(int32_t counts_per_motor_rev,
                                      int32_t gear_ratio,
                                      int32_t wheel_diameter_mm);

/*
 * mm/s -> counts per speed window. Uses symmetric round-half-away-from-zero, not truncation toward zero
 * (truncation makes forward/reverse errors asymmetric and accumulates into a systematic bias).
 */
int32_t fixp_mm_s_to_counts(const fixp_scale_t *s, int32_t mm_s);

/* counts per speed window -> mm/s. */
int32_t fixp_counts_to_mm_s(const fixp_scale_t *s, int32_t counts);

/*
 * ★ Reduce quantization noise: **sum** the counts of N windows and convert to mm/s once.
 * A single window at 50 Hz makes 1 count a 5099 µm/s step (that is the resolution floor of speed feedback);
 * after summing N windows the step drops to 1/N, at the cost of N windows of extra latency.
 * Returns 0 if periods <= 0 (the caller should use the single-window version instead).
 */
int32_t fixp_counts_sum_to_mm_s(const fixp_scale_t *s, int32_t counts_sum,
                                int32_t periods);

/* Accumulated distance: counts -> mm (returns int64, no overflow in long runs). */
int64_t fixp_counts_to_mm(const fixp_scale_t *s, int64_t counts);

/* mm -> counts (returns int64, handy for reverse checks). */
int64_t fixp_mm_to_counts(const fixp_scale_t *s, int64_t mm);

/*
 * Wheel linear speed -> chassis angular velocity (thousandths of a radian, i.e. mrad/s).
 *   A differential relation such as omega_mrad_s = (v_right - v_left) does not apply to mecanum wheels;
 *   this function only does the generic one-dimensional "wheel speed / L" conversion, for calibration and self-tests.
 * Returns 0 when L <= 0, never divides by zero.
 */
int32_t fixp_wheel_mm_s_to_omega_mrad_s(int32_t wheel_mm_s, int32_t l_mm);

/* Counts per wheel revolution (= counts_per_m × circumference mm ÷ 1000), for self-tests and printing. */
int64_t fixp_counts_per_wheel_turn(const fixp_scale_t *s);

#ifdef __cplusplus
}
#endif

#endif /* FIXEDPOINT_H */
