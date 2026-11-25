/*
 * odom.h — arc-integration odometry
 * ---------------------------------------------------------------------------
 *
 * ★★ The function signatures in this file are frozen by 11_interface_contract.md §5.1 —
 *    copy them verbatim, do not change them. (`bridge-dev` is waiting to call these symbols
 *    through ctypes using exactly these signatures.)
 *
 * Ownership: the pose is computed on the Raspberry Pi side (the decision of contract §3), for three reasons:
 *   1. Odometry parameters (wheel diameter / gear ratio) are calibrated repeatedly; keeping them on the STM32
 *      would mean reflashing the firmware for every calibration.
 *   2. Arc integration belongs to the navigation domain and should not be mixed into hard-real-time motor-control firmware.
 *   3. firmware_core is required to be "shared by three places" anyway; built as a shared library it is loaded by the Pi
 *      = one copy of the math, zero duplicated implementations.
 *   So the `E` frame carries **accumulated distance per wheel (mm)**, not a pose; the deltas are taken and integrated here.
 *
 * Design constraints (consistent with the firmware_core iron rules)
 *   - Depends only on <stdint.h> and the sibling "mecanum.h" in this directory.
 *   - All-integer zero floating-point, no dynamic memory, no global mutable state, no recursion.
 *   - Same input sequence → same output sequence (determinism), giving bit-identical results on the STM32F103
 *     and on the Raspberry Pi.
 *
 * Why arc integration is mandatory instead of Euler integration
 *   The Euler form "add this step's body-frame displacement directly to the world coordinates" assumes that
 *   the heading does not change during the step. When one step contains both displacement and rotation, that assumption fails:
 *   the more rotation and the larger the step, the bigger the error; in-place rotation is worse — the tiny lateral
 *   displacement produced by encoder quantization is accumulated toward the same world direction because "the direction does not rotate with it",
 *   and after a full turn that adds up to a real position drift.
 *
 *   Arc integration (this module) rotates this step's body-frame displacement by the **mid-step heading θ + Δθ/2**
 *   into the world frame, and multiplies by a sinc factor k = s(Δθ/2)/(Δθ/2):
 *
 *       ΔX = k · ( Δx·c(θm) − Δy·s(θm) )
 *       ΔY = k · ( Δx·s(θm) + Δy·c(θm) ),   θm = θ + Δθ/2
 *
 *   Here s(·) is the sine and c(·) is the cosine (the names are not spelled out because `check-purity`
 *   does a whole-word forbidden check over the sources, so those two library function names must not appear even in comments).
 *
 *   As Δθ → 0, k → 1 and the formula degenerates to a straight line (so no special-case branch is needed, only
 *   forcing k = 1 when |Δθ| is very small, avoiding 0/0 in the integer domain).
 *
 * One detail about units and dimensions (from the math note in contract §5.1)
 *   `mecanum_forward()` is a linear map: feed it the "four-wheel displacement deltas (mm)" and
 *   the `vx_mm_s / vy_mm_s` fields of the output are actually this step's Δx_mm / Δy_mm,
 *   and the `omega_mrad_s` field is actually this step's Δθ_mrad. The _s suffix in the field names is historical naming;
 *   read the dimension from the input — here the input is a delta, not a rate.
 *   ⇒ Likewise `twist_out` returns **this step's body-frame delta** (mm/step, mrad/step),
 *     not a per-second velocity. A caller that wants velocity must divide by the control period dt itself.
 *     (The contract froze the signature without dt, so this file cannot do the division for you.)
 */

#ifndef ODOM_H
#define ODOM_H

#include <stdint.h>
#include "mecanum.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pose published to ROS2. theta is normalized, safe for long runs without overflow.
 *
 * ★ The wrap amount of theta_mrad must be **one full turn**, otherwise the normalization is no longer an
 *   angle-equivalence transform: pose_theta − total_theta is always a multiple of 6284 mrad (= 2 × 3142).
 *   This is the identity checked by criterion b of test [5], and the basis for "can it be used as yaw directly".
 *   Using a full turn of 6284 while the true value is 2π = 6283.185 loses 0.815 mrad ≈ 0.047° per wrap,
 *   which is the floor of integer mrad representation itself. (180000 was used historically; that is not a full turn,
 *   and the heading jumps 106.48° when crossing it — a real bug, fixed to 3142 by contract §5.1.)
 */
typedef struct {
    int64_t x_mm;        /* accumulated x, mm */
    int64_t y_mm;        /* accumulated y, mm */
    int32_t theta_mrad;  /* normalized to (-3142, 3142], usable directly as yaw */
} odom_pose_t;

/* Accumulated quantities without angle normalization, for debugging and long-run statistics. */
typedef struct {
    int64_t x_mm;
    int64_t y_mm;
    int64_t theta_mrad;
} odom_total_t;

void odom_reset(odom_pose_t *pose, odom_total_t *total);

/*
 * Update the pose from the deltas of "accumulated distance per wheel (mm, order FL,FR,RL,RR)".
 *   - Arc integration (not Euler): advance along the arc when Δθ != 0; otherwise the position
 *     drifts spuriously during in-place rotation — the most typical odometry mistake.
 *   - Returns 1 = updated; 0 = invalid input (invalid geometry / null pointer), pose left unchanged.
 *   - twist_out may be NULL; when non-NULL it receives **this step's body-frame displacement delta**
 *     (mm/step, mrad/step), **not a rate** — this signature has no dt argument, so it cannot produce
 *     a per-second dimension by construction. A caller that wants velocity must divide by the nominal
 *     protocol period 0.020 s itself.
 *     (Contract §5.1 froze this semantics; `bridge-dev`'s check_core.py is written against "delta".)
 *   - Pure functional, no global state, no floating-point, no dynamic memory.
 */
int odom_update(odom_pose_t *pose, odom_total_t *total,
                const mecanum_geom_t *g,
                const int32_t d_now[MECANUM_WHEEL_COUNT],
                const int32_t d_prev[MECANUM_WHEEL_COUNT],
                mecanum_twist_t *twist_out);

#ifdef __cplusplus
}
#endif

#endif /* ODOM_H */
