/*
 * mecanum.h — kinematics of a four-mecanum-wheel omnidirectional chassis
 *             (forward / inverse / clamping / dead zone / mount mapping)
 * ---------------------------------------------------------------------------
 *
 * Design constraints (consistent with the iron rules of this project's firmware_core)
 *   - Zero dependencies beyond the C99 standard library: this file depends only on <stdint.h>.
 *     No HAL headers, ROS2 headers, or third-party libraries.
 *   - All integer, no floating-point. Linear velocity in mm/s, angular velocity in mrad/s (milliradians per second).
 *   - No dynamic memory, no global mutable state, no recursion, no floating-point, no integer types wider than 64 bits.
 *   - Produces bit-identical results on the STM32F103 (Cortex-M3, no FPU) and on a Raspberry Pi 5.
 *     Every division is integer division and only two kinds exist: by a known positive constant, and symmetric rounding.
 *     — determinism is a prerequisite for safety-related code.
 *
 * Coordinate frame (ROS REP-103)
 *   x forward, y left, z up; omega > 0 is counter-clockwise seen from above.
 *   The origin is the geometric centre of the chassis (centre of the rectangle formed by the four axles).
 *
 * Wheel indices (one order throughout this module: FL/FR/RL/RR)
 *   MECANUM_FL = 0   front left     MECANUM_FR = 1   front right
 *   MECANUM_RL = 2   rear left      MECANUM_RR = 3   rear right
 *   Note: the internal enum of ROS2 ros2_controllers' mecanum_drive_controller is
 *   FL, FR, RR, RL (the two rear wheels are swapped). Convert between the two with
 *   mecanum_wheels_from_ros2_order() / mecanum_wheels_to_ros2_order().
 *   WPILib MecanumDriveKinematics uses the same order as this module (FL,FR,RL,RR).
 *
 * Physical model and where the formulas come from (the signs are not guesses)
 *   Adamov, "Geometry and Kinematics of the Mecanum Wheel on a Plane and a
 *   Sphere", Russian J. Nonlinear Dynamics 20(1):43-78, 2024,
 *   DOI 10.20537/nd240201 — when the axial displacement of the roller contact point is ignored (phi=0), a
 *   mecanum wheel reduces to the simplest sliding-disc model:
 *
 *        R * phidot = V_Cx + V_Cy * tan(delta)
 *
 *   delta is the angle of the contacting roller axis relative to the wheel mid-plane. Off-the-shelf mecanum
 *   wheels have delta = ±45 degrees, hence tan(delta) = ±1 — that is exactly where the ±1 entries in S_VY
 *   come from, and the sign is fully determined by the sign of the physical quantity delta. Therefore:
 *
 *        vy_polarity  ==  +1  <=>  delta = +45 degrees
 *        vy_polarity  ==  -1  <=>  delta = -45 degrees   (all four roller orientations reversed)
 *
 *   It is not a patch added just to make the numbers match; it corresponds to a real physical quantity.
 *
 * Sign tables (order FL, FR, RL, RR)
 *   S_VY  = { -1, +1, +1, -1 }   coefficient of the translation y component on each wheel
 *   S_OMG = { -1, +1, -1, +1 }   coefficient of the spin component on each wheel
 *   Both tables have been checked entry by entry against four independent implementations:
 *     - ROS2 ros2_controllers / mecanum_drive_controller
 *     - WPILib MecanumDriveKinematics
 *     - minimec-ros2 / minimeclib MecanumDrive
 *     - akkexd/ros2-mecanum-robot driver_node.py
 *   In addition, a written derivation (in Chinese) and WPILib's real-robot grid file (FL and RR use the
 *   same kind of wheel, FR and RL the same kind of wheel) confirm the S_VY pairing at the physical-assembly level.
 *
 * Formulas (L = half_wheelbase_mm + half_track_mm)
 *   Inverse:
 *       vy  = vy_polarity    * vy_mm_s
 *       rot = omega_polarity * round( omega_mrad_s * L / 1000 )     [mm/s]
 *       wheel[i] = vx_mm_s + S_VY[i] * vy + S_OMG[i] * rot
 *
 *   Forward (equivalent to the Moore-Penrose pseudo-inverse of the design matrix; the system is
 *   overdetermined, so take the least-squares solution):
 *       vx_mm_s      = round( ( w0 + w1 + w2 + w3 ) / 4 )
 *       vy_mm_s      = vy_polarity    * round( ( -w0 + w1 + w2 - w3 ) / 4 )
 *       omega_mrad_s = omega_polarity * round( ( -w0 + w1 - w2 + w3 ) * 1000 / (4L) )
 *
 *   The twist produced by forward and the twist fed into inverse live in the same "command frame":
 *   with one polarity configuration, forward(inverse(t)) == t (within integer-rounding tolerance).
 *
 * Why no Q-format fixed point is needed (measured)
 *   Kinematics rounding introduces about 1 mrad/s of angular-velocity error; encoder quantization at 50 Hz
 *   sampling corresponds to about 17 mrad/s. The error source is roughly 20x coarser than the algorithmic
 *   error, so the kinematics error is completely swamped and integer mm/s is enough. A Q format would only
 *   add complexity and improve no measurable metric.
 *
 * Overflow and safe value ranges
 *   All intermediates are int64_t, so the ranges below have ample headroom:
 *       |vx_mm_s| <= 100000, |vy_mm_s| <= 100000
 *       |omega_mrad_s| <= 20000
 *       0 <= half_wheelbase_mm, half_track_mm, and L <= 1000
 *   Outside these ranges the result is not guaranteed to be meaningful (but no undefined behaviour occurs).
 *   Note that half_wheelbase_mm + half_track_mm must be >= 1, otherwise the divisions in forward kinematics are
 *   meaningless; this module outputs all zeros when L < 1 instead of dividing by zero.
 *   Use mecanum_geom_is_valid() to check.
 */

#ifndef MECANUM_H
#define MECANUM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Wheel indices                                                       */
/* ------------------------------------------------------------------ */

#define MECANUM_FL 0
#define MECANUM_FR 1
#define MECANUM_RL 2
#define MECANUM_RR 3
#define MECANUM_WHEEL_COUNT 4

/* ------------------------------------------------------------------ */
/* Data types                                                          */
/* ------------------------------------------------------------------ */

/* Chassis geometry and polarity. Pure "data", not code — calibration changes numbers, never the algorithm. */
typedef struct {
    int32_t half_wheelbase_mm; /* distance from the geometric centre to the front/rear axle, >= 0 */
    int32_t half_track_mm;     /* distance from the geometric centre to the left/right wheel mid-plane, >= 0 */
    int8_t  vy_polarity;       /* only +1 or -1; physical meaning in the file header (sign of the roller angle delta) */
    int8_t  omega_polarity;    /* only +1 or -1 */
} mecanum_geom_t;

/* Chassis velocity command / forward-kinematics output. */
typedef struct {
    int32_t vx_mm_s;      /* forward positive */
    int32_t vy_mm_s;      /* left positive */
    int32_t omega_mrad_s; /* counter-clockwise positive */
} mecanum_twist_t;

/* Four wheel linear speeds (rim speeds, mm/s), order FL,FR,RL,RR. */
typedef struct {
    int32_t wheel_mm_s[MECANUM_WHEEL_COUNT];
} mecanum_wheels_t;

/*
 * Mount mapping: translates "wheels" into "motors".
 *   motor_of_wheel[i] = which motor channel drives wheel slot i (must be a bijection onto 0..3)
 *   sign_of_wheel[i]  = +1/-1, whether the wheel turns in the positive direction when that motor gets positive PWM
 * Because it is a bijection and the signs are only ±1, apply_mount and undo_mount are exact inverses.
 */
typedef struct {
    uint8_t motor_of_wheel[MECANUM_WHEEL_COUNT];
    int8_t  sign_of_wheel[MECANUM_WHEEL_COUNT];
} mecanum_mount_t;

/*
 * Output clamping and dead zone (both in "wheel linear speed mm/s", not PWM).
 *   wheel_max_mm_s      : wheel linear speed ceiling; <= 0 means unlimited (rarely useful)
 *   wheel_deadzone_mm_s : wheel linear speed equivalent to the friction dead zone; <= 0 means no compensation
 * Requires wheel_deadzone_mm_s <= wheel_max_mm_s (otherwise the two fight each other).
 *
 * Why a deadzone is needed: measured (Aung Khant Ko, Zenodo 10.5281/zenodo.20724812),
 * the motor dead zone is PWM 30-35 on loop-pile carpet and 15-20 on hard floor — 75% wider on carpet —
 * and a standard navigation stack never notices. The fix given there is the same as this module's: scale all
 * four wheels by one factor, keeping the velocity direction unchanged.
 */
typedef struct {
    int32_t wheel_max_mm_s;
    int32_t wheel_deadzone_mm_s;
} mecanum_limits_t;

/* Complete result of mecanum_solve(). */
typedef struct {
    mecanum_wheels_t wheels;                     /* four wheel linear speeds after clamping / dead zone */
    int32_t          motor_mm_s[MECANUM_WHEEL_COUNT]; /* mapped into motor order by the mount mapping */
    uint8_t          saturated;                  /* 1 = saturation normalization fired (scaled down) */
    uint8_t          lifted;                     /* 1 = dead-zone lift fired (scaled up) */
    uint8_t          idle;                       /* 1 = all four motor commands are 0 */
} mecanum_solve_result_t;

/* ------------------------------------------------------------------ */
/* Defaults and zeroing                                                */
/* ------------------------------------------------------------------ */

/*
 * Default geometry: half_wheelbase_mm = 150, half_track_mm = 150.
 * !! 150/150 are placeholder values. Replace them with a tape-measure measurement once the chassis is built.
 *    Get these two wrong and in-place rotation draws a circle around a point away from the geometric centre.
 */
mecanum_geom_t mecanum_default_geom(void);

/* Identity mount: wheel i -> motor i, all signs +1. Also a placeholder. */
mecanum_mount_t mecanum_identity_mount(void);

/*
 * Default limits: wheel_max_mm_s = 1000 (placeholder; replace with the measured maximum wheel speed),
 *          wheel_deadzone_mm_s = 0 (dead-zone compensation off by default — it changes behaviour,
 *          so only enable it after measuring the dead zone of your own robot).
 */
mecanum_limits_t mecanum_default_limits(void);

mecanum_twist_t mecanum_zero_twist(void);
void            mecanum_zero_wheels(mecanum_wheels_t *out);

/* ------------------------------------------------------------------ */
/* Parameter validation                                                */
/* ------------------------------------------------------------------ */

/* Returns 1 if valid. Checks non-negative values, L >= 1, and both polarities being ±1 only. */
int mecanum_geom_is_valid(const mecanum_geom_t *g);

/* Returns 1 if valid. Checks motor_of_wheel is a bijection onto 0..3 and the signs are ±1 only. */
int mecanum_mount_is_valid(const mecanum_mount_t *m);

/* L = half_wheelbase_mm + half_track_mm. */
int32_t mecanum_l_mm(const mecanum_geom_t *g);

/* ------------------------------------------------------------------ */
/* Kinematics                                                          */
/* ------------------------------------------------------------------ */

/*
 * Inverse: chassis velocity -> four wheel linear speeds.
 * If g or twist is NULL, or L < 1, outputs all zeros (no division by zero, no undefined behaviour).
 */
void mecanum_inverse(const mecanum_geom_t *g, const mecanum_twist_t *twist,
                     mecanum_wheels_t *out);

/*
 * Forward: four wheel linear speeds -> chassis velocity (least-squares pseudo-inverse).
 * Shares the same polarity as mecanum_inverse, so forward(inverse(t)) == t for one configuration.
 */
void mecanum_forward(const mecanum_geom_t *g, const mecanum_wheels_t *wheels,
                     mecanum_twist_t *out);

/* ------------------------------------------------------------------ */
/* Output shaping: two proportional scalings, opposite in direction,   */
/* same principle                                                      */
/* ------------------------------------------------------------------ */

/*
 * Saturation normalization (upper bound): when any wheel exceeds limit_abs_mm_s, scale all of them down by one factor.
 * Returns 1 if scaling happened, 0 if the input was copied unchanged.
 *
 * Why not clamp each wheel independently: per-wheel clamping breaks the ratio between the four wheels,
 * so the direction of motion changes — driving straight turns into driving at an angle. A real-world
 * counterexample exists: akkexd/ros2-mecanum-robot's driver_node.py clamps each wheel with
 * max(-MAX, min(MAX, w)).
 * limit_abs_mm_s <= 0 means no limit: copy unchanged and return 0.
 */
int mecanum_desaturate(const mecanum_wheels_t *in, int32_t limit_abs_mm_s,
                       mecanum_wheels_t *out);

/*
 * Dead-zone lift (lower bound): when the four-wheel peak is below deadzone_abs_mm_s, scale all of them up
 * to that value. Returns 1 if scaling happened, 0 if the input was copied unchanged.
 *
 * Safety semantics (important):
 *   - If all four inputs are 0, the output must be all 0 and the return value 0. That is, a stationary robot
 *     must never be "lifted" into motion by dead-zone compensation. This is a hard constraint, not an optimization.
 *   - The operation makes a "commanded but tiny" case run faster than commanded. That is a deliberate trade-off:
 *     better to move slightly too fast than to sit still because of static friction dead zone, while keeping the
 *     direction correct.
 *   - Hence it is off by default (0 in mecanum_default_limits); measure the dead zone of your own robot first,
 *     then enable it.
 * deadzone_abs_mm_s <= 0 means no compensation: copy unchanged and return 0.
 */
int mecanum_lift_deadzone(const mecanum_wheels_t *in, int32_t deadzone_abs_mm_s,
                          mecanum_wheels_t *out);

/* ------------------------------------------------------------------ */
/* Mount mapping                                                       */
/* ------------------------------------------------------------------ */

/* wheel -> motor: motor_of_wheel permutes channels, sign_of_wheel flips direction. */
void mecanum_apply_mount(const mecanum_mount_t *m, const mecanum_wheels_t *wheels,
                         int32_t motor_mm_s[MECANUM_WHEEL_COUNT]);

/* motor -> wheel: the inverse of apply_mount. */
void mecanum_undo_mount(const mecanum_mount_t *m,
                        const int32_t motor_mm_s[MECANUM_WHEEL_COUNT],
                        mecanum_wheels_t *wheels);

/* ------------------------------------------------------------------ */
/* One-call solver (the entry point the firmware actually calls)       */
/* ------------------------------------------------------------------ */

/*
 * Inverse -> saturation normalization -> dead-zone lift -> mount mapping, all in one pass.
 * The order must not be reversed: scale down first, then up. The deadzone <= max constraint guarantees
 * that "lifting to deadzone" can never exceed max.
 * g / lim / mnt may be NULL (lim NULL means neither clamping nor dead-zone compensation).
 */
void mecanum_solve(const mecanum_geom_t *g, const mecanum_limits_t *lim,
                   const mecanum_mount_t *mnt, const mecanum_twist_t *twist,
                   mecanum_solve_result_t *out);

/* ------------------------------------------------------------------ */
/* Wheel-order conversion with ROS2                                    */
/* ------------------------------------------------------------------ */

/*
 * ROS2 ros2_controllers uses the wheel order [FL, FR, RR, RL]; this module uses [FL, FR, RL, RR].
 * When wiring into the ros2_control command interface, or comparing against its logs, RL and RR must be swapped.
 * The consequence of forgetting this swap: the robot drives and spins fine, but strafing and diagonal motion
 * go the wrong way.
 */
void mecanum_wheels_from_ros2_order(const int32_t ros2_in[MECANUM_WHEEL_COUNT],
                                    mecanum_wheels_t *out);
void mecanum_wheels_to_ros2_order(const mecanum_wheels_t *in,
                                  int32_t ros2_out[MECANUM_WHEEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* MECANUM_H */
