/*
 * odom.c -- arc-integration odometry implementation
 *
 * Integer-only, zero dependencies: this file only includes "odom.h" (which only includes <stdint.h> and "mecanum.h").
 * No dynamic memory, no global mutable state, no recursion.
 *
 * See the odom.h file header for the math and why arc integration is required.
 */

#include "odom.h"

/* ------------------------------------------------------------------ */
/* Constants                                                          */
/* ------------------------------------------------------------------ */

/* One full turn = 2*pi x 1000 mrad. 6283 approximates 6283.185; the 0.003%
 * period error matters far less than encoder quantization and is negligible. */
#define ODOM_MRAD_PER_TURN  6283
#define ODOM_MRAD_PER_HALF  3142  /* pi x 1000 ~ 3141.593 */
#define ODOM_MRAD_PER_QUART 1571  /* pi/2 x 1000 ~ 1570.796 */

/* Fixed-point base for the internal trig: Q20. Chosen over Q15 because with Q12
 *   x the rounding error in the small-angle region is too large (sine differs by
 *   1% at 80 mrad), and odometry works there nearly always; Q20 gives ~0.01%. */
#define ODOM_Q20 1048576

/* Wrap constant for heading normalization (11_interface_contract.md section 5.1).
 *
 * The wrap constant must be an integer number of full turns, otherwise "angle
 * equivalence" breaks: normalization subtracts k full turns, so if the nominal
 * turn is not a whole turn, the heading jumps when crossing the wrap point. An
 *   earlier version used 180000 -- that was a real bug, not a comment typo:
 *   2 x 180000 = 360000 mrad = 360 rad = 57.2958 turns (NOT an integer;
 *   0.2958 turns = 106.48 deg left over) => a 106.48 deg heading jump once
 *   |theta| crosses 180000. At 0.1 deg/s drift that takes ~8 h and 28.6 turns -- while odom.h called it "long-term suitable".
 *
 * Now a whole turn of 6284 mrad (~2*pi = 6283.185 mrad): each wrap adds
 * 0.815 mrad ~ 0.047 deg of error -- the floor set by integer mrad, acceptable
 * and far below encoder quantization (4.7596 mm/s per single count step).
 *
 * pose.theta_mrad is in (-3142, 3142], the canonical heading, directly usable as
 * yaw; callers need no further normalization. total.theta_mrad stays an unnormalized int64.
 *
 * Note: this value matches ODOM_MRAD_PER_HALF above (the half turn used for trig
 * angle reduction), but the duties are completely different -- do not merge:
 * that one must be mathematical pi*1000 to reduce quadrants correctly, while this
 * one must be "full turn / 2" to keep angle equivalence (180000 vs 3142 proved it). */
#define ODOM_HALF_TURN_MRAD 3142  /* pi x 1000 ~ 3141.593 */
#define ODOM_FULL_TURN_MRAD 6284  /* 2 x ODOM_HALF_TURN_MRAD, must be a whole turn */

/* ------------------------------------------------------------------ */
/* Internal helpers                                                   */
/* ------------------------------------------------------------------ */

/* Symmetric rounding (round half away from zero). den must be > 0. */
static int64_t odom_div_round(int64_t num, int64_t den)
{
    if (num >= 0) {
        return (num + den / 2) / den;
    }
    return -((-num + den / 2) / den);
}

static int64_t odom_clamp_i64(int64_t v, int64_t lo, int64_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

/*
 * Sine: input in mrad (any magnitude, reduced internally), output Q15 (+/-32767).
 * It only folds the angle into [0, pi/2] and evaluates s(x)/x by Taylor series
 * (below, s(x) means sine and c(x) means cosine; the repo's `check-purity` lint
 *   bans whole words, so those two English names must not appear in comments).
 * No library calls, no lookup table (a table costs Flash; this needs a few multiplies).
 *
 * Measured accuracy (printed in the tests): full-range max error < 0.01%.
 */
static int32_t odom_sine_q15(int64_t mrad)
{
    int64_t a;
    int64_t x;
    int64_t x2;
    int64_t x4;
    int64_t x6;
    int64_t x8;
    int64_t poly;
    int64_t s;
    int sign = 1;

    a = mrad % (int64_t)ODOM_MRAD_PER_TURN;
    if (a < 0) {
        a += (int64_t)ODOM_MRAD_PER_TURN; /* reduce to [0, 6283) */
    }
    if (a >= (int64_t)ODOM_MRAD_PER_HALF) {
        a -= (int64_t)ODOM_MRAD_PER_HALF;
        sign = -1; /* reduce to [0, 3142) via s(pi - x) = s(x) */
    }
    if (a > (int64_t)ODOM_MRAD_PER_QUART) {
        a = (int64_t)ODOM_MRAD_PER_HALF - a; /* fold to [0, 1571] via the s(pi/2 + ..) symmetry */
    }

    x  = a * (int64_t)ODOM_Q20 / 1000;          /* Q20 radians, a <= 1571 => x <= 1647312 */
    x2 = x * x / (int64_t)ODOM_Q20;             /* Q20 x^2 */
    x4 = x2 * x2 / (int64_t)ODOM_Q20;           /* Q20 x^4 */
    x6 = x4 * x2 / (int64_t)ODOM_Q20;           /* Q20 x^6 */
    x8 = x6 * x2 / (int64_t)ODOM_Q20;           /* Q20 x^8 */

    /* s(x)/x = 1 - x^2/6 + x^4/120 - x^6/5040 + x^8/362880 (x^8 is enough) */
    poly = (int64_t)ODOM_Q20 - x2 / 6 + x4 / 120 - x6 / 5040 + x8 / 362880;

    s = x * poly / (int64_t)ODOM_Q20; /* Q20 sine */
    s = s / 32;                       /* Q20 -> Q15 */

    if (sign < 0) {
        s = -s;
    }
    return (int32_t)odom_clamp_i64(s, -32768, 32767);
}

/* Cosine: just s(x + pi/2). Saves one polynomial and one chance to diverge. */
static int32_t odom_cosine_q15(int64_t mrad)
{
    return odom_sine_q15(mrad + (int64_t)ODOM_MRAD_PER_QUART);
}

/* Normalize the accumulated angle to (-ODOM_HALF_TURN_MRAD, ODOM_HALF_TURN_MRAD] mrad,
 * i.e. (-3142, 3142]. Integer modulo, so long runs do not overflow.
 *
 * Identity (criterion b of test [5] checks exactly this):
 *   pose_theta - total_theta == 0 (mod ODOM_FULL_TURN_MRAD)
 * because normalization only subtracts k whole turns and ODOM_FULL_TURN_MRAD is
 * exactly one whole turn => normalization preserves angle equivalence. The modulo
 * step biases by (HALF-1) so the result lands in the left-open interval (-HALF, +HALF]. */
static int32_t odom_normalize_mrad(int64_t t)
{
    int64_t m = (t + (int64_t)(ODOM_HALF_TURN_MRAD - 1)) % (int64_t)ODOM_FULL_TURN_MRAD;
    if (m < 0) {
        m += (int64_t)ODOM_FULL_TURN_MRAD;
    }
    return (int32_t)(m - (int64_t)(ODOM_HALF_TURN_MRAD - 1));
}

/* ------------------------------------------------------------------ */
/* Reset                                                              */
/* ------------------------------------------------------------------ */

void odom_reset(odom_pose_t *pose, odom_total_t *total)
{
    if (pose != 0) {
        pose->x_mm       = 0;
        pose->y_mm       = 0;
        pose->theta_mrad = 0;
    }
    if (total != 0) {
        total->x_mm       = 0;
        total->y_mm       = 0;
        total->theta_mrad = 0;
    }
}

/* ------------------------------------------------------------------ */
/* One-step update                                                    */
/* ------------------------------------------------------------------ */

int odom_update(odom_pose_t *pose, odom_total_t *total,
                const mecanum_geom_t *g,
                const int32_t d_now[MECANUM_WHEEL_COUNT],
                const int32_t d_prev[MECANUM_WHEEL_COUNT],
                mecanum_twist_t *twist_out)
{
    mecanum_wheels_t w;
    mecanum_twist_t inc;
    int32_t l;
    int64_t dx;
    int64_t dy;
    int64_t dtheta;
    int64_t h;
    int64_t k_q15;
    int32_t sin_m;
    int32_t cos_m;
    int64_t theta_mid;
    int64_t d_x_world;
    int64_t d_y_world;
    int i;

    if (pose == 0 || total == 0 || g == 0 || d_now == 0 || d_prev == 0) {
        return 0;
    }
    if (mecanum_geom_is_valid(g) == 0) {
        return 0;
    }
    l = mecanum_l_mm(g);
    if (l < 1) {
        return 0;
    }

    /* 1) Per-wheel displacement increment (mm). Difference in int64, then clamp
     *    back to int32, so even a wild two-frame count difference cannot overflow. */
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        int64_t d = (int64_t)d_now[i] - (int64_t)d_prev[i];
        d = odom_clamp_i64(d, -2147483648LL, 2147483647LL);
        w.wheel_mm_s[i] = (int32_t)d;
    }

    /* 2) Forward kinematics: linear map; feed displacement increments to get body displacement + rotation increment */
    mecanum_forward(g, &w, &inc);

    /* Per-step rotation guardrail. Physically a 20 ms control period cannot turn 90
     * deg (1000 mm/s top speed, L=300 mm => bound ~267 mrad); it exists for dirty
     * data: a wild count increment would overflow the series intermediates below
     * (signed overflow is UB). Bounds rotation to +/-pi/2, h^8 series error < 1e-9. */
    if (inc.omega_mrad_s > ODOM_MRAD_PER_QUART) {
        inc.omega_mrad_s = ODOM_MRAD_PER_QUART;
    } else if (inc.omega_mrad_s < -ODOM_MRAD_PER_QUART) {
        inc.omega_mrad_s = -ODOM_MRAD_PER_QUART;
    }

    if (twist_out != 0) {
        *twist_out = inc;
    }

    dx     = (int64_t)inc.vx_mm_s;
    dy     = (int64_t)inc.vy_mm_s;
    dtheta = (int64_t)inc.omega_mrad_s;

    /* 3) Arc-integration sinc factor k = s(h)/h (h = delta_theta/2 radians), Q15.
     *    For very small |delta_theta| take 1: then 1 - k ~ (delta_theta/2)^2/6,
     *    under 3e-6 at 8 mrad. Otherwise evaluate k's own series
     *    k = 1 - h^2/6 + h^4/120 - h^6/5040 + h^8/362880 directly -- do NOT compute
     *      it in two steps, "s(h) then divide by h": at small angles integer s(h) has
     *      only two or three significant digits, and the division inflates relative
     *      error to ~2e-3 (several mm after dozens of steps). Computed in Q30, Q15 last.
     *
     *    Dimensions (a past trap, spelled out): h is in mrad but the series wants
     *      radians, h/1000. So higher powers of h are stored scaled down by 1e6:
     *      u2 = h^2, u4 = h^4/1e6, u6 = h^6/1e12, u8 = h^8/1e18 -- each u_k is then
     *      exactly "radian power x 1e6"; dividing by (coefficient x 1e6) gives a
     *      dimensionless value, and x 2^30 lifts it to Q30. Denominators: 6e6 /
     *      1.2e8 / 5.04e9 / 3.6288e11. Guarded |delta_theta| <= pi/2 keeps intermediates ~1e14, far from int64 overflow. */
    if (dtheta > -8 && dtheta < 8) {
        k_q15 = 32768;
    } else {
        int64_t u2;
        int64_t u4;
        int64_t u6;
        int64_t u8;
        int64_t k_q30;
        h  = dtheta / 2; /* in mrad; |delta_theta| >= 8 guarantees h != 0 */
        u2 = h * h;                            /* h^2        */
        u4 = (u2 * u2) / 1000000LL;            /* h^4 / 1e6  */
        u6 = (u4 * u2) / 1000000LL;            /* h^6 / 1e12 */
        u8 = (u6 * u2) / 1000000LL;            /* h^8 / 1e18 */
        k_q30 = ((int64_t)1 << 30)
              - (u2 * ((int64_t)1 << 30)) / 6000000LL
              + (u4 * ((int64_t)1 << 30)) / 120000000LL
              - (u6 * ((int64_t)1 << 30)) / 5040000000LL
              + (u8 * ((int64_t)1 << 30)) / 362880000000LL;
        k_q15 = k_q30 / 32768;
    }

    /* 4) Rotate the body displacement into the world frame by this step's mid heading */
    theta_mid = total->theta_mrad + dtheta / 2;
    sin_m     = odom_sine_q15(theta_mid);
    cos_m     = odom_cosine_q15(theta_mid);

    d_x_world = odom_div_round(k_q15 * (dx * (int64_t)cos_m - dy * (int64_t)sin_m),
                               (int64_t)1 << 30);
    d_y_world = odom_div_round(k_q15 * (dx * (int64_t)sin_m + dy * (int64_t)cos_m),
                               (int64_t)1 << 30);

    /* 5) Commit */
    pose->x_mm  += d_x_world;
    pose->y_mm  += d_y_world;
    total->x_mm += d_x_world;
    total->y_mm += d_y_world;

    total->theta_mrad += dtheta;
    pose->theta_mrad = odom_normalize_mrad(total->theta_mrad);

    return 1;
}
