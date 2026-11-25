/*
 * test_odom.c - test bench for the odom module (★ acceptance criterion #5)
 *
 * Coverage:
 *   [1] Reset
 *   [2] ★ Straight line 1000 mm -> x error <= 2 mm
 *   [3] ★ In-place spin 360 deg -> position drift <= 5 mm (only arc integration passes)
 *   [4] ★ Known arc trajectory -> error split into 5 independent criteria by source:
 *         A continuous arc integration vs analytic ground truth  (the algorithm itself, exact in theory)
 *         B integer version vs per-step-rounding reference  (fixed-point trig + sinc)
 *         C integer version vs continuous reference  (quantization from rounding to 1 mm each step)
 *         D integer version vs analytic ground truth  (composition of A/B/C, self-consistency cross-check)
 *         E naive Euler vs analytic ground truth  (control, showing the criteria are meaningful)
 *       plus a "step-count experiment": 4/40/400 steps with the same step length, proving arc
 *       integration has no discretization error; all position error comes from per-step rounding
 *   [5] Angle normalization and pose/total consistency
 *        incl. contract section 5.1 add-on: after >= 2 full turns check "pose - total is a
 *        nonzero multiple of one turn 6284 mrad" (normalization keeps angle equivalence), criteria a/b/c
 *   [6] Invalid input (null pointers / invalid geometry)
 *   [7] Semantics of twist_out (this step's increment, not a rate)
 *   [8] Zero increment and determinism
 *
 * Note: this is a test file, so double / math.h are allowed as an "external reference".
 *       src/ and include/ remain float-free and third-party-dependency-free.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "mecanum.h"
#include "odom.h"

static int g_checks = 0;
static int g_fails  = 0;

static void ck_true(const char *tag, int cond)
{
    ++g_checks;
    if (cond == 0) {
        ++g_fails;
        if (g_fails <= 40) {
            printf("  [FAIL] %s\n", tag);
        }
    }
}

static int64_t iabs64(int64_t v)
{
    return (v < 0) ? -v : v;
}

/* forward declaration: geometry uses the default parameters (L = half_wheelbase + half_track = 300 mm).
   It is defined later in the file, but run_arc() above needs it -- C99 has no implicit declarations,
   and -Werror turns "use before declaration" into a hard compile error. */
static mecanum_geom_t geom(void);

static void ck_near64(const char *tag, int64_t got, int64_t want, int64_t tol)
{
    int64_t err = iabs64(got - want);
    ++g_checks;
    if (err > tol) {
        ++g_fails;
        if (g_fails <= 40) {
            printf("  [FAIL] %s: got %ld, want %ld, tol %ld, err %ld\n",
                   tag, (long)got, (long)want, (long)tol, (long)err);
        }
    }
}

/*
 * Double-precision arc integration reference: rounds each step to whole mm (aligned with the integer
 * version's "semantics") but takes true trig values from libm. It is this module's independent
 * external reference -- odom.c uses a home-grown Q20 polynomial fixed-point sine, a different impl.
 */
static void ref_arc_step(double *rx, double *ry, int64_t *rth,
                         int32_t dx, int32_t dy, int32_t dth_mrad)
{
    double dth = (double)dth_mrad / 1000.0;
    double k;
    double thm;
    double d_x;
    double d_y;

    if (dth == 0.0) {
        k = 1.0;
    } else {
        k = sin(dth / 2.0) / (dth / 2.0);
    }
    thm = ((double)(*rth) + (double)dth_mrad / 2.0) / 1000.0;
    d_x = k * ((double)dx * cos(thm) - (double)dy * sin(thm));
    d_y = k * ((double)dx * sin(thm) + (double)dy * cos(thm));

    *rx += (double)llround(d_x);
    *ry += (double)llround(d_y);
    *rth += dth_mrad;
}

/* naive Euler accumulation: add the body displacement straight to world coordinates (without
 * rotating by the heading). This is exactly the wrong method arc integration replaces. */
static void naive_step(double *nx, double *ny, int32_t dx, int32_t dy)
{
    *nx += (double)dx;
    *ny += (double)dy;
}

/*
 * Continuous arc integration reference: same formulas as ref_arc_step, but **no per-step rounding**.
 *
 * Its purpose is to separate two completely different error sources:
 *   (1) the arc integration algorithm itself vs analytic ground truth  -- measured by this function
 *   (2) quantization from rounding world displacement to 1 mm each step -- measured by "integer vs this"
 *
 * ★ Conclusion (see the step-count experiment in [4]): arc integration is **mathematically exact**
 *   for **constant-curvature** paths -- summing N analytic steps cancels the 2*sin(dtheta/2) terms at
 *   both ends, giving exactly (R*s(Theta), R*(1-c(Theta))), independent of N. So whether N is 40 or 400,
 *   this function stays right on the analytic truth; all position error comes from "(2) per-step rounding".
 */
static void ref_arc_step_exact(double *rx, double *ry, int64_t *rth,
                               int32_t dx, int32_t dy, int32_t dth_mrad)
{
    double dth = (double)dth_mrad / 1000.0;
    double k;
    double thm;

    if (dth == 0.0) {
        k = 1.0;
    } else {
        k = sin(dth / 2.0) / (dth / 2.0);
    }
    thm = ((double)(*rth) + (double)dth_mrad / 2.0) / 1000.0;
    *rx += k * ((double)dx * cos(thm) - (double)dy * sin(thm));
    *ry += k * ((double)dx * sin(thm) + (double)dy * cos(thm));
    *rth += dth_mrad;
}

/*
 * Run N steps of the same arc (each step: 50 mm forward + 20 mrad turn, R = 2500 mm),
 * producing both the "integer odometry" and the "continuous double reference" endpoints.
 * Used to split the error by source: reference hugs truth = algorithm fine; integer drifts = rounding.
 */
static void run_arc(int steps, int64_t *ix, int64_t *iy, double *cx, double *cy)
{
    mecanum_geom_t g = geom();
    odom_pose_t p;
    odom_total_t t;
    const int32_t pattern[MECANUM_WHEEL_COUNT] = { 44, 56, 44, 56 };
    int32_t prev[MECANUM_WHEEL_COUNT];
    int32_t now[MECANUM_WHEEL_COUNT];
    double lx = 0.0;
    double ly = 0.0;
    int64_t lth = 0;
    int step;
    int i;

    odom_reset(&p, &t);
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        prev[i] = 0;
    }
    for (step = 0; step < steps; ++step) {
        mecanum_twist_t inc;
        for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
            now[i] = prev[i] + pattern[i];
        }
        (void)odom_update(&p, &t, &g, now, prev, &inc);
        ref_arc_step_exact(&lx, &ly, &lth, inc.vx_mm_s, inc.vy_mm_s, inc.omega_mrad_s);
        for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
            prev[i] = now[i];
        }
    }
    *ix = p.x_mm;
    *iy = p.y_mm;
    *cx = lx;
    *cy = ly;
}


static mecanum_geom_t geom(void)
{
    return mecanum_default_geom(); /* L = 300 mm (half wheelbase 150 + half track 150) */
}

/* ================================================================== */
/* [1] Reset                                                           */
/* ================================================================== */

static void test_reset(void)
{
    odom_pose_t p;
    odom_total_t t;

    printf("[1] Reset\n");

    p.x_mm = 1;
    p.y_mm = 2;
    p.theta_mrad = 3;
    t.x_mm = 4;
    t.y_mm = 5;
    t.theta_mrad = 6;
    odom_reset(&p, &t);
    ck_near64("[1] pose.x zeroed", p.x_mm, 0, 0);
    ck_near64("[1] pose.y zeroed", p.y_mm, 0, 0);
    ck_near64("[1] pose.theta zeroed", (int64_t)p.theta_mrad, 0, 0);
    ck_near64("[1] total.x zeroed", t.x_mm, 0, 0);
    ck_near64("[1] total.theta zeroed", t.theta_mrad, 0, 0);

    odom_reset(0, 0); /* must not crash */
    odom_reset(&p, 0);
    odom_reset(0, &t);
    ck_true("[1] odom_reset tolerates null pointers", 1);
}

/* ================================================================== */
/* [2] ★ Straight line 1000 mm                                         */
/* ================================================================== */

static void test_straight(void)
{
    mecanum_geom_t g = geom();
    odom_pose_t p;
    odom_total_t t;
    double rx = 0.0;
    double ry = 0.0;
    int64_t rth = 0;
    int32_t prev[MECANUM_WHEEL_COUNT];
    int32_t now[MECANUM_WHEEL_COUNT];
    int step;
    int i;

    printf("[2] ★ straight line 1000 mm (100 steps x 10 mm per wheel)\n");

    odom_reset(&p, &t);
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        prev[i] = 0;
    }

    for (step = 0; step < 100; ++step) {
        mecanum_twist_t inc;
        for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
            now[i] = prev[i] + 10;
        }
        ck_true("[2] odom_update returns 1", odom_update(&p, &t, &g, now, prev, &inc) == 1);
        ck_true("[2] no rotation on a straight line", inc.vx_mm_s == 10 && inc.vy_mm_s == 0 &&
                                     inc.omega_mrad_s == 0);
        ref_arc_step(&rx, &ry, &rth, inc.vx_mm_s, inc.vy_mm_s, inc.omega_mrad_s);
        for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
            prev[i] = now[i];
        }
    }

    printf("  [info] integer odometry: x = %ld mm, y = %ld mm, theta = %ld mrad\n",
           (long)p.x_mm, (long)p.y_mm, (long)p.theta_mrad);
    printf("  [info] double-precision arc reference: x = %ld mm, y = %ld mm\n",
           (long)rx, (long)ry);

    ck_near64("[2] x error <= 2 mm (against truth 1000 mm)", p.x_mm, 1000, 2);
    ck_near64("[2] y error <= 2 mm", p.y_mm, 0, 2);
    ck_near64("[2] theta is 0", (int64_t)p.theta_mrad, 0, 0);
    ck_near64("[2] matches the double reference (<= 2 mm)", p.x_mm, (int64_t)rx, 2);
    ck_near64("[2] y matches the double reference (<= 2 mm)", p.y_mm, (int64_t)ry, 2);
}

/* ================================================================== */
/* [3] ★ In-place spin 360 deg                                         */
/* ================================================================== */

static void test_spin_in_place(void)
{
    mecanum_geom_t g = geom();
    odom_pose_t p;
    odom_total_t t;
    /* wheel displacement pattern for a pure spin = S_OMG x rot.
     * A small pairwise sign-flipped bias is added on purpose, equivalent to encoder quantization
     * leaking a little lateral displacement into every step: { -20, +20, -18, +18 }
     *   -> forward vy = 1 mm/step (lateral bias), turn = 63 mrad/step. */
    const int32_t pattern[MECANUM_WHEEL_COUNT] = { -20, 20, -18, 18 };
    int32_t prev[MECANUM_WHEEL_COUNT];
    int32_t now[MECANUM_WHEEL_COUNT];
    double rx = 0.0;
    double ry = 0.0;
    double nx = 0.0;
    double ny = 0.0;
    int64_t rth = 0;
    int64_t theta_total;
    int step;
    int i;
    double arc_drift;
    double naive_drift;

    printf("[3] ★ in-place spin 360 deg (100 steps, 1 mm lateral quantization bias per step)\n");

    odom_reset(&p, &t);
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        prev[i] = 0;
    }

    for (step = 0; step < 100; ++step) {
        mecanum_twist_t inc;
        for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
            now[i] = prev[i] + pattern[i];
        }
        ck_true("[3] update succeeded", odom_update(&p, &t, &g, now, prev, &inc) == 1);
        ck_near64("[3] no forward body displacement per step (no x expected)", inc.vx_mm_s, 0, 0);
        ck_near64("[3] lateral increment is 1 mm per step", inc.vy_mm_s, 1, 0);
        ref_arc_step(&rx, &ry, &rth, inc.vx_mm_s, inc.vy_mm_s, inc.omega_mrad_s);
        naive_step(&nx, &ny, inc.vx_mm_s, inc.vy_mm_s);
        for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
            prev[i] = now[i];
        }
    }

    theta_total = t.theta_mrad;
    arc_drift   = sqrt((double)p.x_mm * (double)p.x_mm + (double)p.y_mm * (double)p.y_mm);
    naive_drift = sqrt(nx * nx + ny * ny);

    printf("  [info] accumulated turn = %ld mrad (= %.2f deg, target 360 deg = 6283 mrad)\n",
           (long)theta_total, (double)theta_total / 1000.0 * 57.29577951308232);
    printf("  [info] arc-integration position drift = %.3f mm (x=%ld, y=%ld)\n",
           arc_drift, (long)p.x_mm, (long)p.y_mm);
    printf("  [info] double-precision arc reference drift = %.3f mm\n",
           sqrt(rx * rx + ry * ry));
    printf("  [info] naive Euler drift = %.3f mm  <- exactly what arc integration must eliminate\n",
           naive_drift);

    ck_true("[3] arc integration: position drift <= 5 mm", arc_drift <= 5.0);
    ck_true("[3] naive accumulation: drift is far larger (>= 50 mm), so the criterion is meaningful",
            naive_drift >= 50.0);
    ck_near64("[3] matches the double reference (<= 2 mm)", p.x_mm, (int64_t)rx, 2);
    ck_near64("[3] y matches the double reference (<= 2 mm)", p.y_mm, (int64_t)ry, 2);
    ck_true("[3] turn lands near 360 deg (6000..6600 mrad)",
            theta_total >= 6000 && theta_total <= 6600);
}

/* ================================================================== */
/* [4] ★ Known arc trajectory                                          */
/* ================================================================== */

static void test_known_arc(void)
{
    mecanum_geom_t g = geom();
    odom_pose_t p;
    odom_total_t t;
    /* wheel displacement per step {44, 56, 44, 56}:
     *   forward vx = 50 mm, vy = 0, dtheta = 20 mrad
     *   => a standard arc with radius R = vx/omega = 50/0.020 = 2500 mm */
    const int32_t pattern[MECANUM_WHEEL_COUNT] = { 44, 56, 44, 56 };
    const int steps = 40;
    const int32_t dx_step = 50;
    const int32_t dth_step = 20; /* mrad */
    int32_t prev[MECANUM_WHEEL_COUNT];
    int32_t now[MECANUM_WHEEL_COUNT];
    double rx = 0.0;   /* double reference with per-step rounding */
    double ry = 0.0;
    double cx = 0.0;   /* continuous (no rounding) double reference */
    double cy = 0.0;
    double nx = 0.0;
    double ny = 0.0;
    int64_t rth = 0;
    int64_t cth = 0;
    double radius;
    double theta_end;
    double truth_x;
    double truth_y;
    double err_round;  /* integer vs per-step-rounding reference: fixed-point trig + sinc correctness */
    double err_cont;   /* continuous reference vs analytic truth: the arc integration algorithm itself */
    double err_quant;  /* integer vs continuous reference: quantization from 1 mm per-step rounding */
    double err_truth;  /* integer vs analytic truth: composition of the two items above */
    double err_naive;  /* naive Euler vs analytic truth: control proving the criterion is meaningful */
    double quant_bound;
    int step;
    int i;

    printf("[4] ★ known arc trajectory (%d steps x %ld mm forward + %ld mrad turn)\n",
           steps, (long)dx_step, (long)dth_step);

    radius    = (double)dx_step / ((double)dth_step / 1000.0); /* 2500 mm */
    theta_end = (double)steps * (double)dth_step / 1000.0;     /* 0.8 rad */
    truth_x   = radius * sin(theta_end);
    truth_y   = radius * (1.0 - cos(theta_end));

    odom_reset(&p, &t);
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        prev[i] = 0;
    }

    for (step = 0; step < steps; ++step) {
        mecanum_twist_t inc;
        for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
            now[i] = prev[i] + pattern[i];
        }
        ck_true("[4] update succeeded", odom_update(&p, &t, &g, now, prev, &inc) == 1);
        ref_arc_step(&rx, &ry, &rth, inc.vx_mm_s, inc.vy_mm_s, inc.omega_mrad_s);
        ref_arc_step_exact(&cx, &cy, &cth, inc.vx_mm_s, inc.vy_mm_s, inc.omega_mrad_s);
        naive_step(&nx, &ny, inc.vx_mm_s, inc.vy_mm_s);
        for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
            prev[i] = now[i];
        }
    }

    err_round = hypot((double)p.x_mm - rx, (double)p.y_mm - ry);
    err_cont  = hypot(cx - truth_x, cy - truth_y);
    err_quant = hypot((double)p.x_mm - cx, (double)p.y_mm - cy);
    err_truth = hypot((double)p.x_mm - truth_x, (double)p.y_mm - truth_y);
    err_naive = hypot(nx - truth_x, ny - truth_y);

    /* each step loses at most 0.5 mm on each axis, so the strict bound on accumulated Euclidean
       error is 0.5*sqrt(2)*N = 0.7071*N (plus a 1 mm margin). */
    quant_bound = 0.70710678 * (double)steps + 1.0;

    printf("  [info] arc radius R = %.1f mm, accumulated turn = %.4f rad\n", radius, theta_end);
    printf("  [info] analytic truth    : x = %.3f mm, y = %.3f mm\n", truth_x, truth_y);
    printf("  [info] continuous arc ref: x = %.3f mm, y = %.3f mm\n", cx, cy);
    printf("  [info] integer arc integ.: x = %ld mm, y = %ld mm\n",
           (long)p.x_mm, (long)p.y_mm);
    printf("  [info] per-step rounding : x = %ld mm, y = %ld mm\n", (long)rx, (long)ry);
    printf("  [info] error breakdown:\n");
    printf("         A algorithm   (cont ref vs truth)  = %8.4f mm\n", err_cont);
    printf("         B fixed point (int vs rounded ref) = %8.4f mm\n", err_round);
    printf("         C rounding    (int vs cont ref)    = %8.4f mm  (theory bound %.1f mm)\n",
           err_quant, quant_bound);
    printf("         D composite   (int vs truth)       = %8.4f mm\n", err_truth);
    printf("         E naive Euler (control, vs truth)  = %8.4f mm\n", err_naive);

    /* ★ The criterion is split into five independent ones instead of one catch-all "integer vs truth":
       A proves arc integration is exact for constant-curvature paths (should be 0 in theory);
       B proves the home-grown Q20 fixed-point trig + Q30 sinc introduce no visible error;
       C spells out the quantization bound of "round to 1 mm every step";
       D is the composition of A/B/C, cross-checking that the breakdown has no missing term;
       E shows skipping arc integration is not viable (else A-D could all be dismissed as loose). */
    ck_true("[4] A the arc algorithm is exact (cont ref vs truth <= 0.5 mm)", err_cont <= 0.5);
    ck_true("[4] B fixed-point impl correct (int vs rounded ref <= 2 mm)", err_round <= 2.0);
    ck_true("[4] C rounding quantization within the derived bound (<= 0.7071*N + 1 mm)", err_quant <= quant_bound);
    ck_true("[4] D error decomposes (total <= bound C + bound A)",
            err_truth <= quant_bound + 0.5);
    ck_true("[4] E naive accumulation error is far larger (>= 200 mm), so the test is meaningful",
            err_naive >= 200.0);

    /* ---- step-count experiment: same step length (50 mm + 20 mrad), only N changes ----
       If the 6.4 mm came from chord-arc discretization, more steps should shrink the error.
       Measured result is the opposite: the continuous reference hugs the analytic truth at 4 / 40 / 400
       steps (arc integration is analytically exact, independent of N), while the integer version rounds
       every step, so drift grows with N -- directly indicting per-step rounding, not discretization. */
    {
        static const int ntest[3] = { 4, 40, 400 };
        printf("  [info] step-count experiment (same step length, R = 2500 mm):\n");
        for (i = 0; i < 3; ++i) {
            int64_t ix = 0;
            int64_t iy = 0;
            double ox = 0.0;
            double oy = 0.0;
            double th_end = (double)ntest[i] * (double)dth_step / 1000.0;
            double tx = radius * sin(th_end);
            double ty = radius * (1.0 - cos(th_end));
            double e_cont = 0.0;
            double e_quant = 0.0;

            run_arc(ntest[i], &ix, &iy, &ox, &oy);
            e_cont  = hypot(ox - tx, oy - ty);
            e_quant = hypot((double)ix - ox, (double)iy - oy);

            printf("         N = %3d steps: cont ref vs truth %7.4f mm | "
                   "integer vs cont ref %9.3f mm (%.4f mm/step)\n",
                   ntest[i], e_cont, e_quant, e_quant / (double)ntest[i]);

            if (ntest[i] == 40) {
                ck_true("[4] step experiment: at 40 steps the algorithm error is still <= 0.5 mm", e_cont <= 0.5);
                ck_true("[4] step experiment: at 40 steps quantization is visible (>= 1 mm)", e_quant >= 1.0);
                ck_true("[4] step experiment: at 40 steps quantization is within bounds",
                        e_quant <= 0.70710678 * 40.0 + 1.0);
            }
            if (ntest[i] == 400) {
                ck_true("[4] step experiment: at 400 steps the algorithm error is still <= 0.5 mm (no discretization error)",
                        e_cont <= 0.5);
                ck_true("[4] step experiment: at 400 steps quantization is within bounds",
                        e_quant <= 0.70710678 * 400.0 + 1.0);
            }
        }
    }
}

/* ================================================================== */
/* [5] Angle normalization and consistency                             */
/* ================================================================== */

static void test_angle_normalization(void)
{
    odom_pose_t p;
    odom_total_t t;
    int i;

    printf("[5] angle normalization and pose/total consistency\n");

    odom_reset(&p, &t);
    ck_near64("[5] initial theta", (int64_t)p.theta_mrad, 0, 0);

    /* small angles are not normalized: pose equals total itself */
    {
        mecanum_geom_t g = geom();
        int32_t prev[MECANUM_WHEEL_COUNT] = { 0, 0, 0, 0 };
        int32_t now[MECANUM_WHEEL_COUNT]  = { 0, 0, 0, 0 };
        (void)odom_update(&p, &t, &g, now, prev, 0);
        ck_near64("[5] with zero increment pose.theta == total.theta",
                   (int64_t)p.theta_mrad, t.theta_mrad, 0);
    }

    /* verify the normalization boundary directly: push total to 200000 mrad.
     * one turn = 6284 mrad => 200000 - (-1088) = 201088 = 32 x 6284,
     * expected pose.theta = 200000 - 32x6284 = -1088 */
    odom_reset(&p, &t);
    {
        mecanum_geom_t g = geom();
        int32_t prev[MECANUM_WHEEL_COUNT] = { 0, 0, 0, 0 };
        int32_t now[MECANUM_WHEEL_COUNT];
        /* pure spin mode, each step's dtheta set by L; accumulating 200000 mrad takes over
         * three thousand steps, so instead construct a total that has "already turned 200000 mrad"
         * and advance it by one step. */
        t.theta_mrad = 200000 - 63; /* one more +63 step lands exactly on 200000 */
        p.theta_mrad = 0;
        for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
            now[i] = prev[i] + ((i % 2 == 0) ? -19 : 19);
        }
        (void)odom_update(&p, &t, &g, now, prev, 0);
        printf("  [info] total.theta = %ld mrad -> pose.theta = %ld mrad\n",
               (long)t.theta_mrad, (long)p.theta_mrad);
        ck_near64("[5] total keeps accumulating, not zeroed", t.theta_mrad, 200000, 0);
        ck_near64("[5] pose normalized to (-3142, 3142]",
                   (int64_t)p.theta_mrad, -1088, 0);
    }

    /* ------------------------------------------------------------------
     * ★ acceptance add-on (contract section 5.1): after >= 2 full turns of continuous spin, verify that
     *   "normalization is an angle-equivalent transform".
     *
     * criterion b (pose - total is exactly a nonzero integer multiple of one turn) is key -- checking
     * only "the value is inside the interval" is not enough: the old 180000-constant version also passed
     * the interval assertion, but 2x180000 = 360000 mrad = 360 rad = 57.2958 turns (not an integer), so
     * once |theta| passes 180000 the heading jumps 106.48 deg. Criterion b pins two things at once:
     * (1) the wrap truly is a whole turn; (2) wrapping actually happened (multiplier nonzero) -- that is why it beats the interval check.
     * ------------------------------------------------------------------ */
    odom_reset(&p, &t);
    {
        mecanum_geom_t g = geom();
        int32_t prev[MECANUM_WHEEL_COUNT];
        int32_t now[MECANUM_WHEEL_COUNT];
        int step;

        for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
            prev[i] = 0;
        }

        /* symmetric spin mode: wheel odometry in pairwise opposite pairs => vx = vy = 0 (pure rotation).
         * each step dtheta = 63 mrad => 300 steps = 18900 mrad = 3.008 turns (> 2 turns). */
        for (step = 0; step < 300; ++step) {
            for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
                now[i] = prev[i] + ((i % 2 == 0) ? -19 : 19);
            }
            (void)odom_update(&p, &t, &g, now, prev, 0);
            for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
                prev[i] = now[i];
            }
        }

        printf("  [info] spin 300 steps: total.theta = %ld mrad (%.3f turns),"
               "pose.theta = %ld mrad\n",
               (long)t.theta_mrad, (double)t.theta_mrad / 6283.185,
               (long)p.theta_mrad);

        /* criterion a: the canonical heading lies in (-3142, 3142] */
        ck_true("[5] criterion a: pose.theta in (-3142, 3142]",
                p.theta_mrad > -3142 && p.theta_mrad <= 3142);

        /* criterion b: the difference is a nonzero integer multiple of one turn 6284 => normalization
         * preserves angle equivalence. Only error source: using 6284 for the true 6283.185 (2.4 mrad over 3 turns). */
        {
            int64_t diff = (int64_t)p.theta_mrad - t.theta_mrad;
            int64_t turns = diff / 6284;
            printf("  [info] pose - total = %ld mrad = %ld x 6284 (%ld turns)\n",
                   (long)diff, (long)turns, (long)turns);
            ck_true("[5] criterion b: difference is divisible by one turn 6284", (diff % 6284) == 0);
            ck_true("[5] criterion b: multiplier is nonzero (wrapping really happened)", turns != 0);
        }

        /* criterion c: a pure spin produces no spurious position drift (basic arc integration requirement) */
        ck_near64("[5] criterion c: x drift <= 5 mm after 3 turns", p.x_mm, 0, 5);
        ck_near64("[5] criterion c: y drift <= 5 mm after 3 turns", p.y_mm, 0, 5);
    }
}

/* ================================================================== */
/* [6] Invalid input                                                   */
/* ================================================================== */

static void test_invalid_input(void)
{
    mecanum_geom_t g = geom();
    odom_pose_t p;
    odom_total_t t;
    int32_t a[MECANUM_WHEEL_COUNT] = { 0, 0, 0, 0 };
    int32_t b[MECANUM_WHEEL_COUNT] = { 0, 0, 0, 0 };

    printf("[6] Invalid input\n");

    odom_reset(&p, &t);
    p.x_mm = 7;

    ck_true("[6] pose=NULL returns 0", odom_update(0, &t, &g, a, b, 0) == 0);
    ck_true("[6] total=NULL returns 0", odom_update(&p, 0, &g, a, b, 0) == 0);
    ck_true("[6] geom=NULL returns 0", odom_update(&p, &t, 0, a, b, 0) == 0);
    ck_true("[6] d_now=NULL returns 0", odom_update(&p, &t, &g, 0, b, 0) == 0);
    ck_true("[6] d_prev=NULL returns 0", odom_update(&p, &t, &g, a, 0, 0) == 0);

    {
        mecanum_geom_t bad = geom();
        bad.half_wheelbase_mm = 0;
        bad.half_track_mm = 0;
        ck_true("[6] invalid geometry returns 0", odom_update(&p, &t, &bad, a, b, 0) == 0);
    }

    ck_near64("[6] invalid input leaves the pose untouched", p.x_mm, 7, 0);
    ck_true("[6] twist_out may be NULL", odom_update(&p, &t, &g, a, b, 0) == 1);
}

/* ================================================================== */
/* [7] Semantics of twist_out                                          */
/* ================================================================== */

static void test_twist_out(void)
{
    mecanum_geom_t g = geom();
    odom_pose_t p;
    odom_total_t t;
    int32_t prev[MECANUM_WHEEL_COUNT] = { 100, 100, 100, 100 };
    int32_t now[MECANUM_WHEEL_COUNT]  = { 110, 110, 110, 110 };
    mecanum_twist_t inc;

    printf("[7] twist_out semantics (it holds this step's increment, not a rate)\n");

    odom_reset(&p, &t);
    ck_true("[7] update succeeded", odom_update(&p, &t, &g, now, prev, &inc) == 1);
    ck_near64("[7] vx field = this step's dx (mm/step)", inc.vx_mm_s, 10, 0);
    ck_near64("[7] vy field = this step's dy", inc.vy_mm_s, 0, 0);
    ck_near64("[7] omega field = this step's dtheta", inc.omega_mrad_s, 0, 0);
    ck_near64("[7] pose x advances in sync", p.x_mm, 10, 0);
}

/* ================================================================== */
/* [8] Zero increment and determinism                                  */
/* ================================================================== */

static void test_zero_and_determinism(void)
{
    mecanum_geom_t g = geom();
    odom_pose_t p1;
    odom_total_t t1;
    odom_pose_t p2;
    odom_total_t t2;
    int32_t zero[MECANUM_WHEEL_COUNT]  = { 0, 0, 0, 0 };
    int32_t same[MECANUM_WHEEL_COUNT]  = { 5, 5, 5, 5 };
    int32_t samep[MECANUM_WHEEL_COUNT] = { 5, 5, 5, 5 };
    int i;
    int identical = 1;

    printf("[8] Zero increment and determinism\n");

    odom_reset(&p1, &t1);
    ck_true("[8] zero increment still returns 1 (not an error)",
            odom_update(&p1, &t1, &g, zero, zero, 0) == 1);
    ck_near64("[8] zero increment does not move the pose", p1.x_mm, 0, 0);

    odom_reset(&p1, &t1);
    odom_reset(&p2, &t2);
    for (i = 0; i < 200; ++i) {
        (void)odom_update(&p1, &t1, &g, same, samep, 0);
        (void)odom_update(&p2, &t2, &g, same, samep, 0);
        samep[0] = same[0];
        samep[1] = same[1];
        samep[2] = same[2];
        samep[3] = same[3];
        same[0] += 1;
        same[1] += 1;
        same[2] += 1;
        same[3] += 1;
    }
    if (p1.x_mm != p2.x_mm || p1.y_mm != p2.y_mm || p1.theta_mrad != p2.theta_mrad) {
        identical = 0;
    }
    ck_true("[8] both runs are identical", identical != 0);
    ck_near64("[8] 200 steps x 1 mm = 200 mm", p1.x_mm, 200, 2);
}

/* ================================================================== */

int main(void)
{
    printf("=== odom arc-integration odometry tests ===\n");

    test_reset();
    test_straight();
    test_spin_in_place();
    test_known_arc();
    test_angle_normalization();
    test_invalid_input();
    test_twist_out();
    test_zero_and_determinism();

    printf("--------------------------------------------------\n");
    printf("checks = %d, failures = %d\n", g_checks, g_fails);
    if (g_fails == 0) {
        printf("RESULT: PASSED\n");
        return EXIT_SUCCESS;
    }
    printf("RESULT: FAILED\n");
    return EXIT_FAILURE;
}
