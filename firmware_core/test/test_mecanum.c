/*
 * test_mecanum.c - test bench for the mecanum module
 *
 * Coverage:
 *   [1] Inverse -> forward kinematics round trip (all polarity combinations)
 *   [2] Structural cases (pure vx / pure vy / pure omega / zero speed / polarity flip)
 *   [3] Saturation normalization (same scale on all wheels, direction kept, idempotent)
 *   [4] Dead-zone lifting (incl. the hard safety rule "all-zero wheels are never lifted")
 *   [5] Mounting map (round trip over all 24 wheel orders x 16 sign combinations)
 *   [6] ROS2 wheel order swap
 *   [7] Parameter validation and invalid input (incl. L=0 divide-by-zero guard, NULL pointers)
 *   [8] One-shot solve mechanism (flags, ordering, stop semantics)
 *   [9] Cross-check against two independent implementations: ROS2 / WPILib
 *  [10] Resolution reference (print only)
 *
 * Note: this is a test file, so double is allowed - but only in [9] / [10] as an "external reference"
 *      and for "printing". src/ and include/ stay float-free and third-party-dependency-free
 *      (enforced by make check-purity).
 */

#include <stdio.h>
#include <stdlib.h>

#include "mecanum.h"

static int g_checks = 0;
static int g_fails  = 0;

#define TOL_V_MM_S 1
#define TOL_O_MRAD_S 3

static int32_t iabs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static int32_t imax4(const int32_t v[4])
{
    int32_t m = iabs32(v[0]);
    int i;
    for (i = 1; i < 4; ++i) {
        if (iabs32(v[i]) > m) {
            m = iabs32(v[i]);
        }
    }
    return m;
}

static void check_close(const char *tag, int idx, int32_t got, int32_t want, int32_t tol)
{
    int32_t err = iabs32(got - want);
    ++g_checks;
    if (err > tol) {
        ++g_fails;
        if (g_fails <= 40) {
            printf("  [FAIL] %s #%d: got %ld, want %ld, tol %ld, err %ld\n",
                   tag, idx, (long)got, (long)want, (long)tol, (long)err);
        }
    }
}

static void check_eq(const char *tag, int idx, int32_t got, int32_t want)
{
    check_close(tag, idx, got, want, 0);
}

static void check_u8(const char *tag, int idx, uint8_t got, uint8_t want)
{
    ++g_checks;
    if (got != want) {
        ++g_fails;
        if (g_fails <= 40) {
            printf("  [FAIL] %s #%d: got %u, want %u\n", tag, idx, (unsigned)got,
                   (unsigned)want);
        }
    }
}

/* ================================================================== */
/* [1] Round-trip consistency                                          */
/* ================================================================== */

static void test_roundtrip(void)
{
    static const int32_t V[] = { -5000, -2000, -999, -1, 0, 1, 999, 2000, 5000 };
    static const int32_t W[] = { -5000, -2000, -999, -1, 0, 1, 999, 2000, 5000 };
    const int nv = (int)(sizeof(V) / sizeof(V[0]));
    const int nw = (int)(sizeof(W) / sizeof(W[0]));

    int32_t max_ev = 0;
    int32_t max_eo = 0;
    long cases = 0;
    int pv, po, i, j, k;

    printf("[1] inverse -> forward kinematics round trip\n");

    for (pv = 0; pv < 2; ++pv) {
        for (po = 0; po < 2; ++po) {
            mecanum_geom_t g = mecanum_default_geom();
            g.vy_polarity    = (pv == 0) ? 1 : -1;
            g.omega_polarity = (po == 0) ? 1 : -1;

            for (i = 0; i < nv; ++i) {
                for (j = 0; j < nv; ++j) {
                    for (k = 0; k < nw; ++k) {
                        mecanum_twist_t t;
                        mecanum_twist_t back;
                        mecanum_wheels_t w;
                        int32_t ev;
                        int32_t eo;

                        t.vx_mm_s      = V[i];
                        t.vy_mm_s      = V[j];
                        t.omega_mrad_s = W[k];

                        mecanum_inverse(&g, &t, &w);
                        mecanum_forward(&g, &w, &back);

                        ev = iabs32(back.vx_mm_s - t.vx_mm_s);
                        if (iabs32(back.vy_mm_s - t.vy_mm_s) > ev) {
                            ev = iabs32(back.vy_mm_s - t.vy_mm_s);
                        }
                        eo = iabs32(back.omega_mrad_s - t.omega_mrad_s);
                        if (ev > max_ev) { max_ev = ev; }
                        if (eo > max_eo) { max_eo = eo; }

                        check_close("roundtrip.vx", (int)cases, back.vx_mm_s, t.vx_mm_s,
                                    TOL_V_MM_S);
                        check_close("roundtrip.vy", (int)cases, back.vy_mm_s, t.vy_mm_s,
                                    TOL_V_MM_S);
                        check_close("roundtrip.omega", (int)cases, back.omega_mrad_s,
                                    t.omega_mrad_s, TOL_O_MRAD_S);
                        ++cases;
                    }
                }
            }
        }
    }
    printf("  [info] round-trip cases %ld (4 polarity combos)\n", cases);
    printf("  [info] measured max error: linear %ld mm/s, angular %ld mrad/s\n",
           (long)max_ev, (long)max_eo);
}

/* ================================================================== */
/* [2] Structural cases                                                */
/* ================================================================== */

static void test_structure(void)
{
    mecanum_geom_t g = mecanum_default_geom(); /* 150/150 => L = 300 */
    mecanum_twist_t t;
    mecanum_wheels_t w;
    int i;

    printf("[2] Structural cases\n");

    /* pure vx: all four wheels same value and same sign */
    t.vx_mm_s = 500;
    t.vy_mm_s = 0;
    t.omega_mrad_s = 0;
    mecanum_inverse(&g, &t, &w);
    for (i = 0; i < 4; ++i) {
        check_eq("purevx.wheelspeed", i, w.wheel_mm_s[i], 500);
    }

    /* pure vy: signs must be - + + - (FL FR RL RR) */
    t.vx_mm_s = 0;
    t.vy_mm_s = 400;
    t.omega_mrad_s = 0;
    mecanum_inverse(&g, &t, &w);
    {
        static const int32_t want[4] = { -400, 400, 400, -400 };
        for (i = 0; i < 4; ++i) {
            check_eq("purevy.wheelspeed", i, w.wheel_mm_s[i], want[i]);
        }
    }

    /* pure omega = 1000 mrad/s, L = 300 => rot = 300 mm/s, signs - + - + */
    t.vx_mm_s = 0;
    t.vy_mm_s = 0;
    t.omega_mrad_s = 1000;
    mecanum_inverse(&g, &t, &w);
    {
        static const int32_t want[4] = { -300, 300, -300, 300 };
        for (i = 0; i < 4; ++i) {
            check_eq("pureomega.wheelspeed", i, w.wheel_mm_s[i], want[i]);
        }
    }

    /* zero speed => all four wheels zero */
    t = mecanum_zero_twist();
    mecanum_inverse(&g, &t, &w);
    for (i = 0; i < 4; ++i) {
        check_eq("zerospeed.wheelspeed", i, w.wheel_mm_s[i], 0);
    }

    /* forward kinematics recovers each pure component */
    w.wheel_mm_s[0] = 500;
    w.wheel_mm_s[1] = 500;
    w.wheel_mm_s[2] = 500;
    w.wheel_mm_s[3] = 500;
    mecanum_forward(&g, &w, &t);
    check_eq("fwd.purevx", 0, t.vx_mm_s, 500);
    check_eq("fwd.purevy", 1, t.vy_mm_s, 0);
    check_eq("fwd.pureomega", 2, t.omega_mrad_s, 0);

    /* polarity flip: with vy_polarity = -1 a pure vy command gives the opposite wheel pattern */
    {
        mecanum_geom_t g2 = g;
        static const int32_t want[4] = { 400, -400, -400, 400 };
        g2.vy_polarity = -1;
        t.vx_mm_s = 0;
        t.vy_mm_s = 400;
        t.omega_mrad_s = 0;
        mecanum_inverse(&g2, &t, &w);
        for (i = 0; i < 4; ++i) {
            check_eq("polarity-1.purevy", i, w.wheel_mm_s[i], want[i]);
        }
        /* forward kinematics must flip too to keep the round trip consistent */
        {
            mecanum_twist_t back;
            mecanum_forward(&g2, &w, &back);
            check_eq("polarity-1.fwdvy", 0, back.vy_mm_s, 400);
        }
    }
}

/* ================================================================== */
/* [3] Saturation normalization                                        */
/* ================================================================== */

static void test_desaturate(void)
{
    mecanum_wheels_t in;
    mecanum_wheels_t out;
    int r;
    int i;

    printf("[3] Saturation normalization\n");

    /* within limit: copy as-is, return 0 */
    in.wheel_mm_s[0] = 100;
    in.wheel_mm_s[1] = -200;
    in.wheel_mm_s[2] = 300;
    in.wheel_mm_s[3] = 0;
    r = mecanum_desaturate(&in, 500, &out);
    check_eq("within-limit.ret", 0, r, 0);
    for (i = 0; i < 4; ++i) {
        check_eq("within-limit.wheelspeed", i, out.wheel_mm_s[i], in.wheel_mm_s[i]);
    }

    /* over limit: peak pinned exactly at the limit, ratios kept, direction unchanged */
    in.wheel_mm_s[0] = 2000;
    in.wheel_mm_s[1] = -4000;
    in.wheel_mm_s[2] = 1000;
    in.wheel_mm_s[3] = -6000;
    r = mecanum_desaturate(&in, 3000, &out);
    check_eq("over-limit.ret", 0, r, 1);
    check_eq("over-limit.peak-at-limit", 0, out.wheel_mm_s[3], -3000);
    check_eq("over-limit.samescale0", 0, out.wheel_mm_s[0], 1000);
    check_eq("over-limit.samescale1", 0, out.wheel_mm_s[1], -2000);
    check_eq("over-limit.samescale2", 0, out.wheel_mm_s[2], 500);
    check_eq("over-limit.peak", 0, imax4(out.wheel_mm_s), 3000);

    /* idempotent: renormalizing an already normalized set changes nothing */
    {
        mecanum_wheels_t out2;
        r = mecanum_desaturate(&out, 3000, &out2);
        check_eq("idempotent.ret", 0, r, 0);
        for (i = 0; i < 4; ++i) {
            check_eq("idempotent.wheelspeed", i, out2.wheel_mm_s[i], out.wheel_mm_s[i]);
        }
    }

    /* limit <= 0 means no limiting */
    r = mecanum_desaturate(&in, 0, &out);
    check_eq("no-limit.ret", 0, r, 0);
    check_eq("no-limit.wheelspeed", 3, out.wheel_mm_s[3], -6000);

    /* zero input: peak 0, must not divide by zero */
    {
        mecanum_wheels_t z;
        mecanum_zero_wheels(&z);
        r = mecanum_desaturate(&z, 3000, &out);
        check_eq("zero-input.ret", 0, r, 0);
        check_eq("zero-input.wheelspeed", 0, out.wheel_mm_s[0], 0);
    }
}

/* ================================================================== */
/* [4] Dead-zone lifting                                               */
/* ================================================================== */

static void test_lift_deadzone(void)
{
    mecanum_wheels_t in;
    mecanum_wheels_t out;
    int r;
    int i;

    printf("[4] Dead-zone lifting\n");

    /* hard safety rule: all-zero wheels are never lifted (else a parked car would creep) */
    for (i = 0; i < 4; ++i) {
        in.wheel_mm_s[i] = 0;
    }
    r = mecanum_lift_deadzone(&in, 300, &out);
    check_eq("all-zero.ret", 0, r, 0);
    for (i = 0; i < 4; ++i) {
        check_eq("all-zero.wheelspeed", i, out.wheel_mm_s[i], 0);
    }

    /* peak already above the dead zone: leave untouched */
    in.wheel_mm_s[0] = 400;
    in.wheel_mm_s[1] = -100;
    in.wheel_mm_s[2] = 250;
    in.wheel_mm_s[3] = -50;
    r = mecanum_lift_deadzone(&in, 300, &out);
    check_eq("above-deadzone.ret", 0, r, 0);
    for (i = 0; i < 4; ++i) {
        check_eq("above-deadzone.wheelspeed", i, out.wheel_mm_s[i], in.wheel_mm_s[i]);
    }

    /* peak below the dead zone: scale everything up to the dead zone, direction kept */
    in.wheel_mm_s[0] = 40;
    in.wheel_mm_s[1] = -80;
    in.wheel_mm_s[2] = 20;
    in.wheel_mm_s[3] = 0;
    r = mecanum_lift_deadzone(&in, 400, &out);
    check_eq("below-deadzone.ret", 0, r, 1);
    check_eq("below-deadzone.peak-eq-deadzone", 0, out.wheel_mm_s[1], -400);
    check_eq("below-deadzone.scale0", 0, out.wheel_mm_s[0], 200);
    check_eq("below-deadzone.scale2", 0, out.wheel_mm_s[2], 100);
    check_eq("below-deadzone.zero-stays-zero", 0, out.wheel_mm_s[3], 0);

    /* dead zone disabled */
    r = mecanum_lift_deadzone(&in, 0, &out);
    check_eq("disabled.ret", 0, r, 0);
    check_eq("disabled.wheelspeed", 1, out.wheel_mm_s[1], -80);

    /* single nonzero wheel: still scaled up (a single requested wheel must also clear the dead zone) */
    in.wheel_mm_s[0] = 10;
    in.wheel_mm_s[1] = 0;
    in.wheel_mm_s[2] = 0;
    in.wheel_mm_s[3] = 0;
    r = mecanum_lift_deadzone(&in, 400, &out);
    check_eq("single-wheel.ret", 0, r, 1);
    check_eq("single-wheel.wheelspeed", 0, out.wheel_mm_s[0], 400);
    for (i = 1; i < 4; ++i) {
        check_eq("single-wheel.rest-zero", i, out.wheel_mm_s[i], 0);
    }
}

/* ================================================================== */
/* [5] Mounting map                                                    */
/* ================================================================== */

static const uint8_t k_perms[24][4] = {
    {0,1,2,3},{0,1,3,2},{0,2,1,3},{0,2,3,1},{0,3,1,2},{0,3,2,1},
    {1,0,2,3},{1,0,3,2},{1,2,0,3},{1,2,3,0},{1,3,0,2},{1,3,2,0},
    {2,0,1,3},{2,0,3,1},{2,1,0,3},{2,1,3,0},{2,3,0,1},{2,3,1,0},
    {3,0,1,2},{3,0,2,1},{3,1,0,2},{3,1,2,0},{3,2,0,1},{3,2,1,0}
};

static void test_mount(void)
{
    int p;
    int s;
    long cases = 0;

    printf("[5] Mounting map\n");

    for (p = 0; p < 24; ++p) {
        for (s = 0; s < 16; ++s) {
            mecanum_mount_t m;
            mecanum_wheels_t w;
            mecanum_wheels_t back;
            int32_t motor[4];
            int i;

            for (i = 0; i < 4; ++i) {
                m.motor_of_wheel[i] = k_perms[p][i];
                m.sign_of_wheel[i]  = ((s >> i) & 1) ? -1 : 1;
            }
            check_eq("mount-table-valid", (int)cases, mecanum_mount_is_valid(&m), 1);

            w.wheel_mm_s[0] = 111;
            w.wheel_mm_s[1] = -222;
            w.wheel_mm_s[2] = 333;
            w.wheel_mm_s[3] = -444;

            mecanum_apply_mount(&m, &w, motor);
            mecanum_undo_mount(&m, motor, &back);

            for (i = 0; i < 4; ++i) {
                check_eq("map-roundtrip.wheelspeed", (int)cases, back.wheel_mm_s[i],
                         w.wheel_mm_s[i]);
            }
            /* every motor channel must actually be written (guards against all-zero false passes) */
            for (i = 0; i < 4; ++i) {
                check_eq("map.motor-nonzero", (int)cases, motor[i] != 0 ? 1 : 0, 1);
            }
            ++cases;
        }
    }
    printf("  [info] %ld wheel-order x sign combinations, all round trips exact\n", cases);

    /* under the identity map the motor command must equal the wheel speed */
    {
        mecanum_mount_t m = mecanum_identity_mount();
        mecanum_wheels_t w;
        int32_t motor[4];
        int i;
        w.wheel_mm_s[0] = 10;
        w.wheel_mm_s[1] = 20;
        w.wheel_mm_s[2] = 30;
        w.wheel_mm_s[3] = 40;
        mecanum_apply_mount(&m, &w, motor);
        for (i = 0; i < 4; ++i) {
            check_eq("identity-map.motor", i, motor[i], w.wheel_mm_s[i]);
        }
    }

    /* invalid maps must not crash and must not pass as valid */
    {
        mecanum_mount_t bad = mecanum_identity_mount();
        mecanum_wheels_t w;
        int32_t motor[4];
        bad.motor_of_wheel[1] = 0; /* collides with FL, not a bijection */
        check_eq("invalid-map.check", 0, mecanum_mount_is_valid(&bad), 0);

        w.wheel_mm_s[0] = 1;
        w.wheel_mm_s[1] = 2;
        w.wheel_mm_s[2] = 3;
        w.wheel_mm_s[3] = 4;
        motor[0] = motor[1] = motor[2] = motor[3] = 0;
        mecanum_apply_mount(&bad, &w, motor);
        /* FL and FR fight over the same motor channel: later write wins, no out-of-bounds,
           no crash. Invalid tables are not rejected here -- mount_is_valid must block them. */
        check_eq("invalid-map.nocrash0", 0, motor[0], 2);
        check_eq("invalid-map.nocrash2", 0, motor[2], 3);
        check_eq("invalid-map.nocrash3", 0, motor[3], 4);
    }
}

/* ================================================================== */
/* [6] ROS2 wheel order swap                                           */
/* ================================================================== */

static void test_ros2_order(void)
{
    int32_t ros2[4];
    mecanum_wheels_t w;
    int32_t back[4];
    int i;

    printf("[6] ROS2 wheel order swap\n");

    /* ROS2: 0=FL 1=FR 2=RR 3=RL */
    ros2[0] = 10; /* FL */
    ros2[1] = 20; /* FR */
    ros2[2] = 30; /* RR */
    ros2[3] = 40; /* RL */

    mecanum_wheels_from_ros2_order(ros2, &w);
    check_eq("ros2->local.FL", 0, w.wheel_mm_s[MECANUM_FL], 10);
    check_eq("ros2->local.FR", 0, w.wheel_mm_s[MECANUM_FR], 20);
    check_eq("ros2->local.RL", 0, w.wheel_mm_s[MECANUM_RL], 40);
    check_eq("ros2->local.RR", 0, w.wheel_mm_s[MECANUM_RR], 30);

    mecanum_wheels_to_ros2_order(&w, back);
    for (i = 0; i < 4; ++i) {
        check_eq("local->ros2 roundtrip", i, back[i], ros2[i]);
    }

    /* a wheel-order mistake must be caught by this test:
       if RL/RR are not swapped, the pure vy pattern becomes - + - + instead of - + + - */
    {
        mecanum_geom_t g = mecanum_default_geom();
        mecanum_twist_t t = mecanum_zero_twist();
        t.vy_mm_s = 400;
        mecanum_inverse(&g, &t, &w);
        mecanum_wheels_to_ros2_order(&w, back);
        check_eq("ros2order.purevy.0", 0, back[0], -400);
        check_eq("ros2order.purevy.1", 0, back[1], 400);
        check_eq("ros2order.purevy.2", 0, back[2], -400); /* RR */
        check_eq("ros2order.purevy.3", 0, back[3], 400);  /* RL */
    }
}

/* ================================================================== */
/* [7] Parameter validation                                            */
/* ================================================================== */

static void test_validation(void)
{
    mecanum_geom_t g;
    mecanum_wheels_t w;
    mecanum_twist_t t;

    printf("[7] Parameter validation and invalid input\n");

    g = mecanum_default_geom();
    check_eq("default-geom.valid", 0, mecanum_geom_is_valid(&g), 1);
    check_eq("L", 0, mecanum_l_mm(&g), 300);

    g.half_track_mm = -1;
    check_eq("neg-half-track.invalid", 0, mecanum_geom_is_valid(&g), 0);

    g = mecanum_default_geom();
    g.half_wheelbase_mm = 0;
    g.half_track_mm = 0; /* L = 0 */
    check_eq("L=0.invalid", 0, mecanum_geom_is_valid(&g), 0);

    /* L = 0 must never divide by zero: forward kinematics must output all zeros */
    w.wheel_mm_s[0] = 1000;
    w.wheel_mm_s[1] = 1000;
    w.wheel_mm_s[2] = 1000;
    w.wheel_mm_s[3] = 1000;
    mecanum_forward(&g, &w, &t);
    check_eq("L=0.fwdvx", 0, t.vx_mm_s, 0);
    check_eq("L=0.fwdvy", 0, t.vy_mm_s, 0);
    check_eq("L=0.fwdomega", 0, t.omega_mrad_s, 0);

    /* L = 0 inverse kinematics outputs all zeros */
    t = mecanum_zero_twist();
    t.vx_mm_s = 500;
    mecanum_inverse(&g, &t, &w);
    check_eq("L=0.inverse", 0, w.wheel_mm_s[0], 0);

    g = mecanum_default_geom();
    g.vy_polarity = 0;
    check_eq("polarity0.invalid", 0, mecanum_geom_is_valid(&g), 0);
    g.vy_polarity = 2;
    check_eq("polarity2.invalid", 0, mecanum_geom_is_valid(&g), 0);

    /* NULL pointers must not crash */
    mecanum_zero_wheels(&w);
    mecanum_inverse(0, 0, &w);
    check_eq("NULL-inverse", 0, w.wheel_mm_s[0], 0);
    t = mecanum_zero_twist();
    t.vx_mm_s = 77;
    mecanum_forward(0, 0, &t);
    check_eq("NULL-forward", 0, t.vx_mm_s, 0);
    mecanum_forward(&g, 0, &t);
    check_eq("NULL-wheelspeed", 0, t.vx_mm_s, 0);
    mecanum_inverse(0, 0, 0);
    mecanum_inverse(&g, &t, 0);
    mecanum_forward(&g, &w, 0);
    mecanum_zero_wheels(0);
    mecanum_apply_mount(0, 0, 0);
    mecanum_undo_mount(0, 0, 0);
    mecanum_solve(0, 0, 0, 0, 0);
    check_eq("NULL-nothing-crashes", 0, 1, 1);
}

/* ================================================================== */
/* [8] One-shot solve                                                  */
/* ================================================================== */

static void test_solve(void)
{
    mecanum_geom_t g = mecanum_default_geom();
    mecanum_mount_t m = mecanum_identity_mount();
    mecanum_limits_t lim = mecanum_default_limits();
    mecanum_twist_t t;
    mecanum_solve_result_t r;
    int i;

    printf("[8] One-shot solve\n");

    /* zero command + dead-zone compensation on => must still be stopped */
    lim.wheel_max_mm_s      = 1000;
    lim.wheel_deadzone_mm_s = 400;
    t = mecanum_zero_twist();
    mecanum_solve(&g, &lim, &m, &t, &r);
    check_u8("zero-command.idle", 0, r.idle, 1);
    check_u8("zero-command.saturated", 0, r.saturated, 0);
    check_u8("zero-command.lifted", 0, r.lifted, 0);
    for (i = 0; i < 4; ++i) {
        check_eq("zero-command.motor", i, r.motor_mm_s[i], 0);
    }

    /*
     * Large command: saturation.
     * t = (2000, 1000, 4000),L = 300 => rot = 1200
     * Wheels = -200, +4200, +1800, +2200, peak 4200 -> scaled to 1000
     *      = -48, +1000, +429, +524
     * Back through forward kinematics = vx 476, vy 238, omega 953 (= original x 1000/4200, rounded)
     */
    t.vx_mm_s      = 2000;
    t.vy_mm_s      = 1000;
    t.omega_mrad_s = 4000;
    mecanum_solve(&g, &lim, &m, &t, &r);
    check_u8("large-command.saturated", 0, r.saturated, 1);
    check_u8("large-command.lifted", 0, r.lifted, 0);
    check_u8("large-command.idle", 0, r.idle, 0);
    check_eq("large-command.peak-at-limit", 0, imax4(r.motor_mm_s), 1000);
    check_eq("large-command.wheelspeed0", 0, r.motor_mm_s[0], -48);
    check_eq("large-command.wheelspeed1", 0, r.motor_mm_s[1], 1000);
    check_eq("large-command.wheelspeed2", 0, r.motor_mm_s[2], 429);
    check_eq("large-command.wheelspeed3", 0, r.motor_mm_s[3], 524);

    {
        mecanum_twist_t back;
        mecanum_wheels_t w;
        mecanum_undo_mount(&m, r.motor_mm_s, &w);
        mecanum_forward(&g, &w, &back);
        check_close("after-saturation.vx scaled", 0, back.vx_mm_s, 476, 2);
        check_close("after-saturation.vy scaled", 0, back.vy_mm_s, 238, 2);
        check_close("after-saturation.omega scaled", 0, back.omega_mrad_s, 953, 4);
        /* direction unchanged: vy/vx ratio matches the original command */
        check_close("after-saturation.direction-kept", 0, back.vy_mm_s * 2, back.vx_mm_s, 3);
    }

    /* small command: lifted to the dead zone; peak exactly equals the dead zone */
    lim.wheel_max_mm_s      = 3000;
    lim.wheel_deadzone_mm_s = 500;
    t.vx_mm_s      = 60;
    t.vy_mm_s      = 0;
    t.omega_mrad_s = 0;
    mecanum_solve(&g, &lim, &m, &t, &r);
    check_u8("small-command.saturated", 0, r.saturated, 0);
    check_u8("small-command.lifted", 0, r.lifted, 1);
    for (i = 0; i < 4; ++i) {
        check_eq("small-command.motor-lifted-to-deadzone", i, r.motor_mm_s[i], 500);
    }

    /* with dead zone <= limit, the lifted peak never exceeds the limit */
    {
        mecanum_limits_t l2;
        l2.wheel_max_mm_s      = 500;
        l2.wheel_deadzone_mm_s = 500;
        t.vx_mm_s = 60;
        t.vy_mm_s = 0;
        t.omega_mrad_s = 0;
        mecanum_solve(&g, &l2, &m, &t, &r);
        for (i = 0; i < 4; ++i) {
            check_eq("deadzone=limit.no-overshoot", i, r.motor_mm_s[i], 500);
        }
    }

    /* lim is NULL: no shaping at all, pure kinematics + mounting */
    t.vx_mm_s = 2000;
    t.vy_mm_s = 0;
    t.omega_mrad_s = 0;
    mecanum_solve(&g, 0, &m, &t, &r);
    check_u8("no-limit.saturated", 0, r.saturated, 0);
    check_u8("no-limit.lifted", 0, r.lifted, 0);
    for (i = 0; i < 4; ++i) {
        check_eq("no-limit.motor", i, r.motor_mm_s[i], 2000);
    }

    /* invalid geometry: must stop, not drive randomly */
    {
        mecanum_geom_t bad = mecanum_default_geom();
        bad.half_wheelbase_mm = 0;
        bad.half_track_mm = 0;
        t.vx_mm_s = 1000;
        t.vy_mm_s = 1000;
        t.omega_mrad_s = 1000;
        mecanum_solve(&bad, &lim, &m, &t, &r);
        check_u8("invalid-geometry.stopped", 0, r.idle, 1);
        check_eq("invalid-geometry.motor", 0, r.motor_mm_s[0], 0);
    }
}

/* ================================================================== */
/* [9] Cross-check vs ROS2 / WPILib                                    */
/* ================================================================== */

typedef struct { double v[4]; } d4;
typedef struct { double v[3]; } d3;

static double dabs(double x) { return (x < 0.0) ? -x : x; }

static int32_t dround(double x) { return (int32_t)((x < 0.0) ? (x - 0.5) : (x + 0.5)); }

static void check_double(const char *tag, long idx, double got, double want, double tol)
{
    ++g_checks;
    if (dabs(got - want) > tol) {
        ++g_fails;
        if (g_fails <= 40) {
            printf("  [FAIL] %s #%ld: got %.10g, want %.10g, tol %.3g\n", tag, idx, got,
                   want, tol);
        }
    }
}

/*
 * Reference A: ROS2 ros2_controllers / mecanum_drive_controller
 *   S = sum_of_robot_center_projection_on_X_Y_axis = lx + ly
 *   internal wheel order FL, FR, RR, RL (the last two are swapped vs ours)
 *   with S in mm and wheel speed in mm/s the third component comes out in rad/s (r and 1/r cancel)
 */
static void ros2_inverse(double vx, double vy, double w_rad, double S, d4 *w)
{
    w->v[0] = vx - vy - S * w_rad; /* FL */
    w->v[1] = vx + vy + S * w_rad; /* FR */
    w->v[2] = vx + vy - S * w_rad; /* RL (note: placed in our order here) */
    w->v[3] = vx - vy + S * w_rad; /* RR */
}

/* verbatim port of Odometry::update(FL, RL, RR, FR, dt) */
static void ros2_forward(double FL, double RL, double RR, double FR, double S, d3 *c)
{
    c->v[0] = 0.25 * (FL + RL + RR + FR);
    c->v[1] = 0.25 * (-FL + RL - RR + FR);
    c->v[2] = 0.25 / S * (-FL - RL + RR + FR); /* rad/s */
}

/*
 * Reference B: WPILib MecanumDriveKinematics
 *   setRow(0, 0, 1, -1, -(fl.x + fl.y));   FL = (+lx, +ly)
 *   setRow(1, 0, 1,  1,  (fr.x - fr.y));   FR = (+lx, -ly)
 *   setRow(2, 0, 1,  1,  (rl.x - rl.y));   RL = (-lx, +ly)
 *   setRow(3, 0, 1, -1, -(rr.x + rr.y));   RR = (-lx, -ly)
 *   forwardKinematics = inverseKinematics.pseudoInverse();
 * WPILib's wheel order matches this module: FL, FR, RL, RR.
 */
static void wpilib_matrix(double lx, double ly, double M[4][3])
{
    const double flx = +lx, fly = +ly;
    const double frx = +lx, fry = -ly;
    const double rlx = -lx, rly = +ly;
    const double rrx = -lx, rry = -ly;

    M[0][0] = 1.0; M[0][1] = -1.0; M[0][2] = -(flx + fly);
    M[1][0] = 1.0; M[1][1] = +1.0; M[1][2] =  (frx - fry);
    M[2][0] = 1.0; M[2][1] = +1.0; M[2][2] =  (rlx - rly);
    M[3][0] = 1.0; M[3][1] = -1.0; M[3][2] = -(rrx + rry);
}

/* note: C99 does not allow implicit double(*)[3] -> const double(*)[3],
   so the first parameter here is not const (-Wpedantic would reject it). */
static int inv3(double A[3][3], double out[3][3])
{
    double det;
    det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
        - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
        + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    if (dabs(det) < 1e-12) {
        return 0;
    }
    out[0][0] =  (A[1][1] * A[2][2] - A[1][2] * A[2][1]) / det;
    out[0][1] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]) / det;
    out[0][2] =  (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det;
    out[1][0] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]) / det;
    out[1][1] =  (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det;
    out[1][2] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]) / det;
    out[2][0] =  (A[1][0] * A[2][1] - A[1][1] * A[2][0]) / det;
    out[2][1] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]) / det;
    out[2][2] =  (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det;
    return 1;
}

/* actually compute (M^T M)^-1 M^T, with no analytic simplification */
static int wpilib_pinv(double lx, double ly, double P[3][4])
{
    double M[4][3];
    double Mt[3][4];
    double A[3][3];
    double Ainv[3][3];
    int i, j, k;

    wpilib_matrix(lx, ly, M);
    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 3; ++j) {
            Mt[j][i] = M[i][j];
        }
    }
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            double s = 0.0;
            for (k = 0; k < 4; ++k) {
                s += Mt[i][k] * M[k][j];
            }
            A[i][j] = s;
        }
    }
    if (inv3(A, Ainv) == 0) {
        return 0;
    }
    for (i = 0; i < 3; ++i) {
        for (k = 0; k < 4; ++k) {
            double s = 0.0;
            for (j = 0; j < 3; ++j) {
                s += Ainv[i][j] * Mt[j][k];
            }
            P[i][k] = s;
        }
    }
    return 1;
}

static void test_cross_reference(void)
{
    static const int32_t VX[] = { -2000, -700, -1, 0, 1, 123, 2000 };
    static const int32_t VY[] = { -1500, -333, 0, 250, 1500 };
    static const int32_t WW[] = { -3000, -450, -1, 0, 1, 450, 3000 };
    const int nx = (int)(sizeof(VX) / sizeof(VX[0]));
    const int ny = (int)(sizeof(VY) / sizeof(VY[0]));
    const int nw = (int)(sizeof(WW) / sizeof(WW[0]));

    const double lx = 150.0;
    const double ly = 150.0;
    const double K  = lx + ly;
    double P[3][4];
    double M[4][3];
    long cases = 0;
    double worst_a = 0.0;
    double worst_b = 0.0;
    double worst_cv = 0.0;
    double worst_cw = 0.0;
    int pv, po, i, j, k, q;

    printf("[9] Cross-check vs ROS2 / WPILib\n");

    if (wpilib_pinv(lx, ly, P) == 0) {
        printf("  [FAIL] WPILib pseudo-inverse solve failed\n");
        ++g_fails;
        ++g_checks;
        return;
    }

    wpilib_matrix(lx, ly, M);
    printf("  [info] WPILib matrix third column (FL,FR,RL,RR) = %.6g, %.6g, %.6g, %.6g\n",
           M[0][2], M[1][2], M[2][2], M[3][2]);
    printf("  [info] theoretical -(lx+ly), +(lx+ly), -(lx+ly), +(lx+ly) = %.6g, %.6g, %.6g, %.6g\n",
           -K, K, -K, K);
    printf("  [info] pseudo-inverse row 3 / K = %.10g, %.10g, %.10g, %.10g\n",
           P[2][0] / K, P[2][1] / K, P[2][2] / K, P[2][3] / K);

    for (pv = 0; pv < 2; ++pv) {
        for (po = 0; po < 2; ++po) {
            mecanum_geom_t g = mecanum_default_geom();
            g.vy_polarity    = (pv == 0) ? 1 : -1;
            g.omega_polarity = (po == 0) ? 1 : -1;

            for (i = 0; i < nx; ++i) {
                for (j = 0; j < ny; ++j) {
                    for (k = 0; k < nw; ++k) {
                        /* command frame -> physical frame */
                        const double vy_phys = (double)g.vy_polarity * (double)VY[j];
                        const double w_rad   = (double)g.omega_polarity
                                             * (double)WW[k] / 1000.0;

                        mecanum_twist_t t;
                        mecanum_wheels_t mine;
                        d4 a;
                        d4 b;
                        d3 ca;
                        d3 cb;
                        double d;

                        t.vx_mm_s      = VX[i];
                        t.vy_mm_s      = VY[j];
                        t.omega_mrad_s = WW[k];
                        mecanum_inverse(&g, &t, &mine);

                        /* --- A1: my integer inverse solve vs ROS2 closed form --- */
                        ros2_inverse((double)VX[i], vy_phys, w_rad, K, &a);
                        d = 0.0;
                        for (q = 0; q < 4; ++q) {
                            double e = dabs((double)mine.wheel_mm_s[q] - a.v[q]);
                            if (e > d) {
                                d = e;
                            }
                        }
                        if (d > worst_a) {
                            worst_a = d;
                        }
                        ++g_checks;
                        if (d > 1.0) {
                            ++g_fails;
                            if (g_fails <= 40) {
                                printf("  [FAIL] A1 #%ld: max deviation %.6g mm/s\n", cases,
                                       d);
                            }
                        }

                        /* --- A2: WPILib matrix vs ROS2 closed form (both are double) --- */
                        for (q = 0; q < 4; ++q) {
                            b.v[q] = M[q][0] * (double)VX[i] + M[q][1] * vy_phys
                                   + M[q][2] * w_rad;
                        }
                        for (q = 0; q < 4; ++q) {
                            check_double("A2.ROS2 vs WPILib inverse solve", cases, a.v[q], b.v[q],
                                         1e-9);
                        }

                        /* --- B: WPILib pseudo-inverse forward vs ROS2 closed-form forward --- */
                        ros2_forward(a.v[0], a.v[2], a.v[3], a.v[1], K, &ca);
                        cb.v[0] = P[0][0] * a.v[0] + P[0][1] * a.v[1]
                                + P[0][2] * a.v[2] + P[0][3] * a.v[3];
                        cb.v[1] = P[1][0] * a.v[0] + P[1][1] * a.v[1]
                                + P[1][2] * a.v[2] + P[1][3] * a.v[3];
                        cb.v[2] = P[2][0] * a.v[0] + P[2][1] * a.v[1]
                                + P[2][2] * a.v[2] + P[2][3] * a.v[3];
                        /* the third column is -+K, so the pseudo-inverse third component is in rad/s */
                        check_double("B1.pseudo-inverse vs closed-form vx", cases, cb.v[0], ca.v[0], 1e-6);
                        check_double("B1.pseudo-inverse vs closed-form vy", cases, cb.v[1], ca.v[1], 1e-6);
                        check_double("B1.pseudo-inverse vs closed-form omega", cases, cb.v[2] * 1000.0,
                                     ca.v[2] * 1000.0, 1e-6);
                        d = dabs(cb.v[0] - ca.v[0]);
                        if (dabs(cb.v[1] - ca.v[1]) > d) {
                            d = dabs(cb.v[1] - ca.v[1]);
                        }
                        if (dabs(cb.v[2] * 1000.0 - ca.v[2] * 1000.0) > d) {
                            d = dabs(cb.v[2] * 1000.0 - ca.v[2] * 1000.0);
                        }
                        if (d > worst_b) {
                            worst_b = d;
                        }

                        /* --- C: my integer forward solve vs external reference --- */
                        {
                            mecanum_twist_t back;
                            mecanum_wheels_t wl;
                            double dvx, dvy, dw;
                            for (q = 0; q < 4; ++q) {
                                wl.wheel_mm_s[q] = dround(a.v[q]);
                            }
                            mecanum_forward(&g, &wl, &back);

                            dvx = dabs((double)back.vx_mm_s - ca.v[0]);
                            dvy = dabs((double)back.vy_mm_s
                                       - (double)g.vy_polarity * ca.v[1]);
                            dw  = dabs((double)back.omega_mrad_s
                                       - (double)g.omega_polarity * ca.v[2] * 1000.0);
                            if (dvx > worst_cv) { worst_cv = dvx; }
                            if (dvy > worst_cv) { worst_cv = dvy; }
                            if (dw > worst_cw)  { worst_cw = dw; }
                            check_double("C.my-forward vx", cases, (double)back.vx_mm_s,
                                         ca.v[0], 1.0);
                            check_double("C.my-forward vy", cases, (double)back.vy_mm_s,
                                         (double)g.vy_polarity * ca.v[1], 1.0);
                            /* omega tolerance 8: 0.5 rounding error per wheel, amplified by 4L */
                            check_double("C.my-forward omega", cases,
                                         (double)back.omega_mrad_s,
                                         (double)g.omega_polarity * ca.v[2] * 1000.0,
                                         8.0);
                        }

                        ++cases;
                    }
                }
            }
        }
    }

    printf("  [info] cross-check cases %ld (4 polarity combos)\n", cases);
    printf("  [info] A1 my inverse vs ROS2          max deviation %.6g mm/s\n", worst_a);
    printf("  [info] B1 WPILib pseudo-inverse vs ROS2 closed form  max deviation %.6g\n", worst_b);
    printf("  [info] C  my forward vs reference linear   max deviation %.6g mm/s\n", worst_cv);
    printf("  [info] C  my forward vs reference angular  max deviation %.6g mrad/s\n", worst_cw);
    printf("  [info] C linear error is at floating-point noise level; this is no coincidence:\n");
    printf("         symmetric rounding is odd (round(-t) = -round(t)), and S_VY / S_OMG\n");
    printf("         flip sign in pairs on (FL,RR) and (FR,RL), so first-order rounding errors cancel exactly in the forward sum.\n");
}

/* ================================================================== */
/* [10] Resolution reference                                           */
/* ================================================================== */

static void print_resolution(void)
{
    const double L = 300.0;
    const double counts_per_m = 10505.0; /* 11 PPR x 4 quadrature x 60 gear / (pi*80mm), = 1:60 */
    const double fs_hz = 50.0;
    const double kin_mrad = 500.0 / L; /* rot half an LSB (0.5 mm/s) expressed in mrad/s */
    const double step_mm_s = fs_hz * 1000.0 / counts_per_m;
    const double omega_step = step_mm_s * 1000.0 / L;

    printf("[10] Resolution reference (L = %.0f mm)\n", L);
    printf("  [info] kinematics: rot rounding at most 0.5 mm/s => omega error %.3f mrad/s\n",
           kin_mrad);
    printf("  [info] encoder: %.0f counts/m, @%.0f Hz single-count step = %.4f mm/s\n",
           counts_per_m, fs_hz, step_mm_s);
    printf("  [info] encoder: worst case of one count on each wheel => omega step %.3f mrad/s\n",
           omega_step);
    printf("  [info] ratio: encoder quantization is %.1f x coarser than kinematic rounding\n", omega_step / kin_mrad);
    printf("  [info] conclusion: encoder quantization drowns the kinematic error; integer mm/s is enough, no Q-format fixed point needed.\n");
}

/* ================================================================== */

int main(void)
{
    printf("=== mecanum four-wheel kinematics tests ===\n");

    test_roundtrip();
    test_structure();
    test_desaturate();
    test_lift_deadzone();
    test_mount();
    test_ros2_order();
    test_validation();
    test_solve();
    test_cross_reference();
    print_resolution();

    printf("--------------------------------------------------\n");
    printf("checks = %d, failures = %d\n", g_checks, g_fails);
    if (g_fails == 0) {
        printf("RESULT: PASSED\n");
        return EXIT_SUCCESS;
    }
    printf("RESULT: FAILED\n");
    return EXIT_FAILURE;
}
