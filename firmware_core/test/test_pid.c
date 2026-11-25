/*
 * test_pid.c - test bench for the pid module
 *
 * Coverage:
 *   [1] Step response (closed-loop convergence on a first-order inertial plant)
 *   [2] Output limiting (|out| <= out_max for any input)
 *   [3] ★ Anti-windup: desaturation time, with anti-windup vs naive PI
 *   [4] Integral separation (no integration during a large-error phase)
 *   [5] Derivative on measurement (no derivative kick)
 *   [6] Reset semantics
 *   [7] Gain validation and invalid input
 *   [8] Null-pointer safety
 *   [9] Determinism (same input sequence -> same output sequence, bit for bit)
 *   [10] Saturation flag
 *
 * Note: this is a test file, so printf / the integer plant model are allowed. src/ and include/
 *       remain float-free and third-party-dependency-free (make check-purity checks this).
 */

#include <stdio.h>
#include <stdlib.h>

#include "pid.h"

static int g_checks = 0;
static int g_fails  = 0;

static int32_t iabs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

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

static void ck_eq32(const char *tag, int32_t got, int32_t want)
{
    ++g_checks;
    if (got != want) {
        ++g_fails;
        if (g_fails <= 40) {
            printf("  [FAIL] %s: got %ld, want %ld\n", tag, (long)got, (long)want);
        }
    }
}

/* First-order inertial plant: v[k+1] = v[k] + (gain*u[k] - v[k]) / tau
 * gain = steady-state speed per unit of control (mm/s), tau = time constant (steps).
 * Using it instead of "v = u" gives the loop real dynamics, so overshoot and settling are measurable. */
static int32_t plant_step(int32_t u, int32_t v, int32_t gain, int32_t tau)
{
    int64_t d = ((int64_t)gain * (int64_t)u - (int64_t)v);
    return (int32_t)((int64_t)v + d / tau);
}

/* ================================================================== */
/* [1] Step response                                                   */
/* ================================================================== */

static void test_step_response(void)
{
    pid_gains_t g = pid_default_gains();
    pid_state_t st;
    int32_t v = 0;
    int32_t sp = 300;
    int32_t peak = 0;
    int32_t final_err;
    int32_t i;
    int settled_at = -1;

    printf("[1] step response (kp=0.5 ki=0.02, plant gain=1 tau=5 steps)\n");

    pid_reset(&st);
    for (i = 0; i < 1000; ++i) {
        uint32_t fl = 0u;
        int32_t u = pid_step(&g, &st, sp, v, &fl);
        ck_true("[1] |out| <= out_max", iabs32(u) <= g.out_max);
        v = plant_step(u, v, 1, 5);
        if (v > peak) {
            peak = v;
        }
        if (settled_at < 0 && iabs32(sp - v) <= 3) {
            settled_at = i;
        }
    }
    final_err = iabs32(sp - v);

    printf("  [info] peak %ld mm/s (overshoot %ld permille), steady-state error %ld mm/s, first within +/-3 mm/s at step %d\n",
           (long)peak, (long)((peak - sp) * 1000 / sp), (long)final_err, settled_at);

    ck_true("[1] converges to the setpoint (steady-state error <= 2 mm/s)", final_err <= 2);
    ck_true("[1] no large overshoot (peak <= 1.2 x setpoint)", peak <= sp + sp / 5);
    ck_true("[1] settles within +/-3 mm/s in 1000 steps", settled_at >= 0);
}

/* ================================================================== */
/* [2] Output limiting                                                 */
/* ================================================================== */

static void test_output_limit(void)
{
    pid_gains_t g = pid_default_gains();
    pid_state_t st;
    int32_t sp_list[7] = { 0, 1, -1, 100, -100, 100000, -100000 };
    int32_t i;
    int32_t j;

    printf("[2] output limiting\n");

    for (i = 0; i < 7; ++i) {
        pid_reset(&st);
        for (j = 0; j < 50; ++j) {
            uint32_t fl = 0u;
            int32_t u = pid_step(&g, &st, sp_list[i], 0, &fl);
            ck_true("[2] |out| <= out_max", u <= g.out_max && u >= g.out_min);
        }
    }

    /* asymmetric limits must hold too */
    {
        pid_gains_t g2 = pid_default_gains();
        pid_state_t st2;
        pid_reset(&st2);
        g2.out_min = -5;
        g2.out_max = 7;
        for (i = 0; i < 50; ++i) {
            uint32_t fl = 0u;
            int32_t u = pid_step(&g2, &st2, 5000, 0, &fl);
            ck_true("[2] asymmetric limiting", u == 7);
        }
    }
}

/* ================================================================== */
/* [3] ★ Anti-windup                                                   */
/* ================================================================== */

/* Test-only "naive PI" reference: no integral clamping, no conditional integration.
 * It lives in the test, not in src/ -- its only job is to measure the gap; it is not product code. */
static int32_t naive_pi(int32_t sp, int32_t meas, int64_t *integ, int32_t kp_q16,
                        int32_t ki_q16, int32_t lo, int32_t hi)
{
    int64_t e = (int64_t)sp - (int64_t)meas;
    int64_t out;
    *integ += e;
    out = (int64_t)kp_q16 * e / PID_Q16_ONE + (int64_t)ki_q16 * (*integ) / PID_Q16_ONE;
    if (out > hi) {
        out = hi;
    }
    if (out < lo) {
        out = lo;
    }
    return (int32_t)out;
}

static int steps_to_calm(int32_t use_real)
{
    pid_gains_t g = pid_default_gains();
    pid_state_t st;
    int64_t naive_integ = 0;
    int32_t v = 0;
    int32_t i;
    int32_t found = -1;

    g.kp_q16 = PID_Q16_ONE / 2;      /* 0.5  */
    g.ki_q16 = PID_Q16_ONE / 2;      /* 0.5 -- deliberately large so saturation is clearly visible */
    g.integ_limit = 1000;
    pid_reset(&st);

    /* phase 1: command something physically unreachable (plant ceiling 1000 mm/s) for 500 steps */
    for (i = 0; i < 500; ++i) {
        int32_t u;
        if (use_real != 0) {
            uint32_t fl = 0u;
            u = pid_step(&g, &st, 2000, v, &fl);
        } else {
            u = naive_pi(2000, v, &naive_integ, g.kp_q16, g.ki_q16, g.out_min, g.out_max);
        }
        v = plant_step(u, v, 1, 5);
    }

    /* phase 2: drop the command to zero and see in how many steps the speed really falls */
    for (i = 0; i < 3000; ++i) {
        int32_t u;
        if (use_real != 0) {
            uint32_t fl = 0u;
            u = pid_step(&g, &st, 0, v, &fl);
        } else {
            u = naive_pi(0, v, &naive_integ, g.kp_q16, g.ki_q16, g.out_min, g.out_max);
        }
        v = plant_step(u, v, 1, 5);
        if (v <= 10) {
            found = i;
            break;
        }
    }
    return found;
}

static void test_anti_windup(void)
{
    pid_gains_t g = pid_default_gains();
    pid_state_t st;
    int32_t v = 0;
    int32_t i;
    int32_t real_steps;
    int32_t naive_steps;
    int64_t bound;

    printf("[3] ★ anti-windup (command 2000 mm/s, plant ceiling 1000 mm/s)\n");

    /* with anti-windup the integral accumulator must be clamped */
    g.kp_q16 = PID_Q16_ONE / 2;
    g.ki_q16 = PID_Q16_ONE / 2;
    g.integ_limit = 1000;
    pid_reset(&st);
    for (i = 0; i < 500; ++i) {
        uint32_t fl = 0u;
        int32_t u = pid_step(&g, &st, 2000, v, &fl);
        v = plant_step(u, v, 1, 5);
    }
    bound = (int64_t)g.integ_limit * PID_Q16_ONE / g.ki_q16;
    printf("  [info] integral accumulator = %ld, theoretical bound = %ld\n", (long)st.integ_raw, (long)bound);
    ck_true("[3] integral accumulator clamped to the bound", st.integ_raw <= bound && st.integ_raw >= -bound);
    ck_true("[3] integral does not run away while saturated", st.integ_raw <= bound);

    real_steps  = steps_to_calm(1);
    naive_steps = steps_to_calm(0);
    printf("  [info] steps to desaturate to v<=10 mm/s: with anti-windup %d steps, naive PI %d steps\n",
           real_steps, naive_steps);

    ck_true("[3] with anti-windup it desaturates within 300 steps", real_steps >= 0 && real_steps < 300);
    ck_true("[3] and no slower than naive PI (should be clearly faster at the same gains)",
            naive_steps < 0 || real_steps <= naive_steps);
}

/* ================================================================== */
/* [4] Integral separation                                             */
/* ================================================================== */

static void test_integral_separation(void)
{
    pid_gains_t g = pid_default_gains();
    pid_state_t st;
    int32_t i;
    int sep_seen = 0;

    printf("[4] integral separation (threshold 50 mm/s, error 1000 mm/s)\n");

    g.integ_sep_mm_s = 50;
    pid_reset(&st);
    for (i = 0; i < 20; ++i) {
        uint32_t fl = 0u;
        (void)pid_step(&g, &st, 1000, 0, &fl);
        if ((fl & PID_FLAG_INTEG_SEP) != 0u) {
            sep_seen = 1;
        }
    }
    ck_true("[4] INTEG_SEP flag set on a large error", sep_seen != 0);
    ck_eq32("[4] integral accumulator stays 0 during separation", (int32_t)st.integ_raw, 0);

    /* once the error is back under the threshold, integration must resume */
    pid_reset(&st);
    for (i = 0; i < 20; ++i) {
        uint32_t fl = 0u;
        (void)pid_step(&g, &st, 10, 0, &fl);
        ck_true("[4] INTEG_SEP flag clear on a small error", (fl & PID_FLAG_INTEG_SEP) == 0u);
    }
    ck_true("[4] integral accumulates normally on a small error", st.integ_raw > 0);
}

/* ================================================================== */
/* [5] Derivative on measurement                                       */
/* ================================================================== */

static void test_derivative(void)
{
    pid_gains_t g = pid_default_gains();
    pid_state_t st;
    int32_t u0;
    int32_t u1;

    printf("[5] derivative on measurement (kd = 1.0, ki = 0)\n");

    g.ki_q16 = 0;
    g.kd_q16 = PID_Q16_ONE; /* 1.0 */
    pid_reset(&st);

    /* step 1: setpoint jumps 0 -> 100 while the feedback is still 0, so the derivative term must be 0 (no kick) */
    u0 = pid_step(&g, &st, 100, 0, 0);
    ck_eq32("[5] no derivative kick on the setpoint step (expected 0.5*100 = 50)", u0, 50);

    /* step 2: feedback 0 -> 10, derivative term = -1.0*(10-0) = -10
     *         error 90 -> P term 45, total 35 */
    u1 = pid_step(&g, &st, 100, 10, 0);
    ck_eq32("[5] derivative opposes a rising feedback (expected 45-10 = 35)", u1, 35);

    /* constant feedback means a zero derivative term */
    ck_eq32("[5] zero derivative term with constant feedback (expected 45)", pid_step(&g, &st, 100, 10, 0), 45);

    /* with ki = 0 the integral accumulator must stay put */
    ck_eq32("[5] integral accumulator stays at 0 with ki=0", (int32_t)st.integ_raw, 0);
}

/* ================================================================== */
/* [6] Reset semantics                                                 */
/* ================================================================== */

static void test_reset(void)
{
    pid_gains_t g = pid_default_gains();
    pid_state_t st;
    int32_t i;

    printf("[6] reset semantics\n");

    pid_reset(&st);
    for (i = 0; i < 10; ++i) {
        (void)pid_step(&g, &st, 100, 0, 0);
    }
    ck_true("[6] integral is nonzero after a run", st.integ_raw != 0);
    pid_reset(&st);
    ck_eq32("[6] reset clears the integral", (int32_t)st.integ_raw, 0);
    ck_eq32("[6] reset clears prev_meas", st.prev_meas, 0);
    ck_eq32("[6] reset clears started", (int32_t)st.started, 0);

    /* the first step after reset must match a fresh state (no leftover derivative term) */
    {
        pid_state_t a;
        pid_state_t b;
        int32_t ua;
        int32_t ub;
        pid_reset(&a);
        pid_reset(&b);
        ua = pid_step(&g, &a, 42, 7, 0);
        pid_reset(&b);
        (void)pid_step(&g, &b, 999, 999, 0);
        ub = pid_step(&g, &b, 42, 7, 0);
        ck_eq32("[6] first step after reset matches a fresh state", ub, ua);
    }

    pid_reset(0); /* must not crash */
    ck_true("[6] pid_reset(0) is safe", 1);
}

/* ================================================================== */
/* [7] Gain validation                                                 */
/* ================================================================== */

static void test_validation(void)
{
    pid_gains_t g = pid_default_gains();

    printf("[7] gain validation\n");

    ck_true("[7] default gains are valid", pid_gains_is_valid(&g) == 1);

    g.out_min = 100;
    g.out_max = 100;
    ck_true("[7] out_min >= out_max is invalid", pid_gains_is_valid(&g) == 0);

    g = pid_default_gains();
    g.kp_q16 = -1;
    ck_true("[7] negative gains are invalid", pid_gains_is_valid(&g) == 0);

    g = pid_default_gains();
    g.ki_q16 = -1;
    ck_true("[7] negative ki is invalid", pid_gains_is_valid(&g) == 0);

    g = pid_default_gains();
    g.kd_q16 = -1;
    ck_true("[7] negative kd is invalid", pid_gains_is_valid(&g) == 0);

    ck_true("[7] NULL is invalid", pid_gains_is_valid(0) == 0);
}

/* ================================================================== */
/* [8] Null-pointer safety                                             */
/* ================================================================== */

static void test_null_safety(void)
{
    pid_gains_t g = pid_default_gains();
    pid_state_t st;

    printf("[8] null-pointer safety\n");

    pid_reset(&st);
    ck_eq32("[8] g=NULL returns 0", pid_step(0, &st, 100, 0, 0), 0);
    ck_eq32("[8] st=NULL returns 0", pid_step(&g, 0, 100, 0, 0), 0);
    ck_eq32("[8] flags=NULL is acceptable", pid_step(&g, &st, 0, 0, 0), 0);
    ck_true("[8] those calls did not corrupt the state", st.started == 0 || st.started == 1);
}

/* ================================================================== */
/* [9] Determinism                                                     */
/* ================================================================== */

static void test_determinism(void)
{
    pid_gains_t g = pid_default_gains();
    pid_state_t a;
    pid_state_t b;
    int32_t va = 0;
    int32_t vb = 0;
    int32_t i;
    int same = 1;

    printf("[9] determinism\n");

    pid_reset(&a);
    pid_reset(&b);
    for (i = 0; i < 200; ++i) {
        int32_t ua = pid_step(&g, &a, 250, va, 0);
        int32_t ub = pid_step(&g, &b, 250, vb, 0);
        if (ua != ub) {
            same = 0;
        }
        va = plant_step(ua, va, 1, 5);
        vb = plant_step(ub, vb, 1, 5);
    }
    ck_true("[9] both runs are identical", same != 0);
}

/* ================================================================== */
/* [10] Saturation flag                                                */
/* ================================================================== */

static void test_saturation_flag(void)
{
    pid_gains_t g = pid_default_gains();
    pid_state_t st;
    uint32_t fl = 0u;
    int32_t u;

    printf("[10] saturation flag\n");

    g.kp_q16 = PID_Q16_ONE; /* 1.0, error 1000 -> the P term lands exactly on the limit */
    g.ki_q16 = 0;
    g.kd_q16 = 0;
    pid_reset(&st);
    u = pid_step(&g, &st, 1000, 0, &fl);
    ck_eq32("[10] error 1000 -> output exactly at the limit", u, 1000);
    ck_true("[10] right on the limit boundary still sets SATURATED", (fl & PID_FLAG_SATURATED) != 0u);

    /* ★ This is the case the old code missed: when out_raw is **exactly** on the limit, the old
       `out != out_raw` check was false, so the flag never got set. Semantically "more error cannot push
       it any further" is the same thing as "already clipped", so the criterion must be "output pinned to the limit". */
    pid_reset(&st);
    u = pid_step(&g, &st, 5000, 0, &fl);
    ck_eq32("[10] error 5000 -> output clipped to the limit", u, 1000);
    ck_true("[10] clipping sets SATURATED", (fl & PID_FLAG_SATURATED) != 0u);

    pid_reset(&st);
    u = pid_step(&g, &st, -5000, 0, &fl);
    ck_eq32("[10] negative side clipped to the lower limit", u, -1000);
    ck_true("[10] negative clipping sets SATURATED", (fl & PID_FLAG_SATURATED) != 0u);

    pid_reset(&st);
    u = pid_step(&g, &st, -1000, 0, &fl);
    ck_eq32("[10] negative side exactly at the lower limit", u, -1000);
    ck_true("[10] negative boundary also sets SATURATED", (fl & PID_FLAG_SATURATED) != 0u);

    /* must stay clear when not pinned to the limit -- otherwise the flag is always-true noise */
    pid_reset(&st);
    u = pid_step(&g, &st, 999, 0, &fl);
    ck_eq32("[10] error 999 -> not saturated", u, 999);
    ck_true("[10] one count short of the limit does not set SATURATED", (fl & PID_FLAG_SATURATED) == 0u);

    g.kp_q16 = 1;
    pid_reset(&st);
    u = pid_step(&g, &st, 10, 0, &fl);
    ck_true("[10] no flag while unsaturated", (fl & PID_FLAG_SATURATED) == 0u);
    ck_eq32("[10] output on a small error", u, 0);
}

/* ================================================================== */

int main(void)
{
    printf("=== pid speed-loop PID tests ===\n");

    test_step_response();
    test_output_limit();
    test_anti_windup();
    test_integral_separation();
    test_derivative();
    test_reset();
    test_validation();
    test_null_safety();
    test_determinism();
    test_saturation_flag();

    printf("--------------------------------------------------\n");
    printf("checks = %d, failures = %d\n", g_checks, g_fails);
    if (g_fails == 0) {
        printf("RESULT: PASSED\n");
        return EXIT_SUCCESS;
    }
    printf("RESULT: FAILED\n");
    return EXIT_FAILURE;
}
