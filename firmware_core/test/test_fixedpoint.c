/*
 * test_fixedpoint.c - test bench for the fixedpoint module
 *
 * Coverage:
 *   [1] Default parameters and parameter validation
 *   [2] Derivation of counts/m (★ the only source of this constant; arithmetic printed here)
 *   [3] Bidirectional speed conversion with symmetric rounding
 *   [4] Single-count step (resolution floor of speed feedback)
 *   [5] ★ Multi-window accumulation lowers quantization noise
 *   [6] Accumulated distance conversion and round-trip consistency
 *   [7] Large values do not overflow (clamp protection)
 *   [8] One-turn count self-check (must match 44 x 60 = 2640)
 *   [9] One-dimensional angular velocity conversion
 *  [10] Invalid input and determinism
 */

#include <stdio.h>
#include <stdlib.h>

#include "fixedpoint.h"

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

static void ck_i32(const char *tag, int32_t got, int32_t want)
{
    ++g_checks;
    if (got != want) {
        ++g_fails;
        if (g_fails <= 40) {
            printf("  [FAIL] %s: got %ld, want %ld\n",
                   tag, (long)got, (long)want);
        }
    }
}

static void ck_i64(const char *tag, int64_t got, int64_t want, int64_t tol)
{
    int64_t err = iabs64(got - want);
    ++g_checks;
    if (err > tol) {
        ++g_fails;
        if (g_fails <= 40) {
            printf("  [FAIL] %s: got %lld, want %lld, tol %lld\n",
                   tag, (long long)got, (long long)want, (long long)tol);
        }
    }
}

/* ================================================================== */

static void test_default_and_valid(void)
{
    fixp_scale_t s;
    fixp_scale_t bad;

    printf("[1] Default parameters and parameter validation\n");

    s = fixp_default_scale();
    printf("  [info] defaults: counts_per_m = %ld, wheel dia = %ld mm, period = %ld ms\n",
           (long)s.counts_per_m, (long)s.wheel_diameter_mm, (long)s.period_ms);

    ck_i32("[1] counts_per_m nominal 10505 (1:60)", s.counts_per_m, 10505);
    ck_i32("[1] wheel diameter 80 mm", s.wheel_diameter_mm, 80);
    ck_i32("[1] speed window 20 ms", s.period_ms, 20);
    ck_true("[1] defaults are valid", fixp_scale_is_valid(&s) == 1);

    ck_true("[1] NULL is invalid", fixp_scale_is_valid(0) == 0);

    bad = s;
    bad.counts_per_m = 0;
    ck_true("[1] counts_per_m=0 is invalid", fixp_scale_is_valid(&bad) == 0);
    bad = s;
    bad.counts_per_m = -1;
    ck_true("[1] counts_per_m<0 is invalid", fixp_scale_is_valid(&bad) == 0);
    bad = s;
    bad.wheel_diameter_mm = 0;
    ck_true("[1] wheel diameter=0 is invalid", fixp_scale_is_valid(&bad) == 0);
    bad = s;
    bad.period_ms = 0;
    ck_true("[1] window=0 is invalid", fixp_scale_is_valid(&bad) == 0);
}

static void test_counts_per_m_derivation(void)
{
    int32_t c60;
    int32_t c56;
    int32_t c_err;

    printf("[2] ★ counts/m derivation (the only source of this constant)\n");

    /* 44 counts per motor revolution (11 PPR x 4 quadrature) */
    c60 = fixp_counts_per_m_from_params(44, 60, 80);
    c56 = fixp_counts_per_m_from_params(44, 56, 80);

    printf("  [info] 1:60  -> 44 x 60 = 2640 counts/turn; 2640 / 0.2513274 m = 10504.2 -> API returns %ld\n",
           (long)c60);
    printf("  [info] 1:56  -> 44 x 56 = 2464 counts/turn; 2464 / 0.2513274 m =  9804.6 -> API returns %ld\n",
           (long)c56);
    printf("  [info] they differ by %.1f%%; nominal value is 10505 from 1:60, calibrate on the real vehicle\n",
           (double)(c60 - c56) * 100.0 / (double)c60);

    ck_i32("[2] exact 1:60 derivation = 10504", c60, 10504);
    ck_i32("[2] exact 1:56 derivation = 9804", c56, 9804);
    ck_true("[2] only 1 off the nominal 10505 (0.01%)", iabs64((int64_t)c60 - 10505) <= 1);
    ck_true("[2] 1:56 is about 7 percent off the nominal value",
            iabs64((int64_t)c56 - 10505) * 100 >= 10505 * 6);

    printf("  [info] note: 9805/9806 appear in the docs; they differ only by rounding with real pi vs 355/113,\n");
    printf("         difference < 0.02%%, no practical effect on odometry; 10505 is the project nominal.\n");

    c_err = fixp_counts_per_m_from_params(0, 60, 80);
    ck_i32("[2] counts_per_motor_rev=0 returns 0", c_err, 0);
    ck_i32("[2] gear=0 returns 0", fixp_counts_per_m_from_params(44, 0, 80), 0);
    ck_i32("[2] wheel diameter=0 returns 0", fixp_counts_per_m_from_params(44, 60, 0), 0);
    ck_i32("[2] negative returns 0", fixp_counts_per_m_from_params(-44, 60, 80), 0);
}

static void test_velocity_conversion(void)
{
    fixp_scale_t s = fixp_default_scale();

    printf("[3] Bidirectional speed conversion (symmetric rounding)\n");

    /* 100 mm/s over one 20 ms window = 2 mm = 21.01 counts -> 21 */
    ck_i32("[3] 100 mm/s -> 21 counts/window", fixp_mm_s_to_counts(&s, 100), 21);
    ck_i32("[3] 21 counts -> 100 mm/s", fixp_counts_to_mm_s(&s, 21), 100);
    ck_i32("[3] zero speed <-> zero counts", fixp_mm_s_to_counts(&s, 0), 0);
    ck_i32("[3] zero counts -> zero speed", fixp_counts_to_mm_s(&s, 0), 0);

    /* negative values must mirror positive ones exactly, else a systematic bias accumulates */
    ck_i32("[3] -100 mm/s is symmetric", fixp_mm_s_to_counts(&s, -100), -21);
    ck_i32("[3] -21 counts is symmetric", fixp_counts_to_mm_s(&s, -21), -100);
    ck_i32("[3] 2000 mm/s -> 420 counts", fixp_mm_s_to_counts(&s, 2000), 420);
    ck_i32("[3] 420 counts -> 1999 mm/s", fixp_counts_to_mm_s(&s, 420), 1999);

    {
        int i;
        int symmetric = 1;
        for (i = 1; i <= 500; i += 7) {
            if (fixp_mm_s_to_counts(&s, i) != -fixp_mm_s_to_counts(&s, -i)) {
                symmetric = 0;
            }
        }
        ck_true("[3] symmetric over the full range", symmetric != 0);
    }
}

static void test_single_count_step(void)
{
    fixp_scale_t s = fixp_default_scale();
    int32_t one;

    printf("[4] Single-count step (resolution floor of speed feedback)\n");

    one = fixp_counts_to_mm_s(&s, 1);
    printf("  [info] 1 count / 20 ms = 1000 / (10505 x 0.02) = 4.7596 mm/s; integer API returns %ld mm/s\n",
           (long)one);
    printf("  [info] reverse: 1 count means the wheel moved 0.09519 mm\n");
    ck_i32("[4] 1 count -> 4.76 mm/s (rounds to 5)", one, 5);

    /* de-quantizing the step: a true 102 mm/s never lands on an integer grid point */
    printf("  [info] true 102 mm/s: one window quantizes to %ld counts -> back to %ld mm/s (error %ld)\n",
           (long)fixp_mm_s_to_counts(&s, 102),
           (long)fixp_counts_to_mm_s(&s, fixp_mm_s_to_counts(&s, 102)),
           (long)((int64_t)fixp_counts_to_mm_s(&s, fixp_mm_s_to_counts(&s, 102)) - 102));
    ck_i32("[4] 102 mm/s single window -> 21 counts", fixp_mm_s_to_counts(&s, 102), 21);
    ck_i32("[4] 21 counts converts back to only 100 mm/s (2 short)",
           fixp_counts_to_mm_s(&s, 21), 100);
}

static void test_multi_window_average(void)
{
    fixp_scale_t s = fixp_default_scale();
    int32_t single;
    int32_t average;

    printf("[5] ★ Multi-window accumulation lowers quantization noise\n");

    /* 10 windows (200 ms) hold 214 counts in total; convert once */
    single  = fixp_counts_to_mm_s(&s, 21);           /* a single window can only do this */
    average = fixp_counts_sum_to_mm_s(&s, 214, 10);  /* accumulate 10 windows */

    printf("  [info] true 102 mm/s:\n");
    printf("         single window 21 counts -> %ld mm/s (error %ld)\n",
           (long)single, (long)((int64_t)single - 102));
    printf("         10 windows 214 counts -> %ld mm/s (error %ld)\n",
           (long)average, (long)((int64_t)average - 102));

    ck_i32("[5] 10-window sum -> 102 mm/s (exact)", average, 102);
    ck_true("[5] accumulated error is strictly smaller than single-window",
            iabs64((int64_t)average - 102) < iabs64((int64_t)single - 102));

    /* periods = 1 must be exactly equivalent to a single window (no extra rounding) */
    ck_i32("[5] periods=1 is equivalent to a single window",
           fixp_counts_sum_to_mm_s(&s, 21, 1), fixp_counts_to_mm_s(&s, 21));

    /* the multi-window integer result must stay near the single-window result */
    ck_i64("[5] sum result is within 5 percent of the single-window result",
           (int64_t)average, (int64_t)single, 5);

    ck_i32("[5] periods=0 returns 0", fixp_counts_sum_to_mm_s(&s, 214, 0), 0);
    ck_i32("[5] periods<0 returns 0", fixp_counts_sum_to_mm_s(&s, 214, -3), 0);

    {
        int i;
        int monotonic = 1;
        int32_t prev = fixp_counts_sum_to_mm_s(&s, 0, 5);
        for (i = 1; i <= 300; ++i) {
            int32_t now = fixp_counts_sum_to_mm_s(&s, i, 5);
            if (now < prev) {
                monotonic = 0;
            }
            prev = now;
        }
        ck_true("[5] monotonic non-decreasing in counts_sum", monotonic != 0);
    }
}

static void test_distance(void)
{
    fixp_scale_t s = fixp_default_scale();

    printf("[6] Accumulated distance conversion and round-trip consistency\n");

    ck_i64("[6] 10505 counts -> 1000 mm", fixp_counts_to_mm(&s, 10505), 1000, 0);
    ck_i64("[6] 1000 mm -> 10505 counts", fixp_mm_to_counts(&s, 1000), 10505, 0);
    ck_i64("[6] one turn 2640 counts -> 251 mm", fixp_counts_to_mm(&s, 2640), 251, 0);
    ck_i64("[6] 100 mm -> 1051 counts", fixp_mm_to_counts(&s, 100), 1051, 0);
    ck_i64("[6] negative is symmetric: -10505 counts -> -1000 mm",
           fixp_counts_to_mm(&s, -10505), -1000, 0);

    {
        int64_t mm;
        int round_trip_ok = 1;
        for (mm = 0; mm <= 20000; mm += 137) {
            int64_t c = fixp_mm_to_counts(&s, mm);
            int64_t back = fixp_counts_to_mm(&s, c);
            if (iabs64(back - mm) > 1) {
                round_trip_ok = 0;
            }
        }
        ck_true("[6] mm -> counts -> mm round trip within 1 mm", round_trip_ok != 0);
    }

    {
        /* 1 km: 10505 x 1000 = 10,505,000 counts */
        int64_t km_counts = 10505000;
        ck_i64("[6] 1 km is 10,505,000 counts -> 1,000,000 mm",
               fixp_counts_to_mm(&s, km_counts), 1000000, 0);
    }
}

static void test_overflow_guard(void)
{
    fixp_scale_t s = fixp_default_scale();
    int64_t big;

    printf("[7] Large values do not overflow (clamp protection)\n");

    /* clamped to 9e12, x1000 = 9e15, still inside int64 */
    big = fixp_counts_to_mm(&s, 9223372036854775807LL);
    printf("  [info] INT64_MAX counts after clamping -> %lld mm (no UB, no negative overflow)\n",
           (long long)big);
    ck_true("[7] INT64_MAX produces no negative value", big > 0);
    ck_true("[7] INT64_MAX result is sane (order 8.5e11 mm)",
            big > 800000000000LL && big < 900000000000LL);

    big = fixp_counts_to_mm(&s, -9223372036854775807LL);
    ck_true("[7] INT64_MIN produces no positive value", big < 0);
    ck_true("[7] INT64_MIN mirrors INT64_MAX",
            iabs64(big + fixp_counts_to_mm(&s, 9223372036854775807LL)) <= 1);

    ck_true("[7] mm_to_counts does not crash on large values",
            fixp_mm_to_counts(&s, 9223372036854775807LL) > 0);

    /* distance uses int64 accumulation; long runs do not overflow */
    ck_i64("[7] 100 km accumulation stays exact",
           fixp_counts_to_mm(&s, 1050500000LL), 100000000, 0);
}

static void test_counts_per_turn_selfcheck(void)
{
    fixp_scale_t s = fixp_default_scale();
    int64_t per_turn;

    printf("[8] One-turn count self-check (must match 44 x 60 = 2640)\n");

    per_turn = fixp_counts_per_wheel_turn(&s);
    printf("  [info] counts/turn = 10505 x pi x 80 / 1000 = %.1f -> %lld\n",
           10505.0 * 3.14159265358979 * 80.0 / 1000.0, (long long)per_turn);

    ck_i64("[8] one turn = 2640 counts (= 11 PPR x 4 quadrature x 60)", per_turn, 2640, 0);
    ck_true("[8] consistent with the 1:60 premise", per_turn == 44LL * 60LL);
}

static void test_omega(void)
{
    printf("[9] One-dimensional angular velocity conversion\n");

    ck_i32("[9] 500 mm/s / 300 mm -> 1667 mrad/s",
           fixp_wheel_mm_s_to_omega_mrad_s(500, 300), 1667);
    ck_i32("[9] negative is symmetric",
           fixp_wheel_mm_s_to_omega_mrad_s(-500, 300), -1667);
    ck_i32("[9] zero speed -> 0", fixp_wheel_mm_s_to_omega_mrad_s(0, 300), 0);
    ck_i32("[9] L=0 returns 0 (no division by zero)", fixp_wheel_mm_s_to_omega_mrad_s(500, 0), 0);
    ck_i32("[9] L<0 returns 0", fixp_wheel_mm_s_to_omega_mrad_s(500, -1), 0);
    /* for L = 300, invert the forward map: omega=1000 mrad/s is wheel speed 300 mm/s */
    ck_i32("[9] 300 mm/s / 300 mm -> 1000 mrad/s",
           fixp_wheel_mm_s_to_omega_mrad_s(300, 300), 1000);
}

static void test_invalid_and_determinism(void)
{
    fixp_scale_t s = fixp_default_scale();
    fixp_scale_t zero;
    int i;
    int deterministic = 1;

    printf("[10] Invalid input and determinism\n");

    zero = s;
    zero.counts_per_m = 0;

    ck_i32("[10] NULL -> 0 (speed)", fixp_mm_s_to_counts(0, 100), 0);
    ck_i32("[10] NULL -> 0 (convert back)", fixp_counts_to_mm_s(0, 100), 0);
    ck_i32("[10] NULL -> 0 (accumulate)", fixp_counts_sum_to_mm_s(0, 100, 2), 0);
    ck_i64("[10] NULL -> 0 (distance)", fixp_counts_to_mm(0, 100), 0, 0);
    ck_i64("[10] NULL -> 0 (inverse)", fixp_mm_to_counts(0, 100), 0, 0);
    ck_i64("[10] NULL -> 0 (one turn)", fixp_counts_per_wheel_turn(0), 0, 0);

    ck_i32("[10] counts_per_m=0 -> 0", fixp_mm_s_to_counts(&zero, 100), 0);
    ck_i32("[10] counts_per_m=0 convert back -> 0", fixp_counts_to_mm_s(&zero, 100), 0);
    ck_i32("[10] counts_per_m=0 accumulate -> 0", fixp_counts_sum_to_mm_s(&zero, 100, 2), 0);
    ck_i64("[10] counts_per_m=0 distance -> 0", fixp_counts_to_mm(&zero, 100), 0, 0);

    {
        fixp_scale_t badp = s;
        badp.period_ms = 0;
        ck_i32("[10] period_ms=0 -> 0", fixp_mm_s_to_counts(&badp, 100), 0);
        ck_i32("[10] period_ms=0 convert back -> 0", fixp_counts_to_mm_s(&badp, 100), 0);
    }

    /* 777 mm/s over 20 ms = 15.54 mm = 163.25 counts -> 163 */
    for (i = 0; i < 1000; ++i) {
        if (fixp_mm_s_to_counts(&s, 777) != 163) {
            deterministic = 0;
        }
    }
    ck_true("[10] same input always gives the same output", deterministic != 0);
    ck_i32("[10] 777 mm/s -> 163 counts", fixp_mm_s_to_counts(&s, 777), 163);
    ck_i32("[10] 163 counts -> 776 mm/s", fixp_counts_to_mm_s(&s, 163), 776);
}

/* ================================================================== */

int main(void)
{
    printf("=== fixedpoint unit conversion tests ===\n");

    test_default_and_valid();
    test_counts_per_m_derivation();
    test_velocity_conversion();
    test_single_count_step();
    test_multi_window_average();
    test_distance();
    test_overflow_guard();
    test_counts_per_turn_selfcheck();
    test_omega();
    test_invalid_and_determinism();

    printf("--------------------------------------------------\n");
    printf("checks = %d, failures = %d\n", g_checks, g_fails);
    if (g_fails == 0) {
        printf("RESULT: PASSED\n");
        return EXIT_SUCCESS;
    }
    printf("RESULT: FAILED\n");
    return EXIT_FAILURE;
}
