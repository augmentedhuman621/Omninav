/*
 * test_fixedpoint.c — fixedpoint 模块的测试台
 *
 * 覆盖：
 *   [1] 默认参数与参数校验
 *   [2] counts/m 的推导（★ 全项目唯一的常数来源，这里把算术打印出来）
 *   [3] 速度双向换算与对称四舍五入
 *   [4] 单计数台阶（速度反馈的分辨率下限）
 *   [5] ★ 多窗口累加降量化噪声
 *   [6] 累计里程换算与往返自洽
 *   [7] 大数值不溢出（clamp 保护）
 *   [8] 一圈计数自检（必须对上 44 × 60 = 2640）
 *   [9] 一维角速度换算
 *  [10] 非法输入与确定性
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

    printf("[1] 默认参数与参数校验\n");

    s = fixp_default_scale();
    printf("  [info] 默认：counts_per_m = %ld, 轮径 = %ld mm, 测速窗口 = %ld ms\n",
           (long)s.counts_per_m, (long)s.wheel_diameter_mm, (long)s.period_ms);

    ck_i32("[1] counts_per_m 名义值 10505（1:60）", s.counts_per_m, 10505);
    ck_i32("[1] 轮径 80 mm", s.wheel_diameter_mm, 80);
    ck_i32("[1] 测速窗口 20 ms", s.period_ms, 20);
    ck_true("[1] 默认为合法", fixp_scale_is_valid(&s) == 1);

    ck_true("[1] NULL 不合法", fixp_scale_is_valid(0) == 0);

    bad = s;
    bad.counts_per_m = 0;
    ck_true("[1] counts_per_m=0 不合法", fixp_scale_is_valid(&bad) == 0);
    bad = s;
    bad.counts_per_m = -1;
    ck_true("[1] counts_per_m<0 不合法", fixp_scale_is_valid(&bad) == 0);
    bad = s;
    bad.wheel_diameter_mm = 0;
    ck_true("[1] 轮径=0 不合法", fixp_scale_is_valid(&bad) == 0);
    bad = s;
    bad.period_ms = 0;
    ck_true("[1] 窗口=0 不合法", fixp_scale_is_valid(&bad) == 0);
}

static void test_counts_per_m_derivation(void)
{
    int32_t c60;
    int32_t c56;
    int32_t c_err;

    printf("[2] ★ counts/m 推导（全项目唯一的常数来源）\n");

    /* 44 counts/电机转（11 PPR × 4 倍频） */
    c60 = fixp_counts_per_m_from_params(44, 60, 80);
    c56 = fixp_counts_per_m_from_params(44, 56, 80);

    printf("  [info] 1:60  -> 44 × 60 = 2640 counts/轮转；2640 ÷ 0.2513274 m = 10504.2 -> 接口给 %ld\n",
           (long)c60);
    printf("  [info] 1:56  -> 44 × 56 = 2464 counts/轮转；2464 ÷ 0.2513274 m =  9804.6 -> 接口给 %ld\n",
           (long)c56);
    printf("  [info] 两者相差 %.1f%%，名义值取 1:60 的 10505，实车必须标定\n",
           (double)(c60 - c56) * 100.0 / (double)c60);

    ck_i32("[2] 1:60 的精确推导结果 = 10504", c60, 10504);
    ck_i32("[2] 1:56 的精确推导结果 = 9804", c56, 9804);
    ck_true("[2] 与名义值 10505 只差 1（0.01%）", iabs64((int64_t)c60 - 10505) <= 1);
    ck_true("[2] 1:56 与名义值差约 7%",
            iabs64((int64_t)c56 - 10505) * 100 >= 10505 * 6);

    printf("  [info] 注：文档里出现过 9805/9806，是用真 π 而非 355/113 时的取整差异，\n");
    printf("         差异 < 0.02%%，对里程计无实际影响，以 10505 为项目名义值。\n");

    c_err = fixp_counts_per_m_from_params(0, 60, 80);
    ck_i32("[2] counts_per_motor_rev=0 返回 0", c_err, 0);
    ck_i32("[2] gear=0 返回 0", fixp_counts_per_m_from_params(44, 0, 80), 0);
    ck_i32("[2] 轮径=0 返回 0", fixp_counts_per_m_from_params(44, 60, 0), 0);
    ck_i32("[2] 负数返回 0", fixp_counts_per_m_from_params(-44, 60, 80), 0);
}

static void test_velocity_conversion(void)
{
    fixp_scale_t s = fixp_default_scale();

    printf("[3] 速度双向换算（对称四舍五入）\n");

    /* 100 mm/s 走一个 20 ms 窗口 = 2 mm = 21.01 counts -> 21 */
    ck_i32("[3] 100 mm/s -> 21 counts/窗口", fixp_mm_s_to_counts(&s, 100), 21);
    ck_i32("[3] 21 counts -> 100 mm/s", fixp_counts_to_mm_s(&s, 21), 100);
    ck_i32("[3] 零速 <-> 零计数", fixp_mm_s_to_counts(&s, 0), 0);
    ck_i32("[3] 零计数 -> 零速", fixp_counts_to_mm_s(&s, 0), 0);

    /* 负值必须与正值严格对称，否则长期会累出系统性偏差 */
    ck_i32("[3] -100 mm/s 对称", fixp_mm_s_to_counts(&s, -100), -21);
    ck_i32("[3] -21 counts 对称", fixp_counts_to_mm_s(&s, -21), -100);
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
        ck_true("[3] 全量程正负对称", symmetric != 0);
    }
}

static void test_single_count_step(void)
{
    fixp_scale_t s = fixp_default_scale();
    int32_t one;

    printf("[4] 单计数台阶（速度反馈的分辨率下限）\n");

    one = fixp_counts_to_mm_s(&s, 1);
    printf("  [info] 1 count / 20 ms = 1000 ÷ (10505 × 0.02) = 4.7596 mm/s；整数接口给 %ld mm/s\n",
           (long)one);
    printf("  [info] 反向：1 count 对应轮子走了 0.09519 mm\n");
    ck_i32("[4] 1 count -> 4.76 mm/s（取整后 5）", one, 5);

    /* 台阶去量化：真实 102 清不到整数格点上 */
    printf("  [info] 真实 102 mm/s：单窗口量化成 %ld counts -> 回算 %ld mm/s（误差 %ld）\n",
           (long)fixp_mm_s_to_counts(&s, 102),
           (long)fixp_counts_to_mm_s(&s, fixp_mm_s_to_counts(&s, 102)),
           (long)((int64_t)fixp_counts_to_mm_s(&s, fixp_mm_s_to_counts(&s, 102)) - 102));
    ck_i32("[4] 102 mm/s 单窗口 -> 21 counts", fixp_mm_s_to_counts(&s, 102), 21);
    ck_i32("[4] 21 counts 回算只有 100 mm/s（少 2）",
           fixp_counts_to_mm_s(&s, 21), 100);
}

static void test_multi_window_average(void)
{
    fixp_scale_t s = fixp_default_scale();
    int32_t single;
    int32_t average;

    printf("[5] ★ 多窗口累加降量化噪声\n");

    /* 10 个窗口（200 ms）的计数和是 214，一次换算 */
    single  = fixp_counts_to_mm_s(&s, 21);           /* 单个窗口只能这样 */
    average = fixp_counts_sum_to_mm_s(&s, 214, 10);  /* 10 个窗口累加 */

    printf("  [info] 真实 102 mm/s：\n");
    printf("         单窗口 21 counts -> %ld mm/s（误差 %ld）\n",
           (long)single, (long)((int64_t)single - 102));
    printf("         10 窗口 214 counts -> %ld mm/s（误差 %ld）\n",
           (long)average, (long)((int64_t)average - 102));

    ck_i32("[5] 10 窗口累加 -> 102 mm/s（精确）", average, 102);
    ck_true("[5] 累加后的误差严格小于单窗口",
            iabs64((int64_t)average - 102) < iabs64((int64_t)single - 102));

    /* periods = 1 必须与单窗口完全等价（不引入额外取整） */
    ck_i32("[5] periods=1 等价于单窗口",
           fixp_counts_sum_to_mm_s(&s, 21, 1), fixp_counts_to_mm_s(&s, 21));

    /* 多窗口的整数结果必须落在单窗口结果的合理邻域内 */
    ck_i64("[5] 累加结果与单窗口结果相差 <= 5%",
           (int64_t)average, (int64_t)single, 5);

    ck_i32("[5] periods=0 返回 0", fixp_counts_sum_to_mm_s(&s, 214, 0), 0);
    ck_i32("[5] periods<0 返回 0", fixp_counts_sum_to_mm_s(&s, 214, -3), 0);

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
        ck_true("[5] 关于 counts_sum 单调不减", monotonic != 0);
    }
}

static void test_distance(void)
{
    fixp_scale_t s = fixp_default_scale();

    printf("[6] 累计里程换算与往返自洽\n");

    ck_i64("[6] 10505 counts -> 1000 mm", fixp_counts_to_mm(&s, 10505), 1000, 0);
    ck_i64("[6] 1000 mm -> 10505 counts", fixp_mm_to_counts(&s, 1000), 10505, 0);
    ck_i64("[6] 一圈 2640 counts -> 251 mm", fixp_counts_to_mm(&s, 2640), 251, 0);
    ck_i64("[6] 100 mm -> 1051 counts", fixp_mm_to_counts(&s, 100), 1051, 0);
    ck_i64("[6] 负值对称：-10505 counts -> -1000 mm",
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
        ck_true("[6] mm -> counts -> mm 全程往返误差 <= 1 mm", round_trip_ok != 0);
    }

    {
        /* 1 公里：10505 × 1000 = 10,505,000 counts */
        int64_t km_counts = 10505000;
        ck_i64("[6] 1 km 对应 10,505,000 counts -> 1,000,000 mm",
               fixp_counts_to_mm(&s, km_counts), 1000000, 0);
    }
}

static void test_overflow_guard(void)
{
    fixp_scale_t s = fixp_default_scale();
    int64_t big;

    printf("[7] 大数值不溢出（clamp 保护）\n");

    /* 夹到 9e12 后 ×1000 = 9e15，仍在 int64 内 */
    big = fixp_counts_to_mm(&s, 9223372036854775807LL);
    printf("  [info] INT64_MAX counts 被夹住后 -> %lld mm（没有 UB、没有负溢出）\n",
           (long long)big);
    ck_true("[7] INT64_MAX 不产生负值", big > 0);
    ck_true("[7] INT64_MAX 结果合理（8.5e11 mm 量级）",
            big > 800000000000LL && big < 900000000000LL);

    big = fixp_counts_to_mm(&s, -9223372036854775807LL);
    ck_true("[7] INT64_MIN 不产生正值", big < 0);
    ck_true("[7] INT64_MIN 与 INT64_MAX 对称",
            iabs64(big + fixp_counts_to_mm(&s, 9223372036854775807LL)) <= 1);

    ck_true("[7] mm_to_counts 大值不崩",
            fixp_mm_to_counts(&s, 9223372036854775807LL) > 0);

    /* 距离换算用 int64 累计，长时间运行不溢出 */
    ck_i64("[7] 100 km 累计仍精确",
           fixp_counts_to_mm(&s, 1050500000LL), 100000000, 0);
}

static void test_counts_per_turn_selfcheck(void)
{
    fixp_scale_t s = fixp_default_scale();
    int64_t per_turn;

    printf("[8] 一圈计数自检（必须对上 44 × 60 = 2640）\n");

    per_turn = fixp_counts_per_wheel_turn(&s);
    printf("  [info] counts/轮转 = 10505 × π × 80 ÷ 1000 = %.1f -> %lld\n",
           10505.0 * 3.14159265358979 * 80.0 / 1000.0, (long long)per_turn);

    ck_i64("[8] 一圈 = 2640 counts（= 11 PPR × 4 倍频 × 60）", per_turn, 2640, 0);
    ck_true("[8] 与 1:60 的前提自洽", per_turn == 44LL * 60LL);
}

static void test_omega(void)
{
    printf("[9] 一维角速度换算\n");

    ck_i32("[9] 500 mm/s ÷ 300 mm -> 1667 mrad/s",
           fixp_wheel_mm_s_to_omega_mrad_s(500, 300), 1667);
    ck_i32("[9] 负值对称",
           fixp_wheel_mm_s_to_omega_mrad_s(-500, 300), -1667);
    ck_i32("[9] 零速 -> 0", fixp_wheel_mm_s_to_omega_mrad_s(0, 300), 0);
    ck_i32("[9] L=0 返回 0（不除零）", fixp_wheel_mm_s_to_omega_mrad_s(500, 0), 0);
    ck_i32("[9] L<0 返回 0", fixp_wheel_mm_s_to_omega_mrad_s(500, -1), 0);
    /* L = 300 时，正解 L 反推：ω=1000 mrad/s 对应轮速 300 mm/s */
    ck_i32("[9] 300 mm/s ÷ 300 mm -> 1000 mrad/s",
           fixp_wheel_mm_s_to_omega_mrad_s(300, 300), 1000);
}

static void test_invalid_and_determinism(void)
{
    fixp_scale_t s = fixp_default_scale();
    fixp_scale_t zero;
    int i;
    int deterministic = 1;

    printf("[10] 非法输入与确定性\n");

    zero = s;
    zero.counts_per_m = 0;

    ck_i32("[10] NULL -> 0（速度）", fixp_mm_s_to_counts(0, 100), 0);
    ck_i32("[10] NULL -> 0（回算）", fixp_counts_to_mm_s(0, 100), 0);
    ck_i32("[10] NULL -> 0（累加）", fixp_counts_sum_to_mm_s(0, 100, 2), 0);
    ck_i64("[10] NULL -> 0（里程）", fixp_counts_to_mm(0, 100), 0, 0);
    ck_i64("[10] NULL -> 0（反算）", fixp_mm_to_counts(0, 100), 0, 0);
    ck_i64("[10] NULL -> 0（一圈）", fixp_counts_per_wheel_turn(0), 0, 0);

    ck_i32("[10] counts_per_m=0 -> 0", fixp_mm_s_to_counts(&zero, 100), 0);
    ck_i32("[10] counts_per_m=0 回算 -> 0", fixp_counts_to_mm_s(&zero, 100), 0);
    ck_i32("[10] counts_per_m=0 累加 -> 0", fixp_counts_sum_to_mm_s(&zero, 100, 2), 0);
    ck_i64("[10] counts_per_m=0 里程 -> 0", fixp_counts_to_mm(&zero, 100), 0, 0);

    {
        fixp_scale_t badp = s;
        badp.period_ms = 0;
        ck_i32("[10] period_ms=0 -> 0", fixp_mm_s_to_counts(&badp, 100), 0);
        ck_i32("[10] period_ms=0 回算 -> 0", fixp_counts_to_mm_s(&badp, 100), 0);
    }

    /* 777 mm/s 走 20 ms = 15.54 mm = 163.25 counts -> 163 */
    for (i = 0; i < 1000; ++i) {
        if (fixp_mm_s_to_counts(&s, 777) != 163) {
            deterministic = 0;
        }
    }
    ck_true("[10] 同一输入永远同一输出", deterministic != 0);
    ck_i32("[10] 777 mm/s -> 163 counts", fixp_mm_s_to_counts(&s, 777), 163);
    ck_i32("[10] 163 counts -> 776 mm/s", fixp_counts_to_mm_s(&s, 163), 776);
}

/* ================================================================== */

int main(void)
{
    printf("=== 定点单位换算测试 ===\n");

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
