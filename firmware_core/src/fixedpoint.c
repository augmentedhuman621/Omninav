/*
 * fixedpoint.c — 定点单位换算实现
 *
 * 纯整数、零依赖：本文件只 include "fixedpoint.h"（后者只 include <stdint.h>）。
 * 无动态内存、无全局可变状态、无递归。
 */

#include "fixedpoint.h"

/* 圆周率用 355/113 近似（误差 2.7e-7）。折算到 80 mm 轮径上，
 * 周长误差不到 0.0001 mm —— 比轮径本身的公差小三个数量级。
 * 用有理数是为了全程整数，不用任何库函数。 */
#define FIXP_PI_NUM 355
#define FIXP_PI_DEN 113

#define FIXP_ONE_MILLION 1000000LL

/* 乘法前的安全上限：保证 num 侧不会溢出 int64（见各函数的注释）。 */
#define FIXP_MUL_GUARD 9000000000000LL /* 9e12 */

static int64_t fixp_div_round(int64_t num, int64_t den)
{
    if (den <= 0) {
        return 0;
    }
    if (num >= 0) {
        return (num + den / 2) / den;
    }
    return -((-num + den / 2) / den);
}

static int64_t fixp_clamp(int64_t v, int64_t lo, int64_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static int32_t fixp_to_i32(int64_t v)
{
    return (int32_t)fixp_clamp(v, -2147483648LL, 2147483647LL);
}

/* ------------------------------------------------------------------ */
/* 参数                                                               */
/* ------------------------------------------------------------------ */

fixp_scale_t fixp_default_scale(void)
{
    fixp_scale_t s;
    s.counts_per_m      = FIXP_COUNTS_PER_M_NOMINAL;
    s.wheel_diameter_mm = FIXP_WHEEL_DIAMETER_MM_NOMINAL;
    s.period_ms         = FIXP_CONTROL_PERIOD_MS_NOMINAL;
    return s;
}

int fixp_scale_is_valid(const fixp_scale_t *s)
{
    if (s == 0) {
        return 0;
    }
    if (s->counts_per_m <= 0) {
        return 0;
    }
    if (s->wheel_diameter_mm <= 0) {
        return 0;
    }
    if (s->period_ms <= 0) {
        return 0;
    }
    return 1;
}

int32_t fixp_counts_per_m_from_params(int32_t counts_per_motor_rev,
                                      int32_t gear_ratio,
                                      int32_t wheel_diameter_mm)
{
    int64_t num;
    int64_t den;

    if (counts_per_motor_rev <= 0 || gear_ratio <= 0 || wheel_diameter_mm <= 0) {
        return 0;
    }
    /* counts/m = cpr × gear × 1000 / (π × D)
     *          = cpr × gear × 1000 × 113 / (355 × D) */
    num = (int64_t)counts_per_motor_rev * (int64_t)gear_ratio * 1000LL
        * (int64_t)FIXP_PI_DEN;
    den = (int64_t)FIXP_PI_NUM * (int64_t)wheel_diameter_mm;
    if (den <= 0) {
        return 0;
    }
    return fixp_to_i32(fixp_div_round(num, den));
}

/* ------------------------------------------------------------------ */
/* 速度换算                                                            */
/* ------------------------------------------------------------------ */

int32_t fixp_mm_s_to_counts(const fixp_scale_t *s, int32_t mm_s)
{
    int64_t num;
    int64_t den;

    if (s == 0 || s->counts_per_m <= 0 || s->period_ms <= 0) {
        return 0;
    }
    /* counts = mm_s × counts_per_m × period_ms / 1e6
     * |mm_s| ≤ 2^31，乘数 ≤ 2.1e5 ⇒ 积 ≤ 4.5e14，int64 安全 */
    num = (int64_t)mm_s * (int64_t)s->counts_per_m * (int64_t)s->period_ms;
    den = FIXP_ONE_MILLION;
    return fixp_to_i32(fixp_div_round(num, den));
}

int32_t fixp_counts_to_mm_s(const fixp_scale_t *s, int32_t counts)
{
    int64_t num;
    int64_t den;

    if (s == 0 || s->counts_per_m <= 0 || s->period_ms <= 0) {
        return 0;
    }
    /* mm_s = counts × 1e6 / (counts_per_m × period_ms) */
    num = (int64_t)counts * FIXP_ONE_MILLION;
    den = (int64_t)s->counts_per_m * (int64_t)s->period_ms;
    if (den <= 0) {
        return 0;
    }
    return fixp_to_i32(fixp_div_round(num, den));
}

int32_t fixp_counts_sum_to_mm_s(const fixp_scale_t *s, int32_t counts_sum,
                                int32_t periods)
{
    int64_t num;
    int64_t total_ms;
    int64_t den;

    if (s == 0 || s->counts_per_m <= 0 || s->period_ms <= 0) {
        return 0;
    }
    if (periods <= 0) {
        return 0;
    }
    /* counts_sum 覆盖 periods 个窗口，总时长 = period_ms × periods。
     * mm_s = counts_sum × 1e6 / (counts_per_m × period_ms × periods)
     * —— 直接按「总窗口」算，不先除 periods，避免多一次取整丢精度。 */
    total_ms = (int64_t)s->period_ms * (int64_t)periods;
    if (total_ms <= 0) {
        return 0;
    }
    num = (int64_t)counts_sum * FIXP_ONE_MILLION;
    den = (int64_t)s->counts_per_m * total_ms;
    if (den <= 0) {
        return 0;
    }
    return fixp_to_i32(fixp_div_round(num, den));
}

/* ------------------------------------------------------------------ */
/* 距离换算                                                            */
/* ------------------------------------------------------------------ */

int64_t fixp_counts_to_mm(const fixp_scale_t *s, int64_t counts)
{
    int64_t c;

    if (s == 0 || s->counts_per_m <= 0) {
        return 0;
    }
    /* mm = counts × 1000 / counts_per_m；先夹住 counts 保证乘 1000 不溢出 */
    c = fixp_clamp(counts, -FIXP_MUL_GUARD, FIXP_MUL_GUARD);
    return fixp_div_round(c * 1000LL, (int64_t)s->counts_per_m);
}

int64_t fixp_mm_to_counts(const fixp_scale_t *s, int64_t mm)
{
    int64_t m;

    if (s == 0 || s->counts_per_m <= 0) {
        return 0;
    }
    /* counts = mm × counts_per_m / 1000 */
    m = fixp_clamp(mm, -FIXP_MUL_GUARD, FIXP_MUL_GUARD);
    return fixp_div_round(m * (int64_t)s->counts_per_m, 1000LL);
}

/* ------------------------------------------------------------------ */
/* 一维角速度换算与自检量                                              */
/* ------------------------------------------------------------------ */

int32_t fixp_wheel_mm_s_to_omega_mrad_s(int32_t wheel_mm_s, int32_t l_mm)
{
    if (l_mm <= 0) {
        return 0; /* 不除零 */
    }
    /* omega[mrad/s] = v[mm/s] / L[mm] × 1000 */
    return fixp_to_i32(fixp_div_round((int64_t)wheel_mm_s * 1000LL, (int64_t)l_mm));
}

int64_t fixp_counts_per_wheel_turn(const fixp_scale_t *s)
{
    int64_t num;
    int64_t den;

    if (s == 0 || s->counts_per_m <= 0 || s->wheel_diameter_mm <= 0) {
        return 0;
    }
    /* counts/轮转 = counts_per_m × π × D / 1000
     *             = counts_per_m × 355 × D / (113 × 1000) */
    num = (int64_t)s->counts_per_m * (int64_t)FIXP_PI_NUM
        * (int64_t)s->wheel_diameter_mm;
    den = (int64_t)FIXP_PI_DEN * 1000LL;
    return fixp_div_round(num, den);
}
