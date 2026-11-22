/*
 * odom.c — 圆弧积分里程计实现
 *
 * 纯整数、零依赖：本文件只 include "odom.h"（后者只 include <stdint.h> 与 "mecanum.h"）。
 * 无动态内存、无全局可变状态、无递归。
 *
 * 数学与「为什么必须圆弧积分」的完整说明见 odom.h 文件头。
 */

#include "odom.h"

/* ------------------------------------------------------------------ */
/* 常数                                                                */
/* ------------------------------------------------------------------ */

/* 一整圈 = 2π × 1000 mrad。用 6283 近似 6283.185，周期误差 0.003%，
 * 对里程计的影响远小于编码器量化，可忽略。 */
#define ODOM_MRAD_PER_TURN  6283
#define ODOM_MRAD_PER_HALF  3142  /* π × 1000 ≈ 3141.593 */
#define ODOM_MRAD_PER_QUART 1571  /* π/2 × 1000 ≈ 1570.796 */

/* 三角函数内部用的定点基数：Q20。选 Q20 而不是 Q15，是因为
 *   x 用 Q12 时在「小角度」区取整误差太大（80 mrad 处正弦能差 1%），
 *   而里程计恰恰长期工作在小角度区。Q20 下全量程误差约 0.01%。 */
#define ODOM_Q20 1048576

/* ★ 航向角归一化用的回绕量（11_接口契约.md §5.1 定案）。
 *
 * 回绕量必须是**整圈**的整数倍，否则「角度等价」会被破坏：归一化做的事
 * 就是「减去 k 圈」，若一圈的标称值不是整圈，跨过回绕点时航向会跳变。
 * 曾经用过 180000，那是个真 bug 而不是注释笔误 ——
 *   2 × 180000 = 360000 mrad = 360 rad = 57.2958 圈（**非整数**，
 *   余 0.2958 圈 = **106.48°**）⇒ |θ| 一旦越过 180000，航向直接跳 106.48°。
 *   按 0.1°/s 的 yaw 漂移，约 8 小时就能累积到 28.6 圈踩上去，
 *   而 odom.h 给这行写的用途恰恰是「适合长期运行」。
 *
 * 现在取整圈 6284 mrad（≈ 2π = 6283.185 mrad）：每回绕一次引入
 * 0.815 mrad ≈ 0.047° 的误差 —— 这是 mrad 整数化本身的下限，可接受，
 * 且远小于编码器量化（单计数台阶 4.7596 mm/s）。
 *
 * pose.theta_mrad ∈ (-3142, 3142]，是**规范航向角，可以直接当 yaw 用**，
 * 调用方不需要再归一化；total.theta_mrad 仍是不归一化的 int64 累计量。
 *
 * 注：本值与上面 `ODOM_MRAD_PER_HALF`（三角函数角度归约用的半圈）数值相同，
 * 但**职责完全不同、别合并**：那个必须是数学上的 π×1000 才能正确归约象限，
 * 这个必须是「整圈 / 2」才能保证归一化的角度等价性。以前两者不相等
 * （180000 vs 3142）恰恰说明二者不该用同一个宏。 */
#define ODOM_HALF_TURN_MRAD 3142  /* π × 1000 ≈ 3141.593 */
#define ODOM_FULL_TURN_MRAD 6284  /* 2 × ODOM_HALF_TURN_MRAD，必须是一整圈 */

/* ------------------------------------------------------------------ */
/* 内部工具                                                            */
/* ------------------------------------------------------------------ */

/* 对称四舍五入（round half away from zero）。den 必须 > 0。 */
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
 * 正弦，输入「毫弧度」（任意大小，内部先归约），输出 Q15（±32767）。
 * 只做「把角度折到 [0, π/2]，再用泰勒级数求 s(x)/x」这一件事
 * （下文一律用 s(x) 记正弦、c(x) 记余弦 —— 目录里的 `check-purity`
 *   对源码做了整词禁用检查，注释里也不能出现那两个英文函数名）。
 * 不引任何库函数，也不用查表 —— 表会占 Flash，而这个多项式只需要几个乘法。
 *
 * 精度实测（在测试里会打印）：全量程最大误差 < 0.01%。
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
        a += (int64_t)ODOM_MRAD_PER_TURN; /* 归约到 [0, 6283) */
    }
    if (a >= (int64_t)ODOM_MRAD_PER_HALF) {
        a -= (int64_t)ODOM_MRAD_PER_HALF;
        sign = -1; /* 归约到 [0, 3142)，用 s(π−x) = s(x) */
    }
    if (a > (int64_t)ODOM_MRAD_PER_QUART) {
        a = (int64_t)ODOM_MRAD_PER_HALF - a; /* 折到 [0, 1571]，用 s(π/2+..) 的对称 */
    }

    x  = a * (int64_t)ODOM_Q20 / 1000;          /* Q20 弧度，a <= 1571 => x <= 1647312 */
    x2 = x * x / (int64_t)ODOM_Q20;             /* Q20 x² */
    x4 = x2 * x2 / (int64_t)ODOM_Q20;           /* Q20 x⁴ */
    x6 = x4 * x2 / (int64_t)ODOM_Q20;           /* Q20 x⁶ */
    x8 = x6 * x2 / (int64_t)ODOM_Q20;           /* Q20 x⁸ */

    /* s(x)/x = 1 − x²/6 + x⁴/120 − x⁶/5040 + x⁸/362880 （到 x⁸ 就够） */
    poly = (int64_t)ODOM_Q20 - x2 / 6 + x4 / 120 - x6 / 5040 + x8 / 362880;

    s = x * poly / (int64_t)ODOM_Q20; /* Q20 正弦 */
    s = s / 32;                       /* Q20 -> Q15 */

    if (sign < 0) {
        s = -s;
    }
    return (int32_t)odom_clamp_i64(s, -32768, 32767);
}

/* 余弦：直接用 s(x + π/2)。省一份多项式，也少一处不一致的可能。 */
static int32_t odom_cosine_q15(int64_t mrad)
{
    return odom_sine_q15(mrad + (int64_t)ODOM_MRAD_PER_QUART);
}

/* 把累计角度归一化到 (-ODOM_HALF_TURN_MRAD, ODOM_HALF_TURN_MRAD] mrad，
 * 即 (-3142, 3142]。用整数取模，长期运行不溢出。
 *
 * 恒等式（测试 [5] 的判据 b 就是验这个）：
 *   pose_theta − total_theta ≡ 0 (mod ODOM_FULL_TURN_MRAD)
 * 因为归一化只做「减去 k 个整圈」，而 ODOM_FULL_TURN_MRAD 恰好是一整圈
 * ⇒ 归一化保持角度等价。取模那步用了 (HALF−1) 做偏置，正是为了让结果
 * 落在左开右闭区间 (-HALF, +HALF]。 */
static int32_t odom_normalize_mrad(int64_t t)
{
    int64_t m = (t + (int64_t)(ODOM_HALF_TURN_MRAD - 1)) % (int64_t)ODOM_FULL_TURN_MRAD;
    if (m < 0) {
        m += (int64_t)ODOM_FULL_TURN_MRAD;
    }
    return (int32_t)(m - (int64_t)(ODOM_HALF_TURN_MRAD - 1));
}

/* ------------------------------------------------------------------ */
/* 复位                                                                */
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
/* 一步更新                                                            */
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

    /* 1) 四轮位移增量（mm）。用 int64 做差再夹回 int32，
     *    这样即使两帧计数差得离谱也不会溢出。 */
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        int64_t d = (int64_t)d_now[i] - (int64_t)d_prev[i];
        d = odom_clamp_i64(d, -2147483648LL, 2147483647LL);
        w.wheel_mm_s[i] = (int32_t)d;
    }

    /* 2) 正解：线性映射，喂位移增量就得到「机体位移 + 转角增量」 */
    mecanum_forward(g, &w, &inc);

    /* 单步转角护栏。物理上一个 20 ms 控制周期转不过 90°（按最高轮速
     * 1000 mm/s、L=300 mm 估算上限约 267 mrad），这条护栏专为脏数据准备：
     * 计数增量一旦离谱，下面的级数中间量会溢出，而有符号溢出是 UB。
     * 护栏把「单步转角」限制在 ±π/2 内，级数到 h⁸ 在这个区间内相对误差 < 1e-9。 */
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

    /* 3) 圆弧积分的 sinc 因子 k = s(h)/h（h = Δθ/2 的弧度值），Q15。
     *    |Δθ| 很小时直接取 1：此时 1 − k ≈ (Δθ/2)²/6，Δθ=8 mrad 时不到 3e-6。
     *    否则用 k 自己的级数 k = 1 − h²/6 + h⁴/120 − h⁶/5040 + h⁸/362880 直接算 ——
     *    ★ 不要写成「先求 s(h) 再除以 h」两步算：小角度时 s(h) 的整数结果
     *      只有两三位有效数字，再拿去做除法会把相对误差放大到千分之二量级，
     *      几十步累积下来就是几毫米的系统性位置偏差。这里在 Q30 上算，
     *      最后一步才降到 Q15。
     *
     *    ★ 量纲（踩过坑，写清楚）：h 的单位是 mrad，而级数里要的是弧度 h/1000。
     *      所以把 h 的高次幂降 6 位存：u2 = h²，u4 = h⁴/1e6，u6 = h⁶/1e12，
     *      u8 = h⁸/1e18 —— 这样每一项 u_k 恰好等于「弧度幂 × 1e6」，
     *      除以 (系数 × 1e6) 就回到无量纲，再乘 2^30 升到 Q30。
     *      即分母依次是 6e6 / 1.2e8 / 5.04e9 / 3.6288e11。
     *      护栏已把 |Δθ| 限在 π/2 内，此处中间量最大约 1e14，远离 int64 溢出。 */
    if (dtheta > -8 && dtheta < 8) {
        k_q15 = 32768;
    } else {
        int64_t u2;
        int64_t u4;
        int64_t u6;
        int64_t u8;
        int64_t k_q30;
        h  = dtheta / 2; /* 单位 mrad；|Δθ| >= 8 保证 h != 0 */
        u2 = h * h;                            /* h²        */
        u4 = (u2 * u2) / 1000000LL;            /* h⁴ / 1e6  */
        u6 = (u4 * u2) / 1000000LL;            /* h⁶ / 1e12 */
        u8 = (u6 * u2) / 1000000LL;            /* h⁸ / 1e18 */
        k_q30 = ((int64_t)1 << 30)
              - (u2 * ((int64_t)1 << 30)) / 6000000LL
              + (u4 * ((int64_t)1 << 30)) / 120000000LL
              - (u6 * ((int64_t)1 << 30)) / 5040000000LL
              + (u8 * ((int64_t)1 << 30)) / 362880000000LL;
        k_q15 = k_q30 / 32768;
    }

    /* 4) 按本步「中间航向」把机体位移旋到世界系 */
    theta_mid = total->theta_mrad + dtheta / 2;
    sin_m     = odom_sine_q15(theta_mid);
    cos_m     = odom_cosine_q15(theta_mid);

    d_x_world = odom_div_round(k_q15 * (dx * (int64_t)cos_m - dy * (int64_t)sin_m),
                               (int64_t)1 << 30);
    d_y_world = odom_div_round(k_q15 * (dx * (int64_t)sin_m + dy * (int64_t)cos_m),
                               (int64_t)1 << 30);

    /* 5) 提交 */
    pose->x_mm  += d_x_world;
    pose->y_mm  += d_y_world;
    total->x_mm += d_x_world;
    total->y_mm += d_y_world;

    total->theta_mrad += dtheta;
    pose->theta_mrad = odom_normalize_mrad(total->theta_mrad);

    return 1;
}
