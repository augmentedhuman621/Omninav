/*
 * test_odom.c — odom 模块的测试台（★ 验收标准第 5 条）
 *
 * 覆盖：
 *   [1] 复位
 *   [2] ★ 直线 1000 mm -> x 误差 <= 2 mm
 *   [3] ★ 原地自转 360° -> 位置漂移 <= 5 mm（圆弧积分才过得去的判据）
 *   [4] ★ 已知圆弧轨迹 -> 误差按来源拆成 5 条独立判据：
 *         A 连续圆弧积分 vs 解析真值   （算法本身，理论上精确）
 *         B 整数版 vs 逐步取整参照     （定点三角 + sinc）
 *         C 整数版 vs 连续参照         （每步取整到 1 mm 的量化）
 *         D 整数版 vs 解析真值         （A/B/C 的合成，自洽性交叉验证）
 *         E 朴素欧拉 vs 解析真值       （对照，证明判据有意义）
 *       外加「步数实验」：4/40/400 步同一步长，证明圆弧积分没有离散化误差，
 *       位置误差全部来自每步取整（随步数增长）。
 *   [5] 角度归一化与 pose/total 一致性
 *        含契约 §5.1 的验收增量：自转 ≥ 2 整圈后验「pose − total 恰为整圈
 *        6284 mrad 的非零整数倍」（归一化保持角度等价），见判据 a/b/c
 *   [6] 非法输入（空指针 / 几何非法）
 *   [7] twist_out 的语义（本步增量，不是速率）
 *   [8] 零增量与确定性
 *
 * 说明：本文件是测试，允许用 double / math.h 作为「外部参照」。
 *       src/ 与 include/ 依旧零浮点、零第三方依赖。
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

/* 前置声明：几何取默认参数（L = half_wheelbase + half_track = 300 mm）。
   定义在文件后段，但前面的 run_arc() 就要用 —— C99 没有隐式声明，
   -Werror 会直接把「先使用后声明」判成编译错误。 */
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
 * 双精度圆弧积分参照（把每步结果四舍五入到整数毫米，与整数版的「语义」对齐，
 * 但三角函数用 libm 的真值）。它是本模块的独立外部参照 ——
 * odom.c 用的是自研 Q20 多项式定点正弦，两者实现完全不同。
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

/* 朴素欧拉累加：把机体位移直接加到世界坐标上（不按航向旋转）。
 * 这就是圆弧积分要替代的那个错法。 */
static void naive_step(double *nx, double *ny, int32_t dx, int32_t dy)
{
    *nx += (double)dx;
    *ny += (double)dy;
}

/*
 * 连续圆弧积分参照：与 ref_arc_step 同一套公式，但**不做每步取整**。
 *
 * 它的用处是把两件完全不同的误差分开：
 *   ① 圆弧积分算法本身 vs 解析真值     —— 用本函数量
 *   ② 每步把世界系位移取整到 1 mm 的量化 —— 用「整数版 vs 本函数」量
 *
 * ★ 结论（见 [4] 的步数实验）：圆弧积分对**常曲率**路径在数学上是精确的 ——
 *   把 N 步的解析和算出来，两端的 2·sin(Δθ/2) 会互相约掉，恒等于
 *   (R·s(Θ), R·(1−c(Θ)))，与步数 N 无关。所以本函数无论 40 步还是 400 步
 *   都紧贴解析真值；位置误差全部来自「②每步取整」。
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
 * 跑 N 步同一段圆弧（每步前进 50 mm + 转 20 mrad，R = 2500 mm），
 * 同时给出「整数里程计」与「连续双精度参照」的终点。
 * 用来把误差按来源拆开：连续参照贴真值 = 算法没问题，整数版偏离 = 取整量化。
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
    return mecanum_default_geom(); /* L = 300 mm（半轴距 150 + 半轮距 150） */
}

/* ================================================================== */
/* [1] 复位                                                            */
/* ================================================================== */

static void test_reset(void)
{
    odom_pose_t p;
    odom_total_t t;

    printf("[1] 复位\n");

    p.x_mm = 1;
    p.y_mm = 2;
    p.theta_mrad = 3;
    t.x_mm = 4;
    t.y_mm = 5;
    t.theta_mrad = 6;
    odom_reset(&p, &t);
    ck_near64("[1] pose.x 清零", p.x_mm, 0, 0);
    ck_near64("[1] pose.y 清零", p.y_mm, 0, 0);
    ck_near64("[1] pose.theta 清零", (int64_t)p.theta_mrad, 0, 0);
    ck_near64("[1] total.x 清零", t.x_mm, 0, 0);
    ck_near64("[1] total.theta 清零", t.theta_mrad, 0, 0);

    odom_reset(0, 0); /* 必须不崩 */
    odom_reset(&p, 0);
    odom_reset(0, &t);
    ck_true("[1] odom_reset 支持空指针", 1);
}

/* ================================================================== */
/* [2] ★ 直线 1000 mm                                                  */
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

    printf("[2] ★ 直线 1000 mm（100 步 × 四轮各 10 mm）\n");

    odom_reset(&p, &t);
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        prev[i] = 0;
    }

    for (step = 0; step < 100; ++step) {
        mecanum_twist_t inc;
        for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
            now[i] = prev[i] + 10;
        }
        ck_true("[2] odom_update 返回 1", odom_update(&p, &t, &g, now, prev, &inc) == 1);
        ck_true("[2] 直线上无转角", inc.vx_mm_s == 10 && inc.vy_mm_s == 0 &&
                                     inc.omega_mrad_s == 0);
        ref_arc_step(&rx, &ry, &rth, inc.vx_mm_s, inc.vy_mm_s, inc.omega_mrad_s);
        for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
            prev[i] = now[i];
        }
    }

    printf("  [info] 整数里程计：x = %ld mm, y = %ld mm, theta = %ld mrad\n",
           (long)p.x_mm, (long)p.y_mm, (long)p.theta_mrad);
    printf("  [info] 双精度圆弧参照：x = %ld mm, y = %ld mm\n",
           (long)rx, (long)ry);

    ck_near64("[2] x 误差 <= 2 mm（对 1000 mm 真值）", p.x_mm, 1000, 2);
    ck_near64("[2] y 误差 <= 2 mm", p.y_mm, 0, 2);
    ck_near64("[2] theta 为 0", (int64_t)p.theta_mrad, 0, 0);
    ck_near64("[2] 与双精度参照一致（<=2 mm）", p.x_mm, (int64_t)rx, 2);
    ck_near64("[2] 与双精度参照一致 y（<=2 mm）", p.y_mm, (int64_t)ry, 2);
}

/* ================================================================== */
/* [3] ★ 原地自转 360°                                                 */
/* ================================================================== */

static void test_spin_in_place(void)
{
    mecanum_geom_t g = geom();
    odom_pose_t p;
    odom_total_t t;
    /* 纯自转的轮位移模式 = S_OMG × rot。
     * 这里故意在四轮上加一个「成对反号的小偏置」，等价于编码器量化在
     * 每步都漏进一点点横向位移：{ -20, +20, -18, +18 }
     *   -> 正解 vy = 1 mm/步（横向偏置），转角 = 63 mrad/步。 */
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

    printf("[3] ★ 原地自转 360°（100 步，每步带 1 mm 横向量化偏置）\n");

    odom_reset(&p, &t);
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        prev[i] = 0;
    }

    for (step = 0; step < 100; ++step) {
        mecanum_twist_t inc;
        for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
            now[i] = prev[i] + pattern[i];
        }
        ck_true("[3] 更新成功", odom_update(&p, &t, &g, now, prev, &inc) == 1);
        ck_near64("[3] 每步机体前向位移为 0（不该有 x）", inc.vx_mm_s, 0, 0);
        ck_near64("[3] 每步横向增量为 1 mm", inc.vy_mm_s, 1, 0);
        ref_arc_step(&rx, &ry, &rth, inc.vx_mm_s, inc.vy_mm_s, inc.omega_mrad_s);
        naive_step(&nx, &ny, inc.vx_mm_s, inc.vy_mm_s);
        for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
            prev[i] = now[i];
        }
    }

    theta_total = t.theta_mrad;
    arc_drift   = sqrt((double)p.x_mm * (double)p.x_mm + (double)p.y_mm * (double)p.y_mm);
    naive_drift = sqrt(nx * nx + ny * ny);

    printf("  [info] 累计转角 = %ld mrad（= %.2f 度，目标 360 度 = 6283 mrad）\n",
           (long)theta_total, (double)theta_total / 1000.0 * 57.29577951308232);
    printf("  [info] 圆弧积分位置漂移 = %.3f mm（x=%ld, y=%ld）\n",
           arc_drift, (long)p.x_mm, (long)p.y_mm);
    printf("  [info] 双精度圆弧参照漂移 = %.3f mm\n",
           sqrt(rx * rx + ry * ry));
    printf("  [info] 朴素欧拉累加漂移 = %.3f mm  <- 圆弧积分要消灭的就是它\n",
           naive_drift);

    ck_true("[3] 圆弧积分：位置漂移 <= 5 mm", arc_drift <= 5.0);
    ck_true("[3] 朴素累加：漂移显著更大（>= 50 mm），证明这条判据有意义",
            naive_drift >= 50.0);
    ck_near64("[3] 与双精度参照一致（<=2 mm）", p.x_mm, (int64_t)rx, 2);
    ck_near64("[3] 与双精度参照一致 y（<=2 mm）", p.y_mm, (int64_t)ry, 2);
    ck_true("[3] 转角落在 360 度附近（6000~6600 mrad）",
            theta_total >= 6000 && theta_total <= 6600);
}

/* ================================================================== */
/* [4] ★ 已知圆弧轨迹                                                  */
/* ================================================================== */

static void test_known_arc(void)
{
    mecanum_geom_t g = geom();
    odom_pose_t p;
    odom_total_t t;
    /* 每步四轮位移 {44, 56, 44, 56}：
     *   正解 vx = 50 mm，vy = 0，Δθ = 20 mrad
     *   => 半径 R = vx/ω = 50/0.020 = 2500 mm 的标准圆弧 */
    const int32_t pattern[MECANUM_WHEEL_COUNT] = { 44, 56, 44, 56 };
    const int steps = 40;
    const int32_t dx_step = 50;
    const int32_t dth_step = 20; /* mrad */
    int32_t prev[MECANUM_WHEEL_COUNT];
    int32_t now[MECANUM_WHEEL_COUNT];
    double rx = 0.0;   /* 逐步取整的双精度参照 */
    double ry = 0.0;
    double cx = 0.0;   /* 连续（不取整）的双精度参照 */
    double cy = 0.0;
    double nx = 0.0;
    double ny = 0.0;
    int64_t rth = 0;
    int64_t cth = 0;
    double radius;
    double theta_end;
    double truth_x;
    double truth_y;
    double err_round;  /* 整数版 vs 逐步取整参照：定点三角 + sinc 的正确性 */
    double err_cont;   /* 连续参照 vs 解析真值：圆弧积分算法本身 */
    double err_quant;  /* 整数版 vs 连续参照：每步取整到 1 mm 的量化 */
    double err_truth;  /* 整数版 vs 解析真值：上面两项的合成 */
    double err_naive;  /* 朴素欧拉 vs 解析真值：对照，证明这条判据有意义 */
    double quant_bound;
    int step;
    int i;

    printf("[4] ★ 已知圆弧轨迹（%d 步 × 前进 %ld mm + 转 %ld mrad）\n",
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
        ck_true("[4] 更新成功", odom_update(&p, &t, &g, now, prev, &inc) == 1);
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

    /* 每步两个轴各损失不超过 0.5 mm，所以累计欧氏误差的严格上界是
       0.5·sqrt(2)·N = 0.7071·N（再加 1 mm 的余量）。 */
    quant_bound = 0.70710678 * (double)steps + 1.0;

    printf("  [info] 圆弧半径 R = %.1f mm，累计转角 = %.4f rad\n", radius, theta_end);
    printf("  [info] 解析真值       ：x = %.3f mm, y = %.3f mm\n", truth_x, truth_y);
    printf("  [info] 连续圆弧参照   ：x = %.3f mm, y = %.3f mm\n", cx, cy);
    printf("  [info] 整数圆弧积分   ：x = %ld mm, y = %ld mm\n",
           (long)p.x_mm, (long)p.y_mm);
    printf("  [info] 逐步取整参照   ：x = %ld mm, y = %ld mm\n", (long)rx, (long)ry);
    printf("  [info] 误差分解：\n");
    printf("         A 算法本身  (连续参照 vs 真值)   = %8.4f mm\n", err_cont);
    printf("         B 定点实现  (整数 vs 逐步取整参照) = %8.4f mm\n", err_round);
    printf("         C 逐步取整  (整数 vs 连续参照)   = %8.4f mm  (理论上界 %.1f mm)\n",
           err_quant, quant_bound);
    printf("         D 合成      (整数 vs 真值)       = %8.4f mm\n", err_truth);
    printf("         E 朴素欧拉  (对照，vs 真值)      = %8.4f mm\n", err_naive);

    /* ★ 判据拆成五条独立的，不再用「整数 vs 真值」一条大杂烩去卡：
       A 证明圆弧积分算法对常曲率路径是精确的（理论上应为 0）；
       B 证明自研 Q20 定点三角 + Q30 sinc 没有引入可见误差；
       C 把「每步取整到 1 mm」这一项的量化界写清楚；
       D 是 A/B/C 的合成，用来交叉验证分解没有漏项；
       E 证明不做圆弧积分根本不行（否则 A~D 全都可以怀疑是判据太松）。 */
    ck_true("[4] A 圆弧积分算法本身精确（连续参照 vs 真值 <= 0.5 mm）", err_cont <= 0.5);
    ck_true("[4] B 定点实现正确（整数 vs 逐步取整参照 <= 2 mm）", err_round <= 2.0);
    ck_true("[4] C 逐步取整量化在推导界内（<= 0.7071*N + 1 mm）", err_quant <= quant_bound);
    ck_true("[4] D 误差可分解（总量 <= C 的界 + A 的界）",
            err_truth <= quant_bound + 0.5);
    ck_true("[4] E 朴素累加误差显著更大（>= 200 mm），这条判据有意义",
            err_naive >= 200.0);

    /* ---- 步数实验：同一步长（50 mm + 20 mrad），只改步数 ----
       如果那 6.4 mm 是「弦-弧离散化」造成的，那么步数越多误差应越小。
       实测恰好相反：连续参照在 4 / 40 / 400 步下都紧贴解析真值（圆弧积分对
       常曲率路径是解析精确的，与 N 无关），而整数版每步都要取整一次，
       偏离随步数增长 —— 直接指认凶手是每步取整，不是离散化。 */
    {
        static const int ntest[3] = { 4, 40, 400 };
        printf("  [info] 步数实验（同一步长，R = 2500 mm）：\n");
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

            printf("         N = %3d 步：连续参照 vs 真值 %7.4f mm ｜ "
                   "整数 vs 连续参照 %9.3f mm（%.4f mm/步）\n",
                   ntest[i], e_cont, e_quant, e_quant / (double)ntest[i]);

            if (ntest[i] == 40) {
                ck_true("[4] 步数实验：40 步下算法误差仍 <= 0.5 mm", e_cont <= 0.5);
                ck_true("[4] 步数实验：40 步下量化可见（>= 1 mm）", e_quant >= 1.0);
                ck_true("[4] 步数实验：40 步量化在界内",
                        e_quant <= 0.70710678 * 40.0 + 1.0);
            }
            if (ntest[i] == 400) {
                ck_true("[4] 步数实验：400 步下算法误差仍 <= 0.5 mm（没有离散化误差）",
                        e_cont <= 0.5);
                ck_true("[4] 步数实验：400 步量化在界内",
                        e_quant <= 0.70710678 * 400.0 + 1.0);
            }
        }
    }
}

/* ================================================================== */
/* [5] 角度归一化与一致性                                              */
/* ================================================================== */

static void test_angle_normalization(void)
{
    odom_pose_t p;
    odom_total_t t;
    int i;

    printf("[5] 角度归一化与 pose/total 一致性\n");

    odom_reset(&p, &t);
    ck_near64("[5] 初始 theta", (int64_t)p.theta_mrad, 0, 0);

    /* 小角度不归一化：pose 就是 total 本身 */
    {
        mecanum_geom_t g = geom();
        int32_t prev[MECANUM_WHEEL_COUNT] = { 0, 0, 0, 0 };
        int32_t now[MECANUM_WHEEL_COUNT]  = { 0, 0, 0, 0 };
        (void)odom_update(&p, &t, &g, now, prev, 0);
        ck_near64("[5] 零增量时 pose.theta == total.theta",
                   (int64_t)p.theta_mrad, t.theta_mrad, 0);
    }

    /* 直接验证归一化的边界：把 total 推到 200000 mrad。
     * 整圈 = 6284 mrad ⇒ 200000 − (−1088) = 201088 = 32 × 6284，
     * 期望 pose.theta = 200000 − 32×6284 = -1088 */
    odom_reset(&p, &t);
    {
        mecanum_geom_t g = geom();
        int32_t prev[MECANUM_WHEEL_COUNT] = { 0, 0, 0, 0 };
        int32_t now[MECANUM_WHEEL_COUNT];
        /* 用纯自转模式，每步 Δθ 由 L 决定；一整套自转累到 200000 mrad 需要
         * 三千多步，所以这里直接构造一个「已经转过 200000 mrad」的 total
         * 再用一步推进。 */
        t.theta_mrad = 200000 - 63; /* 再走一步 +63 就正好 200000 */
        p.theta_mrad = 0;
        for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
            now[i] = prev[i] + ((i % 2 == 0) ? -19 : 19);
        }
        (void)odom_update(&p, &t, &g, now, prev, 0);
        printf("  [info] total.theta = %ld mrad -> pose.theta = %ld mrad\n",
               (long)t.theta_mrad, (long)p.theta_mrad);
        ck_near64("[5] total 继续累加不清零", t.theta_mrad, 200000, 0);
        ck_near64("[5] pose 归一化到 (-3142, 3142]",
                   (int64_t)p.theta_mrad, -1088, 0);
    }

    /* ------------------------------------------------------------------
     * ★ 验收增量（契约 §5.1 定案）：连续自转 ≥ 2 整圈后验「归一化是角度
     *   等价变换」。
     *
     * 判据 b（pose − total 恰为整圈的非零整数倍）是关键 —— 只验「值落在
     * 区间内」是不够的：旧常量 180000 那版也能让区间断言通过，但
     * 2×180000 = 360000 mrad = 360 rad = 57.2958 圈（非整数），一旦 |θ| 越过
     * 180000 航向就会跳 106.48°。判据 b 同时钉住两件事：① 回绕量确实是整圈；
     * ② 确实发生过回绕（倍数非零）。这就是为什么它比区间断言强。
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

        /* 对称自转模式：四轮里程成对反向 ⇒ vx = vy = 0（纯转动）。
         * 每步 Δθ = 63 mrad ⇒ 300 步 = 18900 mrad = 3.008 圈（> 2 圈）。 */
        for (step = 0; step < 300; ++step) {
            for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
                now[i] = prev[i] + ((i % 2 == 0) ? -19 : 19);
            }
            (void)odom_update(&p, &t, &g, now, prev, 0);
            for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
                prev[i] = now[i];
            }
        }

        printf("  [info] 自转 300 步：total.theta = %ld mrad（%.3f 圈），"
               "pose.theta = %ld mrad\n",
               (long)t.theta_mrad, (double)t.theta_mrad / 6283.185,
               (long)p.theta_mrad);

        /* 判据 a：规范航向角落在 (-3142, 3142] */
        ck_true("[5] 判据a pose.theta ∈ (-3142, 3142]",
                p.theta_mrad > -3142 && p.theta_mrad <= 3142);

        /* 判据 b：差值恰为整圈 6284 的非零整数倍 ⇒ 归一化保持角度等价。
         * 误差来源只有「用 6284 代替真值 6283.185」，3 圈累计 2.4 mrad。 */
        {
            int64_t diff = (int64_t)p.theta_mrad - t.theta_mrad;
            int64_t turns = diff / 6284;
            printf("  [info] pose − total = %ld mrad = %ld × 6284（%ld 圈）\n",
                   (long)diff, (long)turns, (long)turns);
            ck_true("[5] 判据b 差值能被整圈 6284 整除", (diff % 6284) == 0);
            ck_true("[5] 判据b 倍数非零（确实回绕过）", turns != 0);
        }

        /* 判据 c：纯自转不产生虚假位置漂移（圆弧积分的基本要求） */
        ck_near64("[5] 判据c 自转 3 圈后 x 漂移 <= 5 mm", p.x_mm, 0, 5);
        ck_near64("[5] 判据c 自转 3 圈后 y 漂移 <= 5 mm", p.y_mm, 0, 5);
    }
}

/* ================================================================== */
/* [6] 非法输入                                                        */
/* ================================================================== */

static void test_invalid_input(void)
{
    mecanum_geom_t g = geom();
    odom_pose_t p;
    odom_total_t t;
    int32_t a[MECANUM_WHEEL_COUNT] = { 0, 0, 0, 0 };
    int32_t b[MECANUM_WHEEL_COUNT] = { 0, 0, 0, 0 };

    printf("[6] 非法输入\n");

    odom_reset(&p, &t);
    p.x_mm = 7;

    ck_true("[6] pose=NULL 返回 0", odom_update(0, &t, &g, a, b, 0) == 0);
    ck_true("[6] total=NULL 返回 0", odom_update(&p, 0, &g, a, b, 0) == 0);
    ck_true("[6] geom=NULL 返回 0", odom_update(&p, &t, 0, a, b, 0) == 0);
    ck_true("[6] d_now=NULL 返回 0", odom_update(&p, &t, &g, 0, b, 0) == 0);
    ck_true("[6] d_prev=NULL 返回 0", odom_update(&p, &t, &g, a, 0, 0) == 0);

    {
        mecanum_geom_t bad = geom();
        bad.half_wheelbase_mm = 0;
        bad.half_track_mm = 0;
        ck_true("[6] 几何无效返回 0", odom_update(&p, &t, &bad, a, b, 0) == 0);
    }

    ck_near64("[6] 非法输入不改动位姿", p.x_mm, 7, 0);
    ck_true("[6] twist_out 允许为 NULL", odom_update(&p, &t, &g, a, b, 0) == 1);
}

/* ================================================================== */
/* [7] twist_out 的语义                                                */
/* ================================================================== */

static void test_twist_out(void)
{
    mecanum_geom_t g = geom();
    odom_pose_t p;
    odom_total_t t;
    int32_t prev[MECANUM_WHEEL_COUNT] = { 100, 100, 100, 100 };
    int32_t now[MECANUM_WHEEL_COUNT]  = { 110, 110, 110, 110 };
    mecanum_twist_t inc;

    printf("[7] twist_out 语义（写入的是本步增量，不是速率）\n");

    odom_reset(&p, &t);
    ck_true("[7] 更新成功", odom_update(&p, &t, &g, now, prev, &inc) == 1);
    ck_near64("[7] vx 字段 = 本步 Δx（mm/步）", inc.vx_mm_s, 10, 0);
    ck_near64("[7] vy 字段 = 本步 Δy", inc.vy_mm_s, 0, 0);
    ck_near64("[7] omega 字段 = 本步 Δθ", inc.omega_mrad_s, 0, 0);
    ck_near64("[7] 位姿 x 同步前进", p.x_mm, 10, 0);
}

/* ================================================================== */
/* [8] 零增量与确定性                                                  */
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

    printf("[8] 零增量与确定性\n");

    odom_reset(&p1, &t1);
    ck_true("[8] 零增量仍返回 1（不是错误）",
            odom_update(&p1, &t1, &g, zero, zero, 0) == 1);
    ck_near64("[8] 零增量不改变位姿", p1.x_mm, 0, 0);

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
    ck_true("[8] 两遍完全一致", identical != 0);
    ck_near64("[8] 200 步 × 1 mm = 200 mm", p1.x_mm, 200, 2);
}

/* ================================================================== */

int main(void)
{
    printf("=== odom 圆弧积分里程计测试 ===\n");

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
