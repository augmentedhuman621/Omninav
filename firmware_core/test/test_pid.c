/*
 * test_pid.c — pid 模块的测试台
 *
 * 覆盖：
 *   [1] 阶跃响应（一阶惯性对象上的闭环收敛）
 *   [2] 输出限幅（任何输入下 |out| <= out_max）
 *   [3] ★ 抗积分饱和：对比「有抗饱和」与「朴素 PI」的退饱和时间
 *   [4] 积分分离（大偏差阶段不许积分）
 *   [5] 微分作用在测量值上（无微分尖峰）
 *   [6] 复位语义
 *   [7] 参数校验与非法输入
 *   [8] 空指针安全
 *   [9] 确定性（同输入序列 -> 同输出序列，逐位一致）
 *   [10] 饱和标志位
 *
 * 说明：本文件是测试，允许用 printf / 整数对象模型。src/ 与 include/ 依旧
 *       零浮点、零第三方依赖（make check-purity 会检查）。
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

/* 一阶惯性对象：v[k+1] = v[k] + (gain*u[k] - v[k]) / tau
 * gain = 每个控制量对应的稳态速度（mm/s），tau = 时间常数（步）。
 * 用它而不是「v = u」是为了让闭环真的有动态，才测得出超调与整定。 */
static int32_t plant_step(int32_t u, int32_t v, int32_t gain, int32_t tau)
{
    int64_t d = ((int64_t)gain * (int64_t)u - (int64_t)v);
    return (int32_t)((int64_t)v + d / tau);
}

/* ================================================================== */
/* [1] 阶跃响应                                                        */
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

    printf("[1] 阶跃响应（kp=0.5 ki=0.02，对象 gain=1 tau=5 步）\n");

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

    printf("  [info] 峰值 %ld mm/s（超调 %ld‰），稳态误差 %ld mm/s，首次进入 ±3 mm/s 于第 %d 步\n",
           (long)peak, (long)((peak - sp) * 1000 / sp), (long)final_err, settled_at);

    ck_true("[1] 收敛到设定值（稳态误差 <= 2 mm/s）", final_err <= 2);
    ck_true("[1] 无大超调（峰值 <= 1.2 倍设定值）", peak <= sp + sp / 5);
    ck_true("[1] 1000 步内进入 ±3 mm/s", settled_at >= 0);
}

/* ================================================================== */
/* [2] 输出限幅                                                        */
/* ================================================================== */

static void test_output_limit(void)
{
    pid_gains_t g = pid_default_gains();
    pid_state_t st;
    int32_t sp_list[7] = { 0, 1, -1, 100, -100, 100000, -100000 };
    int32_t i;
    int32_t j;

    printf("[2] 输出限幅\n");

    for (i = 0; i < 7; ++i) {
        pid_reset(&st);
        for (j = 0; j < 50; ++j) {
            uint32_t fl = 0u;
            int32_t u = pid_step(&g, &st, sp_list[i], 0, &fl);
            ck_true("[2] |out| <= out_max", u <= g.out_max && u >= g.out_min);
        }
    }

    /* 非对称限幅也要成立 */
    {
        pid_gains_t g2 = pid_default_gains();
        pid_state_t st2;
        pid_reset(&st2);
        g2.out_min = -5;
        g2.out_max = 7;
        for (i = 0; i < 50; ++i) {
            uint32_t fl = 0u;
            int32_t u = pid_step(&g2, &st2, 5000, 0, &fl);
            ck_true("[2] 非对称限幅", u == 7);
        }
    }
}

/* ================================================================== */
/* [3] ★ 抗积分饱和                                                    */
/* ================================================================== */

/* 测试专用的「朴素 PI」参照物：没有积分限幅、没有条件积分。
 * 放在测试里而不是 src/ 里 —— 它只是为了量出差距，不是产品代码。 */
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
    g.ki_q16 = PID_Q16_ONE / 2;      /* 0.5  —— 故意调大，好让饱和明显 */
    g.integ_limit = 1000;
    pid_reset(&st);

    /* 相位 1：给一个物理上够不到的指令（对象上限 1000 mm/s），跑 500 步 */
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

    /* 相位 2：指令归零，看多少步之内速度真的降下来 */
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

    printf("[3] ★ 抗积分饱和（指令 2000 mm/s，对象上限 1000 mm/s）\n");

    /* 有抗饱和：积分累加器必须被夹住 */
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
    printf("  [info] 积分累加器 = %ld，理论上限 = %ld\n", (long)st.integ_raw, (long)bound);
    ck_true("[3] 积分累加器被夹在上限内", st.integ_raw <= bound && st.integ_raw >= -bound);
    ck_true("[3] 饱和期间积分不再增长到天上", st.integ_raw <= bound);

    real_steps  = steps_to_calm(1);
    naive_steps = steps_to_calm(0);
    printf("  [info] 退饱和到 v<=10 mm/s 所需步数：有抗饱和 %d 步，朴素 PI %d 步\n",
           real_steps, naive_steps);

    ck_true("[3] 有抗饱和时能在 300 步内退饱和", real_steps >= 0 && real_steps < 300);
    ck_true("[3] 且不慢于朴素 PI（同参数下应显著更快）",
            naive_steps < 0 || real_steps <= naive_steps);
}

/* ================================================================== */
/* [4] 积分分离                                                        */
/* ================================================================== */

static void test_integral_separation(void)
{
    pid_gains_t g = pid_default_gains();
    pid_state_t st;
    int32_t i;
    int sep_seen = 0;

    printf("[4] 积分分离（阈值 50 mm/s，偏差 1000 mm/s）\n");

    g.integ_sep_mm_s = 50;
    pid_reset(&st);
    for (i = 0; i < 20; ++i) {
        uint32_t fl = 0u;
        (void)pid_step(&g, &st, 1000, 0, &fl);
        if ((fl & PID_FLAG_INTEG_SEP) != 0u) {
            sep_seen = 1;
        }
    }
    ck_true("[4] 大偏差时置 INTEG_SEP 标志", sep_seen != 0);
    ck_eq32("[4] 分离期间积分累加器保持 0", (int32_t)st.integ_raw, 0);

    /* 偏差回到阈值内后，积分必须恢复 */
    pid_reset(&st);
    for (i = 0; i < 20; ++i) {
        uint32_t fl = 0u;
        (void)pid_step(&g, &st, 10, 0, &fl);
        ck_true("[4] 小偏差时不置 INTEG_SEP 标志", (fl & PID_FLAG_INTEG_SEP) == 0u);
    }
    ck_true("[4] 小偏差时积分正常累加", st.integ_raw > 0);
}

/* ================================================================== */
/* [5] 微分作用在测量值上                                              */
/* ================================================================== */

static void test_derivative(void)
{
    pid_gains_t g = pid_default_gains();
    pid_state_t st;
    int32_t u0;
    int32_t u1;

    printf("[5] 微分作用在测量值上（kd = 1.0，ki = 0）\n");

    g.ki_q16 = 0;
    g.kd_q16 = PID_Q16_ONE; /* 1.0 */
    pid_reset(&st);

    /* 第一步：设定值突变 0 -> 100，但反馈还是 0，微分项必须为 0（无尖峰） */
    u0 = pid_step(&g, &st, 100, 0, 0);
    ck_eq32("[5] 设定值阶跃首步无微分尖峰（应为 0.5*100 = 50）", u0, 50);

    /* 第二步：反馈 0 -> 10，微分项 = -1.0*(10-0) = -10
     *        误差 90 -> P 项 45，合计 35 */
    u1 = pid_step(&g, &st, 100, 10, 0);
    ck_eq32("[5] 反馈上升时微分项反向（应为 45-10 = 35）", u1, 35);

    /* 反馈不变则微分项为 0 */
    ck_eq32("[5] 反馈不变时微分项为 0（应为 45）", pid_step(&g, &st, 100, 10, 0), 45);

    /* ki = 0 时积分累加器必须原地不动 */
    ck_eq32("[5] ki=0 时积分累加器不动", (int32_t)st.integ_raw, 0);
}

/* ================================================================== */
/* [6] 复位语义                                                        */
/* ================================================================== */

static void test_reset(void)
{
    pid_gains_t g = pid_default_gains();
    pid_state_t st;
    int32_t i;

    printf("[6] 复位语义\n");

    pid_reset(&st);
    for (i = 0; i < 10; ++i) {
        (void)pid_step(&g, &st, 100, 0, 0);
    }
    ck_true("[6] 跑过之后积分非零", st.integ_raw != 0);
    pid_reset(&st);
    ck_eq32("[6] 复位清积分", (int32_t)st.integ_raw, 0);
    ck_eq32("[6] 复位清 prev_meas", st.prev_meas, 0);
    ck_eq32("[6] 复位清 started", (int32_t)st.started, 0);

    /* 复位后第一步必须与全新状态的第一位一致（微分项不残留） */
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
        ck_eq32("[6] 复位后首步与全新状态一致", ub, ua);
    }

    pid_reset(0); /* 必须不崩 */
    ck_true("[6] pid_reset(0) 安全", 1);
}

/* ================================================================== */
/* [7] 参数校验                                                        */
/* ================================================================== */

static void test_validation(void)
{
    pid_gains_t g = pid_default_gains();

    printf("[7] 参数校验\n");

    ck_true("[7] 默认参数合法", pid_gains_is_valid(&g) == 1);

    g.out_min = 100;
    g.out_max = 100;
    ck_true("[7] out_min >= out_max 非法", pid_gains_is_valid(&g) == 0);

    g = pid_default_gains();
    g.kp_q16 = -1;
    ck_true("[7] 负增益非法", pid_gains_is_valid(&g) == 0);

    g = pid_default_gains();
    g.ki_q16 = -1;
    ck_true("[7] 负 ki 非法", pid_gains_is_valid(&g) == 0);

    g = pid_default_gains();
    g.kd_q16 = -1;
    ck_true("[7] 负 kd 非法", pid_gains_is_valid(&g) == 0);

    ck_true("[7] NULL 非法", pid_gains_is_valid(0) == 0);
}

/* ================================================================== */
/* [8] 空指针安全                                                      */
/* ================================================================== */

static void test_null_safety(void)
{
    pid_gains_t g = pid_default_gains();
    pid_state_t st;

    printf("[8] 空指针安全\n");

    pid_reset(&st);
    ck_eq32("[8] g=NULL 返回 0", pid_step(0, &st, 100, 0, 0), 0);
    ck_eq32("[8] st=NULL 返回 0", pid_step(&g, 0, 100, 0, 0), 0);
    ck_eq32("[8] flags=NULL 可用", pid_step(&g, &st, 0, 0, 0), 0);
    ck_true("[8] 上面几步没有破坏状态", st.started == 0 || st.started == 1);
}

/* ================================================================== */
/* [9] 确定性                                                          */
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

    printf("[9] 确定性\n");

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
    ck_true("[9] 两遍完全一致", same != 0);
}

/* ================================================================== */
/* [10] 饱和标志                                                       */
/* ================================================================== */

static void test_saturation_flag(void)
{
    pid_gains_t g = pid_default_gains();
    pid_state_t st;
    uint32_t fl = 0u;
    int32_t u;

    printf("[10] 饱和标志\n");

    g.kp_q16 = PID_Q16_ONE; /* 1.0，偏差 1000 -> P 项正好等于上限 */
    g.ki_q16 = 0;
    g.kd_q16 = 0;
    pid_reset(&st);
    u = pid_step(&g, &st, 1000, 0, &fl);
    ck_eq32("[10] 偏差 1000 -> 输出正好等于上限", u, 1000);
    ck_true("[10] 恰好落在限幅边界也要置 SATURATED", (fl & PID_FLAG_SATURATED) != 0u);

    /* ★ 这是曾经漏掉的那条：out_raw 只**正好等于**上限时，老的判据
       `out != out_raw` 为假 -> 标志没置。语义上「偏差再大也推不动了」
       与「已经被削掉」是同一件事，所以判据必须是「输出顶在边界」。 */
    pid_reset(&st);
    u = pid_step(&g, &st, 5000, 0, &fl);
    ck_eq32("[10] 偏差 5000 -> 输出被削到上限", u, 1000);
    ck_true("[10] 被削掉时置 SATURATED", (fl & PID_FLAG_SATURATED) != 0u);

    pid_reset(&st);
    u = pid_step(&g, &st, -5000, 0, &fl);
    ck_eq32("[10] 负向被削到下限", u, -1000);
    ck_true("[10] 负向被削掉时置 SATURATED", (fl & PID_FLAG_SATURATED) != 0u);

    pid_reset(&st);
    u = pid_step(&g, &st, -1000, 0, &fl);
    ck_eq32("[10] 负向正好等于下限", u, -1000);
    ck_true("[10] 负向恰好落在边界也置 SATURATED", (fl & PID_FLAG_SATURATED) != 0u);

    /* 没顶到边界时必须不置 —— 否则这个标志就成了"永远为真"的噪声 */
    pid_reset(&st);
    u = pid_step(&g, &st, 999, 0, &fl);
    ck_eq32("[10] 偏差 999 -> 未饱和", u, 999);
    ck_true("[10] 差一个计数就不置 SATURATED", (fl & PID_FLAG_SATURATED) == 0u);

    g.kp_q16 = 1;
    pid_reset(&st);
    u = pid_step(&g, &st, 10, 0, &fl);
    ck_true("[10] 未饱和时不置标志", (fl & PID_FLAG_SATURATED) == 0u);
    ck_eq32("[10] 小偏差输出", u, 0);
}

/* ================================================================== */

int main(void)
{
    printf("=== pid 速度环 PID 测试 ===\n");

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
