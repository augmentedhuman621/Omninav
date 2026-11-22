/*
 * test_mecanum.c — mecanum 模块的测试台
 *
 * 覆盖：
 *   [1] 逆解->正解 往返一致性（全部极性组合）
 *   [2] 结构用例（纯 vx / 纯 vy / 纯 omega / 零速 / 极性翻转）
 *   [3] 饱和归一（四轮同比例，方向不变，幂等）
 *   [4] 死区抬升（含「四轮全零绝不被抬起」的安全硬约束）
 *   [5] 装配映射（24 种轮序 x 16 种符号组合全往返）
 *   [6] ROS2 轮序互换
 *   [7] 参数校验与非法输入（含 L=0 除零防护、NULL 指针）
 *   [8] 一键求解机制（flags、顺序、停车语义）
 *   [9] 与 ROS2 / WPILib 两个独立实现的交叉验证
 *   [10] 分辨率参考（打印）
 *
 * 说明：本文件是测试，允许使用 double —— 但只在 [9] / [10] 里作为「外部参照」
 *      与「打印」使用。src/ 与 include/ 依旧零浮点、零第三方依赖
 *      （make check-purity 会检查）。
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
/* [1] 往返一致性                                                      */
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

    printf("[1] 逆解 -> 正解 往返一致性\n");

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

                        check_close("往返.vx", (int)cases, back.vx_mm_s, t.vx_mm_s,
                                    TOL_V_MM_S);
                        check_close("往返.vy", (int)cases, back.vy_mm_s, t.vy_mm_s,
                                    TOL_V_MM_S);
                        check_close("往返.omega", (int)cases, back.omega_mrad_s,
                                    t.omega_mrad_s, TOL_O_MRAD_S);
                        ++cases;
                    }
                }
            }
        }
    }
    printf("  [info] 往返用例 %ld 组（4 种极性组合）\n", cases);
    printf("  [info] 实测最大误差：线速度 %ld mm/s，角速度 %ld mrad/s\n",
           (long)max_ev, (long)max_eo);
}

/* ================================================================== */
/* [2] 结构用例                                                        */
/* ================================================================== */

static void test_structure(void)
{
    mecanum_geom_t g = mecanum_default_geom(); /* 150/150 => L = 300 */
    mecanum_twist_t t;
    mecanum_wheels_t w;
    int i;

    printf("[2] 结构用例\n");

    /* 纯 vx：四轮同值同号 */
    t.vx_mm_s = 500;
    t.vy_mm_s = 0;
    t.omega_mrad_s = 0;
    mecanum_inverse(&g, &t, &w);
    for (i = 0; i < 4; ++i) {
        check_eq("纯vx.轮速", i, w.wheel_mm_s[i], 500);
    }

    /* 纯 vy：符号必须是 - + + -（FL FR RL RR） */
    t.vx_mm_s = 0;
    t.vy_mm_s = 400;
    t.omega_mrad_s = 0;
    mecanum_inverse(&g, &t, &w);
    {
        static const int32_t want[4] = { -400, 400, 400, -400 };
        for (i = 0; i < 4; ++i) {
            check_eq("纯vy.轮速", i, w.wheel_mm_s[i], want[i]);
        }
    }

    /* 纯 omega = 1000 mrad/s，L = 300 => rot = 300 mm/s，符号 - + - + */
    t.vx_mm_s = 0;
    t.vy_mm_s = 0;
    t.omega_mrad_s = 1000;
    mecanum_inverse(&g, &t, &w);
    {
        static const int32_t want[4] = { -300, 300, -300, 300 };
        for (i = 0; i < 4; ++i) {
            check_eq("纯omega.轮速", i, w.wheel_mm_s[i], want[i]);
        }
    }

    /* 零速 => 四轮全零 */
    t = mecanum_zero_twist();
    mecanum_inverse(&g, &t, &w);
    for (i = 0; i < 4; ++i) {
        check_eq("零速.轮速", i, w.wheel_mm_s[i], 0);
    }

    /* 正解的纯分量还原 */
    w.wheel_mm_s[0] = 500;
    w.wheel_mm_s[1] = 500;
    w.wheel_mm_s[2] = 500;
    w.wheel_mm_s[3] = 500;
    mecanum_forward(&g, &w, &t);
    check_eq("正解.纯vx", 0, t.vx_mm_s, 500);
    check_eq("正解.纯vx", 1, t.vy_mm_s, 0);
    check_eq("正解.纯vx", 2, t.omega_mrad_s, 0);

    /* 极性翻转：vy_polarity = -1 时纯 vy 命令给出相反的轮速图案 */
    {
        mecanum_geom_t g2 = g;
        static const int32_t want[4] = { 400, -400, -400, 400 };
        g2.vy_polarity = -1;
        t.vx_mm_s = 0;
        t.vy_mm_s = 400;
        t.omega_mrad_s = 0;
        mecanum_inverse(&g2, &t, &w);
        for (i = 0; i < 4; ++i) {
            check_eq("极性-1.纯vy", i, w.wheel_mm_s[i], want[i]);
        }
        /* 正解也必须跟着翻，才能保持往返一致 */
        {
            mecanum_twist_t back;
            mecanum_forward(&g2, &w, &back);
            check_eq("极性-1.正解vy", 0, back.vy_mm_s, 400);
        }
    }
}

/* ================================================================== */
/* [3] 饱和归一                                                        */
/* ================================================================== */

static void test_desaturate(void)
{
    mecanum_wheels_t in;
    mecanum_wheels_t out;
    int r;
    int i;

    printf("[3] 饱和归一\n");

    /* 未超限：原样复制，返回 0 */
    in.wheel_mm_s[0] = 100;
    in.wheel_mm_s[1] = -200;
    in.wheel_mm_s[2] = 300;
    in.wheel_mm_s[3] = 0;
    r = mecanum_desaturate(&in, 500, &out);
    check_eq("未超限.返回", 0, r, 0);
    for (i = 0; i < 4; ++i) {
        check_eq("未超限.轮速", i, out.wheel_mm_s[i], in.wheel_mm_s[i]);
    }

    /* 超限：峰值精确卡到上限，比例保持，方向不变 */
    in.wheel_mm_s[0] = 2000;
    in.wheel_mm_s[1] = -4000;
    in.wheel_mm_s[2] = 1000;
    in.wheel_mm_s[3] = -6000;
    r = mecanum_desaturate(&in, 3000, &out);
    check_eq("超限.返回", 0, r, 1);
    check_eq("超限.峰值卡上限", 0, out.wheel_mm_s[3], -3000);
    check_eq("超限.同比例0", 0, out.wheel_mm_s[0], 1000);
    check_eq("超限.同比例1", 0, out.wheel_mm_s[1], -2000);
    check_eq("超限.同比例2", 0, out.wheel_mm_s[2], 500);
    check_eq("超限.峰值", 0, imax4(out.wheel_mm_s), 3000);

    /* 幂等：对已归一的再归一不变 */
    {
        mecanum_wheels_t out2;
        r = mecanum_desaturate(&out, 3000, &out2);
        check_eq("幂等.返回", 0, r, 0);
        for (i = 0; i < 4; ++i) {
            check_eq("幂等.轮速", i, out2.wheel_mm_s[i], out.wheel_mm_s[i]);
        }
    }

    /* limit <= 0 视为不限制 */
    r = mecanum_desaturate(&in, 0, &out);
    check_eq("不限制.返回", 0, r, 0);
    check_eq("不限制.轮速", 3, out.wheel_mm_s[3], -6000);

    /* 零输入：峰值 0，不得除零 */
    {
        mecanum_wheels_t z;
        mecanum_zero_wheels(&z);
        r = mecanum_desaturate(&z, 3000, &out);
        check_eq("零输入.返回", 0, r, 0);
        check_eq("零输入.轮速", 0, out.wheel_mm_s[0], 0);
    }
}

/* ================================================================== */
/* [4] 死区抬升                                                        */
/* ================================================================== */

static void test_lift_deadzone(void)
{
    mecanum_wheels_t in;
    mecanum_wheels_t out;
    int r;
    int i;

    printf("[4] 死区抬升\n");

    /* 安全硬约束：四轮全零绝不能被抬起来（否则停着的车会自己走） */
    for (i = 0; i < 4; ++i) {
        in.wheel_mm_s[i] = 0;
    }
    r = mecanum_lift_deadzone(&in, 300, &out);
    check_eq("全零.返回", 0, r, 0);
    for (i = 0; i < 4; ++i) {
        check_eq("全零.轮速", i, out.wheel_mm_s[i], 0);
    }

    /* 峰值已高于死区：不动 */
    in.wheel_mm_s[0] = 400;
    in.wheel_mm_s[1] = -100;
    in.wheel_mm_s[2] = 250;
    in.wheel_mm_s[3] = -50;
    r = mecanum_lift_deadzone(&in, 300, &out);
    check_eq("高于死区.返回", 0, r, 0);
    for (i = 0; i < 4; ++i) {
        check_eq("高于死区.轮速", i, out.wheel_mm_s[i], in.wheel_mm_s[i]);
    }

    /* 峰值低于死区：整体同比例放大到死区，方向保持 */
    in.wheel_mm_s[0] = 40;
    in.wheel_mm_s[1] = -80;
    in.wheel_mm_s[2] = 20;
    in.wheel_mm_s[3] = 0;
    r = mecanum_lift_deadzone(&in, 400, &out);
    check_eq("低于死区.返回", 0, r, 1);
    check_eq("低于死区.峰值等于死区", 0, out.wheel_mm_s[1], -400);
    check_eq("低于死区.比例0", 0, out.wheel_mm_s[0], 200);
    check_eq("低于死区.比例2", 0, out.wheel_mm_s[2], 100);
    check_eq("低于死区.零仍是零", 0, out.wheel_mm_s[3], 0);

    /* 死区关闭 */
    r = mecanum_lift_deadzone(&in, 0, &out);
    check_eq("关闭.返回", 0, r, 0);
    check_eq("关闭.轮速", 1, out.wheel_mm_s[1], -80);

    /* 单轮非零：照常放大（只有一个轮子被请求时也必须越过死区） */
    in.wheel_mm_s[0] = 10;
    in.wheel_mm_s[1] = 0;
    in.wheel_mm_s[2] = 0;
    in.wheel_mm_s[3] = 0;
    r = mecanum_lift_deadzone(&in, 400, &out);
    check_eq("单轮.返回", 0, r, 1);
    check_eq("单轮.轮速", 0, out.wheel_mm_s[0], 400);
    for (i = 1; i < 4; ++i) {
        check_eq("单轮.其余为零", i, out.wheel_mm_s[i], 0);
    }
}

/* ================================================================== */
/* [5] 装配映射                                                        */
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

    printf("[5] 装配映射\n");

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
            check_eq("装配表合法", (int)cases, mecanum_mount_is_valid(&m), 1);

            w.wheel_mm_s[0] = 111;
            w.wheel_mm_s[1] = -222;
            w.wheel_mm_s[2] = 333;
            w.wheel_mm_s[3] = -444;

            mecanum_apply_mount(&m, &w, motor);
            mecanum_undo_mount(&m, motor, &back);

            for (i = 0; i < 4; ++i) {
                check_eq("映射往返.轮速", (int)cases, back.wheel_mm_s[i],
                         w.wheel_mm_s[i]);
            }
            /* 各路电机都必须真的被写到过（防止恒零的假通过） */
            for (i = 0; i < 4; ++i) {
                check_eq("映射.电机非空", (int)cases, motor[i] != 0 ? 1 : 0, 1);
            }
            ++cases;
        }
    }
    printf("  [info] 轮序x符号组合 %ld 种，全部往返无差\n", cases);

    /* 恒等映射下电机指令必须等于轮速 */
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
            check_eq("恒等映射.电机", i, motor[i], w.wheel_mm_s[i]);
        }
    }

    /* 非法映射不能崩、不能被当合法 */
    {
        mecanum_mount_t bad = mecanum_identity_mount();
        mecanum_wheels_t w;
        int32_t motor[4];
        bad.motor_of_wheel[1] = 0; /* 与 FL 撞车，不是双射 */
        check_eq("非法映射.校验", 0, mecanum_mount_is_valid(&bad), 0);

        w.wheel_mm_s[0] = 1;
        w.wheel_mm_s[1] = 2;
        w.wheel_mm_s[2] = 3;
        w.wheel_mm_s[3] = 4;
        motor[0] = motor[1] = motor[2] = motor[3] = 0;
        mecanum_apply_mount(&bad, &w, motor);
        /* FL 与 FR 抢同一路电机：后写覆盖前者，但不越界、不崩。
           这也说明「非法表不会被拦下来」——必须靠 mount_is_valid 在装载时挡。 */
        check_eq("非法映射.不崩0", 0, motor[0], 2);
        check_eq("非法映射.不崩2", 0, motor[2], 3);
        check_eq("非法映射.不崩3", 0, motor[3], 4);
    }
}

/* ================================================================== */
/* [6] ROS2 轮序互换                                                   */
/* ================================================================== */

static void test_ros2_order(void)
{
    int32_t ros2[4];
    mecanum_wheels_t w;
    int32_t back[4];
    int i;

    printf("[6] ROS2 轮序互换\n");

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
        check_eq("local->ros2 往返", i, back[i], ros2[i]);
    }

    /* 轮序错位的后果必须被这个测试挡住：
       如果 RL/RR 不换，纯 vy 的图案会变成 - + - + 而不是 - + + - */
    {
        mecanum_geom_t g = mecanum_default_geom();
        mecanum_twist_t t = mecanum_zero_twist();
        t.vy_mm_s = 400;
        mecanum_inverse(&g, &t, &w);
        mecanum_wheels_to_ros2_order(&w, back);
        check_eq("ros2序.纯vy.0", 0, back[0], -400);
        check_eq("ros2序.纯vy.1", 0, back[1], 400);
        check_eq("ros2序.纯vy.2", 0, back[2], -400); /* RR */
        check_eq("ros2序.纯vy.3", 0, back[3], 400);  /* RL */
    }
}

/* ================================================================== */
/* [7] 参数校验                                                        */
/* ================================================================== */

static void test_validation(void)
{
    mecanum_geom_t g;
    mecanum_wheels_t w;
    mecanum_twist_t t;

    printf("[7] 参数校验与非法输入\n");

    g = mecanum_default_geom();
    check_eq("默认几何.合法", 0, mecanum_geom_is_valid(&g), 1);
    check_eq("L", 0, mecanum_l_mm(&g), 300);

    g.half_track_mm = -1;
    check_eq("负半轮距.非法", 0, mecanum_geom_is_valid(&g), 0);

    g = mecanum_default_geom();
    g.half_wheelbase_mm = 0;
    g.half_track_mm = 0; /* L = 0 */
    check_eq("L=0.非法", 0, mecanum_geom_is_valid(&g), 0);

    /* L = 0 绝不能除零：正解必须输出全零 */
    w.wheel_mm_s[0] = 1000;
    w.wheel_mm_s[1] = 1000;
    w.wheel_mm_s[2] = 1000;
    w.wheel_mm_s[3] = 1000;
    mecanum_forward(&g, &w, &t);
    check_eq("L=0.正解vx", 0, t.vx_mm_s, 0);
    check_eq("L=0.正解vy", 0, t.vy_mm_s, 0);
    check_eq("L=0.正解omega", 0, t.omega_mrad_s, 0);

    /* L = 0 逆解输出全零 */
    t = mecanum_zero_twist();
    t.vx_mm_s = 500;
    mecanum_inverse(&g, &t, &w);
    check_eq("L=0.逆解", 0, w.wheel_mm_s[0], 0);

    g = mecanum_default_geom();
    g.vy_polarity = 0;
    check_eq("极性0.非法", 0, mecanum_geom_is_valid(&g), 0);
    g.vy_polarity = 2;
    check_eq("极性2.非法", 0, mecanum_geom_is_valid(&g), 0);

    /* NULL 指针不得崩 */
    mecanum_zero_wheels(&w);
    mecanum_inverse(0, 0, &w);
    check_eq("NULL逆解", 0, w.wheel_mm_s[0], 0);
    t = mecanum_zero_twist();
    t.vx_mm_s = 77;
    mecanum_forward(0, 0, &t);
    check_eq("NULL正解", 0, t.vx_mm_s, 0);
    mecanum_forward(&g, 0, &t);
    check_eq("NULL轮速", 0, t.vx_mm_s, 0);
    mecanum_inverse(0, 0, 0);
    mecanum_inverse(&g, &t, 0);
    mecanum_forward(&g, &w, 0);
    mecanum_zero_wheels(0);
    mecanum_apply_mount(0, 0, 0);
    mecanum_undo_mount(0, 0, 0);
    mecanum_solve(0, 0, 0, 0, 0);
    check_eq("NULL 全部不崩", 0, 1, 1);
}

/* ================================================================== */
/* [8] 一键求解                                                        */
/* ================================================================== */

static void test_solve(void)
{
    mecanum_geom_t g = mecanum_default_geom();
    mecanum_mount_t m = mecanum_identity_mount();
    mecanum_limits_t lim = mecanum_default_limits();
    mecanum_twist_t t;
    mecanum_solve_result_t r;
    int i;

    printf("[8] 一键求解\n");

    /* 零指令 + 打开死区补偿 => 必须仍然停车 */
    lim.wheel_max_mm_s      = 1000;
    lim.wheel_deadzone_mm_s = 400;
    t = mecanum_zero_twist();
    mecanum_solve(&g, &lim, &m, &t, &r);
    check_u8("零指令.idle", 0, r.idle, 1);
    check_u8("零指令.saturated", 0, r.saturated, 0);
    check_u8("零指令.lifted", 0, r.lifted, 0);
    for (i = 0; i < 4; ++i) {
        check_eq("零指令.电机", i, r.motor_mm_s[i], 0);
    }

    /*
     * 大指令：饱和。
     * t = (2000, 1000, 4000)，L = 300 => rot = 1200
     * 四轮 = -200, +4200, +1800, +2200，峰值 4200 -> 缩到 1000
     *      = -48, +1000, +429, +524
     * 正解回去 = vx 476, vy 238, omega 953（= 原值 x 1000/4200，取整后）
     */
    t.vx_mm_s      = 2000;
    t.vy_mm_s      = 1000;
    t.omega_mrad_s = 4000;
    mecanum_solve(&g, &lim, &m, &t, &r);
    check_u8("大指令.saturated", 0, r.saturated, 1);
    check_u8("大指令.lifted", 0, r.lifted, 0);
    check_u8("大指令.idle", 0, r.idle, 0);
    check_eq("大指令.峰值卡上限", 0, imax4(r.motor_mm_s), 1000);
    check_eq("大指令.轮速0", 0, r.motor_mm_s[0], -48);
    check_eq("大指令.轮速1", 0, r.motor_mm_s[1], 1000);
    check_eq("大指令.轮速2", 0, r.motor_mm_s[2], 429);
    check_eq("大指令.轮速3", 0, r.motor_mm_s[3], 524);

    {
        mecanum_twist_t back;
        mecanum_wheels_t w;
        mecanum_undo_mount(&m, r.motor_mm_s, &w);
        mecanum_forward(&g, &w, &back);
        check_close("饱和后.vx 等比例缩小", 0, back.vx_mm_s, 476, 2);
        check_close("饱和后.vy 等比例缩小", 0, back.vy_mm_s, 238, 2);
        check_close("饱和后.omega 等比例缩小", 0, back.omega_mrad_s, 953, 4);
        /* 方向不变：vy/vx 比值与原指令一致 */
        check_close("饱和后.方向保持", 0, back.vy_mm_s * 2, back.vx_mm_s, 3);
    }

    /* 小指令：抬升到死区；峰值精确等于死区 */
    lim.wheel_max_mm_s      = 3000;
    lim.wheel_deadzone_mm_s = 500;
    t.vx_mm_s      = 60;
    t.vy_mm_s      = 0;
    t.omega_mrad_s = 0;
    mecanum_solve(&g, &lim, &m, &t, &r);
    check_u8("小指令.saturated", 0, r.saturated, 0);
    check_u8("小指令.lifted", 0, r.lifted, 1);
    for (i = 0; i < 4; ++i) {
        check_eq("小指令.电机抬到死区", i, r.motor_mm_s[i], 500);
    }

    /* 死区 <= 上限 时，抬升后的峰值绝不会突破上限 */
    {
        mecanum_limits_t l2;
        l2.wheel_max_mm_s      = 500;
        l2.wheel_deadzone_mm_s = 500;
        t.vx_mm_s = 60;
        t.vy_mm_s = 0;
        t.omega_mrad_s = 0;
        mecanum_solve(&g, &l2, &m, &t, &r);
        for (i = 0; i < 4; ++i) {
            check_eq("死区=上限.不越界", i, r.motor_mm_s[i], 500);
        }
    }

    /* lim 为 NULL：不做任何整形，纯运动学 + 装配 */
    t.vx_mm_s = 2000;
    t.vy_mm_s = 0;
    t.omega_mrad_s = 0;
    mecanum_solve(&g, 0, &m, &t, &r);
    check_u8("无限制.saturated", 0, r.saturated, 0);
    check_u8("无限制.lifted", 0, r.lifted, 0);
    for (i = 0; i < 4; ++i) {
        check_eq("无限制.电机", i, r.motor_mm_s[i], 2000);
    }

    /* 非法几何：必须停车而不是乱走 */
    {
        mecanum_geom_t bad = mecanum_default_geom();
        bad.half_wheelbase_mm = 0;
        bad.half_track_mm = 0;
        t.vx_mm_s = 1000;
        t.vy_mm_s = 1000;
        t.omega_mrad_s = 1000;
        mecanum_solve(&bad, &lim, &m, &t, &r);
        check_u8("非法几何.停车", 0, r.idle, 1);
        check_eq("非法几何.电机", 0, r.motor_mm_s[0], 0);
    }
}

/* ================================================================== */
/* [9] 与 ROS2 / WPILib 交叉验证                                       */
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
 * 参照 A：ROS2 ros2_controllers / mecanum_drive_controller
 *   S = sum_of_robot_center_projection_on_X_Y_axis = lx + ly
 *   内部轮序 FL, FR, RR, RL（后两路与我们的顺序相反）
 *   S 用 mm、轮速用 mm/s 时，第三分量得到的就是 rad/s（因子 r 与 1/r 抵消）
 */
static void ros2_inverse(double vx, double vy, double w_rad, double S, d4 *w)
{
    w->v[0] = vx - vy - S * w_rad; /* FL */
    w->v[1] = vx + vy + S * w_rad; /* FR */
    w->v[2] = vx + vy - S * w_rad; /* RL（注意：这里按我们的顺序放） */
    w->v[3] = vx - vy + S * w_rad; /* RR */
}

/* Odometry::update(FL, RL, RR, FR, dt) 的逐字搬运 */
static void ros2_forward(double FL, double RL, double RR, double FR, double S, d3 *c)
{
    c->v[0] = 0.25 * (FL + RL + RR + FR);
    c->v[1] = 0.25 * (-FL + RL - RR + FR);
    c->v[2] = 0.25 / S * (-FL - RL + RR + FR); /* rad/s */
}

/*
 * 参照 B：WPILib MecanumDriveKinematics
 *   setRow(0, 0, 1, -1, -(fl.x + fl.y));   FL = (+lx, +ly)
 *   setRow(1, 0, 1,  1,  (fr.x - fr.y));   FR = (+lx, -ly)
 *   setRow(2, 0, 1,  1,  (rl.x - rl.y));   RL = (-lx, +ly)
 *   setRow(3, 0, 1, -1, -(rr.x + rr.y));   RR = (-lx, -ly)
 *   forwardKinematics = inverseKinematics.pseudoInverse();
 * WPILib 的轮序与本模块一致：FL, FR, RL, RR。
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

/* 注意：C99 不允许 double(*)[3] 隐式转成 const double(*)[3]，
   所以这里第一个参数不加 const（-Wpedantic 会拦下来）。 */
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

/* 真的算一遍 (M^T M)^-1 M^T，不做任何解析化简 */
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

    printf("[9] 与 ROS2 / WPILib 交叉验证\n");

    if (wpilib_pinv(lx, ly, P) == 0) {
        printf("  [FAIL] WPILib 伪逆求解失败\n");
        ++g_fails;
        ++g_checks;
        return;
    }

    wpilib_matrix(lx, ly, M);
    printf("  [info] WPILib 矩阵第三列 (FL,FR,RL,RR) = %.6g, %.6g, %.6g, %.6g\n",
           M[0][2], M[1][2], M[2][2], M[3][2]);
    printf("  [info] 理论 -(lx+ly), +(lx+ly), -(lx+ly), +(lx+ly) = %.6g, %.6g, %.6g, %.6g\n",
           -K, K, -K, K);
    printf("  [info] 伪逆第 3 行 / K = %.10g, %.10g, %.10g, %.10g\n",
           P[2][0] / K, P[2][1] / K, P[2][2] / K, P[2][3] / K);

    for (pv = 0; pv < 2; ++pv) {
        for (po = 0; po < 2; ++po) {
            mecanum_geom_t g = mecanum_default_geom();
            g.vy_polarity    = (pv == 0) ? 1 : -1;
            g.omega_polarity = (po == 0) ? 1 : -1;

            for (i = 0; i < nx; ++i) {
                for (j = 0; j < ny; ++j) {
                    for (k = 0; k < nw; ++k) {
                        /* 命令系 -> 物理系 */
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

                        /* --- A1: 我的整数逆解 vs ROS2 闭式 --- */
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
                                printf("  [FAIL] A1 #%ld: 最大偏差 %.6g mm/s\n", cases,
                                       d);
                            }
                        }

                        /* --- A2: WPILib 矩阵 vs ROS2 闭式（两者都是 double） --- */
                        for (q = 0; q < 4; ++q) {
                            b.v[q] = M[q][0] * (double)VX[i] + M[q][1] * vy_phys
                                   + M[q][2] * w_rad;
                        }
                        for (q = 0; q < 4; ++q) {
                            check_double("A2.ROS2 vs WPILib 逆解", cases, a.v[q], b.v[q],
                                         1e-9);
                        }

                        /* --- B: WPILib 伪逆正解 vs ROS2 闭式正解 --- */
                        ros2_forward(a.v[0], a.v[2], a.v[3], a.v[1], K, &ca);
                        cb.v[0] = P[0][0] * a.v[0] + P[0][1] * a.v[1]
                                + P[0][2] * a.v[2] + P[0][3] * a.v[3];
                        cb.v[1] = P[1][0] * a.v[0] + P[1][1] * a.v[1]
                                + P[1][2] * a.v[2] + P[1][3] * a.v[3];
                        cb.v[2] = P[2][0] * a.v[0] + P[2][1] * a.v[1]
                                + P[2][2] * a.v[2] + P[2][3] * a.v[3];
                        /* 第三列是 ∓K，故伪逆解出的第三分量是 rad/s */
                        check_double("B1.伪逆 vs 闭式 vx", cases, cb.v[0], ca.v[0], 1e-6);
                        check_double("B1.伪逆 vs 闭式 vy", cases, cb.v[1], ca.v[1], 1e-6);
                        check_double("B1.伪逆 vs 闭式 omega", cases, cb.v[2] * 1000.0,
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

                        /* --- C: 我的整数正解 vs 外部参照 --- */
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
                            check_double("C.我的正解 vx", cases, (double)back.vx_mm_s,
                                         ca.v[0], 1.0);
                            check_double("C.我的正解 vy", cases, (double)back.vy_mm_s,
                                         (double)g.vy_polarity * ca.v[1], 1.0);
                            /* 角速度容差 8：四个轮各 0.5 的取整误差经 4L 放大 */
                            check_double("C.我的正解 omega", cases,
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

    printf("  [info] 交叉验证用例 %ld 组（4 种极性组合）\n", cases);
    printf("  [info] A1 我的逆解 vs ROS2          最大偏差 %.6g mm/s\n", worst_a);
    printf("  [info] B1 WPILib 伪逆 vs ROS2 闭式  最大偏差 %.6g\n", worst_b);
    printf("  [info] C  我的正解 vs 参照 线速度   最大偏差 %.6g mm/s\n", worst_cv);
    printf("  [info] C  我的正解 vs 参照 角速度   最大偏差 %.6g mrad/s\n", worst_cw);
    printf("  [info] C 的线速度误差只有浮点噪声量级，原因不是巧合：\n");
    printf("         对称四舍五入是奇函数（round(-t) = -round(t)），而 S_VY / S_OMG\n");
    printf("         在 (FL,RR) 与 (FR,RL) 上成对反号，一阶取整误差在正解求和时精确抵消。\n");
}

/* ================================================================== */
/* [10] 分辨率参考                                                     */
/* ================================================================== */

static void print_resolution(void)
{
    const double L = 300.0;
    const double counts_per_m = 10505.0; /* 11 PPR x 4 倍频 x 60 减速比 / (pi*80mm)，= 1:60 */
    const double fs_hz = 50.0;
    const double kin_mrad = 500.0 / L; /* rot 半个 LSB（0.5 mm/s）折算成 mrad/s */
    const double step_mm_s = fs_hz * 1000.0 / counts_per_m;
    const double omega_step = step_mm_s * 1000.0 / L;

    printf("[10] 分辨率参考（L = %.0f mm）\n", L);
    printf("  [info] 运动学：rot 取整最大 0.5 mm/s => 角速度误差 %.3f mrad/s\n",
           kin_mrad);
    printf("  [info] 编码器：%.0f counts/m，@%.0f Hz 单计数台阶 = %.4f mm/s\n",
           counts_per_m, fs_hz, step_mm_s);
    printf("  [info] 编码器：四轮各跳 1 个计数的最坏组合 => 角速度台阶 %.3f mrad/s\n",
           omega_step);
    printf("  [info] 比值：编码器量化比运动学取整粗 %.1f 倍\n", omega_step / kin_mrad);
    printf("  [info] 结论：运动学误差被编码器量化淹没，mm/s 整数域足够，无需 Q 格式定点。\n");
}

/* ================================================================== */

int main(void)
{
    printf("=== mecanum 四麦克纳姆轮运动学测试 ===\n");

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
