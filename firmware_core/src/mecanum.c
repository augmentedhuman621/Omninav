/*
 * mecanum.c — 四麦克纳姆轮运动学实现
 *
 * 纯整数、零依赖：本文件只 include "mecanum.h"（后者只 include <stdint.h>）。
 * 不 include HAL / ROS2 / 任何第三方头。无浮点，无动态内存，无全局状态。
 */

#include "mecanum.h"

/* ------------------------------------------------------------------ */
/* 符号表（顺序 FL, FR, RL, RR）                                        */
/* 来历见 mecanum.h 文件头：S_VY 的 ±1 就是 Adamov 模型里的 tan(delta)，  */
/* delta = ±45 度。这两张表已与 ROS2 / WPILib / minimec / akkexd 核对。 */
/* ------------------------------------------------------------------ */

static const int8_t k_s_vy[MECANUM_WHEEL_COUNT]  = { -1, +1, +1, -1 };
static const int8_t k_s_omg[MECANUM_WHEEL_COUNT] = { -1, +1, -1, +1 };

/* 取整用的常数 */
#define MECANUM_MRAD_PER_RAD 1000

/* ------------------------------------------------------------------ */
/* 内部工具                                                            */
/* ------------------------------------------------------------------ */

/*
 * 对称四舍五入（round half away from zero）。
 * 选它而不是 C 的整数除法（向零截断），是因为截断会引入符号相关的偏差，
 * 让正反转的误差不对称。要求 den > 0；用 int64 中间量，故 num 为
 * INT32_MIN 时也不会溢出（这是相对 32 位版本的一个实际改进）。
 */
static int32_t div_round(int64_t num, int64_t den)
{
    if (num >= 0) {
        return (int32_t)((num + den / 2) / den);
    }
    return (int32_t)(-((-num + den / 2) / den));
}

/* 按极性翻转。极性只允许 ±1。 */
static int32_t apply_polarity(int8_t pol, int32_t v)
{
    return (pol < 0) ? -v : v;
}

/* 四轮绝对值的峰值，永不为负。用 int64 以免 INT32_MIN 取负时溢出。 */
static int64_t wheels_peak_abs(const mecanum_wheels_t *w)
{
    int64_t peak = 0;
    int i;
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        int64_t a = (int64_t)w->wheel_mm_s[i];
        if (a < 0) {
            a = -a;
        }
        if (a > peak) {
            peak = a;
        }
    }
    return peak;
}

static void copy_wheels(const mecanum_wheels_t *src, mecanum_wheels_t *dst)
{
    int i;
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        dst->wheel_mm_s[i] = src->wheel_mm_s[i];
    }
}

/* 四轮同比例缩放到目标峰值。调用者保证 peak > 0 且 new_peak > 0。 */
static void scale_wheels_to_peak(const mecanum_wheels_t *in, int32_t new_peak,
                                 int64_t peak, mecanum_wheels_t *out)
{
    int i;
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        out->wheel_mm_s[i] = div_round((int64_t)in->wheel_mm_s[i] * (int64_t)new_peak,
                                       peak);
    }
}

/* ------------------------------------------------------------------ */
/* 默认值与清零                                                        */
/* ------------------------------------------------------------------ */

mecanum_geom_t mecanum_default_geom(void)
{
    mecanum_geom_t g;
    g.half_wheelbase_mm = 150; /* 占位，必须实测 */
    g.half_track_mm     = 150; /* 占位，必须实测 */
    g.vy_polarity       = 1;
    g.omega_polarity    = 1;
    return g;
}

mecanum_mount_t mecanum_identity_mount(void)
{
    mecanum_mount_t m = { { 0 }, { 0 } }; /* 先整体清零，杜绝“可能未初始化”告警 */
    int i;
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        m.motor_of_wheel[i] = (uint8_t)i;
        m.sign_of_wheel[i]  = 1;
    }
    return m;
}

mecanum_limits_t mecanum_default_limits(void)
{
    mecanum_limits_t l;
    l.wheel_max_mm_s      = 1000; /* 占位，必须按实测最高轮速替换 */
    l.wheel_deadzone_mm_s = 0;    /* 默认关闭死区补偿 */
    return l;
}

mecanum_twist_t mecanum_zero_twist(void)
{
    mecanum_twist_t t;
    t.vx_mm_s      = 0;
    t.vy_mm_s      = 0;
    t.omega_mrad_s = 0;
    return t;
}

void mecanum_zero_wheels(mecanum_wheels_t *out)
{
    int i;
    if (out == 0) {
        return;
    }
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        out->wheel_mm_s[i] = 0;
    }
}

/* ------------------------------------------------------------------ */
/* 参数校验                                                            */
/* ------------------------------------------------------------------ */

int mecanum_geom_is_valid(const mecanum_geom_t *g)
{
    if (g == 0) {
        return 0;
    }
    if (g->half_wheelbase_mm < 0 || g->half_track_mm < 0) {
        return 0;
    }
    /* L 必须 >= 1：正解要除以 4L，L = 0 会除零。 */
    if (g->half_wheelbase_mm + g->half_track_mm < 1) {
        return 0;
    }
    if (g->vy_polarity != 1 && g->vy_polarity != -1) {
        return 0;
    }
    if (g->omega_polarity != 1 && g->omega_polarity != -1) {
        return 0;
    }
    return 1;
}

int mecanum_mount_is_valid(const mecanum_mount_t *m)
{
    uint8_t seen[MECANUM_WHEEL_COUNT];
    int i;

    if (m == 0) {
        return 0;
    }
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        seen[i] = 0;
    }
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        uint8_t mi = m->motor_of_wheel[i];
        if (mi >= MECANUM_WHEEL_COUNT) {
            return 0;
        }
        if (seen[mi] != 0) {
            return 0; /* 有两路轮子映射到同一路电机 => 不是双射 */
        }
        seen[mi] = 1;
        if (m->sign_of_wheel[i] != 1 && m->sign_of_wheel[i] != -1) {
            return 0;
        }
    }
    return 1;
}

int32_t mecanum_l_mm(const mecanum_geom_t *g)
{
    if (g == 0) {
        return 0;
    }
    return g->half_wheelbase_mm + g->half_track_mm;
}

/* ------------------------------------------------------------------ */
/* 逆解                                                                */
/* ------------------------------------------------------------------ */

void mecanum_inverse(const mecanum_geom_t *g, const mecanum_twist_t *twist,
                     mecanum_wheels_t *out)
{
    int32_t l;
    int32_t vy;
    int32_t rot;
    int i;

    if (out == 0) {
        return;
    }
    if (g == 0 || twist == 0) {
        mecanum_zero_wheels(out);
        return;
    }

    l = g->half_wheelbase_mm + g->half_track_mm;
    if (l < 1) {
        /* 几何非法：宁可不走，也不要除零或给出无意义的轮速。 */
        mecanum_zero_wheels(out);
        return;
    }

    vy  = apply_polarity(g->vy_polarity, twist->vy_mm_s);
    rot = apply_polarity(g->omega_polarity,
                         div_round((int64_t)twist->omega_mrad_s * (int64_t)l,
                                   MECANUM_MRAD_PER_RAD));

    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        out->wheel_mm_s[i] = twist->vx_mm_s
                           + (int32_t)k_s_vy[i] * vy
                           + (int32_t)k_s_omg[i] * rot;
    }
}

/* ------------------------------------------------------------------ */
/* 正解（最小二乘伪逆）                                                 */
/* ------------------------------------------------------------------ */

void mecanum_forward(const mecanum_geom_t *g, const mecanum_wheels_t *wheels,
                     mecanum_twist_t *out)
{
    int32_t l;
    int64_t s_vx  = 0;
    int64_t s_vy  = 0;
    int64_t s_omg = 0;
    int i;

    if (out == 0) {
        return;
    }
    if (g == 0 || wheels == 0) {
        out->vx_mm_s      = 0;
        out->vy_mm_s      = 0;
        out->omega_mrad_s = 0;
        return;
    }

    l = g->half_wheelbase_mm + g->half_track_mm;
    if (l < 1) {
        out->vx_mm_s      = 0;
        out->vy_mm_s      = 0;
        out->omega_mrad_s = 0;
        return;
    }

    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        int64_t w = (int64_t)wheels->wheel_mm_s[i];
        s_vx  += w;
        s_vy  += (int64_t)k_s_vy[i] * w;
        s_omg += (int64_t)k_s_omg[i] * w;
    }

    out->vx_mm_s = div_round(s_vx, 4);
    out->vy_mm_s = apply_polarity(g->vy_polarity, div_round(s_vy, 4));
    out->omega_mrad_s = apply_polarity(
        g->omega_polarity,
        div_round(s_omg * MECANUM_MRAD_PER_RAD, (int64_t)4 * (int64_t)l));
}

/* ------------------------------------------------------------------ */
/* 饱和归一                                                            */
/* ------------------------------------------------------------------ */

int mecanum_desaturate(const mecanum_wheels_t *in, int32_t limit_abs_mm_s,
                       mecanum_wheels_t *out)
{
    int64_t peak;

    if (out == 0) {
        return 0;
    }
    if (in == 0) {
        mecanum_zero_wheels(out);
        return 0;
    }
    if (limit_abs_mm_s <= 0) {
        copy_wheels(in, out);
        return 0;
    }

    peak = wheels_peak_abs(in);
    if (peak <= (int64_t)limit_abs_mm_s) {
        copy_wheels(in, out);
        return 0;
    }

    scale_wheels_to_peak(in, limit_abs_mm_s, peak, out);
    return 1;
}

/* ------------------------------------------------------------------ */
/* 死区抬升                                                            */
/* ------------------------------------------------------------------ */

int mecanum_lift_deadzone(const mecanum_wheels_t *in, int32_t deadzone_abs_mm_s,
                          mecanum_wheels_t *out)
{
    int64_t peak;

    if (out == 0) {
        return 0;
    }
    if (in == 0) {
        mecanum_zero_wheels(out);
        return 0;
    }
    if (deadzone_abs_mm_s <= 0) {
        copy_wheels(in, out);
        return 0;
    }

    peak = wheels_peak_abs(in);

    /*
     * 安全硬约束：四轮全零就是「停车指令」，绝不能被抬起来。
     * 若这里返回放大结果，静止的车会自己开始走。
     */
    if (peak == 0) {
        copy_wheels(in, out);
        return 0;
    }

    if (peak >= (int64_t)deadzone_abs_mm_s) {
        copy_wheels(in, out);
        return 0;
    }

    scale_wheels_to_peak(in, deadzone_abs_mm_s, peak, out);
    return 1;
}

/* ------------------------------------------------------------------ */
/* 装配映射                                                            */
/* ------------------------------------------------------------------ */

void mecanum_apply_mount(const mecanum_mount_t *m, const mecanum_wheels_t *wheels,
                         int32_t motor_mm_s[MECANUM_WHEEL_COUNT])
{
    int i;

    if (motor_mm_s == 0) {
        return;
    }
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        motor_mm_s[i] = 0;
    }
    if (m == 0 || wheels == 0) {
        return;
    }

    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        uint8_t mi = m->motor_of_wheel[i];
        int32_t v;
        if (mi >= MECANUM_WHEEL_COUNT) {
            continue; /* 映射表非法：该轮丢弃，其余仍按表走 */
        }
        v = wheels->wheel_mm_s[i];
        motor_mm_s[mi] = (m->sign_of_wheel[i] < 0) ? -v : v;
    }
}

void mecanum_undo_mount(const mecanum_mount_t *m,
                        const int32_t motor_mm_s[MECANUM_WHEEL_COUNT],
                        mecanum_wheels_t *wheels)
{
    int i;

    if (wheels == 0) {
        return;
    }
    mecanum_zero_wheels(wheels);
    if (m == 0 || motor_mm_s == 0) {
        return;
    }

    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        uint8_t mi = m->motor_of_wheel[i];
        int32_t v;
        if (mi >= MECANUM_WHEEL_COUNT) {
            continue;
        }
        v = motor_mm_s[mi];
        wheels->wheel_mm_s[i] = (m->sign_of_wheel[i] < 0) ? -v : v;
    }
}

/* ------------------------------------------------------------------ */
/* 一键求解                                                            */
/* ------------------------------------------------------------------ */

void mecanum_solve(const mecanum_geom_t *g, const mecanum_limits_t *lim,
                   const mecanum_mount_t *mnt, const mecanum_twist_t *twist,
                   mecanum_solve_result_t *out)
{
    mecanum_wheels_t w;
    int i;

    if (out == 0) {
        return;
    }

    out->saturated = 0;
    out->lifted    = 0;
    out->idle      = 1;
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        out->wheels.wheel_mm_s[i] = 0;
        out->motor_mm_s[i]        = 0;
    }

    mecanum_inverse(g, twist, &w);

    if (lim != 0) {
        mecanum_wheels_t tmp;
        if (mecanum_desaturate(&w, lim->wheel_max_mm_s, &tmp) != 0) {
            out->saturated = 1;
        }
        w = tmp;
        if (mecanum_lift_deadzone(&w, lim->wheel_deadzone_mm_s, &tmp) != 0) {
            out->lifted = 1;
        }
        w = tmp;
    }

    out->wheels = w;
    mecanum_apply_mount(mnt, &w, out->motor_mm_s);

    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        if (out->motor_mm_s[i] != 0) {
            out->idle = 0;
            break;
        }
    }
}

/* ------------------------------------------------------------------ */
/* 与 ROS2 的轮序互换                                                  */
/* ------------------------------------------------------------------ */

void mecanum_wheels_from_ros2_order(const int32_t ros2_in[MECANUM_WHEEL_COUNT],
                                    mecanum_wheels_t *out)
{
    if (out == 0) {
        return;
    }
    if (ros2_in == 0) {
        mecanum_zero_wheels(out);
        return;
    }
    /* ROS2 顺序：0=FL, 1=FR, 2=RR, 3=RL */
    out->wheel_mm_s[MECANUM_FL] = ros2_in[0];
    out->wheel_mm_s[MECANUM_FR] = ros2_in[1];
    out->wheel_mm_s[MECANUM_RR] = ros2_in[2];
    out->wheel_mm_s[MECANUM_RL] = ros2_in[3];
}

void mecanum_wheels_to_ros2_order(const mecanum_wheels_t *in,
                                  int32_t ros2_out[MECANUM_WHEEL_COUNT])
{
    if (ros2_out == 0) {
        return;
    }
    if (in == 0) {
        ros2_out[0] = 0;
        ros2_out[1] = 0;
        ros2_out[2] = 0;
        ros2_out[3] = 0;
        return;
    }
    /* ROS2 顺序：0=FL, 1=FR, 2=RR, 3=RL */
    ros2_out[0] = in->wheel_mm_s[MECANUM_FL];
    ros2_out[1] = in->wheel_mm_s[MECANUM_FR];
    ros2_out[2] = in->wheel_mm_s[MECANUM_RR];
    ros2_out[3] = in->wheel_mm_s[MECANUM_RL];
}
