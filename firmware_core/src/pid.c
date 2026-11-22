/*
 * pid.c — 速度环 PID 实现
 *
 * 纯整数、零依赖：本文件只 include "pid.h"（后者只 include <stdint.h>）。
 * 无动态内存、无全局可变状态、无递归、无浮点。
 *
 * 三项的算式与抗积分饱和的三重手段见 pid.h 文件头 —— 那里解释了
 * 「为什么是这样做」，这里只写实现。
 */

#include "pid.h"

/* ------------------------------------------------------------------ */
/* 内部工具                                                            */
/* ------------------------------------------------------------------ */

static int64_t iabs64(int64_t v)
{
    return (v < 0) ? -v : v;
}

static int64_t clamp_i64(int64_t v, int64_t lo, int64_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

/* ------------------------------------------------------------------ */
/* 默认值与状态                                                        */
/* ------------------------------------------------------------------ */

pid_gains_t pid_default_gains(void)
{
    pid_gains_t g;
    g.kp_q16         = PID_Q16_ONE / 2;  /* 0.5  */
    g.ki_q16         = PID_Q16_ONE / 50; /* 0.02 */
    g.kd_q16         = 0;                /* 默认关微分：编码器量化噪声会被放大 */
    g.out_min        = -1000;
    g.out_max        = 1000;
    g.integ_sep_mm_s = 0;                /* 默认不分离 */
    g.integ_limit    = 1000;             /* 积分项单独限幅，与输出同量级 */
    return g;
}

void pid_reset(pid_state_t *st)
{
    if (st == 0) {
        return;
    }
    st->integ_raw = 0;
    st->prev_meas = 0;
    st->started   = (uint8_t)0;
}

int pid_gains_is_valid(const pid_gains_t *g)
{
    if (g == 0) {
        return 0;
    }
    if (g->out_min >= g->out_max) {
        return 0;
    }
    if (g->kp_q16 < 0 || g->ki_q16 < 0 || g->kd_q16 < 0) {
        return 0;
    }
    return 1;
}

/* ------------------------------------------------------------------ */
/* 主步进                                                              */
/* ------------------------------------------------------------------ */

int32_t pid_step(const pid_gains_t *g, pid_state_t *st,
                 int32_t setpoint, int32_t meas, uint32_t *flags)
{
    int64_t e;
    int64_t p_term;
    int64_t d_term;
    int64_t i_term;
    int64_t out_raw;
    int64_t out;
    int64_t integ_bound = 0;
    int64_t integ_cand;
    int64_t out_lo;
    int64_t out_hi;
    int64_t i_limit_abs;
    uint8_t sep     = (uint8_t)0;
    uint8_t frozen  = (uint8_t)0;
    uint8_t clipped = (uint8_t)0; /* out_raw 越界被削掉（抗饱和用） */
    uint8_t sat     = (uint8_t)0; /* 输出顶在限幅边界（标志位用） */

    if (flags != 0) {
        *flags = 0u;
    }
    if (g == 0 || st == 0) {
        return 0;
    }

    out_lo = (int64_t)g->out_min;
    out_hi = (int64_t)g->out_max;
    e      = (int64_t)setpoint - (int64_t)meas;

    /* --- P --- */
    p_term = (int64_t)g->kp_q16 * e / (int64_t)PID_Q16_ONE;

    /* --- D：作用在测量值上，避免设定值突变时的微分尖峰 --- */
    if (st->started != (uint8_t)0) {
        int64_t dmeas = (int64_t)meas - (int64_t)st->prev_meas;
        d_term = (int64_t)g->kd_q16 * (-dmeas) / (int64_t)PID_Q16_ONE;
    } else {
        d_term = 0;
    }

    /* --- 积分分离判定 --- */
    if (g->integ_sep_mm_s > 0 && iabs64(e) > (int64_t)g->integ_sep_mm_s) {
        sep = (uint8_t)1;
    }

    /* --- 积分累加器的上限：由 integ_limit 反推 --- */
    if (g->ki_q16 > 0) {
        if (g->integ_limit > 0) {
            i_limit_abs = (int64_t)g->integ_limit;
        } else if (out_hi > -out_lo) {
            i_limit_abs = out_hi;
        } else {
            i_limit_abs = -out_lo;
        }
        integ_bound = i_limit_abs * (int64_t)PID_Q16_ONE / (int64_t)g->ki_q16;
    }

    /* --- 拟提交的积分累加器 --- */
    integ_cand = st->integ_raw;
    if (g->ki_q16 == 0) {
        /* 积分关闭：累加器保持不动，避免「关掉 ki 后残留值又突然生效」 */
        sep = (uint8_t)0;
    } else if (sep == (uint8_t)0) {
        integ_cand = clamp_i64(st->integ_raw + e, -integ_bound, integ_bound);
    } else {
        /* 分离期间不新增，但仍要把历史值夹回合法范围 */
        integ_cand = clamp_i64(st->integ_raw, -integ_bound, integ_bound);
    }

    i_term  = (int64_t)g->ki_q16 * integ_cand / (int64_t)PID_Q16_ONE;
    out_raw = p_term + i_term + d_term;
    out     = clamp_i64(out_raw, out_lo, out_hi);
    clipped = (out != out_raw) ? (uint8_t)1 : (uint8_t)0;

    /* --- 条件积分（clamping 抗饱和）：**真的被削掉**、且误差仍在往深处推就回滚。
           判据必须是 out_raw 越界，不能是「顶在边界上」——否则 out_raw 恰好
           等于 out_max 的正常工作点会被误判成 windup。 --- */
    if (sep == (uint8_t)0 && clipped != (uint8_t)0) {
        uint8_t push_deeper = (uint8_t)0;
        if (e > 0 && out_raw > out_hi) {
            push_deeper = (uint8_t)1;
        } else if (e < 0 && out_raw < out_lo) {
            push_deeper = (uint8_t)1;
        }
        if (push_deeper != (uint8_t)0) {
            frozen     = (uint8_t)1;
            integ_cand = st->integ_raw;
            i_term     = (int64_t)g->ki_q16 * integ_cand / (int64_t)PID_Q16_ONE;
            out_raw    = p_term + i_term + d_term;
            out        = clamp_i64(out_raw, out_lo, out_hi);
        }
    }

    /* --- 饱和标志：「输出顶在限幅边界上」。恰好落在边界（out_raw == out_max）
           也算 —— 此时误差再大也推不动，对上层诊断（PWM 满占空比）同样有意义。
           ★ 这里和上面的 `clipped` 是两个不同的判据，别合并。 --- */
    if (out >= out_hi || out <= out_lo) {
        sat = (uint8_t)1;
    }

    st->integ_raw = integ_cand;
    st->prev_meas = meas;
    st->started   = (uint8_t)1;

    if (flags != 0) {
        uint32_t f = 0u;
        if (sat != (uint8_t)0) {
            f |= PID_FLAG_SATURATED;
        }
        if (frozen != (uint8_t)0) {
            f |= PID_FLAG_INTEG_FROZEN;
        }
        if (sep != (uint8_t)0) {
            f |= PID_FLAG_INTEG_SEP;
        }
        *flags = f;
    }

    return (int32_t)out;
}
