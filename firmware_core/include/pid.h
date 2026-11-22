/*
 * pid.h — 速度环 PID（纯整数，Q16 增益）
 * ---------------------------------------------------------------------------
 *
 * 设计约束（与本项目 firmware_core 的铁律一致）
 *   - 只 include <stdint.h>。不许 HAL / ROS2 / 第三方 / <stdio.h> / <math.h>。
 *   - 全整数零浮点。增益用 Q16 表示（寄存器里的整数 = 真实值 × 65536）。
 *   - 无动态内存、无全局可变状态、无递归。
 *   - 状态全部放在调用者给的 pid_state_t 里 —— 四路轮子就是四个实例，
 *     不需要也不允许模块级全局量。
 *
 * 单位约定（跨模块统一，见 STM32_README.md §5）
 *   设定值 / 反馈值 : mm/s（int32_t）
 *   输出           : 「PWM 单位」，对称范围通常 -1000 ~ +1000（int32_t）
 *   误差 e = setpoint - meas，单位 mm/s
 *
 * 为什么增益用 Q16 而不是整数比值
 *   速度环的 kp 典型量级是 0.1 ~ 2.0。用整数比值表达不了 0.35 这种值；
 *   用 Q16 后 0.35 就是 22938，全部运算落在 int64 中间量上，既精确又无溢出。
 *
 * 三项各自的处理（每一处都是刻意的，不是默认写法）
 *   P : p_term   = (kp_q16 * e) >> 16
 *   I : 累加器存「误差的原始和」Σe（单位 mm/s·次），而不是已经乘过 ki 的
 *       结果 —— 这样改动 ki 不会污染历史累积，且小 ki 不会因取整被吃掉。
 *       积分项 i_term = (ki_q16 * integ_raw) >> 16，并单独限幅。
 *   D : ★ 微分作用在「测量值」上，不在误差上（derivative-on-measurement）：
 *       d_term = (kd_q16 * (-(meas - prev_meas))) >> 16
 *       这样设定值突变时不会产生微分尖峰（derivative kick）。
 *       代价是设定值斜坡跟踪时少了一点超前，速度环里这个取舍是标准的。
 *
 * 抗积分饱和（三重，缺一不可）
 *   1) 积分项单独限幅 integ_limit —— 防止积分项本身把输出顶死。
 *   2) 积分分离：|e| > integ_sep_mm_s 时暂停积分 —— 大偏差阶段（起步、
 *      阶跃）本来就不该积分；这同时是最省事的抗超调手段。
 *   3) 条件积分（clamping）：本步算出的输出若已饱和、且误差方向还在把
 *      输出往饱和深处推，则**回滚**这一步的积分累加。
 *      —— 没有第 3 条时，指令长期超出物理可达范围会让积分涨到天上，
 *         等指令反向时车要先「等积分退出来」才动，表现为长时间不响应。
 *
 * 饱和标志
 *   pid_step() 通过 flags 回传本步发生的事，供上层记日志 / 判 PWM 使能：
 *     PID_FLAG_SATURATED   输出**顶在限幅边界上**（out >= out_max 或
 *                          out <= out_min）。语义取「已到轨」而不是
 *                          「本步被削掉」——因为对上层而言，恰好落在边界
 *                          （out_raw == out_max）与真正越界一样，都意味着
 *                          「误差再大也推不动了」，这正是判 PWM 满占空比
 *                          与上报 `A,<text>` 时想要的信息。
 *                          注意：抗积分饱和（下面第 3 条）用的是更严格的
 *                          「out_raw 真的越界」判据，两者不是一回事。
 *     PID_FLAG_INTEG_FROZEN 本步积分被回滚（条件积分触发）
 *     PID_FLAG_INTEG_SEP   本步因积分分离而暂停积分
 *   flags 允许为 NULL。
 *
 * 溢出与取值范围（内部一律 int64 中间量）
 *   |e| <= 2^20（约 100 万 mm/s，远超实际）
 *   |integ_raw| 被限幅在 integ_bound 内，见 pid.c
 *   |out| <= max(|out_min|, |out_max|)
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Q16 的 1.0 */
#define PID_Q16_ONE 65536

/* pid_step() 回传的标志位 */
#define PID_FLAG_SATURATED    0x01u
#define PID_FLAG_INTEG_FROZEN 0x02u
#define PID_FLAG_INTEG_SEP    0x04u

/* 增益与限幅配置。全部是「数据」，标定只改数字，不动算法。 */
typedef struct {
    int32_t kp_q16;          /* 比例增益，Q16 */
    int32_t ki_q16;          /* 积分增益，Q16；0 = 关闭积分 */
    int32_t kd_q16;          /* 微分增益，Q16；0 = 关闭微分 */
    int32_t out_min;         /* 输出下限（通常 -1000）。必须 < out_max */
    int32_t out_max;         /* 输出上限（通常 +1000） */
    int32_t integ_sep_mm_s;  /* 积分分离阈值：|e| 大于它则暂停积分；<=0 表示不分离 */
    int32_t integ_limit;     /* 积分项输出的绝对值上限；<=0 表示用 out_max/-out_min 推导 */
} pid_gains_t;

/* 一个控制通道的运行状态。四路轮子 = 四个独立实例。 */
typedef struct {
    int64_t integ_raw;  /* Σe，单位 mm/s·次 */
    int32_t prev_meas;  /* 上一步的反馈值，用于微分项 */
    uint8_t started;    /* 0 = 还没跑过第一步（微分项按 0 处理） */
} pid_state_t;

/* 默认增益：kp = 0.5, ki = 1/50 = 0.02, kd = 0，输出 ±1000。
 * ！！这些只是让固件能跑起来的起点，实车必须整定。 */
pid_gains_t pid_default_gains(void);

/* 清零状态。允许 st 为 0（空操作）。 */
void pid_reset(pid_state_t *st);

/*
 * 参数合法性：out_min < out_max、三个增益非负、integ_sep_mm_s 与
 * integ_limit 允许为 0 / 负数（表示关闭）。
 * 返回 1 = 合法。
 */
int pid_gains_is_valid(const pid_gains_t *g);

/*
 * 跑一步。返回限幅后的输出。
 *   g / st 为 0 时返回 0（不崩、不写任何东西）。
 *   flags 允许为 0。
 *   setpoint / meas 单位 mm/s。
 *
 * 本函数是纯状态机：同样输入序列 → 同样输出序列，与调用时刻无关。
 */
int32_t pid_step(const pid_gains_t *g, pid_state_t *st,
                 int32_t setpoint, int32_t meas, uint32_t *flags);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
