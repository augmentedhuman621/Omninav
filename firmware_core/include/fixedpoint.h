/*
 * fixedpoint.h — 定点单位换算（编码器计数 ↔ 毫米 / 毫米每秒）
 * ---------------------------------------------------------------------------
 *
 * 设计约束（与 firmware_core 铁律一致）
 *   - 只 include <stdint.h>。全整数零浮点、无动态内存、无全局可变状态。
 *
 * ★★ 本项目「编码器分辨率」这个数字的**唯一来源就在本文件**
 *    （契约 §6：严禁在第二个地方再写一份）。
 *    其他地方要这个数，一律调 fixp_default_scale()，不要自己写常量。
 *
 * 这套数字是怎么来的（详细算术在测试里会打印）
 *   电机 11 PPR 霍尔编码器，经 4 倍频（A/B 双相正交）得到
 *     11 × 4 = 44 counts / 电机转
 *   减速箱 1:60（实际采购档位，见 03_底盘物料清单.md）→
 *     44 × 60 = 2640 counts / 轮转
 *   轮径 80 mm → 周长 251.3274 mm
 *     2640 ÷ 0.2513274 = 10504.2 → **项目名义值取 10505**（+0.008%，
 *     远小于下面 7% 的齿数歧义，且这两个数字本来就要实车标定）
 *
 *   ⚠️ 歧义提示：卖家按空载转速把同一款齿轮箱标成「1:56」。
 *      若实车标定确认是 1:56，则
 *        44 × 56 = 2464 counts / 轮转 → 9806 counts / 米
 *      两者差 7%。所以 **实车必须实测标定**，本文件的常数只是名义值。
 *      改的时候只改这里的 FIXP_COUNTS_PER_M_NOMINAL 一行。
 *
 * 单位约定
 *   线速度 mm/s、角速度 mrad/s、距离 mm、控制周期 ms。
 */

#ifndef FIXEDPOINT_H
#define FIXEDPOINT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 名义值：1:60 档位（实际采购）。★ 全项目唯一的定义处。 */
#define FIXP_COUNTS_PER_M_NOMINAL 10505

/* 若实车标定确认齿轮箱是 1:56，把这个注释打开、把上一行改掉（二选一，别同时用）
 * #define FIXP_COUNTS_PER_M_NOMINAL 9806
 */

#define FIXP_WHEEL_DIAMETER_MM_NOMINAL 80
#define FIXP_CONTROL_PERIOD_MS_NOMINAL 20  /* 100 Hz 控制环里跑 50 Hz 速度反馈 */
#define FIXP_ENCODER_PPR_NOMINAL 11
#define FIXP_GEAR_RATIO_NOMINAL 60
#define FIXP_QUADRATURE_FACTOR 4

/* 换算参数。全部是数据，标定只改数字。 */
typedef struct {
    int32_t counts_per_m;    /* 编码器每米计数，必须 >= 1 */
    int32_t wheel_diameter_mm;
    int32_t period_ms;       /* 测速窗口，ms，必须 >= 1 */
} fixp_scale_t;

/* 默认参数：用上面那几个名义常数拼出来。 */
fixp_scale_t fixp_default_scale(void);

/* 参数合法性：三个字段都必须 >= 1。返回 1 = 合法。 */
int fixp_scale_is_valid(const fixp_scale_t *s);

/*
 * 由「每电机转计数（PPR×倍频）」和「减速比」推出 counts/m。
 *   counts_per_m = ppr_x_quadrature × gear_ratio × 1000 / (π × 轮径)
 * 这里不引入任何库函数：用 355/113 近似圆周率（误差 2.7e-7，
 * 折算到 251.327 mm 上不到 0.0001 mm）。
 * 返回 0 表示入参不合法（分母为 0 等），调用者应保留原值。
 */
int32_t fixp_counts_per_m_from_params(int32_t counts_per_motor_rev,
                                      int32_t gear_ratio,
                                      int32_t wheel_diameter_mm);

/*
 * mm/s -> counts / 测速窗口。用对称四舍五入，不是向零截断
 * （截断会让正反转误差不对称，长期累积成系统性偏差）。
 */
int32_t fixp_mm_s_to_counts(const fixp_scale_t *s, int32_t mm_s);

/* counts / 测速窗口 -> mm/s。 */
int32_t fixp_counts_to_mm_s(const fixp_scale_t *s, int32_t counts);

/*
 * ★ 降量化噪声：把 N 个窗口的计数**求和**再一次性换算成 mm/s。
 * 单窗口在 50 Hz 下 1 个计数 = 5099 µm/s 的台阶（这就是速度反馈的分辨率下限）；
 * 累加 N 个窗口后台阶降到 1/N，代价是延迟增加 N 个窗口。
 * 返回 0 表示 periods <= 0（调用者应改用单窗口版本）。
 */
int32_t fixp_counts_sum_to_mm_s(const fixp_scale_t *s, int32_t counts_sum,
                                int32_t periods);

/* 累计里程：计数 -> mm（返回 int64，长期运行不溢出）。 */
int64_t fixp_counts_to_mm(const fixp_scale_t *s, int64_t counts);

/* mm -> 计数（返回 int64，便于反向校验）。 */
int64_t fixp_mm_to_counts(const fixp_scale_t *s, int64_t mm);

/*
 * 轮线速度 -> 底盘角速度（rad 的千分之一，即 mrad/s）。
 *   omega_mrad_s = (v_right - v_left) 这类差速关系不适用于麦轮，
 *   本函数只做「轮速 / L」这个通用的一维换算，供标定与自检用。
 * L <= 0 时返回 0，不除零。
 */
int32_t fixp_wheel_mm_s_to_omega_mrad_s(int32_t wheel_mm_s, int32_t l_mm);

/* 一圈多少计数（= counts_per_m × 周长 mm ÷ 1000），供自检与打印用。 */
int64_t fixp_counts_per_wheel_turn(const fixp_scale_t *s);

#ifdef __cplusplus
}
#endif

#endif /* FIXEDPOINT_H */
