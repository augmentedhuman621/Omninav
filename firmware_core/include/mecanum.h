/*
 * mecanum.h — 四麦克纳姆轮全向底盘 运动学（正解 / 逆解 / 限幅 / 死区 / 装配映射）
 * ---------------------------------------------------------------------------
 *
 * 设计约束（与本项目 firmware_core 的铁律一致）
 *   - C99 标准库以外零依赖：本文件只 include <stdint.h>。
 *     不许 include HAL 头、ROS2 头、任何第三方库。
 *   - 全整数，不用浮点。线速度单位 mm/s，角速度单位 mrad/s（毫弧度/秒）。
 *   - 无动态内存、无全局可变状态、无递归、无浮点、无 64 位以外的宽类型要求。
 *   - 可在 STM32F103（Cortex-M3，无 FPU）与树莓派 5 上得到完全相同的结果。
 *     所有除法都是整数除法且只有两种：除以已知正整数、以及对称四舍五入。
 *     ——确定性是安全相关代码的必要条件。
 *
 * 坐标系（ROS REP-103）
 *   x 朝前，y 朝左，z 朝上；omega > 0 为从上往下看的逆时针。
 *   原点取底盘几何中心（四条轮轴所成矩形的中心）。
 *
 * 轮位索引（本模块统一顺序，FL/FR/RL/RR）
 *   MECANUM_FL = 0   左前      MECANUM_FR = 1   右前
 *   MECANUM_RL = 2   左后      MECANUM_RR = 3   右后
 *   注意：ROS2 ros2_controllers 的 mecanum_drive_controller 内部枚举是
 *   FL, FR, RR, RL（后轮两路顺序相反）。二者互换用
 *   mecanum_wheels_from_ros2_order() / mecanum_wheels_to_ros2_order()。
 *   WPILib MecanumDriveKinematics 的顺序与本模块一致（FL,FR,RL,RR）。
 *
 * 物理模型与公式来源（不是拍脑袋定的符号）
 *   Adamov, "Geometry and Kinematics of the Mecanum Wheel on a Plane and a
 *   Sphere", Russian J. Nonlinear Dynamics 20(1):43-78, 2024,
 *   DOI 10.20537/nd240201 —— 在忽略辊子接触点轴向位移（phi=0）时，麦轮退化为
 *   最简滑动圆盘模型：
 *
 *        R * phidot = V_Cx + V_Cy * tan(delta)
 *
 *   delta 是接触辊子轴线相对轮中面的偏角。成品麦轮 delta = ±45 度，
 *   于是 tan(delta) = ±1 —— 这正是本模块 S_VY 里那些 ±1 的来历，
 *   并且符号完全由物理量 delta 的符号决定。所以：
 *
 *        vy_polarity  ==  +1  <=>  delta = +45 度
 *        vy_polarity  ==  -1  <=>  delta = -45 度   （四轮辊子朝向整体相反）
 *
 *   它不是一个为了凑对而加的补丁，它有对应的物理量。
 *
 * 符号表（顺序 FL, FR, RL, RR）
 *   S_VY  = { -1, +1, +1, -1 }   平移 y 分量在各轮上的系数
 *   S_OMG = { -1, +1, -1, +1 }   自转分量在各轮上的系数
 *   这两张表已与四个独立实现逐项核对一致：
 *     - ROS2 ros2_controllers / mecanum_drive_controller
 *     - WPILib MecanumDriveKinematics
 *     - minimec-ros2 / minimeclib MecanumDrive
 *     - akkexd/ros2-mecanum-robot driver_node.py
 *   另有中文推导与 WPILib 的实车网格文件（FL 与 RR 同一种轮、
 *   FR 与 RL 同一种轮）在物理装配层面印证 S_VY 的配对关系。
 *
 * 公式（L = half_wheelbase_mm + half_track_mm）
 *   逆解：
 *       vy  = vy_polarity    * vy_mm_s
 *       rot = omega_polarity * round( omega_mrad_s * L / 1000 )     [mm/s]
 *       wheel[i] = vx_mm_s + S_VY[i] * vy + S_OMG[i] * rot
 *
 *   正解（等价于设计矩阵的 Moore-Penrose 伪逆；该系统超定，取最小二乘解）：
 *       vx_mm_s      = round( ( w0 + w1 + w2 + w3 ) / 4 )
 *       vy_mm_s      = vy_polarity    * round( ( -w0 + w1 + w2 - w3 ) / 4 )
 *       omega_mrad_s = omega_polarity * round( ( -w0 + w1 - w2 + w3 ) * 1000 / (4L) )
 *
 *   正解输出的 twist 与逆解输入的 twist 处在同一个「命令系」：
 *   同一组极性配置下 forward(inverse(t)) == t（在整数取整容差内）。
 *
 * 为什么不需要 Q 格式定点（已实测）
 *   运动学取整引入的角速度误差约 1 mrad/s；而编码器量化在 50 Hz 采样下
 *   对应约 17 mrad/s。误差源比算法误差粗约 20 倍，运动学误差被完全淹没，
 *   mm/s 整数域足够。引入 Q 格式只是增加复杂度，不改善任何可测量指标。
 *
 * 溢出与安全取值范围
 *   内部中间量一律用 int64_t，因此下面的范围有充足余量：
 *       |vx_mm_s| <= 100000, |vy_mm_s| <= 100000
 *       |omega_mrad_s| <= 20000
 *       0 <= half_wheelbase_mm, half_track_mm, 且 L <= 1000
 *   超出这个范围不保证结果有意义（但不会产生未定义行为）。
 *   注意 half_wheelbase_mm + half_track_mm 必须 >= 1，否则正解的除法无意义；
 *   本模块在 L < 1 时一律输出全零而不会除零。请用 mecanum_geom_is_valid() 检查。
 */

#ifndef MECANUM_H
#define MECANUM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* 轮位索引                                                            */
/* ------------------------------------------------------------------ */

#define MECANUM_FL 0
#define MECANUM_FR 1
#define MECANUM_RL 2
#define MECANUM_RR 3
#define MECANUM_WHEEL_COUNT 4

/* ------------------------------------------------------------------ */
/* 数据类型                                                            */
/* ------------------------------------------------------------------ */

/* 底盘几何与极性。全部是「数据」，不是代码——标定只改数字，不动算法。 */
typedef struct {
    int32_t half_wheelbase_mm; /* 几何中心到前/后轮轴的距离，>= 0 */
    int32_t half_track_mm;     /* 几何中心到左/右轮中面的距离，>= 0 */
    int8_t  vy_polarity;       /* 只许 +1 或 -1，物理含义见文件头（辊子偏角 delta 的符号） */
    int8_t  omega_polarity;    /* 只许 +1 或 -1 */
} mecanum_geom_t;

/* 底盘速度指令 / 正解输出。*/
typedef struct {
    int32_t vx_mm_s;      /* 前进为正 */
    int32_t vy_mm_s;      /* 左移为正 */
    int32_t omega_mrad_s; /* 逆时针为正 */
} mecanum_twist_t;

/* 四轮线速度（轮缘线速度，mm/s），顺序 FL,FR,RL,RR。 */
typedef struct {
    int32_t wheel_mm_s[MECANUM_WHEEL_COUNT];
} mecanum_wheels_t;

/*
 * 装配映射：把「轮」翻译成「电机」。
 *   motor_of_wheel[i] = 第 i 个轮位由哪一路电机驱动（必须是 0..3 的双射）
 *   sign_of_wheel[i]  = +1/-1，给该路电机正 PWM 时轮子是否朝正方向转
 * 因为是双射且符号只有 ±1，apply_mount 与 undo_mount 互为逆运算。
 */
typedef struct {
    uint8_t motor_of_wheel[MECANUM_WHEEL_COUNT];
    int8_t  sign_of_wheel[MECANUM_WHEEL_COUNT];
} mecanum_mount_t;

/*
 * 输出限幅与死区（单位都是「轮线速度 mm/s」，不是 PWM）。
 *   wheel_max_mm_s      : 轮线速度上限，<= 0 表示不限制（一般不这么用）
 *   wheel_deadzone_mm_s : 摩擦死区的等效轮线速度，<= 0 表示不做补偿
 * 必须满足 wheel_deadzone_mm_s <= wheel_max_mm_s（否则两者互相打架）。
 *
 * 为什么需要 deadzone：实测（Aung Khant Ko, Zenodo 10.5281/zenodo.20724812）
 * 环形绒毛地毯上的电机死区是 PWM 30-35，硬地板是 15-20，地毯上宽 75%，
 * 而且标准导航栈察觉不到。该文给出的修法与本模块一致：四轮同比例缩放，
 * 保持速度方向不变。
 */
typedef struct {
    int32_t wheel_max_mm_s;
    int32_t wheel_deadzone_mm_s;
} mecanum_limits_t;

/* mecanum_solve() 的完整结果。 */
typedef struct {
    mecanum_wheels_t wheels;                     /* 限幅/死区处理后的四轮线速度 */
    int32_t          motor_mm_s[MECANUM_WHEEL_COUNT]; /* 已按装配映射换算到电机序 */
    uint8_t          saturated;                  /* 1 = 触发过饱和归一（被按比例缩小） */
    uint8_t          lifted;                     /* 1 = 触发过死区抬升（被按比例放大） */
    uint8_t          idle;                       /* 1 = 四路电机指令全为 0 */
} mecanum_solve_result_t;

/* ------------------------------------------------------------------ */
/* 默认值与清零                                                        */
/* ------------------------------------------------------------------ */

/*
 * 默认几何：half_wheelbase_mm = 150, half_track_mm = 150。
 * ！！ 150/150 是占位值。底盘装好后必须用卷尺实测替换。
 *    这两个数错了，原地自转会绕着偏离几何中心的点画圆。
 */
mecanum_geom_t mecanum_default_geom(void);

/* 恒等装配：wheel i -> motor i，符号全 +1。同样是占位值。 */
mecanum_mount_t mecanum_identity_mount(void);

/*
 * 默认限幅：wheel_max_mm_s = 1000（占位值，必须按实测最高轮速替换），
 *          wheel_deadzone_mm_s = 0（默认关闭死区补偿——它会改变行为，
 *          必须在实测出自己车上的死区之后才打开）。
 */
mecanum_limits_t mecanum_default_limits(void);

mecanum_twist_t mecanum_zero_twist(void);
void            mecanum_zero_wheels(mecanum_wheels_t *out);

/* ------------------------------------------------------------------ */
/* 参数校验                                                            */
/* ------------------------------------------------------------------ */

/* 返回 1 = 合法。检查非负、L >= 1、两个极性只能是 ±1。 */
int mecanum_geom_is_valid(const mecanum_geom_t *g);

/* 返回 1 = 合法。检查 motor_of_wheel 是 0..3 的双射、符号只能是 ±1。 */
int mecanum_mount_is_valid(const mecanum_mount_t *m);

/* L = half_wheelbase_mm + half_track_mm。 */
int32_t mecanum_l_mm(const mecanum_geom_t *g);

/* ------------------------------------------------------------------ */
/* 运动学                                                              */
/* ------------------------------------------------------------------ */

/*
 * 逆解：底盘速度 -> 四轮线速度。
 * g 或 twist 为 NULL、或 L < 1 时输出全零（不除零、不产生未定义行为）。
 */
void mecanum_inverse(const mecanum_geom_t *g, const mecanum_twist_t *twist,
                     mecanum_wheels_t *out);

/*
 * 正解：四轮线速度 -> 底盘速度（最小二乘伪逆）。
 * 与 mecanum_inverse 共用同一组极性，因此同一配置下 forward(inverse(t)) == t。
 */
void mecanum_forward(const mecanum_geom_t *g, const mecanum_wheels_t *wheels,
                     mecanum_twist_t *out);

/* ------------------------------------------------------------------ */
/* 输出整形：两个方向相反、原理相同的比例缩放                          */
/* ------------------------------------------------------------------ */

/*
 * 饱和归一（限上限）：四轮超出 limit_abs_mm_s 时整体同比例缩小。
 * 返回 1 = 发生了缩放，0 = 原样复制。
 *
 * 为什么不能逐轮各自截断：逐轮截断会破坏四轮之间的比例关系，
 * 于是运动方向被改变——直行会变成斜行。真实工程里能抓到这个反例：
 * akkexd/ros2-mecanum-robot 的 driver_node.py 就是逐轮
 * max(-MAX, min(MAX, w)) 截断的。
 * limit_abs_mm_s <= 0 视为不限制，原样复制并返回 0。
 */
int mecanum_desaturate(const mecanum_wheels_t *in, int32_t limit_abs_mm_s,
                       mecanum_wheels_t *out);

/*
 * 死区抬升（保下限）：四轮峰值低于 deadzone_abs_mm_s 时整体同比例放大到该值。
 * 返回 1 = 发生了放大，0 = 原样复制。
 *
 * 安全语义（重要）：
 *   - 输入四轮全为 0 时，输出必须全为 0，返回 0。也就是说静止的车绝不会
 *     被死区补偿「抬」起来自己走。这条是硬约束，不是优化。
 *   - 该操作会让「有指令但很小」的情况跑得比指令快。这是刻意的取舍：
 *     宁可动起来但略快，不要因为摩擦静死区而纹丝不动，同时方向保持正确。
 *   - 因此默认关闭（mecanum_default_limits 里为 0），必须先实测自己车上的
 *     死区再打开。
 * deadzone_abs_mm_s <= 0 视为不补偿，原样复制并返回 0。
 */
int mecanum_lift_deadzone(const mecanum_wheels_t *in, int32_t deadzone_abs_mm_s,
                          mecanum_wheels_t *out);

/* ------------------------------------------------------------------ */
/* 装配映射                                                            */
/* ------------------------------------------------------------------ */

/* wheel -> motor：motor_of_wheel 负责换线，sign_of_wheel 负责换向。 */
void mecanum_apply_mount(const mecanum_mount_t *m, const mecanum_wheels_t *wheels,
                         int32_t motor_mm_s[MECANUM_WHEEL_COUNT]);

/* motor -> wheel：apply_mount 的逆运算。 */
void mecanum_undo_mount(const mecanum_mount_t *m,
                        const int32_t motor_mm_s[MECANUM_WHEEL_COUNT],
                        mecanum_wheels_t *wheels);

/* ------------------------------------------------------------------ */
/* 一键求解（固件实际调用的入口）                                       */
/* ------------------------------------------------------------------ */

/*
 * 逆解 -> 饱和归一 -> 死区抬升 -> 装配映射，一趟做完。
 * 顺序不可颠倒：先压低再抬高。因为约束 deadzone <= max 保证了
 * 「抬到 deadzone」永远不会突破 max。
 * g / lim / mnt 允许为 NULL（lim 为 NULL 表示既不限幅也不补死区）。
 */
void mecanum_solve(const mecanum_geom_t *g, const mecanum_limits_t *lim,
                   const mecanum_mount_t *mnt, const mecanum_twist_t *twist,
                   mecanum_solve_result_t *out);

/* ------------------------------------------------------------------ */
/* 与 ROS2 的轮序互换                                                  */
/* ------------------------------------------------------------------ */

/*
 * ROS2 ros2_controllers 的轮序是 [FL, FR, RR, RL]，本模块是 [FL, FR, RL, RR]。
 * 接 ros2_control 的命令接口、或对照它的日志时，RL 与 RR 必须互换。
 * 忘记换这一次的后果是：车能走、能自转，但横移和斜行方向是错的。
 */
void mecanum_wheels_from_ros2_order(const int32_t ros2_in[MECANUM_WHEEL_COUNT],
                                    mecanum_wheels_t *out);
void mecanum_wheels_to_ros2_order(const mecanum_wheels_t *in,
                                  int32_t ros2_out[MECANUM_WHEEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* MECANUM_H */
