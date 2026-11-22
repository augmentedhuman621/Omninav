/**
  ******************************************************************************
  * @file    Src/core_selftest.c
  * @brief   OmniNav 算法层（firmware_core）上板自检
  *
  *          为什么要这个文件：
  *            firmware_core 是纯算法层，放在 .uvprojx 的文件组里，但 Src/main.c
  *            原本一个核心函数都不调用，armlink 会把 5 个目标文件整个丢弃
  *            （Program Size 加不加 firmware_core 完全一样）。这里主动调用每个
  *            模块的公开 API，才有真正的链接与运行期验证。
  *
  *          断言口径：
  *            期望值全部抄自 firmware_core/test/test_*.c —— 那套用例在宿主机上
  *            25344 条断言全绿，所以这里只做"同一输入 → 同一输出"的搬运，
  *            不自己发明期望值。每条断言都能真分辨对错（不是恒真式）。
  *
  *          输出：
  *            每组一行 `   <组名>: N checks, M failed -> PASS/FAIL`
  *            失败时逐条打印 `   FAIL <名称>: got X want Y`
  *            末行汇总 `core_selftest: N checks, M failed`
  *
  *          约束：禁止 %f（uart_printf 不支持），格式串里禁止中文。
  ******************************************************************************
  */

#include "core_selftest.h"

#include "mecanum.h"
#include "pid.h"
#include "odom.h"
#include "fixedpoint.h"
#include "protocol.h"

#include <stdint.h>

/* main.c 提供的串口输出层（只用 32 位，不支持 %f / 64 位） */
void uart_printf(const char *fmt, ...);

/* Private variables ---------------------------------------------------------*/
static int         s_checks;        /* 累计断言数 */
static int         s_fails;         /* 累计失败数 */
static const char *s_group;         /* 当前组名 */
static int         s_group_checks;  /* 组起始断言数 */
static int         s_group_fails;   /* 组起始失败数 */

/* Private functions ---------------------------------------------------------*/

static void group_begin(const char *name)
{
  s_group        = name;
  s_group_checks = s_checks;
  s_group_fails  = s_fails;
  uart_printf("-- %s\r\n", name);
}

static void group_end(void)
{
  int c = s_checks - s_group_checks;
  int f = s_fails - s_group_fails;

  uart_printf("   %s: %d checks, %d failed -> %s\r\n",
              s_group, c, f, (f == 0) ? "PASS" : "FAIL");
}

/* 精确相等断言（32 位） */
static void ck(const char *what, int32_t got, int32_t want)
{
  s_checks++;
  if (got != want)
  {
    s_fails++;
    uart_printf("   FAIL %s: got %d want %d\r\n", what, (int)got, (int)want);
  }
}

/* 区间断言（32 位），用于宿主测试本身也带容差的量 */
static void ck_range(const char *what, int32_t got, int32_t lo, int32_t hi)
{
  s_checks++;
  if ((got < lo) || (got > hi))
  {
    s_fails++;
    uart_printf("   FAIL %s: got %d want [%d,%d]\r\n", what, (int)got, (int)lo, (int)hi);
  }
}

/* 精确相等断言（64 位，里程量用；消息里的值都很小，打印时截到 32 位） */
static void ck64(const char *what, int64_t got, int64_t want)
{
  s_checks++;
  if (got != want)
  {
    s_fails++;
    uart_printf("   FAIL %s: got %d want %d\r\n", what, (int)got, (int)want);
  }
}

/* 逐字节喂一串命令，返回产出的报文条数（最后一条留在 *m 里） */
static int feed_str(proto_parser_t *p, const char *s, proto_msg_t *m)
{
  int i;
  int got = 0;

  for (i = 0; s[i] != '\0'; i++)
  {
    if (proto_feed(p, (uint8_t)s[i], m) != 0)
    {
      got++;
    }
  }
  return got;
}

/* -------------------------------------------------------------------------- */
/*  [1] fixedpoint —— 定点标定与换算                                           */
/* -------------------------------------------------------------------------- */
static void selftest_fixedpoint(void)
{
  fixp_scale_t s = fixp_default_scale();
  fixp_scale_t bad;

  group_begin("fixedpoint");

  ck("counts_per_m nominal", s.counts_per_m, 10505);
  ck("wheel_diameter_mm nominal", s.wheel_diameter_mm, 80);
  ck("period_ms nominal", s.period_ms, 20);
  ck("scale_is_valid(default)", fixp_scale_is_valid(&s), 1);

  bad = s;
  bad.counts_per_m = 0;
  ck("scale_is_valid(counts_per_m=0)", fixp_scale_is_valid(&bad), 0);
  ck("scale_is_valid(NULL)", fixp_scale_is_valid(0), 0);

  ck("counts_per_m_from_params(44,60,80)",
     fixp_counts_per_m_from_params(44, 60, 80), 10504);
  ck("counts_per_m_from_params(44,56,80)",
     fixp_counts_per_m_from_params(44, 56, 80), 9804);
  ck("counts_per_m_from_params(gear=0)",
     fixp_counts_per_m_from_params(44, 0, 80), 0);

  /* 速度换算：100 mm/s 恰好 = 21 counts / 20 ms 窗口 */
  ck("mm_s_to_counts(100)", fixp_mm_s_to_counts(&s, 100), 21);
  ck("counts_to_mm_s(21)", fixp_counts_to_mm_s(&s, 21), 100);
  ck("counts_to_mm_s(1)", fixp_counts_to_mm_s(&s, 1), 5);
  ck("mm_s_to_counts(0)", fixp_mm_s_to_counts(&s, 0), 0);
  ck("mm_s_to_counts(-100)", fixp_mm_s_to_counts(&s, -100), -21);
  ck("counts_to_mm_s(-21)", fixp_counts_to_mm_s(&s, -21), -100);
  ck("mm_s_to_counts(2000)", fixp_mm_s_to_counts(&s, 2000), 420);
  ck("counts_to_mm_s(420)", fixp_counts_to_mm_s(&s, 420), 1999);

  /* 多窗口平均 */
  ck("counts_sum_to_mm_s(214,10)", fixp_counts_sum_to_mm_s(&s, 214, 10), 102);
  ck("counts_sum_to_mm_s(periods=1)",
     fixp_counts_sum_to_mm_s(&s, 21, 1), fixp_counts_to_mm_s(&s, 21));
  ck("counts_sum_to_mm_s(periods=0)", fixp_counts_sum_to_mm_s(&s, 214, 0), 0);
  ck("counts_sum_to_mm_s(periods=-3)", fixp_counts_sum_to_mm_s(&s, 214, -3), 0);

  /* 里程换算 */
  ck64("counts_to_mm(10505)", fixp_counts_to_mm(&s, 10505), 1000);
  ck64("mm_to_counts(1000)", fixp_mm_to_counts(&s, 1000), 10505);
  ck64("counts_to_mm(2640)", fixp_counts_to_mm(&s, 2640), 251);
  ck64("mm_to_counts(100)", fixp_mm_to_counts(&s, 100), 1051);
  ck64("counts_to_mm(-10505)", fixp_counts_to_mm(&s, -10505), -1000);
  ck64("mm_to_counts(-10505 ... -1000mm)", fixp_mm_to_counts(&s, -1000), -10505);
  ck64("counts_per_wheel_turn", fixp_counts_per_wheel_turn(&s), 2640);

  /* 轮速 -> 车体角速度 */
  ck("wheel_mm_s_to_omega(500,300)", fixp_wheel_mm_s_to_omega_mrad_s(500, 300), 1667);
  ck("wheel_mm_s_to_omega(-500,300)", fixp_wheel_mm_s_to_omega_mrad_s(-500, 300), -1667);
  ck("wheel_mm_s_to_omega(300,300)", fixp_wheel_mm_s_to_omega_mrad_s(300, 300), 1000);
  ck("wheel_mm_s_to_omega(500,0)", fixp_wheel_mm_s_to_omega_mrad_s(500, 0), 0);

  group_end();
}

/* -------------------------------------------------------------------------- */
/*  [2] mecanum —— 麦轮逆解 / 正解 / 归一 / 死区 / 安装映射 / 总入口            */
/* -------------------------------------------------------------------------- */
static void selftest_mecanum(void)
{
  mecanum_geom_t         g   = mecanum_default_geom();
  mecanum_mount_t        mnt = mecanum_identity_mount();
  mecanum_limits_t       lim;
  mecanum_twist_t        t;
  mecanum_wheels_t       w;
  mecanum_wheels_t       in;
  mecanum_wheels_t       out;
  mecanum_solve_result_t r;
  int32_t                motor[4];
  int32_t                ros2[4];
  int32_t                back[4];
  int                    i;

  group_begin("mecanum");

  ck("geom_is_valid(default)", mecanum_geom_is_valid(&g), 1);
  ck("l_mm(default 150/150)", mecanum_l_mm(&g), 300);
  ck("mount_is_valid(identity)", mecanum_mount_is_valid(&mnt), 1);

  /* 逆解：纯 vx */
  t = mecanum_zero_twist();
  t.vx_mm_s = 500;
  mecanum_inverse(&g, &t, &w);
  for (i = 0; i < 4; i++)
  {
    ck("inverse vx=500 wheel", w.wheel_mm_s[i], 500);
  }

  /* 逆解：纯 vy=400 -> {-400,+400,+400,-400} */
  t = mecanum_zero_twist();
  t.vy_mm_s = 400;
  mecanum_inverse(&g, &t, &w);
  ck("inverse vy=400 FL", w.wheel_mm_s[MECANUM_FL], -400);
  ck("inverse vy=400 FR", w.wheel_mm_s[MECANUM_FR],  400);
  ck("inverse vy=400 RL", w.wheel_mm_s[MECANUM_RL],  400);
  ck("inverse vy=400 RR", w.wheel_mm_s[MECANUM_RR], -400);

  /* 逆解：纯 omega=1000 mrad/s，L=300 -> rot=300 */
  t = mecanum_zero_twist();
  t.omega_mrad_s = 1000;
  mecanum_inverse(&g, &t, &w);
  ck("inverse omega=1000 FL", w.wheel_mm_s[MECANUM_FL], -300);
  ck("inverse omega=1000 FR", w.wheel_mm_s[MECANUM_FR],  300);
  ck("inverse omega=1000 RL", w.wheel_mm_s[MECANUM_RL], -300);
  ck("inverse omega=1000 RR", w.wheel_mm_s[MECANUM_RR],  300);

  /* 正解：四轮全 500 -> 纯 vx */
  for (i = 0; i < 4; i++)
  {
    w.wheel_mm_s[i] = 500;
  }
  mecanum_forward(&g, &w, &t);
  ck("forward vx", t.vx_mm_s, 500);
  ck("forward vy", t.vy_mm_s, 0);
  ck("forward omega", t.omega_mrad_s, 0);

  /* 极坐标翻转：vy_polarity=-1 */
  {
    mecanum_geom_t gf = mecanum_default_geom();
    gf.vy_polarity = -1;
    t = mecanum_zero_twist();
    t.vy_mm_s = 400;
    mecanum_inverse(&gf, &t, &w);
    ck("flip vy FL", w.wheel_mm_s[MECANUM_FL],  400);
    ck("flip vy FR", w.wheel_mm_s[MECANUM_FR], -400);
    ck("flip vy RR", w.wheel_mm_s[MECANUM_RR],  400);
    mecanum_forward(&gf, &w, &t);
    ck("flip forward vy", t.vy_mm_s, 400);
  }

  /* 非法几何：L=0 不许除零 */
  {
    mecanum_geom_t g0 = g;
    g0.half_wheelbase_mm = 0;
    g0.half_track_mm     = 0;
    ck("geom_is_valid(0,0)", mecanum_geom_is_valid(&g0), 0);
    t = mecanum_zero_twist();
    t.vx_mm_s = 500;
    mecanum_inverse(&g0, &t, &w);
    ck("inverse invalid geom FL", w.wheel_mm_s[MECANUM_FL], 0);
    mecanum_forward(&g0, &w, &t);
    ck("forward invalid geom vx", t.vx_mm_s, 0);
  }

  /* 饱和归一：峰值 6000 -> 3000，比例保持 */
  in.wheel_mm_s[0] =  2000;
  in.wheel_mm_s[1] = -4000;
  in.wheel_mm_s[2] =  1000;
  in.wheel_mm_s[3] = -6000;
  ck("desaturate sat flag", mecanum_desaturate(&in, 3000, &out), 1);
  ck("desat out FL", out.wheel_mm_s[0],  1000);
  ck("desat out FR", out.wheel_mm_s[1], -2000);
  ck("desat out RL", out.wheel_mm_s[2],   500);
  ck("desat out RR", out.wheel_mm_s[3], -3000);
  ck("desaturate again no-op", mecanum_desaturate(&out, 3000, &out), 0);
  ck("desaturate limit=0 no-op", mecanum_desaturate(&in, 0, &out), 0);
  ck("desat limit=0 keeps in RR", out.wheel_mm_s[3], -6000);
  in.wheel_mm_s[0] =  100;
  in.wheel_mm_s[1] = -200;
  in.wheel_mm_s[2] =  300;
  in.wheel_mm_s[3] =    0;
  ck("desaturate under limit no-op", mecanum_desaturate(&in, 500, &out), 0);
  ck("desat under limit FL", out.wheel_mm_s[0], 100);

  /* 死区抬升：全零必须保持全零（安全硬约束） */
  in.wheel_mm_s[0] = 0;
  in.wheel_mm_s[1] = 0;
  in.wheel_mm_s[2] = 0;
  in.wheel_mm_s[3] = 0;
  ck("deadzone all-zero not lifted", mecanum_lift_deadzone(&in, 300, &out), 0);
  ck("deadzone all-zero stays 0", out.wheel_mm_s[0], 0);

  in.wheel_mm_s[0] =  40;
  in.wheel_mm_s[1] = -80;
  in.wheel_mm_s[2] =  20;
  in.wheel_mm_s[3] =   0;
  ck("deadzone lifted flag", mecanum_lift_deadzone(&in, 400, &out), 1);
  ck("deadzone out FL", out.wheel_mm_s[0],  200);
  ck("deadzone out FR", out.wheel_mm_s[1], -400);
  ck("deadzone out RL", out.wheel_mm_s[2],  100);
  ck("deadzone out RR", out.wheel_mm_s[3],    0);

  in.wheel_mm_s[0] =  400;
  in.wheel_mm_s[1] = -100;
  in.wheel_mm_s[2] =  250;
  in.wheel_mm_s[3] =  -50;
  ck("deadzone all above no-op", mecanum_lift_deadzone(&in, 300, &out), 0);
  ck("deadzone no-op keeps FL", out.wheel_mm_s[0], 400);

  /* 安装映射：恒等往返 */
  w.wheel_mm_s[0] =  111;
  w.wheel_mm_s[1] = -222;
  w.wheel_mm_s[2] =  333;
  w.wheel_mm_s[3] = -444;
  mecanum_apply_mount(&mnt, &w, motor);
  ck("mount identity m0", motor[0],  111);
  ck("mount identity m1", motor[1], -222);
  ck("mount identity m3", motor[3], -444);
  mecanum_undo_mount(&mnt, motor, &out);
  ck("undo mount FL", out.wheel_mm_s[0],  111);
  ck("undo mount RR", out.wheel_mm_s[3], -444);

  {
    mecanum_mount_t bad = mecanum_identity_mount();
    bad.motor_of_wheel[1] = 0;          /* 两个轮子抢同一个电机 */
    ck("mount_is_valid(duplicate)", mecanum_mount_is_valid(&bad), 0);
  }

  /* ROS2 轮序：0=FL 1=FR 2=RR 3=RL */
  ros2[0] = 10;
  ros2[1] = 20;
  ros2[2] = 30;
  ros2[3] = 40;
  mecanum_wheels_from_ros2_order(ros2, &w);
  ck("from_ros2 FL", w.wheel_mm_s[MECANUM_FL], 10);
  ck("from_ros2 FR", w.wheel_mm_s[MECANUM_FR], 20);
  ck("from_ros2 RL", w.wheel_mm_s[MECANUM_RL], 40);
  ck("from_ros2 RR", w.wheel_mm_s[MECANUM_RR], 30);
  mecanum_wheels_to_ros2_order(&w, back);
  ck("to_ros2 0", back[0], 10);
  ck("to_ros2 2", back[2], 30);
  ck("to_ros2 3", back[3], 40);

  /* 总入口 solve：零指令 + 死区 -> idle */
  lim.wheel_max_mm_s      = 1000;
  lim.wheel_deadzone_mm_s =  400;
  t = mecanum_zero_twist();
  mecanum_solve(&g, &lim, &mnt, &t, &r);
  ck("solve idle flag", r.idle, 1);
  ck("solve idle saturated", r.saturated, 0);
  ck("solve idle lifted", r.lifted, 0);
  for (i = 0; i < 4; i++)
  {
    ck("solve idle motor", r.motor_mm_s[i], 0);
  }

  /* 总入口 solve：饱和 + 死区，期望值取自 test_mecanum.c */
  t = mecanum_zero_twist();
  t.vx_mm_s      = 2000;
  t.vy_mm_s      = 1000;
  t.omega_mrad_s = 4000;
  mecanum_solve(&g, &lim, &mnt, &t, &r);
  ck("solve saturated flag", r.saturated, 1);
  ck("solve not lifted", r.lifted, 0);
  ck("solve motor0", r.motor_mm_s[0],  -48);
  ck("solve motor1", r.motor_mm_s[1], 1000);
  ck("solve motor2", r.motor_mm_s[2],  429);
  ck("solve motor3", r.motor_mm_s[3],  524);

  /* 总入口 solve：指令落在死区内 -> 抬到死区边界 */
  lim.wheel_max_mm_s      = 3000;
  lim.wheel_deadzone_mm_s =  500;
  t = mecanum_zero_twist();
  t.vx_mm_s = 60;
  mecanum_solve(&g, &lim, &mnt, &t, &r);
  ck("solve lifted flag", r.lifted, 1);
  ck("solve lifted saturated", r.saturated, 0);
  for (i = 0; i < 4; i++)
  {
    ck("solve lifted motor", r.motor_mm_s[i], 500);
  }

  /* 总入口 solve：lim 为 NULL 表示不限幅 */
  t = mecanum_zero_twist();
  t.vx_mm_s = 2000;
  mecanum_solve(&g, 0, &mnt, &t, &r);
  ck("solve NULL lim saturated", r.saturated, 0);
  ck("solve NULL lim lifted", r.lifted, 0);
  ck("solve NULL lim motor0", r.motor_mm_s[0], 2000);

  group_end();
}

/* -------------------------------------------------------------------------- */
/*  [3] pid —— 限幅 / 饱和标志 / 微分 / 积分分离 / 复位                        */
/* -------------------------------------------------------------------------- */
static void selftest_pid(void)
{
  pid_gains_t g;
  pid_state_t st;
  uint32_t    flags;
  int         i;

  group_begin("pid");

  g = pid_default_gains();
  ck("default gains valid", pid_gains_is_valid(&g), 1);
  ck("gains NULL invalid", pid_gains_is_valid(0), 0);
  {
    pid_gains_t b = g;
    b.out_min = b.out_max;
    ck("gains out_min==out_max invalid", pid_gains_is_valid(&b), 0);
  }
  {
    pid_gains_t b = g;
    b.kp_q16 = -1;
    ck("gains negative kp invalid", pid_gains_is_valid(&b), 0);
  }
  {
    pid_gains_t b = g;
    b.kd_q16 = -1;
    ck("gains negative kd invalid", pid_gains_is_valid(&b), 0);
  }

  /* 纯比例：kp=1.0，饱和边界必须靠输出与标志一起分辨 */
  g = pid_default_gains();
  g.kp_q16 = 65536;
  g.ki_q16 = 0;
  g.kd_q16 = 0;

  pid_reset(&st);
  ck("kp=1 step(1000)", pid_step(&g, &st, 1000, 0, &flags), 1000);
  ck("kp=1 saturated(1000)", (int32_t)(flags & PID_FLAG_SATURATED),
     (int32_t)PID_FLAG_SATURATED);

  pid_reset(&st);
  ck("kp=1 step(999)", pid_step(&g, &st, 999, 0, &flags), 999);
  ck("kp=1 not saturated(999)", (int32_t)(flags & PID_FLAG_SATURATED), 0);

  pid_reset(&st);
  ck("kp=1 step(5000)", pid_step(&g, &st, 5000, 0, &flags), 1000);
  ck("kp=1 saturated(5000)", (int32_t)(flags & PID_FLAG_SATURATED),
     (int32_t)PID_FLAG_SATURATED);

  pid_reset(&st);
  ck("kp=1 step(-5000)", pid_step(&g, &st, -5000, 0, &flags), -1000);
  ck("kp=1 saturated(-5000)", (int32_t)(flags & PID_FLAG_SATURATED),
     (int32_t)PID_FLAG_SATURATED);

  pid_reset(&st);
  ck("kp=1 step(-1000)", pid_step(&g, &st, -1000, 0, &flags), -1000);
  ck("kp=1 saturated(-1000)", (int32_t)(flags & PID_FLAG_SATURATED),
     (int32_t)PID_FLAG_SATURATED);

  /* 空指针必须安全返回 0 */
  pid_reset(&st);
  ck("pid_step gains NULL", pid_step(0, &st, 100, 0, &flags), 0);
  ck("pid_step state NULL", pid_step(&g, 0, 100, 0, &flags), 0);

  /* 非对称限幅 */
  {
    pid_gains_t b = pid_default_gains();
    b.kp_q16  = 65536;
    b.ki_q16  = 0;
    b.kd_q16  = 0;
    b.out_min = -5;
    b.out_max =  7;
    pid_reset(&st);
    (void)pid_step(&b, &st, 0, 0, &flags);
    ck("asym limit step1", pid_step(&b, &st, 5000, 0, &flags), 7);
    ck("asym limit step2", pid_step(&b, &st, 5000, 0, &flags), 7);
  }

  /* 微分作用在测量值上：设定值阶跃首步无微分尖峰 */
  g = pid_default_gains();
  g.kp_q16 = 32768;      /* 0.5  */
  g.ki_q16 = 0;
  g.kd_q16 = 65536;      /* 1.0  */
  pid_reset(&st);
  ck("diff step1 (sp100 m0)", pid_step(&g, &st, 100, 0, &flags), 50);
  ck("diff step2 (sp100 m10)", pid_step(&g, &st, 100, 10, &flags), 35);
  ck("diff step3 (sp100 m10)", pid_step(&g, &st, 100, 10, &flags), 45);
  ck64("diff ki=0 integ untouched", st.integ_raw, 0);

  /* 积分分离：偏差 1000 > 50 -> 置标志且不积分 */
  g = pid_default_gains();
  g.integ_sep_mm_s = 50;
  pid_reset(&st);
  (void)pid_step(&g, &st, 1000, 0, &flags);
  ck("integ_sep flag", (int32_t)(flags & PID_FLAG_INTEG_SEP),
     (int32_t)PID_FLAG_INTEG_SEP);
  ck64("integ_sep no windup", st.integ_raw, 0);

  /* 偏差小于阈值 -> 正常积分 */
  pid_reset(&st);
  (void)pid_step(&g, &st, 10, 0, &flags);
  ck("integ_sep not flagged", (int32_t)(flags & PID_FLAG_INTEG_SEP), 0);
  ck("integ_sep integrates", (int32_t)(st.integ_raw != 0), 1);

  /* 复位清状态 */
  g = pid_default_gains();
  pid_reset(&st);
  for (i = 0; i < 10; i++)
  {
    (void)pid_step(&g, &st, 100, 0, &flags);
  }
  ck("integ nonzero before reset", (int32_t)(st.integ_raw != 0), 1);
  pid_reset(&st);
  ck64("reset integ_raw", st.integ_raw, 0);
  ck("reset prev_meas", st.prev_meas, 0);
  ck("reset started", (int32_t)st.started, 0);

  group_end();
}

/* -------------------------------------------------------------------------- */
/*  [4] odom —— 航迹推算（整数圆弧积分 + 角度归一化）                          */
/* -------------------------------------------------------------------------- */
static void selftest_odom(void)
{
  mecanum_geom_t  g = mecanum_default_geom();
  odom_pose_t     pose;
  odom_total_t    total;
  mecanum_twist_t inc;
  int32_t         d_now[4];
  int32_t         d_prev[4];
  int             i;
  int             k;

  group_begin("odom");

  /* 复位 */
  odom_reset(&pose, &total);
  ck64("reset pose.x", pose.x_mm, 0);
  ck64("reset pose.y", pose.y_mm, 0);
  ck("reset pose.theta", pose.theta_mrad, 0);
  ck64("reset total.x", total.x_mm, 0);
  ck64("reset total.theta", total.theta_mrad, 0);
  odom_reset(0, 0);            /* 双 NULL 不许崩 */
  odom_reset(&pose, 0);
  odom_reset(0, &total);
  ck("reset(NULL) survives pose", pose.theta_mrad, 0);

  /* 直线：100 步 × 四轮各 +10 mm -> 前进 1000 mm */
  odom_reset(&pose, &total);
  for (k = 0; k < 4; k++)
  {
    d_prev[k] = 0;
  }
  inc.vx_mm_s      = 0;
  inc.vy_mm_s      = 0;
  inc.omega_mrad_s = 0;
  for (i = 0; i < 100; i++)
  {
    for (k = 0; k < 4; k++)
    {
      d_now[k] = d_prev[k] + 10;
    }
    (void)odom_update(&pose, &total, &g, d_now, d_prev, &inc);
    for (k = 0; k < 4; k++)
    {
      d_prev[k] = d_now[k];
    }
  }
  ck("straight inc.vx", inc.vx_mm_s, 10);
  ck("straight inc.vy", inc.vy_mm_s, 0);
  ck("straight inc.omega", inc.omega_mrad_s, 0);
  ck_range("straight pose.x", (int32_t)pose.x_mm, 998, 1002);
  ck64("straight pose.y", pose.y_mm, 0);
  ck("straight pose.theta", pose.theta_mrad, 0);

  /* twist_out 语义：本步增量，不是速率 */
  odom_reset(&pose, &total);
  for (k = 0; k < 4; k++)
  {
    d_prev[k] = 100;
    d_now[k]  = 110;
  }
  ck("dtwist update ret", odom_update(&pose, &total, &g, d_now, d_prev, &inc), 1);
  ck("dtwist inc.vx", inc.vx_mm_s, 10);
  ck("dtwist inc.vy", inc.vy_mm_s, 0);
  ck("dtwist inc.omega", inc.omega_mrad_s, 0);
  ck64("dtwist pose.x", pose.x_mm, 10);

  /* 零增量：位姿不动 */
  odom_reset(&pose, &total);
  for (k = 0; k < 4; k++)
  {
    d_prev[k] = 0;
    d_now[k]  = 0;
  }
  ck("zero delta ret", odom_update(&pose, &total, &g, d_now, d_prev, &inc), 1);
  ck64("zero delta pose.x", pose.x_mm, 0);
  ck("zero delta pose.theta", pose.theta_mrad, 0);

  /* 角度归一化：total 200000-63 再走一步 +63 */
  odom_reset(&pose, &total);
  total.theta_mrad = 200000 - 63;
  d_prev[0] = 0;
  d_prev[1] = 0;
  d_prev[2] = 0;
  d_prev[3] = 0;
  d_now[0]  = -20;
  d_now[1]  =  20;
  d_now[2]  = -18;
  d_now[3]  =  18;
  ck("spin update ret", odom_update(&pose, &total, &g, d_now, d_prev, &inc), 1);
  ck("spin inc.omega(63)", inc.omega_mrad_s, 63);
  ck("spin inc.vx", inc.vx_mm_s, 0);
  ck("spin inc.vy", inc.vy_mm_s, 1);
  ck64("spin total.theta 200000", total.theta_mrad, 200000);
  ck("spin pose.theta -1088", pose.theta_mrad, -1088);
  ck("spin pose.theta in (-3142,3142]",
     (int32_t)((pose.theta_mrad > -3142) && (pose.theta_mrad <= 3142)), 1);
  ck("spin pose vs total off by 6284*k",
     (int32_t)(((pose.theta_mrad - (int32_t)total.theta_mrad) % 6284) == 0), 1);

  /* 非法输入必须被拒且不改动位姿 */
  odom_reset(&pose, &total);
  for (k = 0; k < 4; k++)
  {
    d_prev[k] = 0;
    d_now[k]  = 10;
  }
  ck("odom NULL pose", odom_update(0, &total, &g, d_now, d_prev, &inc), 0);
  ck("odom NULL total", odom_update(&pose, 0, &g, d_now, d_prev, &inc), 0);
  ck("odom NULL geom", odom_update(&pose, &total, 0, d_now, d_prev, &inc), 0);
  ck("odom NULL d_now", odom_update(&pose, &total, &g, 0, d_prev, &inc), 0);
  ck("odom NULL d_prev", odom_update(&pose, &total, &g, d_now, 0, &inc), 0);
  {
    mecanum_geom_t g0 = g;
    g0.half_wheelbase_mm = 0;
    g0.half_track_mm     = 0;
    ck("odom invalid geom", odom_update(&pose, &total, &g0, d_now, d_prev, &inc), 0);
  }
  ck64("rejected call left pose.x", pose.x_mm, 0);
  ck("odom twist_out NULL ok", odom_update(&pose, &total, &g, d_now, d_prev, 0), 1);

  /* 重复性：同一输入两遍完全一致 */
  odom_reset(&pose, &total);
  for (k = 0; k < 4; k++)
  {
    d_prev[k] = 0;
  }
  for (i = 0; i < 200; i++)
  {
    for (k = 0; k < 4; k++)
    {
      d_now[k] = d_prev[k] + 1;
    }
    (void)odom_update(&pose, &total, &g, d_now, d_prev, &inc);
    for (k = 0; k < 4; k++)
    {
      d_prev[k] = d_now[k];
    }
  }
  ck_range("repeat200 pose.x", (int32_t)pose.x_mm, 198, 202);
  ck64("repeat200 total.x", total.x_mm, pose.x_mm);

  group_end();
}

/* -------------------------------------------------------------------------- */
/*  [5] protocol —— 行解析器                                                   */
/* -------------------------------------------------------------------------- */
static void selftest_protocol(void)
{
  proto_parser_t p;
  proto_msg_t    m;
  int            n;
  int            i;

  group_begin("protocol");

  /* 正常帧 */
  proto_init(&p);
  ck("feed V,300,-100,250 count", feed_str(&p, "V,300,-100,250\n", &m), 1);
  ck("cmd V", (int32_t)m.cmd, (int32_t)PROTO_CMD_V);
  ck("V error none", (int32_t)m.error, 0);
  ck("V argc 3", (int32_t)m.argc, 3);
  ck("V arg0", m.arg[0],  300);
  ck("V arg1", m.arg[1], -100);
  ck("V arg2", m.arg[2],  250);

  /* 半包：换行之前不产出 */
  proto_init(&p);
  n = 0;
  for (i = 0; i < 7; i++)
  {
    n += (proto_feed(&p, (uint8_t)"V,1,2,3"[i], &m) != 0) ? 1 : 0;
  }
  ck("half packet incomplete", n, 0);
  ck("half packet completes at LF", proto_feed(&p, (uint8_t)'\n', &m), 1);
  ck("half packet arg0", m.arg[0], 1);

  /* 粘包：一次喂两帧 */
  proto_init(&p);
  n = feed_str(&p, "V,1,2,3\nV,4,5,6\n", &m);
  ck("sticky packet count", n, 2);
  ck("sticky last cmd", (int32_t)m.cmd, (int32_t)PROTO_CMD_V);
  ck("sticky last arg0", m.arg[0], 4);
  ck("sticky last arg2", m.arg[2], 6);

  /* 行缓冲溢出后必须自愈 */
  proto_init(&p);
  n = 0;
  for (i = 0; i < (PROTO_LINE_MAX + 50); i++)
  {
    n += (proto_feed(&p, (uint8_t)'x', &m) != 0) ? 1 : 0;
  }
  n += (proto_feed(&p, (uint8_t)'\n', &m) != 0) ? 1 : 0;
  ck("overflow no msg", n, 0);
  ck("overflow stat counter", (int32_t)(p.stat_overflow >= 1u), 1);
  ck("overflow stat_lines 0", (int32_t)p.stat_lines, 0);
  ck("recovers after overflow", feed_str(&p, "V,7,8,9\n", &m), 1);
  ck("recovered arg1", m.arg[1], 8);
  ck("recovered error none", (int32_t)m.error, 0);

  /* 非法字符：置 BAD_CHAR 但仍产出，下一帧必须干净 */
  proto_init(&p);
  ck("badchar still produces", feed_str(&p, "V,1" "\x01" ",2,3\n", &m), 1);
  ck("badchar flag", (int32_t)(m.error & PROTO_ERR_BAD_CHAR),
     (int32_t)PROTO_ERR_BAD_CHAR);
  ck("badchar stat counter", (int32_t)(p.stat_bad >= 1u), 1);
  ck("badchar next frame clean", feed_str(&p, "V,5,6,7\n", &m), 1);
  ck("badchar next error none", (int32_t)m.error, 0);

  /* 未知命令 */
  proto_init(&p);
  ck("unknown Z produces", feed_str(&p, "Z,1,2\n", &m), 1);
  ck("unknown cmd", (int32_t)m.cmd, (int32_t)PROTO_CMD_UNKNOWN);
  ck("unknown flag", (int32_t)(m.error & PROTO_ERR_UNKNOWN_CMD),
     (int32_t)PROTO_ERR_UNKNOWN_CMD);
  ck("unknown name starts with U",
     (int32_t)proto_cmd_name(PROTO_CMD_UNKNOWN)[0], (int32_t)'U');
  ck("V300 glued -> unknown", feed_str(&p, "V300,0,0\n", &m), 1);
  ck("V300 cmd unknown", (int32_t)m.cmd, (int32_t)PROTO_CMD_UNKNOWN);

  /* 参数个数 */
  proto_init(&p);
  ck("V,1,2 produces", feed_str(&p, "V,1,2\n", &m), 1);
  ck("V,1,2 argc flag", (int32_t)(m.error & PROTO_ERR_ARGC), (int32_t)PROTO_ERR_ARGC);
  ck("V,1,2 partial arg1", m.arg[1], 2);
  ck("M,1,2,3 argc flag", feed_str(&p, "M,1,2,3\n", &m), 1);
  ck("M,1,2,3 flag", (int32_t)(m.error & PROTO_ERR_ARGC), (int32_t)PROTO_ERR_ARGC);

  /* 长度合法命令：S 零参数 / reset */
  proto_init(&p);
  ck("S produces", feed_str(&p, "S\n", &m), 1);
  ck("S cmd", (int32_t)m.cmd, (int32_t)PROTO_CMD_S);
  ck("S error none", (int32_t)m.error, 0);
  ck("S argc 0", (int32_t)m.argc, 0);
  ck("Sx -> unknown", feed_str(&p, "Sx\n", &m), 1);
  ck("Sx cmd unknown", (int32_t)m.cmd, (int32_t)PROTO_CMD_UNKNOWN);

  proto_init(&p);
  ck("reset produces", feed_str(&p, "reset\n", &m), 1);
  ck("reset cmd", (int32_t)m.cmd, (int32_t)PROTO_CMD_RESET);
  ck("reset error none", (int32_t)m.error, 0);
  ck("RESET upper", feed_str(&p, "RESET\r\n", &m), 1);
  ck("RESET cmd", (int32_t)m.cmd, (int32_t)PROTO_CMD_RESET);
  ck("resett -> unknown", feed_str(&p, "resett\n", &m), 1);
  ck("resett cmd unknown", (int32_t)m.cmd, (int32_t)PROTO_CMD_UNKNOWN);

  /* 数值解析 */
  proto_init(&p);
  ck("M 4-arg produces", feed_str(&p, "M,-1000,1000,-500,0\n", &m), 1);
  ck("M error none", (int32_t)m.error, 0);
  ck("M arg0", m.arg[0], -1000);
  ck("M arg1", m.arg[1],  1000);
  ck("M arg2", m.arg[2],  -500);

  ck("INT32 bounds produces",
     feed_str(&p, "M,-2147483648,2147483647,0,0\n", &m), 1);
  ck("INT32_MIN ok", m.arg[0], INT32_MIN);
  ck("INT32_MAX ok", m.arg[1], INT32_MAX);
  ck("INT32 bounds no RANGE", (int32_t)(m.error & PROTO_ERR_RANGE), 0);

  ck("positive overflow produces", feed_str(&p, "V,99999999999999,0,0\n", &m), 1);
  ck("positive overflow RANGE",
     (int32_t)(m.error & PROTO_ERR_RANGE), (int32_t)PROTO_ERR_RANGE);
  ck("positive overflow clamps", m.arg[0], INT32_MAX);

  ck("negative overflow produces", feed_str(&p, "V,-99999999999999,0,0\n", &m), 1);
  ck("negative overflow RANGE",
     (int32_t)(m.error & PROTO_ERR_RANGE), (int32_t)PROTO_ERR_RANGE);
  ck("negative overflow clamps", m.arg[0], INT32_MIN);

  ck("empty args produces", feed_str(&p, "V,,,\n", &m), 1);
  ck("empty args BAD_NUMBER",
     (int32_t)(m.error & PROTO_ERR_BAD_NUMBER), (int32_t)PROTO_ERR_BAD_NUMBER);
  ck("empty args argc 3", (int32_t)m.argc, 3);

  ck("garbage digits produces", feed_str(&p, "V,1a2,3,4\n", &m), 1);
  ck("garbage digits BAD_NUMBER",
     (int32_t)(m.error & PROTO_ERR_BAD_NUMBER), (int32_t)PROTO_ERR_BAD_NUMBER);
  ck("lone minus produces", feed_str(&p, "V,-,3,4\n", &m), 1);
  ck("lone minus BAD_NUMBER",
     (int32_t)(m.error & PROTO_ERR_BAD_NUMBER), (int32_t)PROTO_ERR_BAD_NUMBER);

  /* 行尾与空白 */
  proto_init(&p);
  ck("CRLF produces", feed_str(&p, "V,300,0,0\r\n", &m), 1);
  ck("CRLF arg0", m.arg[0], 300);
  proto_init(&p);
  n = 0;
  for (i = 0; i < 3; i++)
  {
    n += (proto_feed(&p, (uint8_t)'\n', &m) != 0) ? 1 : 0;
  }
  ck("empty lines no msg", n, 0);
  ck("empty lines counted", (int32_t)p.stat_empty, 3);
  ck("blank line no msg", feed_str(&p, "   \t \n", &m), 0);
  ck("padded frame produces", feed_str(&p, "  V,1,2,3  \n", &m), 1);
  ck("padded frame arg2", m.arg[2], 3);
  ck("padded frame error none", (int32_t)m.error, 0);
  ck("padded frame raw_len>=9", (int32_t)(m.raw_len >= 9u), 1);

  /* 命令名表 */
  ck("cmd_name(V)[0]", (int32_t)proto_cmd_name(PROTO_CMD_V)[0], (int32_t)'V');
  ck("cmd_name(M)[0]", (int32_t)proto_cmd_name(PROTO_CMD_M)[0], (int32_t)'M');
  ck("cmd_name(RESET)[0]", (int32_t)proto_cmd_name(PROTO_CMD_RESET)[0], (int32_t)'r');

  group_end();
}

/* -------------------------------------------------------------------------- */
/*  入口                                                                       */
/* -------------------------------------------------------------------------- */
void core_selftest_run(void)
{
  s_checks = 0;
  s_fails  = 0;

  uart_printf("\r\n");
  uart_printf("========== core_selftest: firmware_core link check ==========\r\n");
  uart_printf("sizeof: geom=%u twist=%u wheels=%u pid_gains=%u pid_state=%u\r\n",
              (unsigned)sizeof(mecanum_geom_t), (unsigned)sizeof(mecanum_twist_t),
              (unsigned)sizeof(mecanum_wheels_t), (unsigned)sizeof(pid_gains_t),
              (unsigned)sizeof(pid_state_t));

  selftest_fixedpoint();
  selftest_mecanum();
  selftest_pid();
  selftest_odom();
  selftest_protocol();

  uart_printf("core_selftest: %d checks, %d failed\r\n", s_checks, s_fails);
  uart_printf("core_selftest: %s\r\n", (s_fails == 0) ? "ALL PASS" : "FAILURES PRESENT");
  uart_printf("============================================================\r\n");
}
