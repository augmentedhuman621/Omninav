/**
  ******************************************************************************
  * @file    Src/app_single_wheel.c
  * @brief   A-02 最小单轮安全固件：首次让一个轮子安全、可控地转起来的最小代码集
  *
  *  安全行为（task-5 硬要求，缺一不可）：
  *    1) 上电默认禁能：STBY=0、PWM=0、IN1=IN2=0（高阻滑行，轮子不转）
  *    2) 输出限幅：占空比上限 OMNINAV_APP_DUTY_MAX（默认 50%），按键步进 5%
  *    3) 超时停车：RUNNING 状态下超过 OMNINAV_CMD_TIMEOUT_MS（默认 200 ms）
  *       没有收到任何有效命令 -> 立即撤输出回 DISABLED，并打印
  *       “最后有效命令 -> 撤输出”的实测时间差（供 A-05 钉死阈值）
  *    4) 单轮调试入口：USART1（电脑调试口）单字符命令，选轮/方向/占空比/停
  *       ★ 不新增任何冻结协议帧；冻结的 USART2 协议口（PA2/PA3）完全不碰
  *    5) 20 ms 控制周期骨架 app_tick_20ms()，与方案 A-3 的 20 ms 速度环对齐
  *
  *  ★ 与 bench.c 阶段 C 的关系：bench.c 是已验收基线（USART2 11/11），
  *    本文件是独立实现、一行都不改 bench.c / usart2_bench.c / core_selftest.c；
  *    电机初始化顺序与 bench.c 阶段 C 保持一致（该顺序已在真机跑通），
  *    并展开为 4 路 PWM + 8 根方向脚（同一时刻只驱动选中的 1 个轮子）。
  *
  *  ★ 最小停止路径（首次非零输出前必须满足）：
  *      DISABLED(STBY=0) 是上电默认态；超时 / 's' / 任何异常都回 DISABLED，
  *      且必须显式按 'g' 才能重新使能（锁存语义，防“抖动反复使能”）。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_single_wheel.h"

#include <stdint.h>

/* Private defines -----------------------------------------------------------*/
#ifndef OMNINAV_CMD_TIMEOUT_MS
#define OMNINAV_CMD_TIMEOUT_MS 200u     /* task-5 裁定默认 200；A-05 实测后可调 150-250 */
#endif
#ifndef OMNINAV_APP_DUTY_MAX
#define OMNINAV_APP_DUTY_MAX 50u        /* 输出限幅：默认 50%（宏可调，A-02 先保守） */
#endif

#define APP_VERSION          "v0.1"
#define APP_DUTY_STEP        5u         /* 按键占空比步进 5% */
#define APP_CTRL_PERIOD_MS   20u        /* 20 ms 调度骨架 */
#define APP_STATUS_MS        500u       /* 周期状态打印 */
#define APP_PWM_ARR          3599u      /* 72MHz/(0+1)/(3599+1) = 20.000 kHz，同 bench.c */
#define APP_DIR_SETTLE_MS    5u         /* 换向死区：PWM=0 后至少 5 ms 才允许再给输出 */

/* 方向脚与 PWM 脚（06 §5.1.1 TFT_LCD 丝印表；永远认丝印不数孔号） */
#define APP_DIR_PINS_E      ((uint16_t)(GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_12))
#define APP_DIR_PINS_D      ((uint16_t)(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9))
#define APP_PWM_PINS_E      ((uint16_t)(GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14))
#define APP_STBY_PIN        GPIO_PIN_15

/* Private types -------------------------------------------------------------*/
typedef struct
{
  GPIO_TypeDef *port;    /* 方向脚所在端口 */
  uint16_t      in1;     /* AIN1 / BIN1 */
  uint16_t      in2;     /* AIN2 / BIN2 */
  uint32_t      ch;      /* TIM1 通道 */
  const char   *name;    /* 丝印 M1..M4 */
} app_wheel_t;

typedef enum
{
  APP_ST_DISABLED = 0,   /* STBY=0 / PWM=0 / IN=0：高阻滑行，上电默认态 */
  APP_ST_ARMED    = 1,   /* STBY=1 / PWM=0：短路制动，等心跳 */
  APP_ST_RUNNING  = 2    /* STBY=1 / PWM=duty：受心跳监管，超时即撤输出 */
} app_state_t;

/* Private variables ---------------------------------------------------------*/
/*
 * 轮序 = M1..M4；引脚与 06 §5.1.1 TFT_LCD 丝印表一致：
 *   M1: PWM PE9 (D6)   AIN1 PE7 (D4)  AIN2 PE8 (D5)
 *   M2: PWM PE11(D8)   BIN1 PE10(D7)  BIN2 PE12(D9)
 *   M3: PWM PE13(D10)  AIN1 PD0 (D2)  AIN2 PD1 (D3)
 *   M4: PWM PE14(D11)  BIN1 PD8 (D13) BIN2 PD9 (D14)
 */
static const app_wheel_t k_wheel[4] =
{
  { GPIOE, GPIO_PIN_7,  GPIO_PIN_8,  TIM_CHANNEL_1, "M1" },
  { GPIOE, GPIO_PIN_10, GPIO_PIN_12, TIM_CHANNEL_2, "M2" },
  { GPIOD, GPIO_PIN_0,  GPIO_PIN_1,  TIM_CHANNEL_3, "M3" },
  { GPIOD, GPIO_PIN_8,  GPIO_PIN_9,  TIM_CHANNEL_4, "M4" }
};

static TIM_HandleTypeDef htim1_app;

static app_state_t g_state;              /* 当前安全状态 */
static uint8_t     g_wheel;              /* 0..3 = M1..M4 */
static uint8_t     g_fwd;                /* 1 = FWD, 0 = REV */
static uint8_t     g_duty;               /* 0..OMNINAV_APP_DUTY_MAX (%) */
static uint32_t    g_last_cmd_tick;      /* 最后一条有效命令的 HAL tick */
static char        g_last_cmd;           /* 最后一条有效命令字符 */
static uint32_t    g_dir_block_until;    /* 换向死区：此刻之前不允许给输出 */
static uint32_t    g_last_revoke_ms;     /* 最近一次“撤输出”实测延迟：最后有效命令 -> 撤 */

/* 外部符号：main.c 提供 */
extern UART_HandleTypeDef huart1;
extern void uart_printf(const char *fmt, ...);

/* Private function prototypes ------------------------------------------------*/
static void app_init(void);
static void app_print_banner(void);
static void app_print_help(void);
static void app_print_status(void);
static const char *app_state_name(void);
static void app_pwm_all(uint32_t ccr);
static void app_dir_all_off(void);
static void app_output_disabled(void);
static void app_output_armed(void);
static void app_output_running(void);
static int  app_getc(char *c);
static void app_revoke_timeout(uint32_t now);
static void app_handle_cmd(char c);
static void app_tick_20ms(uint32_t now);

/* ==========================================================================
 *  打印层（全部走 main.c 的 uart_printf；不用标准 printf、不用 %f）
 * ========================================================================== */

static const char *app_state_name(void)
{
  if (g_state == APP_ST_DISABLED)
  {
    return "DISABLED";
  }
  if (g_state == APP_ST_ARMED)
  {
    return "ARMED";
  }
  return "RUNNING";
}

static void app_print_help(void)
{
  uart_printf("[A02] keys (single char, no Enter):\r\n");
  uart_printf("[A02]   1..4 = select wheel M1..M4 (duty resets to 0)\r\n");
  uart_printf("[A02]   f / r = direction FWD / REV (PWM forced to 0 first)\r\n");
  uart_printf("[A02]   + / - = duty step 5%%   range 0..%lu%%\r\n",
              (unsigned long)OMNINAV_APP_DUTY_MAX);
  uart_printf("[A02]   g = arm (STBY=1, PWM=0 brake)   k = heartbeat\r\n");
  uart_printf("[A02]   b = brake (STBY=1, PWM=0)   s = stop -> DISABLED (STBY=0)\r\n");
  uart_printf("[A02]   p = status   ? = this help\r\n");
  uart_printf("[A02] run : '+' to 10..20%% -> 'g' -> send 'k' every <=100 ms\r\n");
  uart_printf("[A02] stop: stop sending 'k' -> output revoked within %lu ms\r\n",
              (unsigned long)OMNINAV_CMD_TIMEOUT_MS);
}

static void app_print_banner(void)
{
  uart_printf("\r\n");
  uart_printf("[A02] ==================================================\r\n");
  uart_printf("[A02] app=single_wheel %s   (A-02 minimum safe single-wheel)\r\n", APP_VERSION);
  uart_printf("[A02] build : %s %s\r\n", __DATE__, __TIME__);
  uart_printf("[A02] safety: boot STBY=0/PWM=0/IN=0 (coast); timeout=%lu ms; duty_max=%lu%%\r\n",
              (unsigned long)OMNINAV_CMD_TIMEOUT_MS, (unsigned long)OMNINAV_APP_DUTY_MAX);
  uart_printf("[A02] wheel : 1=M1(PE9/PE7,PE8)  2=M2(PE11/PE10,PE12)\r\n");
  uart_printf("[A02]         3=M3(PE13/PD0,PD1) 4=M4(PE14/PD8,PD9)\r\n");
  uart_printf("[A02] port  : USART1 debug (single-char); frozen USART2 NOT touched\r\n");
  uart_printf("[A02] pre   : STBY(PE15) must be wired to TB6612, NOT tied to 3.3V\r\n");
  uart_printf("[A02] ==================================================\r\n");
  app_print_help();
}

static void app_print_status(void)
{
  uint32_t age;

  age = HAL_GetTick() - g_last_cmd_tick;
  uart_printf("[A02] st=%s wheel=%s dir=%s duty=%u%% age=%lu ms last='%c' last_stop=%lu ms\r\n",
              app_state_name(),
              k_wheel[g_wheel].name,
              (g_fwd != 0u) ? "FWD" : "REV",
              (unsigned)g_duty,
              (unsigned long)age,
              (g_last_cmd != 0) ? g_last_cmd : '-',
              (unsigned long)g_last_revoke_ms);
}

/* ==========================================================================
 *  输出层：只允许“先 PWM=0，再改方向，最后给占空比”的顺序
 * ========================================================================== */

static void app_pwm_all(uint32_t ccr)
{
  __HAL_TIM_SET_COMPARE(&htim1_app, TIM_CHANNEL_1, ccr);
  __HAL_TIM_SET_COMPARE(&htim1_app, TIM_CHANNEL_2, ccr);
  __HAL_TIM_SET_COMPARE(&htim1_app, TIM_CHANNEL_3, ccr);
  __HAL_TIM_SET_COMPARE(&htim1_app, TIM_CHANNEL_4, ccr);
}

static void app_dir_all_off(void)
{
  HAL_GPIO_WritePin(GPIOE, APP_DIR_PINS_E, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, APP_DIR_PINS_D, GPIO_PIN_RESET);
}

/* 安全态：STBY=0 -> TB6612 全桥高阻（滑行，不制动），PWM=0、IN=0 */
static void app_output_disabled(void)
{
  app_pwm_all(0u);
  app_dir_all_off();
  HAL_GPIO_WritePin(GPIOE, APP_STBY_PIN, GPIO_PIN_RESET);
}

/* 待命态：STBY=1 + PWM=0 -> 短路制动（刹车），但轮子不会被推动着转 */
static void app_output_armed(void)
{
  app_pwm_all(0u);
  app_dir_all_off();
  HAL_GPIO_WritePin(GPIOE, APP_STBY_PIN, GPIO_PIN_SET);
}

/* 运行态：仅选中轮给方向 + 占空比，其余三轮强制 0 */
static void app_output_running(void)
{
  const app_wheel_t *w;
  uint32_t ccr;

  w   = &k_wheel[g_wheel];
  ccr = ((uint32_t)g_duty * (APP_PWM_ARR + 1u)) / 100u;

  app_pwm_all(0u);        /* 换向/启动死区：先撤 PWM */
  app_dir_all_off();
  if (g_fwd != 0u)
  {
    HAL_GPIO_WritePin(w->port, w->in1, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(w->port, w->in2, GPIO_PIN_SET);
  }
  HAL_GPIO_WritePin(GPIOE, APP_STBY_PIN, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim1_app, w->ch, ccr);
}

/* ==========================================================================
 *  初始化（顺序与 bench.c 阶段 C 一致；真机已验证过该顺序）
 * ========================================================================== */

static void app_init(void)
{
  GPIO_InitTypeDef        g  = {0};
  TIM_OC_InitTypeDef      oc = {0};
  TIM_MasterConfigTypeDef mc = {0};

  g_state           = APP_ST_DISABLED;
  g_wheel           = 0u;
  g_fwd             = 1u;
  g_duty            = 0u;
  g_last_cmd        = 0;
  g_last_cmd_tick   = HAL_GetTick();
  g_dir_block_until = 0u;
  g_last_revoke_ms  = 0u;

  /* AFIO 时钟必须先开，重映射寄存器才写得进去 */
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();

  /* TIM1 完全重映射：CH1=PE9 CH2=PE11 CH3=PE13 CH4=PE14 */
  __HAL_AFIO_REMAP_TIM1_ENABLE();

  /* 先把方向脚 + STBY 写成低，再配成输出：避免上电瞬间误动 */
  HAL_GPIO_WritePin(GPIOE, (uint16_t)(APP_DIR_PINS_E | APP_STBY_PIN), GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, APP_DIR_PINS_D, GPIO_PIN_RESET);

  /* PWM 四脚：复用推挽 */
  g.Pin   = APP_PWM_PINS_E;
  g.Mode  = GPIO_MODE_AF_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &g);

  /* 方向 8 脚 + STBY(PE15)：普通推挽输出 */
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin   = (uint16_t)(APP_DIR_PINS_E | APP_STBY_PIN);
  HAL_GPIO_Init(GPIOE, &g);
  g.Pin   = APP_DIR_PINS_D;
  HAL_GPIO_Init(GPIOD, &g);

  htim1_app.Instance               = TIM1;
  htim1_app.Init.Prescaler         = 0u;
  htim1_app.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1_app.Init.Period            = APP_PWM_ARR;   /* 20 kHz */
  htim1_app.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1_app.Init.RepetitionCounter = 0u;
  htim1_app.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  (void)HAL_TIM_PWM_Init(&htim1_app);

  oc.OCMode       = TIM_OCMODE_PWM1;
  oc.Pulse        = 0u;
  oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode   = TIM_OCFAST_DISABLE;
  oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  (void)HAL_TIM_PWM_ConfigChannel(&htim1_app, &oc, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_ConfigChannel(&htim1_app, &oc, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_ConfigChannel(&htim1_app, &oc, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_ConfigChannel(&htim1_app, &oc, TIM_CHANNEL_4);

  mc.MasterOutputTrigger = TIM_TRGO_RESET;
  mc.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  (void)HAL_TIMEx_MasterConfigSynchronization(&htim1_app, &mc);

  /* ★ 配置完必须显式 Start，否则 GPIO 上永远没有波形（06 §10-15 的坑） */
  (void)HAL_TIM_PWM_Start(&htim1_app, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim1_app, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Start(&htim1_app, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Start(&htim1_app, TIM_CHANNEL_4);

  /* 显式回安全态：STBY=0 / PWM=0 / IN=0 */
  app_output_disabled();
}

/* ==========================================================================
 *  USART1 非阻塞收字节（轮询，不占中断向量；与 bench.c 的 uart_getc 同法）
 * ========================================================================== */

static int app_getc(char *c)
{
  volatile uint32_t tmp;

  /* 溢出错误要先清掉，否则 RXNE 再也不置位 */
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET)
  {
    tmp = huart1.Instance->SR;
    tmp = huart1.Instance->DR;
    (void)tmp;
  }

  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
  {
    *c = (char)(huart1.Instance->DR & 0xFFu);
    return 1;
  }

  return 0;
}

/* ==========================================================================
 *  状态机
 * ========================================================================== */

static void app_revoke_timeout(uint32_t now)
{
  uint32_t dt;

  dt = now - g_last_cmd_tick;
  g_last_revoke_ms = dt;
  g_duty           = 0u;
  g_state          = APP_ST_DISABLED;
  app_output_disabled();

  uart_printf("[A02] STOP timeout: cmd '%c' @%lu ms -> revoked @%lu ms, dt=%lu ms (limit=%lu ms)\r\n",
              (g_last_cmd != 0) ? g_last_cmd : '-',
              (unsigned long)g_last_cmd_tick,
              (unsigned long)now,
              (unsigned long)dt,
              (unsigned long)OMNINAV_CMD_TIMEOUT_MS);
  uart_printf("[A02] state -> DISABLED (STBY=0/PWM=0/IN=0); press 'g' to re-arm\r\n");
}

static void app_tick_20ms(uint32_t now)
{
  uint32_t age;

  age = now - g_last_cmd_tick;

  switch (g_state)
  {
    case APP_ST_DISABLED:
      /* 每 20 ms 强制一次安全态，防止任何意外残留输出 */
      app_output_disabled();
      break;

    case APP_ST_ARMED:
      app_output_armed();
      if (g_duty == 0u)
      {
        break;                                  /* 占空比 0：不启动 */
      }
      if (age >= OMNINAV_CMD_TIMEOUT_MS)
      {
        break;                                  /* 心跳不新：不启动 */
      }
      if ((int32_t)(now - g_dir_block_until) < 0)
      {
        break;                                  /* 换向死区未到：不启动 */
      }
      g_state = APP_ST_RUNNING;
      app_output_running();
      uart_printf("[A02] state -> RUNNING (%s %s duty=%u%%), keep 'k' every <=%lu ms\r\n",
                  k_wheel[g_wheel].name,
                  (g_fwd != 0u) ? "FWD" : "REV",
                  (unsigned)g_duty,
                  (unsigned long)OMNINAV_CMD_TIMEOUT_MS);
      break;

    case APP_ST_RUNNING:
    default:
      if (age >= OMNINAV_CMD_TIMEOUT_MS)
      {
        app_revoke_timeout(now);
      }
      else if (g_duty == 0u)
      {
        g_state = APP_ST_ARMED;
        app_output_armed();
        uart_printf("[A02] duty=0 -> ARMED (STBY=1, PWM=0)\r\n");
      }
      else
      {
        app_output_running();
      }
      break;
  }
}

/* ==========================================================================
 *  命令处理（USART1 单字符；不新增任何协议帧）
 * ========================================================================== */

static void app_handle_cmd(char c)
{
  uint32_t now;

  if ((c == '\r') || (c == '\n'))
  {
    return;
  }

  if ((c >= 'A') && (c <= 'Z'))
  {
    c = (char)(c + ('a' - 'A'));
  }

  now             = HAL_GetTick();
  g_last_cmd      = c;
  g_last_cmd_tick = now;

  switch (c)
  {
    case '1':
    case '2':
    case '3':
    case '4':
      g_wheel = (uint8_t)(c - '1');
      g_duty  = 0u;
      if (g_state == APP_ST_RUNNING)
      {
        g_state = APP_ST_ARMED;
        app_output_armed();
      }
      g_dir_block_until = now + APP_DIR_SETTLE_MS;
      uart_printf("[A02] wheel -> %s, duty=0%%, PWM forced 0\r\n", k_wheel[g_wheel].name);
      break;

    case 'f':
    case 'r':
      if (g_state == APP_ST_RUNNING)
      {
        g_state = APP_ST_ARMED;
        app_output_armed();
      }
      g_fwd             = (c == 'f') ? 1u : 0u;
      g_duty            = 0u;   /* 换向必须 PWM 归零；占空比也清零防“按一下就飞” */
      g_dir_block_until = now + APP_DIR_SETTLE_MS;
      uart_printf("[A02] dir -> %s, duty=0%%, %lu ms dead-time after PWM=0\r\n",
                  (g_fwd != 0u) ? "FWD" : "REV",
                  (unsigned long)APP_DIR_SETTLE_MS);
      break;

    case '+':
      if (g_duty <= (uint8_t)(OMNINAV_APP_DUTY_MAX - APP_DUTY_STEP))
      {
        g_duty = (uint8_t)(g_duty + APP_DUTY_STEP);
      }
      else
      {
        g_duty = (uint8_t)OMNINAV_APP_DUTY_MAX;
      }
      uart_printf("[A02] duty -> %u%% (max %lu%%)\r\n",
                  (unsigned)g_duty, (unsigned long)OMNINAV_APP_DUTY_MAX);
      break;

    case '-':
      if (g_duty >= APP_DUTY_STEP)
      {
        g_duty = (uint8_t)(g_duty - APP_DUTY_STEP);
      }
      else
      {
        g_duty = 0u;
      }
      uart_printf("[A02] duty -> %u%%\r\n", (unsigned)g_duty);
      break;

    case 'g':
      if (g_state == APP_ST_DISABLED)
      {
        g_state = APP_ST_ARMED;
        app_output_armed();
        uart_printf("[A02] state -> ARMED (STBY=1, PWM=0, brake)\r\n");
      }
      else
      {
        uart_printf("[A02] already %s; heartbeat refreshed\r\n", app_state_name());
      }
      break;

    case 'k':
      /* 心跳：只刷新时间戳；是否维持 RUNNING 由 20 ms tick 决定 */
      break;

    case 'b':
      g_duty = 0u;
      if (g_state == APP_ST_DISABLED)
      {
        g_state = APP_ST_ARMED;
      }
      app_output_armed();
      uart_printf("[A02] BRAKE (STBY=1, PWM=0, short brake)\r\n");
      break;

    case 's':
      g_duty  = 0u;
      g_state = APP_ST_DISABLED;
      app_output_disabled();
      uart_printf("[A02] STOP by user -> DISABLED (STBY=0, PWM=0, coast)\r\n");
      break;

    case 'p':
      app_print_status();
      break;

    case '?':
      app_print_help();
      break;

    default:
      uart_printf("[A02] unknown '%c' (try '?')\r\n", (int)c);
      break;
  }
}

/* ==========================================================================
 *  入口：初始化 -> banner -> 主循环（收命令 + 20 ms tick + 周期状态）
 * ========================================================================== */

void app_single_wheel_run(void)
{
  char     c;
  uint32_t now;
  uint32_t t_ctrl;
  uint32_t t_status;

  app_init();
  app_print_banner();

  t_ctrl   = HAL_GetTick();
  t_status = t_ctrl;

  while (1)
  {
    while (app_getc(&c) != 0)
    {
      app_handle_cmd(c);
    }

    now = HAL_GetTick();

    if ((now - t_ctrl) >= APP_CTRL_PERIOD_MS)
    {
      t_ctrl += APP_CTRL_PERIOD_MS;
      if ((now - t_ctrl) > (APP_CTRL_PERIOD_MS * 10u))
      {
        t_ctrl = now;   /* 防长阻塞后“补 tick 风暴” */
      }
      app_tick_20ms(now);
    }

    if ((g_state != APP_ST_DISABLED) &&
        ((HAL_GetTick() - t_status) >= APP_STATUS_MS))
    {
      t_status = HAL_GetTick();
      app_print_status();
    }
  }
}

/* ==========================================================================
 *  附：命令/状态一览（与实现同步维护，改代码时一起改）
 *
 *  命令        作用
 *  1..4        选轮 M1..M4（占空比归 0，RUNNING 时先回 ARMED）
 *  f / r       方向 FWD / REV（PWM 先归 0 + 5 ms 死区，占空比归 0）
 *  + / -       占空比 ±5%，范围 0..OMNINAV_APP_DUTY_MAX
 *  g           使能（STBY=1，PWM=0 制动待命）
 *  k           心跳；RUNNING 下必须每 <= 200 ms 至少一条有效命令
 *  b           刹车（STBY=1，PWM=0，短路制动）
 *  s           停止 -> DISABLED（STBY=0，PWM=0，高阻滑行）
 *  p / ?       打印状态 / 帮助
 *
 *  状态机：DISABLED --g--> ARMED --duty>0 且心跳新鲜--> RUNNING
 *          RUNNING --超时--> DISABLED（锁存，需 'g' 重新使能）
 * ========================================================================== */
