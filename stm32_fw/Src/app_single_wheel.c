/**
  ******************************************************************************
  * @file    Src/app_single_wheel.c
  * @brief   A-02 minimum safe single-wheel firmware: the smallest code set that
  *          first makes one wheel spin safely and under control
  *
  *  Safety behaviour (task-5 hard requirements, all mandatory):
  *    1) Disabled by default at boot: STBY=0, PWM=0, IN1=IN2=0 (high-Z coast, wheel
  *       does not turn)
  *    2) Output limiting: duty cap OMNINAV_APP_DUTY_MAX (default 50%), key step 5%
  *    3) Timeout stop: in RUNNING, if no valid command arrives within
  *       OMNINAV_CMD_TIMEOUT_MS (default 200 ms) -> revoke output immediately, return
  *       to DISABLED, and print the measured "last valid command -> revoke" delta (so
  *       A-05 can pin the threshold down)
  *    4) Single-wheel debug entry: single-character commands on USART1 (PC debug
  *       port) to select wheel / direction / duty / stop
  *       No new frozen protocol frame is added; the frozen USART2 protocol port
  *       (PA2/PA3) is not touched at all
  *    5) 20 ms control period skeleton app_tick_20ms(), aligned with the 20 ms speed
  *       loop of plan A-3
  *
  *  Relationship to bench.c stage C: bench.c is the accepted baseline (USART2
  *  11/11); this file is an independent implementation and changes not one line of
  *  bench.c / usart2_bench.c / core_selftest.c. The motor init sequence matches
  *  bench.c stage C (that sequence is proven on real hardware) and is expanded to
  *  4 PWM channels + 8 direction pins (only the selected wheel is driven at a time).
  *
  *  Minimum stop path (must hold before the first non-zero output):
  *      DISABLED (STBY=0) is the power-up default; timeout / 's' / any fault returns
  *      to DISABLED, and an explicit 'g' is required to re-enable (latched
  *      semantics, prevents "chatter re-enables it").
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_single_wheel.h"

#include <stdint.h>

/* Private defines -----------------------------------------------------------*/
#ifndef OMNINAV_CMD_TIMEOUT_MS
#define OMNINAV_CMD_TIMEOUT_MS 200u     /* task-5 interim default 200; A-05 may tune 150-250 after measuring */
#endif
#ifndef OMNINAV_APP_DUTY_MAX
#define OMNINAV_APP_DUTY_MAX 50u        /* output limit: 50% by default (macro adjustable, A-02 starts conservative) */
#endif

#define APP_VERSION          "v0.1"
#define APP_DUTY_STEP        5u         /* key duty step 5% */
#define APP_CTRL_PERIOD_MS   20u        /* 20 ms scheduling skeleton */
#define APP_STATUS_MS        500u       /* periodic status print */
#define APP_PWM_ARR          3599u      /* 72MHz/(0+1)/(3599+1) = 20.000 kHz, same as bench.c */
#define APP_DIR_SETTLE_MS    5u         /* direction dead-time: output only after 5 ms of PWM=0 */

/* Direction pins and PWM pins (06 section 5.1.1 TFT_LCD silkscreen table; always trust the silkscreen, never count holes) */
#define APP_DIR_PINS_E      ((uint16_t)(GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_12))
#define APP_DIR_PINS_D      ((uint16_t)(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9))
#define APP_PWM_PINS_E      ((uint16_t)(GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14))
#define APP_STBY_PIN        GPIO_PIN_15

/* Private types -------------------------------------------------------------*/
typedef struct
{
  GPIO_TypeDef *port;    /* port the direction pins live on */
  uint16_t      in1;     /* AIN1 / BIN1 */
  uint16_t      in2;     /* AIN2 / BIN2 */
  uint32_t      ch;      /* TIM1 channel */
  const char   *name;    /* silkscreen M1..M4 */
} app_wheel_t;

typedef enum
{
  APP_ST_DISABLED = 0,   /* STBY=0 / PWM=0 / IN=0: high-Z coast, power-up default */
  APP_ST_ARMED    = 1,   /* STBY=1 / PWM=0: short brake, waiting for heartbeat */
  APP_ST_RUNNING  = 2    /* STBY=1 / PWM=duty: supervised by heartbeat, revoked on timeout */
} app_state_t;

/* Private variables ---------------------------------------------------------*/
/*
 * Wheel order = M1..M4; pins follow the 06 section 5.1.1 TFT_LCD silkscreen table:
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

static app_state_t g_state;              /* current safety state */
static uint8_t     g_wheel;              /* 0..3 = M1..M4 */
static uint8_t     g_fwd;                /* 1 = FWD, 0 = REV */
static uint8_t     g_duty;               /* 0..OMNINAV_APP_DUTY_MAX (%) */
static uint32_t    g_last_cmd_tick;      /* HAL tick of the last valid command */
static char        g_last_cmd;           /* last valid command character */
static uint32_t    g_dir_block_until;    /* direction dead-time: no output before this tick */
static uint32_t    g_last_revoke_ms;     /* measured latency of the last revoke: last valid cmd -> revoke */

/* External symbols: provided by main.c */
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
 *  Print layer (everything goes through main.c's uart_printf; no stdio, no %f)
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
 *  Output layer: the only allowed order is "PWM=0 first, then change direction, then apply duty"
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

/* Safe state: STBY=0 -> TB6612 full bridge high-Z (coast, not brake), PWM=0, IN=0 */
static void app_output_disabled(void)
{
  app_pwm_all(0u);
  app_dir_all_off();
  HAL_GPIO_WritePin(GPIOE, APP_STBY_PIN, GPIO_PIN_RESET);
}

/* Armed state: STBY=1 + PWM=0 -> short-circuit brake (braking), the wheel cannot be pushed into turning */
static void app_output_armed(void)
{
  app_pwm_all(0u);
  app_dir_all_off();
  HAL_GPIO_WritePin(GPIOE, APP_STBY_PIN, GPIO_PIN_SET);
}

/* Running state: only the selected wheel gets direction + duty, the other three are forced to 0 */
static void app_output_running(void)
{
  const app_wheel_t *w;
  uint32_t ccr;

  w   = &k_wheel[g_wheel];
  ccr = ((uint32_t)g_duty * (APP_PWM_ARR + 1u)) / 100u;

  app_pwm_all(0u);        /* direction/start dead-time: drop PWM first */
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
 *  Init (same order as bench.c stage C; that order is proven on real hardware)
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

  /* The AFIO clock must be enabled first, or the remap register cannot be written */
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();

  /* TIM1 full remap: CH1=PE9 CH2=PE11 CH3=PE13 CH4=PE14 */
  __HAL_AFIO_REMAP_TIM1_ENABLE();

  /* Write the direction pins + STBY low first, then configure them as outputs: avoids a glitch at power-up */
  HAL_GPIO_WritePin(GPIOE, (uint16_t)(APP_DIR_PINS_E | APP_STBY_PIN), GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, APP_DIR_PINS_D, GPIO_PIN_RESET);

  /* Four PWM pins: alternate-function push-pull */
  g.Pin   = APP_PWM_PINS_E;
  g.Mode  = GPIO_MODE_AF_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &g);

  /* Eight direction pins + STBY(PE15): plain push-pull outputs */
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

  /* Must be started explicitly after configuration, otherwise there is never a
   * waveform on the GPIO (pitfall from 06 section 10-15) */
  (void)HAL_TIM_PWM_Start(&htim1_app, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim1_app, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Start(&htim1_app, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Start(&htim1_app, TIM_CHANNEL_4);

  /* Explicitly return to the safe state: STBY=0 / PWM=0 / IN=0 */
  app_output_disabled();
}

/* ==========================================================================
 *  USART1 non-blocking byte receive (polled, no interrupt vector; same approach
 *  as uart_getc in bench.c)
 * ========================================================================== */

static int app_getc(char *c)
{
  volatile uint32_t tmp;

  /* Clear an overrun error first, otherwise RXNE never gets set again */
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
 *  State machine
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
      /* Force the safe state every 20 ms so no stray output can survive */
      app_output_disabled();
      break;

    case APP_ST_ARMED:
      app_output_armed();
      if (g_duty == 0u)
      {
        break;                                  /* duty 0: do not start */
      }
      if (age >= OMNINAV_CMD_TIMEOUT_MS)
      {
        break;                                  /* heartbeat stale: do not start */
      }
      if ((int32_t)(now - g_dir_block_until) < 0)
      {
        break;                                  /* direction dead-time not elapsed: do not start */
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
 *  Command handling (single characters on USART1; no protocol frame is added)
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
      g_duty            = 0u;   /* changing direction must zero PWM; duty is zeroed too so it cannot jump */
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
      /* Heartbeat: only refreshes the timestamp; the 20 ms tick decides whether RUNNING survives */
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
 *  Entry: init -> banner -> main loop (commands + 20 ms tick + periodic status)
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
        t_ctrl = now;   /* avoid a "catch-up tick storm" after a long block */
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
 *  Appendix: command/state overview (keep in sync with the implementation)
 *
 *  command      action
 *  1..4        select wheel M1..M4 (duty reset to 0; when RUNNING, drop to ARMED first)
 *  f / r       direction FWD / REV (PWM to 0 first + 5 ms dead-time, duty reset to 0)
 *  + / -       duty +/-5%, range 0..OMNINAV_APP_DUTY_MAX
 *  g           arm (STBY=1, PWM=0 brake, standing by)
 *  k           heartbeat; while RUNNING at least one valid command every <= 200 ms
 *  b           brake (STBY=1, PWM=0, short-circuit brake)
 *  s           stop -> DISABLED (STBY=0, PWM=0, high-Z coast)
 *  p / ?       print status / help
 *
 *  State machine: DISABLED --g--> ARMED --duty>0 and heartbeat fresh--> RUNNING
 *                 RUNNING --timeout--> DISABLED (latched, needs 'g' to re-enable)
 * ========================================================================== */
