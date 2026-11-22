/**
  ******************************************************************************
  * @file    Src/bench.c
  * @brief   外设层台架测试：引脚探针 + 单电机开环驱动
  *
  *  ── 信号链 ────────────────────────────────────────────────────────────────
  *   PE9  ── TIM1_CH1（AF 推挽，20 kHz PWM） ──► TB6612 `PWMA`   调速
  *   PE7  ── 普通 GPIO 输出 ────────────────────► TB6612 `AIN1`   方向位 1
  *   PE8  ── 普通 GPIO 输出 ────────────────────► TB6612 `AIN2`   方向位 0
  *   PE15 ── 普通 GPIO 输出 ────────────────────► TB6612 `STBY`   置高才工作
  *   AO1/AO2 ──────────────────────────────────► 电机 白线(M+) / 红线(M−)
  *
  *  ★ TIM1 走【完全重映射】（AFIO_MAPR.TIM1_REMAP = 11）：
  *         ETR=PE7  CH1=PE9  CH2=PE11  CH3=PE13  CH4=PE14
  *         BKIN=PE15  CH1N=PE8  CH2N=PE10  CH3N=PE12
  *    重映射只是把"复用功能"挪到 PE 口上，**引脚不会自动变成复用**——
  *    只有被你使能的那个通道才接管引脚。所以 PE7/PE8/PE15 依旧是普通 GPIO。
  *       · PE7  = TIM1_ETR，ETR 要 SMCR.ECE=1 才生效 ⇒ 默认是 GPIO ✓
  *       · PE8  = TIM1_CH1N，互补输出要 CCER.CC1NE=1 才生效 ⇒ 默认是 GPIO ✓
  *       · PE15 = TIM1_BKIN，刹车输入要 BDTR.BKE=1 才生效 ⇒ 默认是 GPIO ✓
  *
  *  ── TB6612 真值表（Toshiba 数据手册，务必按这个记）───────────────────────
  *   STBY=0                      → 待机，全桥高阻（**惰行/滑行**，唯一的"松"状态）
  *   STBY=1, IN1=1, IN2=0, PWM>0 → 正转
  *   STBY=1, IN1=0, IN2=1, PWM>0 → 反转
  *   STBY=1, PWM=0（IN1≠IN2）    → **短路制动（刹车）** ← 不是滑行！
  *   STBY=1, IN1=IN2（0 或 1）   → **短路制动（刹车）**
  *   ★ 所以：想让车"停下来"用 PWM=0 / IN1=IN2=0（刹得住）；
  *          想让车"自由滑行"**必须把 STBY 拉低**（急停走这一条）。
  *
  *  ── ADC 型的"占空比表"────────────────────────────────────────────────────
  *   万用表直流电压档量 PWM 引脚，读数是平均值 ≈ 3.3 V × 占空比。
  *   20 kHz 远高于表头带宽，读数稳定。这正好是"不用示波器验证 PWM"的办法。
  ******************************************************************************
  */

#include "bench.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern void uart_printf(const char *fmt, ...);   /* 定义在 main.c，自研实现，不支持 %f */

/* Private defines -----------------------------------------------------------*/
#define PROBE_HALF_MS     250u   /* 250 ms 翻转一次 = 2 Hz 方波，表针看得清 */
/* ★ 探针【不自动推进】：每个脚一直翻转，直到你在串口发一个字符才换下一个。
 *   原因：13 个脚 × 1.5 s = 20 s，人根本来不及用万用表逐个点完。
 *   操作：任意字符 = 下一个脚，'p' = 上一个脚，'q' = 跳过剩下的直接进阶段 C。 */
#define PWM_ARR          3599u   /* 72 MHz / (0+1) / (3599+1) = 20.000 kHz */
#define DUTY_STEP           5u   /* 每次 +/- 调 5% */
#define DUTY_MAX           90u   /* 封顶 90%，留余量 */
#define STATUS_MS         500u

/* Private types -------------------------------------------------------------*/
typedef struct
{
  const char   *mcu;    /* MCU 引脚名 */
  const char   *silk;   /* TFT_LCD 排座上的丝印 */
  const char   *pos;    /* 在排座上的物理位置（手工定位用） */
  GPIO_TypeDef *port;
  uint16_t      pin;
} probe_pin_t;

/* Private variables ---------------------------------------------------------*/
static TIM_HandleTypeDef htim1;

/* ★ 顺序 = PE7..PE15 再 PD0/PD1/PD8/PD9；
 *   silk 列是"排座上要去找的丝印"，依据是 FSMC 数据线固定映射：
 *   FSMC_D4..D12 = PE7..PE15（⇒ 丝印 D4..D12），FSMC_D2/D3 = PD0/PD1（丝印 D2/D3），
 *   FSMC_D13/D14 = PD8/PD9（丝印 D13/D14）。用探针实测来最终确认。 */
static const probe_pin_t g_probe[] =
{
  { "PE7",  "D4",  "UPPER row, 5th from left",  GPIOE, GPIO_PIN_7  },
  { "PE8",  "D5",  "LOWER row, 6th from left",  GPIOE, GPIO_PIN_8  },
  { "PE9",  "D6",  "UPPER row, 6th from left",  GPIOE, GPIO_PIN_9  },
  { "PE10", "D7",  "LOWER row, 7th from left",  GPIOE, GPIO_PIN_10 },
  { "PE11", "D8",  "UPPER row, 7th from left",  GPIOE, GPIO_PIN_11 },
  { "PE12", "D9",  "LOWER row, 8th from left",  GPIOE, GPIO_PIN_12 },
  { "PE13", "D10", "UPPER row, 8th from left",  GPIOE, GPIO_PIN_13 },
  { "PE14", "D11", "LOWER row, 9th from left",  GPIOE, GPIO_PIN_14 },
  { "PE15", "D12", "UPPER row, 9th from left",  GPIOE, GPIO_PIN_15 },
  { "PD0",  "D2",  "UPPER row, 4th from left",  GPIOD, GPIO_PIN_0  },
  { "PD1",  "D3",  "LOWER row, 5th from left",  GPIOD, GPIO_PIN_1  },
  { "PD8",  "D13", "LOWER row, 10th from left", GPIOD, GPIO_PIN_8  },
  { "PD9",  "D14", "UPPER row, 10th from left", GPIOD, GPIO_PIN_9  }
};
#define PROBE_N (sizeof(g_probe) / sizeof(g_probe[0]))

static uint8_t g_duty;      /* 0..100 % */
static uint8_t g_fwd;       /* 1 = 正转 */
static uint8_t g_stby;      /* 1 = TB6612 使能 */
static uint8_t g_brake;     /* 1 = 两个 IN 同时拉高（刹车） */

/* Private function prototypes -----------------------------------------------*/
static void stage_a_heartbeat(void);
static void stage_b_probe(void);
static void stage_c_motor(void);
static void motor_init(void);
static void motor_apply(void);
static void motor_print_status(void);
static void motor_help(void);
static int  uart_getc(char *c);

/* ==========================================================================
 *  入口
 * ========================================================================== */

void bench_run(void)
{
  stage_a_heartbeat();
  stage_b_probe();
  stage_c_motor();

  /* 理论到不了这里 */
  while (1)
  {
  }
}

/* ==========================================================================
 *  阶段 A：心跳
 * ========================================================================== */

static void stage_a_heartbeat(void)
{
  uint32_t i;
  uint32_t t0;
  uint32_t dtk;

  uart_printf("\r\n");
  uart_printf("===== [A] HEARTBEAT =====\r\n");
  uart_printf("LED0=PB5  LED1=PE5  should blink 2 Hz for 4 s.\r\n");

  t0 = HAL_GetTick();
  for (i = 0u; i < 8u; i++)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
    HAL_Delay(250u);
  }

  dtk = HAL_GetTick() - t0;
  uart_printf("[A] 8 x HAL_Delay(250) took %lu ms (expect ~2000)\r\n",
              (unsigned long)dtk);
  uart_printf("[A] %s\r\n", (dtk >= 1900u) && (dtk <= 2100u) ? "timebase OK" : "TIMEBASE BAD");
}

/* ==========================================================================
 *  阶段 B：引脚探针
 * ========================================================================== */

static void stage_b_probe(void)
{
  GPIO_InitTypeDef g = {0};
  uint32_t i;
  uint32_t next_toggle;
  char     c;

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;

  for (i = 0u; i < PROBE_N; i++)
  {
    g.Pin = g_probe[i].pin;
    HAL_GPIO_Init(g_probe[i].port, &g);
    HAL_GPIO_WritePin(g_probe[i].port, g_probe[i].pin, GPIO_PIN_RESET);
  }

  uart_printf("\r\n");
  uart_printf("===== [B] PIN PROBE (key driven) =====\r\n");
  uart_printf("Multimeter: DC volt, black probe on board GND.\r\n");
  uart_printf("The pin named below flips 0 <-> 3.3V at 2 Hz and keeps going\r\n");
  uart_printf("until you send a character. %u pins total.\r\n", (unsigned)PROBE_N);
  uart_printf("Keys: any char = next pin   'p' = previous   'q' = skip rest\r\n");
  uart_printf("TFT_LCD header (34 pins, 2 rows of 17), left -> right:\r\n");
  uart_printf("  UPPER: RS RD D0 D2 D4 D6 D8 D10 D12 D14 GND VDD GND VDD MOSI MO CLK\r\n");
  uart_printf("  LOWER: CS WR RST D1 D3 D5 D7 D9 D11 D13 D15 BL VDD GND MISO PEN TCS\r\n");

  i = 0u;
  while (i < PROBE_N)
  {
    uart_printf("\r\n[B] (%u/%u) >>> toggling %s   TFT_LCD silk = %s   [%s] <<<\r\n",
                (unsigned)(i + 1u), (unsigned)PROBE_N,
                g_probe[i].mcu, g_probe[i].silk, g_probe[i].pos);
    uart_printf("[B]     send any char for next, 'p' previous, 'q' skip rest\r\n");

    /* 一直翻转这个脚，直到收到一个字符（回车换行不算） */
    next_toggle = HAL_GetTick();
    for (;;)
    {
      if ((HAL_GetTick() - next_toggle) >= PROBE_HALF_MS)
      {
        next_toggle = HAL_GetTick();
        HAL_GPIO_TogglePin(g_probe[i].port, g_probe[i].pin);
      }

      if (uart_getc(&c) != 0)
      {
        if ((c == '\r') || (c == '\n'))
        {
          continue;
        }
        break;
      }
    }

    /* 离开这个脚前先拉低，避免两个脚同时跳把人看花 */
    HAL_GPIO_WritePin(g_probe[i].port, g_probe[i].pin, GPIO_PIN_RESET);

    if (c == 'q')
    {
      uart_printf("[B] skip rest, go to stage C.\r\n");
      break;
    }
    else if (c == 'p')
    {
      if (i > 0u)
      {
        i--;
      }
      else
      {
        uart_printf("[B] already at the first pin.\r\n");
      }
    }
    else
    {
      i++;
    }
  }

  /* 收尾：所有探针脚拉低，交还干净状态给阶段 C */
  for (i = 0u; i < PROBE_N; i++)
  {
    HAL_GPIO_WritePin(g_probe[i].port, g_probe[i].pin, GPIO_PIN_RESET);
  }

  uart_printf("\r\n[B] probe finished. Note any mismatch between MCU name and silk.\r\n");
  uart_printf("[B] e.g. \"PE9 did NOT show up on the pin marked D6\".\r\n");
}

/* ==========================================================================
 *  阶段 C：单电机开环驱动
 * ========================================================================== */

static void motor_init(void)
{
  GPIO_InitTypeDef        g  = {0};
  TIM_OC_InitTypeDef      oc = {0};
  TIM_MasterConfigTypeDef mc = {0};

  /* AFIO 时钟必须先开，重映射寄存器才写得进去 */
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();

  /* TIM1 完全重映射：CH1=PE9 CH2=PE11 CH3=PE13 CH4=PE14 */
  __HAL_AFIO_REMAP_TIM1_ENABLE();

  /* PE9 = TIM1_CH1，复用推挽 */
  g.Pin   = GPIO_PIN_9;
  g.Mode  = GPIO_MODE_AF_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &g);

  /* PE7 = AIN1 / PE8 = AIN2 / PE15 = STBY，普通推挽输出 */
  g.Mode   = GPIO_MODE_OUTPUT_PP;
  g.Speed  = GPIO_SPEED_FREQ_LOW;
  g.Pin    = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &g);

  /* 上电先让 TB6612 处于待机，电机不通电 */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);

  htim1.Instance               = TIM1;
  htim1.Init.Prescaler         = 0u;
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1.Init.Period            = PWM_ARR;   /* 20 kHz */
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0u;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  (void)HAL_TIM_PWM_Init(&htim1);

  oc.OCMode       = TIM_OCMODE_PWM1;
  oc.Pulse        = 0u;
  oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode   = TIM_OCFAST_DISABLE;
  oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  (void)HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);

  mc.MasterOutputTrigger = TIM_TRGO_RESET;
  mc.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  (void)HAL_TIMEx_MasterConfigSynchronization(&htim1, &mc);

  /* ★ 配置完必须显式 Start，否则 GPIO 上永远没有波形（最经典的坑） */
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  g_duty  = 0u;
  g_fwd   = 1u;
  g_stby  = 0u;
  g_brake = 0u;
  motor_apply();
}

static void motor_apply(void)
{
  GPIO_PinState in1;
  GPIO_PinState in2;

  if (g_stby == 0u)
  {
    /* 待机：TB6612 全桥高阻，方向位给什么都不动，但保持 0 更省心 */
    in1 = GPIO_PIN_RESET;
    in2 = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);

    if (g_brake != 0u)
    {
      in1 = GPIO_PIN_SET;       /* 两个 IN 同时高 ⇒ 短路制动 */
      in2 = GPIO_PIN_SET;
    }
    else if (g_fwd != 0u)
    {
      in1 = GPIO_PIN_SET;
      in2 = GPIO_PIN_RESET;
    }
    else
    {
      in1 = GPIO_PIN_RESET;
      in2 = GPIO_PIN_SET;
    }
  }

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, in1);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, in2);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ((uint32_t)g_duty * PWM_ARR) / 100u);
}

static void motor_print_status(void)
{
  uart_printf("[C] stby=%u dir=%s brake=%u duty=%u%%  (%s)\r\n",
              (unsigned)g_stby,
              (g_fwd != 0u) ? "FWD" : "REV",
              (unsigned)g_brake,
              (unsigned)g_duty,
              (g_stby == 0u) ? "STANDBY / coast" : "ENABLED");
}

static void motor_help(void)
{
  uart_printf("\r\n");
  uart_printf("[C] Single-motor open-loop test. Commands (single char):\r\n");
  uart_printf("      g  go      : STBY high, motor enabled\r\n");
  uart_printf("      s  stop    : duty 0 + STBY low (coast / safe)\r\n");
  uart_printf("      +  duty +5%%      -  duty -5%%\r\n");
  uart_printf("      f  forward        r  reverse\r\n");
  uart_printf("      b  brake (both IN high)\r\n");
  uart_printf("      ?  this help\r\n");
  uart_printf("    Motor only moves after 'g' AND '+'. Nothing moves on power-up.\r\n\r\n");
}

static void stage_c_motor(void)
{
  uint32_t t_status;

  uart_printf("\r\n");
  uart_printf("===== [C] SINGLE MOTOR =====\r\n");
  uart_printf("PE9=TIM1_CH1 PWM 20kHz (silk D6)  PE7=AIN1 (D4)  PE8=AIN2 (D5)  PE15=STBY (D12)\r\n");

  /*
   * ── 接线（TB6612 A 路）────────────────────────────────────────────────
   *   TB6612 VM   -> 电池正极（经总开关）        ★ 绝不接开发板 5V
   *   TB6612 VCC  -> 精英板 3.3V
   *   TB6612 GND  -> 精英板 GND（★ 必须与电池共地）
   *   TB6612 PWMA -> PE9   (TFT_LCD 座丝印 D6)
   *   TB6612 AIN1 -> PE7   (丝印 D4)
   *   TB6612 AIN2 -> PE8   (丝印 D5)
   *   TB6612 STBY -> PE15  (丝印 D12)
   *   TB6612 AO1  -> 电机白线 (M+)
   *   TB6612 AO2  -> 电机红线 (M-)
   *   VM 与 GND 之间并 100~470uF 电解 + 0.1uF 陶瓷
   * ────────────────────────────────────────────────────────────────────
   */

  motor_init();
  motor_help();
  motor_print_status();

  t_status = HAL_GetTick();

  while (1)
  {
    char c;

    if (uart_getc(&c) != 0)
    {
      switch (c)
      {
        case 'g':
        {
          g_stby  = 1u;
          g_brake = 0u;
          motor_apply();
          uart_printf("[C] GO\r\n");
          break;
        }
        case 's':
        {
          g_duty  = 0u;
          g_stby  = 0u;
          g_brake = 0u;
          motor_apply();
          uart_printf("[C] STOP (coast)\r\n");
          break;
        }
        case '+':
        {
          if ((uint32_t)g_duty + DUTY_STEP <= DUTY_MAX)
          {
            g_duty = (uint8_t)(g_duty + DUTY_STEP);
          }
          g_brake = 0u;
          motor_apply();
          break;
        }
        case '-':
        {
          if (g_duty >= DUTY_STEP)
          {
            g_duty = (uint8_t)(g_duty - DUTY_STEP);
          }
          else
          {
            g_duty = 0u;
          }
          motor_apply();
          break;
        }
        case 'f':
        {
          g_fwd = 1u;
          g_brake = 0u;
          motor_apply();
          break;
        }
        case 'r':
        {
          g_fwd = 0u;
          g_brake = 0u;
          motor_apply();
          break;
        }
        case 'b':
        {
          g_brake = 1u;
          g_duty  = 0u;
          motor_apply();
          uart_printf("[C] BRAKE\r\n");
          break;
        }
        case '?':
        {
          motor_help();
          break;
        }
        case '\r':
        case '\n':
        {
          break;
        }
        default:
        {
          uart_printf("[C] unknown '%c' (try '?')\r\n", c);
          break;
        }
      }
    }

    if ((HAL_GetTick() - t_status) >= STATUS_MS)
    {
      t_status = HAL_GetTick();
      motor_print_status();
    }
  }
}

/**
  * @brief  非阻塞读一个串口字节（轮询，不占用中断向量）
  * @retval 1 = 读到；0 = 没有
  */
static int uart_getc(char *c)
{
  /* 溢出错误要先清掉，否则 RXNE 再也不置位 */
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET)
  {
    volatile uint32_t tmp;
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
