/**
  ******************************************************************************
  * @file    Src/bench.c
  * @brief   Peripheral-layer bench test: pin probe + single-motor open-loop drive
  *
  *  -- Signal chain ---------------------------------------------------------
  *   PE9  -- TIM1_CH1 (AF push-pull, 20 kHz PWM) --> TB6612 `PWMA`   speed
  *   PE7  -- plain GPIO output ------------------> TB6612 `AIN1`   direction bit 1
  *   PE8  -- plain GPIO output ------------------> TB6612 `AIN2`   direction bit 0
  *   PE15 -- plain GPIO output ------------------> TB6612 `STBY`   high to enable
  *   AO1/AO2 -----------------------------------> motor white (M+) / red (M-)
  *
  *  NOTE: TIM1 uses FULL remap (AFIO_MAPR.TIM1_REMAP = 11):
  *         ETR=PE7  CH1=PE9  CH2=PE11  CH3=PE13  CH4=PE14
  *         BKIN=PE15  CH1N=PE8  CH2N=PE10  CH3N=PE12
  *    Remapping only moves the "alternate function" onto port E; the pin does
  *    NOT become alternate-function automatically -- only the channel you
  *    enable takes over the pin. So PE7/PE8/PE15 stay plain GPIO:
  *       . PE7  = TIM1_ETR, ETR needs SMCR.ECE=1 to take effect => plain GPIO
  *       . PE8  = TIM1_CH1N, complementary output needs CCER.CC1NE=1 =>
  *                plain GPIO
  *       . PE15 = TIM1_BKIN, brake input needs BDTR.BKE=1 => plain GPIO
  *
  *  -- TB6612 truth table (Toshiba datasheet; memorize this) --------------
  *   STBY=0                      -> standby, full bridge high-Z (**coast**,
  *                                  the only "released" state)
  *   STBY=1, IN1=1, IN2=0, PWM>0 -> forward
  *   STBY=1, IN1=0, IN2=1, PWM>0 -> reverse
  *   STBY=1, PWM=0 (IN1!=IN2)    -> **short-circuit brake** <- NOT coasting!
  *   STBY=1, IN1=IN2 (0 or 1)    -> **short-circuit brake**
  *   NOTE: to make the car "stop", use PWM=0 / IN1=IN2=0 (it brakes);
  *         to make it "coast freely" you **must pull STBY low** (use this for
  *         an emergency stop).
  *
  *  -- ADC-style "duty table" ----------------------------------------------
  *   With a multimeter on DC volts across the PWM pin, the reading is the
  *   average ~ 3.3 V x duty. 20 kHz is far above the meter bandwidth, so the
  *   reading is stable. This is exactly how to verify PWM without a scope.
  ******************************************************************************
  */

#include "bench.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern void uart_printf(const char *fmt, ...);   /* Defined in main.c, custom implementation, no %f support */

/* Private defines -----------------------------------------------------------*/
#define PROBE_HALF_MS     250u   /* Toggle every 250 ms = 2 Hz square wave, easy to read on the needle */
/* NOTE: the probe does [not auto-advance]: each pin keeps toggling until you
 *   send one character on the UART. Reason: 13 pins x 1.5 s = 20 s, far too
 *   fast to touch them all with a multimeter.
 *   Controls: any char = next pin, 'p' = previous pin, 'q' = skip the rest and
 *   go straight to stage C. */
#define PWM_ARR          3599u   /* 72 MHz / (0+1) / (3599+1) = 20.000 kHz */
#define DUTY_STEP           5u   /* +/- 5% per press */
#define DUTY_MAX           90u   /* cap at 90%, keeps some margin */
#define STATUS_MS         500u

/* Private types -------------------------------------------------------------*/
typedef struct
{
  const char   *mcu;    /* MCU pin name */
  const char   *silk;   /* silkscreen on the TFT_LCD header */
  const char   *pos;    /* physical position on the header (for manual location) */
  GPIO_TypeDef *port;
  uint16_t      pin;
} probe_pin_t;

/* Private variables ---------------------------------------------------------*/
static TIM_HandleTypeDef htim1;

/* NOTE: order = PE7..PE15 then PD0/PD1/PD8/PD9;
 *   the silk column is "the silkscreen to look for on the header", based on the
 *   fixed FSMC data-line mapping:
 *   FSMC_D4..D12 = PE7..PE15 (=> silk D4..D12), FSMC_D2/D3 = PD0/PD1 (silk D2/D3),
 *   FSMC_D13/D14 = PD8/PD9 (silk D13/D14). The probe measurement confirms it. */
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
static uint8_t g_fwd;       /* 1 = forward */
static uint8_t g_stby;      /* 1 = TB6612 enabled */
static uint8_t g_brake;     /* 1 = both IN pulled high (brake) */

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
 *  Entry
 * ========================================================================== */

void bench_run(void)
{
  stage_a_heartbeat();
  stage_b_probe();
  stage_c_motor();

  /* Should never get here */
  while (1)
  {
  }
}

/* ==========================================================================
 *  Stage A: heartbeat
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
 *  Stage B: pin probe
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

    /* Keep toggling this pin until a character arrives (CR/LF ignored) */
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

    /* Pull it low before leaving, so two pins never toggle at once and blur the reading */
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

  /* Wrap up: pull all probe pins low and hand a clean state to stage C */
  for (i = 0u; i < PROBE_N; i++)
  {
    HAL_GPIO_WritePin(g_probe[i].port, g_probe[i].pin, GPIO_PIN_RESET);
  }

  uart_printf("\r\n[B] probe finished. Note any mismatch between MCU name and silk.\r\n");
  uart_printf("[B] e.g. \"PE9 did NOT show up on the pin marked D6\".\r\n");
}

/* ==========================================================================
 *  Stage C: single-motor open-loop drive
 * ========================================================================== */

static void motor_init(void)
{
  GPIO_InitTypeDef        g  = {0};
  TIM_OC_InitTypeDef      oc = {0};
  TIM_MasterConfigTypeDef mc = {0};

  /* The AFIO clock must be enabled first, or the remap register cannot be written */
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();

  /* TIM1 full remap: CH1=PE9 CH2=PE11 CH3=PE13 CH4=PE14 */
  __HAL_AFIO_REMAP_TIM1_ENABLE();

  /* PE9 = TIM1_CH1, alternate-function push-pull */
  g.Pin   = GPIO_PIN_9;
  g.Mode  = GPIO_MODE_AF_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &g);

  /* PE7 = AIN1 / PE8 = AIN2 / PE15 = STBY, plain push-pull outputs */
  g.Mode   = GPIO_MODE_OUTPUT_PP;
  g.Speed  = GPIO_SPEED_FREQ_LOW;
  g.Pin    = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &g);

  /* Start with TB6612 in standby, motor unpowered */
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

  /* NOTE: PWM must be started explicitly after configuration, otherwise there
   * is never a waveform on the pin (the classic pitfall) */
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
    /* Standby: TB6612 full bridge is high-Z, it does not move whatever the
     * direction bits are, but holding them 0 is cleaner */
    in1 = GPIO_PIN_RESET;
    in2 = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);

    if (g_brake != 0u)
    {
      in1 = GPIO_PIN_SET;       /* Both IN high => short-circuit brake */
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
   * -- Wiring (TB6612 channel A) ----------------------------------------
   *   TB6612 VM   -> battery positive (through the main switch)
   *                  NOTE: never connect this to the dev board 5V
   *   TB6612 VCC  -> Elite board 3.3V
   *   TB6612 GND  -> Elite board GND (NOTE: must share ground with battery)
   *   TB6612 PWMA -> PE9   (TFT_LCD header silk D6)
   *   TB6612 AIN1 -> PE7   (silk D4)
   *   TB6612 AIN2 -> PE8   (silk D5)
   *   TB6612 STBY -> PE15  (silk D12)
   *   TB6612 AO1  -> motor white wire (M+)
   *   TB6612 AO2  -> motor red wire (M-)
   *   Add 100~470uF electrolytic + 0.1uF ceramic between VM and GND
   * --------------------------------------------------------------------
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
  * @brief  Non-blocking read of one UART byte (polling, no interrupt vector)
  * @retval 1 = received; 0 = nothing
  */
static int uart_getc(char *c)
{
  /* Clear an overrun error first, otherwise RXNE never gets set again */
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
