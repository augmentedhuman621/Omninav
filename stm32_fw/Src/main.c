/**
  ******************************************************************************
  * @file    Src/main.c
  * @brief   OmniNav STM32 phase-0 acceptance program
  *
  *          Target board: ALIENTEK STM32F103ZE Elite board (STM32F103ZET6,
  *          512K Flash / 64K SRAM)
  *          LED0 = PB5, LED1 = PE5 (on-board LEDs are active low)
  *          UART = USART1 (PA9=TX / PA10=RX) 115200 8N1, on-board CH340C ->
  *          COM9 on the Windows side
  *
  *          Acceptance criteria:
  *            1) The two LEDs blink at 1 Hz (toggle every 500 ms)
  *            2) One UART line every 2 s, with the took field steady at 501
  ******************************************************************************
  * NOTE: This file does not use the standard printf -- with MicroLIB disabled,
  *   printf triggers semihosting, and the program hangs as soon as it enters
  *   main. All output goes through uart_printf() below.
  * NOTE: vsnprintf is not used either: format strings are parsed by our own
  *   implementation, which saves library code and keeps the C library's
  *   libspace / heap initialization out of the picture.
  * NOTE: %f is forbidden: the protocol and logs use fixed-point integers only,
  *   so the floating-point library is never pulled into the firmware.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "core_selftest.h"
#include "bench.h"
#include "usart2_bench.h"   /* Power-on default entry: USART2 link self-test (PA2/PA3 -> Raspberry Pi 5) */
#include "app_single_wheel.h"   /* A-02: compile-time switch OMNINAV_APP_SINGLE_WHEEL (default 0 = keep original behavior) */

#include <stdarg.h>

/* Private defines -----------------------------------------------------------*/
#define TXBUF_SZ 128u

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

static char     g_tx_buf[TXBUF_SZ];
static uint16_t g_tx_len;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void timebase_repair(void);
static int  boot_wait_motor_key(uint32_t ms);
static void tx_flush(void);
static void tx_putc(char c);
static void tx_puts(const char *s);
static void tx_putn(uint32_t mag, int neg, uint32_t base, uint32_t width, char pad, int upper);

void uart_printf(const char *fmt, ...);

/* --------------------------------------------------------------------------
 * Timebase repair (located and measured on the real board on 2026-10-07)
 *
 *   Symptom: HAL_Delay(500) took only about 3 ms (measured took=762), and
 *            HAL_GetTick() advanced 254 per millisecond, so every loop ran
 *            more than two hundred times faster than expected.
 *
 *   Root cause: the HAL private variable uwTickFreq (1 byte; see map
 *            0x20000000  uwTickFreq  Data 1  stm32f1xx_hal.o(.data..L_MergedGlobals)
 *         -- it happens to land on the first byte of RAM) was overwritten
 *         during startup by a single **16-bit write of 0xE7FE**, changing its
 *         value from 0x01 to 0xFE (=254). HAL_IncTick() does
 *         uwTick += uwTickFreq, so ticks ran 254 times faster; the wait in
 *         HAL_Delay(500) is 500 + 254 = 754, and 754/254 ~ 3 SysTicks ~ 3 ms,
 *         which matches the measurement exactly.
 *
 *   Evidence:
 *     - The initial value of that byte in the Flash image is confirmed 0x01
 *       (8 bytes at 0x08001b80 = 01 00 00 00 10 00 00 00), the padding is
 *       00 00 00 => the image is correct; the byte is rewritten at runtime.
 *     - Reading 4 bytes at runtime gives FE E7 00 00 (after writing 1 it
 *       becomes 01 E7 00 00) => byte 0 and byte 1 changed together, i.e. one
 *       16-bit write of 0xE7FE (0xE7FE is exactly the Thumb `B .`, which is
 *       also armlink's fallback fill value).
 *     - Staged sampling: at main entry / after HAL_Init / after ClockConfig /
 *       after UART init the value is 1; after the **first UART transmit** it
 *       becomes 254.
 *     - Placing the repair after UART init but before the banner print still
 *       reads back 254; placing it **after the banner print** gives a steady
 *       took of 501 for more than 12 s with no further rewrite.
 *     => **The rewrite point is the first HAL_UART_Transmit, and it happens
 *        only once.**
 *
 *   Fix: call timebase_repair() after all boot prints are done (doing it
 *        earlier is useless, that write overwrites it again).
 *
 *   Long-term plan (to adopt when the BSP layer lands): give the firmware its
 *        own TIM6 millisecond timebase, so the control loop no longer depends
 *        on HAL_GetTick() / HAL_Delay() -- the variable those two read is not
 *        in our own RAM area, and once it is clobbered we can only patch it
 *        after the fact.
 * ------------------------------------------------------------------------ */

/* --------------------------------------------------------------------------
 * Power-on routing: by default enter the USART2 link self-test; pressing 'm'
 * on USART1 within 3 s switches to the motor bench
 *
 *   Why this window is needed: nobody can press a key on the Raspberry Pi
 *   side, so **the default path must run unattended**. Conversely, when doing
 *   motor experiments we do not want to reflash the firmware just to switch
 *   benches.
 *
 *   Why HAL_GetTick() instead of HAL_Delay(): USART1 RXNE must be polled
 *   continuously while waiting, so a blocking delay cannot be used. The
 *   timebase has already been fixed by timebase_repair() in main().
 * ------------------------------------------------------------------------ */
static int boot_wait_motor_key(uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();
  uint32_t last_sec = 0u;
  int hit = 0;

  uart_printf("\r\n[boot] press 'm' on this port (USART1/COM9) within %lu s\r\n",
              (unsigned long)(ms / 1000u));
  uart_printf("[boot]   -> 'm' = MOTOR bench (bench.c)\r\n");
  uart_printf("[boot]   -> no key = USART2 link bench (default, for Raspberry Pi)\r\n");

  while ((HAL_GetTick() - t0) < ms)
  {
    uint32_t el = HAL_GetTick() - t0;

    if ((el / 1000u) != last_sec)
    {
      last_sec = el / 1000u;
      uart_printf("[boot] %lu ...\r\n",
                  (unsigned long)((ms - el + 999u) / 1000u));
    }

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
    {
      char c = (char)(huart1.Instance->DR & 0xFFu);
      uart_printf("[boot] got '%c'\r\n", c);
      if (c == 'm' || c == 'M') { hit = 1; break; }
    }
  }

  uart_printf("[boot] -> %s\r\n\r\n", hit ? "MOTOR bench" : "USART2 link bench");
  return hit;
}

/**
  * @brief  Main program
  * @retval int (never returns)
  */
int main(void)
{
  uint32_t n;
  uint32_t t0;
  uint32_t dtk;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  uart_printf("\r\n");
  uart_printf("OmniNav STM32 alive\r\n");
  uart_printf("  MCU   : STM32F103ZET6 (HD, 512K Flash / 64K SRAM)\r\n");
  uart_printf("  HCLK  : %lu Hz\r\n", (unsigned long)HAL_RCC_GetHCLKFreq());
  uart_printf("  PCLK1 : %lu Hz\r\n", (unsigned long)HAL_RCC_GetPCLK1Freq());
  uart_printf("  PCLK2 : %lu Hz\r\n", (unsigned long)HAL_RCC_GetPCLK2Freq());
  uart_printf("  built : %s %s\r\n", __DATE__, __TIME__);
  uart_printf("----------------------------------------\r\n");

  /* ---- Timebase repair: must run after the first UART transmit ---- */
  timebase_repair();
  uart_printf("SysTick repaired: LOAD=%lu  uwTickFreq=%lu\r\n",
              (unsigned long)SysTick->LOAD, (unsigned long)uwTickFreq);

  /* ---- Algorithm-layer self-test: proves firmware_core is really linked
   *      and running on the board ---- */
  core_selftest_run();

  /* ---- Power-on routing (both branches are infinite loops, never return)
   *
   *   default -> usart2_bench_run(): USART2 (PA2/PA3) link self-test +
   *          firmware_core protocol-layer verification on the real board.
   *          This is the link used by the Raspberry Pi, so it must run
   *          unattended.
   *   'm'     -> bench_run(): A heartbeat -> B 13-pin probe -> C single-motor
   *          PWM (the original bench).
   *   A-02 switch -> app_single_wheel_run(): single-wheel safety firmware.
   *          Takes effect only when OMNINAV_APP_SINGLE_WHEEL in
   *          Inc/app_single_wheel.h is 1; default 0 = original behavior fully
   *          preserved.
   * ------------------------------------------------------------------------ */
#if OMNINAV_APP_SINGLE_WHEEL
  (void)boot_wait_motor_key;     /* Keep the symbol referenced in mode 1, avoids "unused static function" */
  app_single_wheel_run();        /* A-02 single-wheel safety firmware (bench, single-character commands on USART1) */
#else
  if (boot_wait_motor_key(3000u))
  {
    bench_run();
  }
  usart2_bench_run();
#endif

  /*
   * Below is the phase-0 heartbeat loop. bench_run() normally never returns
   * (we only get here when the bench module is stripped out), so this stays as
   * the minimal working fallback.
   */
  n = 0;
  t0 = HAL_GetTick();

  while (1)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);   /* LED0 */
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);   /* LED1 */

    HAL_Delay(500);
    dtk = HAL_GetTick() - t0;   /* HAL_Delay(500) should measure about 501 */
    t0  = HAL_GetTick();
    n++;

    if ((n % 4u) == 0u)   /* One line every 2 s, easy to read */
    {
      uart_printf("n=%lu  HAL_Delay(500) took %lu ms  uf=%lu\r\n",
                  (unsigned long)n, (unsigned long)dtk, (unsigned long)uwTickFreq);
    }
  }
}

/**
  * @brief  Repair the HAL millisecond timebase back to 1 kHz
  *         Must be called after the first UART transmit; see the "Timebase
  *         repair" note in the file header
  */
static void timebase_repair(void)
{
  uwTickFreq = HAL_TICK_FREQ_DEFAULT;    /* == 1 -> 1 kHz */
  (void)HAL_InitTick(TICK_INT_PRIORITY); /* Recompute SysTick LOAD=71999 with uf=1 */
}

/* -------------------------------------------------------------------------- */
/*  UART output layer                                                         */
/* -------------------------------------------------------------------------- */

/**
  * @brief  Flush the transmit buffer to USART1
  */
static void tx_flush(void)
{
  if (g_tx_len > 0u)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)g_tx_buf, g_tx_len, HAL_MAX_DELAY);
    g_tx_len = 0u;
  }
}

/**
  * @brief  Buffer one character, flushing first when full
  */
static void tx_putc(char c)
{
  if (g_tx_len >= (uint16_t)TXBUF_SZ)
  {
    tx_flush();
  }
  g_tx_buf[g_tx_len] = c;
  g_tx_len++;
}

/**
  * @brief  Buffer one string
  */
static void tx_puts(const char *s)
{
  while (*s != '\0')
  {
    tx_putc(*s);
    s++;
  }
}

/**
  * @brief  Emit one integer (32-bit only, no C library dependency)
  */
static void tx_putn(uint32_t mag, int neg, uint32_t base, uint32_t width, char pad, int upper)
{
  char     tmp[11];
  int      i = 0;
  uint32_t len;

  if (mag == 0u)
  {
    tmp[i] = '0';
    i++;
  }
  while (mag != 0u)
  {
    uint32_t d = mag % base;
    mag = mag / base;
    if (d < 10u)
    {
      tmp[i] = (char)('0' + d);
    }
    else
    {
      tmp[i] = (char)((upper != 0 ? 'A' : 'a') + (d - 10u));
    }
    i++;
  }

  len = (uint32_t)i + (neg != 0 ? 1u : 0u);
  while (len < width)
  {
    tx_putc(pad);
    len++;
  }

  if (neg != 0)
  {
    tx_putc('-');
  }
  while (i > 0)
  {
    i--;
    tx_putc(tmp[i]);
  }
}

/**
  * @brief  Formatted UART output (safe replacement for printf)
  * @param  fmt: format string, supports %s %c %% %d %i %u %x %X, including
  *              the '0' flag and field width
  *              NOTE: %f is forbidden (and unsupported here); Chinese text is
  *              also forbidden in format strings
  */
void uart_printf(const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);

  while (*fmt != '\0')
  {
    uint32_t width = 0u;
    char     pad   = ' ';
    char     spec;

    if (*fmt != '%')
    {
      tx_putc(*fmt);
      fmt++;
      continue;
    }
    fmt++;

    if (*fmt == '0')
    {
      pad = '0';
      fmt++;
    }
    while ((*fmt >= '0') && (*fmt <= '9'))
    {
      width = (width * 10u) + (uint32_t)(*fmt - '0');
      fmt++;
    }
    while ((*fmt == 'l') || (*fmt == 'h'))
    {
      fmt++;   /* Length prefixes are accepted and ignored: this implementation is 32-bit only */
    }

    spec = *fmt;

    if (spec == '\0')
    {
      break;
    }

    switch (spec)
    {
      case 's':
      {
        const char *s = va_arg(ap, const char *);
        tx_puts((s != NULL) ? s : "(null)");
        break;
      }
      case 'c':
      {
        char c = (char)va_arg(ap, int);
        tx_putc(c);
        break;
      }
      case '%':
      {
        tx_putc('%');
        break;
      }
      case 'd':
      case 'i':
      {
        int32_t v = va_arg(ap, int32_t);
        if (v < 0)
        {
          tx_putn((uint32_t)(-(v + 1)) + 1u, 1, 10u, width, pad, 0);
        }
        else
        {
          tx_putn((uint32_t)v, 0, 10u, width, pad, 0);
        }
        break;
      }
      case 'u':
      {
        tx_putn((uint32_t)va_arg(ap, uint32_t), 0, 10u, width, pad, 0);
        break;
      }
      case 'x':
      {
        tx_putn((uint32_t)va_arg(ap, uint32_t), 0, 16u, width, pad, 0);
        break;
      }
      case 'X':
      {
        tx_putn((uint32_t)va_arg(ap, uint32_t), 0, 16u, width, pad, 1);
        break;
      }
      default:
      {
        /* Unknown conversion: print it verbatim so misuse is easy to spot */
        tx_putc('%');
        tx_putc(spec);
        break;
      }
    }

    fmt++;
  }

  va_end(ap);
  tx_flush();
}

/* -------------------------------------------------------------------------- */

/**
  * @brief  GPIO init: PB5 / PE5 push-pull outputs (the two on-board LEDs)
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  gpio.Pin   = GPIO_PIN_5;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_Init(GPIOE, &gpio);

  /* Turn them off at power-on (on-board LEDs are active low) */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
  * @brief  USART1 init: PA9=TX / PA10=RX, 115200 8N1
  *         The on-board CH340C is wired to USART1; plug into the "USB to
  *         serial" port at the lower-left corner of the board to see COM9
  */
static void MX_USART1_UART_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  /* PA9 = USART1_TX, alternate-function push-pull */
  gpio.Pin   = GPIO_PIN_9;
  gpio.Mode  = GPIO_MODE_AF_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* PA10 = USART1_RX, floating input */
  gpio.Pin   = GPIO_PIN_10;
  gpio.Mode  = GPIO_MODE_INPUT;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);

  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 115200;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    while (1)
    {
    }
  }
}

/**
  * @brief  System clock: HSE 8 MHz -> PLL x9 -> SYSCLK 72 MHz
  *         AHB/1 = 72 MHz, APB2/1 = 72 MHz, APB1/2 = 36 MHz, Flash 2 wait states
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};

  oscinitstruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState       = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState   = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL     = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&oscinitstruct) != HAL_OK)
  {
    while (1)
    {
    }
  }

  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                             RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2) != HAL_OK)
  {
    while (1)
    {
    }
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Landing point when assert_param fails
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;

  while (1)
  {
  }
}

#endif /* USE_FULL_ASSERT */
