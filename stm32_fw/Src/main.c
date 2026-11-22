/**
  ******************************************************************************
  * @file    Src/main.c
  * @brief   OmniNav STM32 阶段 0 验收程序
  *
  *          目标板：正点原子 STM32F103ZE 精英板（STM32F103ZET6，512K Flash / 64K SRAM）
  *          LED0 = PB5、LED1 = PE5（板载 LED 低电平点亮）
  *          串口 = USART1 (PA9=TX / PA10=RX) 115200 8N1，板载 CH340C → Windows 侧 COM9
  *
  *          验收判据：
  *            1) 两颗 LED 以 1 Hz 闪烁（每 500 ms 翻转一次）
  *            2) 串口每 2 秒一行，且 took 字段稳定在 501
  ******************************************************************************
  * ★ 本文件不使用标准 printf —— Keil 不勾 MicroLIB 时 printf 会触发半主机
  *   (semihosting)，表现为程序一进 main 就卡死。所有输出统一走下面的 uart_printf()。
  * ★ 不使用 vsnprintf：格式串一律走自研实现，既省下库代码，也避免 C 库的
  *   libspace / 堆初始化参与进来。
  * ★ 禁止使用 %f：协议与日志全用定点整数，避免把浮点库拖进固件。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "core_selftest.h"
#include "bench.h"
#include "usart2_bench.h"   /* 上电默认入口：USART2 链路自证（PA2/PA3 → 树莓派 5） */
#include "app_single_wheel.h"   /* A-02：编译期开关 OMNINAV_APP_SINGLE_WHEEL（默认 0 = 保持原行为） */

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
 * 时基修复（2026-10-07 在真机上定位并实测）
 *
 *   现象：HAL_Delay(500) 只花约 3 ms（实测 took=762），HAL_GetTick() 每毫秒
 *         前进 254，于是每圈比预期快两百多倍。
 *
 *   根因：HAL 的私有变量 uwTickFreq（1 字节；见 map
 *           0x20000000  uwTickFreq  Data 1  stm32f1xx_hal.o(.data..L_MergedGlobals)
 *         —— 它正好落在 RAM 的第一个字节）在启动阶段被一次 **16 位写入 0xE7FE**
 *         覆盖，值由 0x01 变成 0xFE(=254)。HAL_IncTick() 是 uwTick += uwTickFreq，
 *         所以 tick 快了 254 倍；HAL_Delay(500) 的 wait = 500 + 254 = 754，
 *         754/254 ≈ 3 个 SysTick ≈ 3 ms，与实测完全吻合。
 *
 *   证据：
 *     - Flash 镜像里该字节初值确认为 0x01（`0x08001b80` 处 8 字节 = 01 00 00 00
 *       10 00 00 00），padding 是 00 00 00 ⇒ 镜像是对的，是运行期被改写。
 *     - 运行期读 4 字节 = FE E7 00 00（写入 1 后变 01 E7 00 00）⇒ 字节 0 与字节 1
 *       同时变了，即一次 16 位写 0xE7FE（0xE7FE 正好是 Thumb 的 `B .`，
 *       也是 armlink 的兜底填充值）。
 *     - 分段采样：main 入口 / HAL_Init 后 / ClockConfig 后 / UART init 后 全是 1，
 *       **第一次 UART 发送之后** 变成 254。
 *     - 把修复放在 UART 初始化之后、banner 打印之前 —— 仍然读出 254；
 *       把修复放在 **banner 打印之后** —— took 稳定 501，连续 12 秒以上不再被改写。
 *     ⇒ **改写点是第一次 HAL_UART_Transmit，且只发生一次。**
 *
 *   处置：所有启动打印做完之后再调 timebase_repair()（提前无效，会被那次写回盖掉）。
 *
 *   长期方案（等 BSP 层落地时换）：固件自带 TIM6 毫秒时基，控制环彻底不依赖
 *         HAL_GetTick() / HAL_Delay()，因为这两个函数读的变量不在我们自己的
 *         RAM 区里，它被踩我们只能事后补救。
 * ------------------------------------------------------------------------ */

/* --------------------------------------------------------------------------
 * 上电选路：默认进 USART2 链路自证，3 秒内在 USART1 上按 'm' 改走电机台架
 *
 *   为什么要有这个窗口：树莓派那边按不了键，所以**默认那条必须无人值守就能跑**。
 *   反过来，做电机实验时又不想为了切台架重新烧一遍固件。
 *
 *   为什么用 HAL_GetTick() 而不是 HAL_Delay()：要在等待期间持续轮询 USART1 的
 *   RXNE，不能用阻塞延时。时基已在 main() 里被 timebase_repair() 修好。
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
  * @brief  主程序
  * @retval int（永不返回）
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

  /* ---- 时基修复：必须在第一次 UART 发送之后 ---- */
  timebase_repair();
  uart_printf("SysTick repaired: LOAD=%lu  uwTickFreq=%lu\r\n",
              (unsigned long)SysTick->LOAD, (unsigned long)uwTickFreq);

  /* ---- 算法层自检：证明 firmware_core 真的被链接并跑在板子上 ---- */
  core_selftest_run();

  /* ---- 上电选路（两个分支都是死循环，永不返回）
   *
   *   默认 → usart2_bench_run()：USART2(PA2/PA3) 链路自证 + firmware_core 协议层
   *          真机验证。这是给树莓派用的那条链路，必须无人值守就能跑。
   *   按 'm' → bench_run()：A 心跳 → B 13 脚探针 → C 单电机 PWM（原台架）。
   *   A-02 开关 → app_single_wheel_run()：单轮安全固件。把 Inc/app_single_wheel.h
   *          里的 OMNINAV_APP_SINGLE_WHEEL 置 1 才生效；默认 0 = 完全保持原行为。
   * ------------------------------------------------------------------------ */
#if OMNINAV_APP_SINGLE_WHEEL
  (void)boot_wait_motor_key;     /* 模式=1 时保持符号引用，避免“未使用静态函数”告警 */
  app_single_wheel_run();        /* A-02 单轮安全固件（台架，USART1 单字符命令） */
#else
  if (boot_wait_motor_key(3000u))
  {
    bench_run();
  }
  usart2_bench_run();
#endif

  /*
   * 下面是阶段 0 的心跳循环。bench_run() 正常不会返回（摘掉 bench 模块时才会
   * 走到这里），留着当作最小可用回退。
   */
  n = 0;
  t0 = HAL_GetTick();

  while (1)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);   /* LED0 */
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);   /* LED1 */

    HAL_Delay(500);
    dtk = HAL_GetTick() - t0;   /* HAL_Delay(500) 实测应约 501 */
    t0  = HAL_GetTick();
    n++;

    if ((n % 4u) == 0u)   /* 每 2 秒一行，便于读数 */
    {
      uart_printf("n=%lu  HAL_Delay(500) took %lu ms  uf=%lu\r\n",
                  (unsigned long)n, (unsigned long)dtk, (unsigned long)uwTickFreq);
    }
  }
}

/**
  * @brief  把 HAL 的毫秒时基修正回 1 kHz
  *         必须在第一次 UART 发送之后调用，见文件头"时基修复"的说明
  */
static void timebase_repair(void)
{
  uwTickFreq = HAL_TICK_FREQ_DEFAULT;    /* == 1 -> 1 kHz */
  (void)HAL_InitTick(TICK_INT_PRIORITY); /* 按 uf=1 重算 SysTick LOAD=71999 */
}

/* -------------------------------------------------------------------------- */
/*  串口输出层                                                                 */
/* -------------------------------------------------------------------------- */

/**
  * @brief  把发送缓冲刷到 USART1
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
  * @brief  缓冲一个字符，满了先刷
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
  * @brief  缓冲一个字符串
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
  * @brief  输出一个整数（只用 32 位，不依赖 C 库）
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
  * @brief  串口格式化输出（printf 的安全替代）
  * @param  fmt: 格式化串，支持 %s %c %% %d %i %u %x %X，含 '0' 标志与字段宽度
  *              ★ 禁止 %f（本实现也不支持），格式串里也禁止放中文
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
      fmt++;   /* 长度前缀接受并忽略：本实现只用 32 位 */
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
        /* 不认识的格式符：原样打出来，方便发现用错 */
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
  * @brief  GPIO 初始化：PB5 / PE5 推挽输出（板载两颗 LED）
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

  /* 上电先熄灭（板载 LED 低电平点亮） */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
  * @brief  USART1 初始化：PA9=TX / PA10=RX，115200 8N1
  *         板载 CH340C 就挂在 USART1 上，插板子左下角「USB转串口」口即可见 COM9
  */
static void MX_USART1_UART_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  /* PA9 = USART1_TX，复用推挽 */
  gpio.Pin   = GPIO_PIN_9;
  gpio.Mode  = GPIO_MODE_AF_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* PA10 = USART1_RX，浮空输入 */
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
  * @brief  系统时钟：HSE 8 MHz → PLL×9 → SYSCLK 72 MHz
  *         AHB/1 = 72 MHz，APB2/1 = 72 MHz，APB1/2 = 36 MHz，Flash 2 等待周期
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
  * @brief  assert_param 失败时的落点
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
