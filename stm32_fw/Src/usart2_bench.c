/**
 * usart2_bench.c —— USART2 链路自证台（OmniNav 下位机 ↔ 树莓派 5 的唯一数据链）
 * =============================================================================
 *
 * 这个文件是干什么的
 * -----------------------------------------------------------------------------
 *   证明「树莓派 5 ↔ STM32」这条线真的通了，并且把 `firmware_core` 的协议层
 *   放到真机上跑一遍。它是**诊断工具**，不是最终固件 —— 最终固件里 USART2 会被
 *   控制环以 50 Hz 使用，而这里是一问一答 + 心跳。
 *
 *   上电默认就进这里（见 main.c 的上电选路）；3 秒内在 USART1(COM9) 上按 'm'
 *   会切到原来的电机/探针台架 bench.c。
 *
 * 硬件接线（已在 2026-10 由用户接好）
 * -----------------------------------------------------------------------------
 *   STM32 PA2 (USART2_TX)  ──►  USB-TTL 模块 RXD
 *   STM32 PA3 (USART2_RX)  ◄──  USB-TTL 模块 TXD
 *   STM32 GND              ◄──► USB-TTL 模块 GND
 *   USB-TTL 模块 VCC       ——   不接（STM32 由 ST-Link/DC 座供电）
 *
 *   ★★★ P5 跳线帽必须拔掉
 * -----------------------------------------------------------------------------
 *   精英版把 PA2 / PA3 引到一个叫 **P5 USART2** 的 4 针座（板子左下角、DC 电源
 *   座旁边），丝印是 2x2：
 *
 *        ┌──────┬──────┐
 *        │ 48T  │ 48R  │   ← 接板载 SP3485（RS485 收发器）
 *        ├──────┼──────┤
 *        │ PA3  │ PA2  │   ← STM32 的 USART2_RX / USART2_TX
 *        └──────┴──────┘
 *
 *   跳线帽 = 上下两针短接。**帽子插着 ⇒ PA2/PA3 挂在 SP3485 上**（485 模式），
 *   此时 TTL 信号被 485 芯片拖拽，现象是"完全收不到"或"满屏乱码"。
 *   **帽子拔掉 ⇒ PA2/PA3 与 485 断开，那两个孔就是纯 3.3 V TTL，可以直接插杜邦线。**
 *   （依据：正点原子论坛 311106「P5 的跳线帽去掉时可以做 RS485 转 TTL 使用」）
 *
 *   30 秒判定帽子拔没拔：万用表蜂鸣档，一表笔点 PA2 孔、另一支点 48R 孔 ——
 *   响 / 0 Ω ⇒ 帽子还插着（必须拔）；不通 ⇒ 已拔（可用）。
 *
 *   另外：只走 TTL 时 **PD7（485 方向使能）完全不用管**。
 *
 * 三个坑（都踩过或在别的文件里记过）
 * -----------------------------------------------------------------------------
 *   1) **绝不能重定向 printf**。Keil 不勾 MicroLIB 时 printf 会触发半主机
 *      (semihosting) 把程序卡死；本文件用一个 60 行的自研格式化器，不碰 <stdio.h>。
 *   2) **USART2 挂在 APB1 = 36 MHz，不是 APB2 的 72 MHz**。HAL 会自己按 PCLK1
 *      算 BRR，但必须把算出来的 BRR 打出来核对（期望 0x0138）。
 *   3) **输出只许 ASCII**（交接单坑 26）。自研格式化器不支持中文，写了会变 '?'。
 *
 * 为什么用"中断收 + 环形缓冲"而不是轮询
 * -----------------------------------------------------------------------------
 *   回显是逐字节阻塞发送的，单字节 @115200 要 ~87 us。若用轮询收，发送期间到达
 *   的字节会被硬件 ORE 丢掉，回显就不再是逐字节可验证的 ⇒ 测出来的"丢字节"分不清
 *   是线路问题还是固件问题。所以：**RXNE 中断 + 256 字节环形缓冲**，
 *   中断里只做"读 DR、进队列"，其余全在主循环，绝不调用 HAL。
 *
 *   同时**不走 HAL 的 HAL_UART_Receive_IT**（它的状态机在 ORE 时会中断接收、
 *   需要 ErrorCallback 里重新武装，很容易把 RX 卡死）。这里自己写 USART2_IRQHandler，
 *   直接读 SR/DR —— 读 SR 再读 DR 一次就把 RXNE 和 ORE 都清了，ORE 永远卡不住我们。
 *   stm32f1xx_it.c 里没有定义过 USART2_IRQHandler，所以不冲突。
 *
 * 怎么用（树莓派侧）
 * -----------------------------------------------------------------------------
 *   设备名：/dev/ttySTM32（udev 规则已装，指向 ttyUSB0）
 *   参数  ：115200 8N1，无流控
 *
 *   看心跳（证明 STM32 → 树莓派 方向）：
 *     python3 -c "import serial;s=serial.Serial('/dev/ttySTM32',115200,timeout=3);print(s.readline())"
 *
 *   发一条运动指令（证明 树莓派 → STM32 方向 + 协议层）：
 *     printf 'V,100,0,0\n' > /dev/ttySTM32
 *     ⇒ 本地会原样回显 `V,100,0,0`，并且 STM32 回一行 `#ACK V 3`
 *
 *   本文件配套脚本：pi_setup/test_stm32_link.py（一次跑完全部自证并给结论）
 * =============================================================================
 */

#include "usart2_bench.h"
#include "main.h"
#include "bench.h"        /* 只为上电选路时切到 bench_run() */
#include "protocol.h"     /* firmware_core，已在 Keil 工程 IncludePath 里 */
#include "mecanum.h"      /* firmware_core：麦轮逆解/正解 + 一键求解（影子模式用） */
#include "fixedpoint.h"   /* firmware_core：mm/s -> 计数/控制窗口（PID 设定值） */
#include <stdarg.h>

/* USART1 的句柄定义在 main.c（同 bench.c 的写法） */
extern UART_HandleTypeDef huart1;

/* 覆盖 startup_stm32f103xe.s 里的 weak 默认实现。
 * ★ stm32f1xx_it.c 里没有定义过它，所以不冲突。 */
void USART2_IRQHandler(void);

/* ===========================================================================
 * 配置
 * =========================================================================== */

#define U2_BAUD          115200u   /* 契约冻结值，不许改 */
#define U2_FIFO_SZ       256u      /* 必须是 2 的幂 */
#define U2_FIFO_MASK     (U2_FIFO_SZ - 1u)
#define HB_PERIOD_MS     1000u     /* 心跳周期 */
#define STAT_PERIOD_MS   5000u     /* 统计行周期 */
#define U2_LINE_MAX      160u      /* 单行输出缓冲 */

/* proto_cmd_t 的取值个数（NONE/V/M/P/S/RESET/UNKNOWN） */
#define U2_CMD_SLOTS     7u

/* ★★ 本地命令前缀。必须是协议字符集之外的字符（协议只用字母/数字/','/'-'/'\n'）。
 *    用 '.' 的理由：它永远不可能出现在协议帧里，所以绝不会误拦正常命令；
 *    即使漏进解析器也只会被判 BAD_CHAR，不会污染后续帧。
 *    ⇒ 本地命令写法：'.?' '.e' '.h' '.a' '.c' '.k'（见 u2_handle_byte() 的注释） */
#define U2_LOCAL_PREFIX  '.'

/* ===========================================================================
 * 状态
 * =========================================================================== */

static UART_HandleTypeDef huart2;

/* --- 接收环形缓冲（中断写 head，主循环读 tail） --- */
static volatile uint8_t  s_fifo[U2_FIFO_SZ];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;
static volatile uint32_t s_drop;      /* 队列满丢掉的字节数 */
static volatile uint32_t s_rxerr;     /* ORE/FE/NE/PE 次数 */

/* --- 统计 --- */
static uint32_t s_rx_bytes;           /* PA3 收到多少字节 */
static uint32_t s_echo_bytes;         /* 回显出去多少字节 */
static uint32_t s_hb_count;           /* 发了多少次心跳 */
static uint32_t s_cmd_count[U2_CMD_SLOTS];

/* --- 开关 --- */
static uint8_t s_echo_on = 1u;        /* '.e' 切换，默认开 */
static uint8_t s_hb_on   = 1u;        /* '.h' 切换，默认开 */
static uint8_t s_ack_on  = 1u;        /* '.a' 切换，默认开 */

/* 影子模式：'.s' 切换，**默认关**。
 * ★ 关着的时候，USART2 上的**逐帧行为**与"11/11 链路自证"基线完全一致
 *   （唯一差别是启动 banner 多了两行文字，那是一次性的、测试脚本不看它）。
 * ★ 打开以后每帧多 5 行 —— logf() 是"两个口都发"（见 send_on/logf），
 *   所以影子结果在 COM9 和树莓派两侧都能看到。 */
static uint8_t s_shadow_on;

/* --- 本地命令前缀状态（收到 U2_LOCAL_PREFIX 后置 1，下一个字节是命令字母） --- */
static uint8_t s_local_mode;

static proto_parser_t s_parser;
static proto_msg_t    s_msg;

/* ===========================================================================
 * 迷你格式化器（故意不支持 %f / 64 位 / 宽度补零 —— 本文件用不到）
 * =========================================================================== */

static void fmt_put(char *out, int cap, int *pn, char ch)
{
    if (*pn < cap - 1) { out[*pn] = ch; }
    (*pn)++;
}

static void fmt_num(char *out, int cap, int *pn, unsigned long v,
                    unsigned long base, int upper)
{
    char t[32];
    int k = 0;

    if (v == 0ul) { t[k++] = '0'; }
    while (v != 0ul)
    {
        unsigned long d = v % base;
        v /= base;
        t[k++] = (char)((d < 10ul) ? ('0' + (int)d)
                                  : ((upper ? 'A' : 'a') + (int)(d - 10ul)));
    }
    while (k > 0) { k--; fmt_put(out, cap, pn, t[k]); }
}

/**
 * 支持的转换：%s %c %d %i %u %x %X %%
 * 其余（宽度、'0' 标志、l/h 前缀）会被跳过，不做补零对齐。
 * 返回值 = 写入长度（不含结尾 NUL）；若 >= cap 说明被截断了。
 */
static int fmt_v(char *out, int cap, const char *fmt, va_list ap)
{
    int n = 0;

    while (*fmt != '\0')
    {
        if (*fmt != '%') { fmt_put(out, cap, &n, *fmt); fmt++; continue; }

        fmt++;
        if (*fmt == '\0') { break; }
        if (*fmt == '%')  { fmt_put(out, cap, &n, '%'); fmt++; continue; }

        /* 跳过标志位与字段宽度（本文件不依赖对齐） */
        while (*fmt == '0' || (*fmt >= '1' && *fmt <= '9')) { fmt++; }
        /* 跳过 l / h 长度前缀 */
        while (*fmt == 'l' || *fmt == 'h') { fmt++; }

        switch (*fmt)
        {
        case 's':
        {
            const char *sp = va_arg(ap, const char *);
            if (sp == 0) { sp = "(null)"; }
            while (*sp != '\0') { fmt_put(out, cap, &n, *sp); sp++; }
            break;
        }
        case 'c':
        {
            int c = va_arg(ap, int);
            fmt_put(out, cap, &n, (char)c);
            break;
        }
        case 'd':
        case 'i':
        {
            int v = va_arg(ap, int);
            if (v < 0)
            {
                fmt_put(out, cap, &n, '-');
                /* INT_MIN 安全写法 */
                fmt_num(out, cap, &n, (unsigned long)(-(v + 1)) + 1ul, 10ul, 0);
            }
            else
            {
                fmt_num(out, cap, &n, (unsigned long)v, 10ul, 0);
            }
            break;
        }
        case 'u':
            fmt_num(out, cap, &n, (unsigned long)va_arg(ap, unsigned int), 10ul, 0);
            break;
        case 'x':
            fmt_num(out, cap, &n, (unsigned long)va_arg(ap, unsigned int), 16ul, 0);
            break;
        case 'X':
            fmt_num(out, cap, &n, (unsigned long)va_arg(ap, unsigned int), 16ul, 1);
            break;
        default:
            /* 不认识的转换：原样吐出去，方便发现写错 */
            fmt_put(out, cap, &n, '%');
            fmt_put(out, cap, &n, *fmt);
            break;
        }
        fmt++;
    }

    if (cap > 0)
    {
        out[(n < cap) ? n : (cap - 1)] = '\0';
    }
    return n;
}

/* ===========================================================================
 * 输出：两个口都发 / 只发 USART2
 * =========================================================================== */

static void send_on(UART_HandleTypeDef *h, const char *s, int n)
{
    if (n <= 0) { return; }
    (void)HAL_UART_Transmit(h, (const uint8_t *)s, (uint16_t)n, HAL_MAX_DELAY);
}

/** 同一行同时打到 USART1（电脑 COM9）和 USART2（树莓派）—— 便于两处对照 */
static void logf(const char *fmt, ...)
{
    static char buf[U2_LINE_MAX];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = fmt_v(buf, (int)sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > (int)sizeof(buf) - 1) { n = (int)sizeof(buf) - 1; }
    send_on(&huart1, buf, n);
    send_on(&huart2, buf, n);
}

/** 只打 USART2 —— 机器可读的行（'#' 开头），不污染 COM9 */
static void u2f(const char *fmt, ...)
{
    static char buf[U2_LINE_MAX];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = fmt_v(buf, (int)sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > (int)sizeof(buf) - 1) { n = (int)sizeof(buf) - 1; }
    send_on(&huart2, buf, n);
}

/* ===========================================================================
 * USART2 初始化
 * =========================================================================== */

static void u2_init(void)
{
    GPIO_InitTypeDef gpio;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    /* PA2 = USART2_TX：复用推挽。PA2/PA3 是 USART2 的**默认**引脚，不需要重映射，
     * 所以这里**不碰 AFIO**（bench.c 里给 TIM1 做的完全重映射与这里无冲突）。 */
    gpio.Pin   = GPIO_PIN_2;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PA3 = USART2_RX：浮空输入（HAL 对 UART 的 RX 要求 INPUT + NOPULL，同 PA10） */
    gpio.Pin   = GPIO_PIN_3;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = U2_BAUD;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        logf("[U2] FATAL: HAL_UART_Init(USART2) failed\r\n");
        for (;;) { }
    }

    /* 只开 RXNE 中断，不走 HAL 的 Receive_IT（理由见文件头） */
    HAL_NVIC_SetPriority(USART2_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/**
 * USART2 中断服务程序。
 * 只做两件事：把收到的字节塞进环形缓冲；把错误标志清掉。
 * ★ 绝不在这里调用任何 HAL 发送函数（HAL_UART_Transmit 不是可重入的）。
 */
void USART2_IRQHandler(void)
{
    uint32_t sr = USART2->SR;

    if ((sr & USART_SR_RXNE) != 0u)
    {
        uint8_t b = (uint8_t)(USART2->DR & 0xFFu);   /* 读 DR 同时清 RXNE / ORE */
        uint16_t next = (uint16_t)((s_head + 1u) & U2_FIFO_MASK);

        if (next == s_tail)
        {
            s_drop++;                                 /* 队列满：丢新的，不覆盖旧的 */
        }
        else
        {
            s_fifo[s_head] = b;
            s_head = next;
        }
    }
    else if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0u)
    {
        (void)USART2->DR;                             /* 清错误标志 */
        s_rxerr++;
    }
}

/** 从环形缓冲取一个字节；返回 1 = 取到 */
static int u2_pop(uint8_t *b)
{
    if (s_tail == s_head) { return 0; }
    *b = s_fifo[s_tail];
    s_tail = (uint16_t)((s_tail + 1u) & U2_FIFO_MASK);
    return 1;
}

/* ===========================================================================
 * 输出内容
 * =========================================================================== */

static void u2_banner(void)
{
    uint32_t brr = huart2.Instance->BRR;

    logf("\r\n");
    logf("===== OmniNav USART2 link bench =====\r\n");
    logf("[U2] PA2=TX  PA3=RX  %u 8N1  no flow control\r\n", (unsigned)U2_BAUD);
    logf("[U2] APB1(PCLK1)=%u Hz   BRR=0x%X (expect 0x138 = %u baud)\r\n",
         (unsigned)HAL_RCC_GetPCLK1Freq(), (unsigned)brr, (unsigned)U2_BAUD);
    logf("[U2] P5 jumper MUST be OFF (PA2/PA3 not tied to SP3485)\r\n");
    logf("[U2] wire: PA2 -> module RXD | PA3 <- module TXD | GND-GND | VCC open\r\n");
    logf("[U2] local keys need the '%c' prefix: %c? stat  %ce echo  %ch heartbeat  %ca ack  %cc clear  %ck kick  %cs shadow\r\n",
         (char)U2_LOCAL_PREFIX, (char)U2_LOCAL_PREFIX, (char)U2_LOCAL_PREFIX,
         (char)U2_LOCAL_PREFIX, (char)U2_LOCAL_PREFIX, (char)U2_LOCAL_PREFIX,
         (char)U2_LOCAL_PREFIX, (char)U2_LOCAL_PREFIX);
    logf("[U2] why a prefix: bare 'e' used to swallow the 'e' in 'reset' -> parser saw 'rst'\r\n");
    logf("[U2] shadow: send V,300,0,0 then '%cs' -- computes 4 wheel targets, drives NOTHING\r\n",
         (char)U2_LOCAL_PREFIX);
    logf("[U2] this text is ALSO on USART1 (COM9) for side-by-side view\r\n");
    logf("\r\n");
}

static void u2_heartbeat(void)
{
    if (s_hb_on == 0u) { return; }
    s_hb_count++;
    /* 机器可读行：'#' 开头，树莓派侧一眼能认出来 */
    u2f("#HB %u %u\r\n", (unsigned)HAL_GetTick(), (unsigned)s_hb_count);
}

static void u2_status(void)
{
    logf("[U2] rx=%u echo=%u hb=%u drop=%u rxerr=%u | "
         "V=%u M=%u P=%u S=%u rst=%u unk=%u | lines=%u over=%u bad=%u\r\n",
         (unsigned)s_rx_bytes, (unsigned)s_echo_bytes, (unsigned)s_hb_count,
         (unsigned)s_drop, (unsigned)s_rxerr,
         (unsigned)s_cmd_count[PROTO_CMD_V],
         (unsigned)s_cmd_count[PROTO_CMD_M],
         (unsigned)s_cmd_count[PROTO_CMD_P],
         (unsigned)s_cmd_count[PROTO_CMD_S],
         (unsigned)s_cmd_count[PROTO_CMD_RESET],
         (unsigned)s_cmd_count[PROTO_CMD_UNKNOWN],
         (unsigned)s_parser.stat_lines, (unsigned)s_parser.stat_overflow,
         (unsigned)s_parser.stat_bad);
}

/** 'k' —— 主动吐一串固定测试模式，让树莓派校验"逐字节一致" */
static void u2_kick(void)
{
    logf("[U2] kick: sending fixed pattern on USART2\r\n");
    u2f("#KICK 0123456789 ABCDEFGHIJKLMNOPQRSTUVWXYZ abcdefghijklmnopqrstuvwxyz\r\n");
}

/* ---------------------------------------------------------------------------
 * 影子模式（'.s' 切换，默认关）—— B1
 *
 *   把一条 `V,<vx>,<vy>,<w>` 真的走一遍**最终固件要走的那条路**：
 *
 *       协议帧(mecanum_twist_t)
 *         -> mecanum_solve()        逆解 + 饱和归一 + 死区抬升 + 装配映射
 *         -> fixp_mm_s_to_counts()  mm/s -> 本控制窗口的编码器计数
 *
 *   ★ 只打印，不驱动任何电机 —— 这是台架，电机还没接、PID 还没写。
 *     打印的 `target=` 就是将来要喂给速度环 PID 的**设定值**，
 *     也就是"协议帧 → 轮子"这条链上唯一还看不见的一环。
 *
 *   ★ 输出走 logf()，而 logf() 是**两个口都发**（USART1/COM9 + USART2/树莓派），
 *     所以影子结果在两侧都能看到。默认关着 ⇒ 对
 *     pi_setup/test_stm32_link.py 的 11/11 **逐帧无影响**；打开后每帧多 5 行，
 *     可能把某个心跳的发送时刻推后十几毫秒（T1 的容差是 30 ms）。
 *
 *   为什么默认关：这个文件首先是"链路自证基线"，基线必须可复现；
 *   影子模式是它的下游消费者，不能改变它的输出。
 * ------------------------------------------------------------------------- */
static void u2_shadow_v(int32_t vx, int32_t vy, int32_t w)
{
    static const char *const wheel_name[MECANUM_WHEEL_COUNT] = { "FL", "FR", "RL", "RR" };

    mecanum_geom_t   geom  = mecanum_default_geom();
    mecanum_limits_t lim   = mecanum_default_limits();
    mecanum_mount_t  mount = mecanum_identity_mount();
    mecanum_twist_t  tw;
    mecanum_solve_result_t res;
    fixp_scale_t     scale = fixp_default_scale();
    int              i;

    tw.vx_mm_s      = vx;
    tw.vy_mm_s      = vy;
    tw.omega_mrad_s = w;

    if (mecanum_geom_is_valid(&geom) != 1)
    {
        logf("[SHADOW] geom INVALID -> 四轮全部置 0（不会输出假速度）\r\n");
        return;
    }

    mecanum_solve(&geom, &lim, &mount, &tw, &res);

    logf("[SHADOW] V,%d,%d,%d -> L=%d mm  scale=%d cnt/m  sat=%u lift=%u idle=%u\r\n",
         (int)vx, (int)vy, (int)w,
         (int)mecanum_l_mm(&geom), (int)scale.counts_per_m,
         (unsigned)res.saturated, (unsigned)res.lifted, (unsigned)res.idle);

    for (i = 0; i < (int)MECANUM_WHEEL_COUNT; i++)
    {
        logf("[SHADOW]   %s wheel=%d mm/s  motor=%d mm/s  target=%d cnt/%dms\r\n",
             wheel_name[i],
             (int)res.wheels.wheel_mm_s[i],
             (int)res.motor_mm_s[i],
             (int)fixp_mm_s_to_counts(&scale, res.motor_mm_s[i]),
             (int)scale.period_ms);
    }
}

/** 一条完整协议报文解出来了 */
static void u2_on_message(const proto_msg_t *m)
{
    char raw[PROTO_LINE_MAX + 1];
    uint8_t i;

    for (i = 0; i < m->raw_len && i < PROTO_LINE_MAX; i++) { raw[i] = m->raw[i]; }
    raw[i] = '\0';

    if (m->cmd < U2_CMD_SLOTS) { s_cmd_count[m->cmd]++; }

    logf("[U2] RX '%s' -> cmd=%s argc=%u err=0x%X\r\n",
         raw, proto_cmd_name(m->cmd), (unsigned)m->argc, (unsigned)m->error);

    if (m->argc > 0u)
    {
        logf("[U2]    arg: %d %d %d %d %d %d\r\n",
             (int)m->arg[0], (int)m->arg[1], (int)m->arg[2],
             (int)m->arg[3], (int)m->arg[4], (int)m->arg[5]);
    }

    /* 影子模式：V 帧真的算一遍（只打印，不驱动；见 u2_shadow_v）
     * 要求 error == NONE 且 argc >= 3 —— 被标了错的帧不许产生"看起来能用"的目标值。 */
    if ((s_shadow_on != 0u) && (m->cmd == PROTO_CMD_V) &&
        (m->error == PROTO_ERR_NONE) && (m->argc >= 3u))
    {
        u2_shadow_v(m->arg[0], m->arg[1], m->arg[2]);
    }

    if (s_ack_on != 0u)
    {
        u2f("#ACK %s %u\r\n", proto_cmd_name(m->cmd), (unsigned)m->argc);
    }
}

/** 收到一个字节：先无条件回显，再判断是本地命令还是协议帧
 *
 *  ★★ 为什么本地命令必须带前缀 '.'，不能是裸字符：
 *     第一版把本地命令写成裸的 ? e h a c k，实机一跑就撞车 —— 协议命令
 *     `reset` 里的第二个字符就是 'e'，于是树莓派发 `reset\n` 时：
 *          r  -> 进解析器（缓冲 "r"）
 *          e  -> 被本地命令吃掉，顺手把回显关了（!!）
 *          s  -> 进解析器
 *          t  -> 进解析器
 *     解析器最后只看到 "rst" -> cmd=UNKNOWN。协议层完全没错，是这一层拦错了。
 *     ⇒ 用 '.' 作前缀：'.' 不在任何协议命令字符集里，永远不可能误拦；
 *       而且 '.' 本身也不是数字/逗号/换行，喂给解析器也只会被判 BAD_CHAR，
 *       所以绝不会污染正常帧。
 * ------------------------------------------------------------------------- */
static void u2_handle_byte(uint8_t b)
{
    s_rx_bytes++;

    /* ① 原样回显（loopback 判据：树莓派写什么就应读到什么） */
    if (s_echo_on != 0u)
    {
        (void)HAL_UART_Transmit(&huart2, &b, 1u, HAL_MAX_DELAY);
        s_echo_bytes++;
    }

    /* ② 前缀已激活：这一个是本地命令字母 */
    if (s_local_mode != 0u)
    {
        s_local_mode = 0u;
        switch (b)
        {
        case '?':
            u2_status();
            u2f("#STAT rx=%u echo=%u hb=%u drop=%u rxerr=%u lines=%u\r\n",
                (unsigned)s_rx_bytes, (unsigned)s_echo_bytes, (unsigned)s_hb_count,
                (unsigned)s_drop, (unsigned)s_rxerr, (unsigned)s_parser.stat_lines);
            return;
        case 'e':
            s_echo_on = (uint8_t)(s_echo_on ? 0u : 1u);
            logf("[U2] echo = %s\r\n", s_echo_on ? "ON" : "OFF");
            return;
        case 'h':
            s_hb_on = (uint8_t)(s_hb_on ? 0u : 1u);
            logf("[U2] heartbeat = %s\r\n", s_hb_on ? "ON" : "OFF");
            return;
        case 'a':
            s_ack_on = (uint8_t)(s_ack_on ? 0u : 1u);
            logf("[U2] ack frame = %s\r\n", s_ack_on ? "ON" : "OFF");
            return;
        case 'c':
            s_rx_bytes = 0u; s_echo_bytes = 0u; s_hb_count = 0u;
            s_drop = 0u; s_rxerr = 0u;
            {
                uint32_t k;
                for (k = 0u; k < U2_CMD_SLOTS; k++) { s_cmd_count[k] = 0u; }
            }
            proto_init(&s_parser);
            logf("[U2] counters cleared\r\n");
            return;
        case 'k':
            u2_kick();
            return;
        case 's':
            s_shadow_on = (uint8_t)(s_shadow_on ? 0u : 1u);
            logf("[U2] shadow (V -> mecanum_solve -> PID target) = %s\r\n",
                 s_shadow_on ? "ON" : "OFF");
            return;
        default:
            logf("[U2] unknown local key '%c' (prefix is '%c')\r\n",
                 (char)b, (char)U2_LOCAL_PREFIX);
            return;
        }
    }

    /* ③ 前缀字节本身：只置位，不进解析器 */
    if (b == (uint8_t)U2_LOCAL_PREFIX)
    {
        s_local_mode = 1u;
        return;
    }

    /* ④ 喂给 firmware_core 的协议解析器（与最终固件用的是同一份代码） */
    if (proto_feed(&s_parser, b, &s_msg) == 1)
    {
        u2_on_message(&s_msg);
    }
}

/* ===========================================================================
 * 主入口
 * =========================================================================== */

void usart2_bench_run(void)
{
    uint32_t t_hb;
    uint32_t t_stat;

    u2_init();
    proto_init(&s_parser);
    proto_msg_init(&s_msg);

    u2_banner();

    t_hb   = HAL_GetTick() + HB_PERIOD_MS;
    t_stat = HAL_GetTick() + STAT_PERIOD_MS;

    for (;;)
    {
        uint8_t b;

        /* ---- 收：把队列排干 ---- */
        while (u2_pop(&b) == 1)
        {
            u2_handle_byte(b);
        }

        /* ---- 心跳：证明 STM32 -> 树莓派 方向 ---- */
        if (s_hb_on != 0u && (int32_t)(HAL_GetTick() - t_hb) >= 0)
        {
            t_hb += HB_PERIOD_MS;
            u2_heartbeat();
        }

        /* ---- 周期统计：打在 COM9 和树莓派两边 ---- */
        if ((int32_t)(HAL_GetTick() - t_stat) >= 0)
        {
            t_stat += STAT_PERIOD_MS;
            u2_status();
        }
    }
}
