/**
 * usart2_bench.c -- USART2 link self-test bench (the only data link between the
 *                  OmniNav lower controller and the Raspberry Pi 5)
 * =============================================================================
 *
 * What this file is for
 * -----------------------------------------------------------------------------
 *   Proves that the "Raspberry Pi 5 <-> STM32" wire really works, and runs the
 *   firmware_core protocol layer once on the real hardware. It is a DIAGNOSTIC
 *   tool, not the final firmware -- in the final firmware USART2 is driven by the
 *   control loop at 50 Hz, whereas here it is request/response + heartbeat.
 *
 *   Power-up boots into here by default (see the boot routing in main.c); pressing
 *   'm' on USART1 (COM9) within 3 s switches to the original motor/probe bench,
 *   bench.c.
 *
 * Hardware wiring (done by the user in 2026-10)
 * -----------------------------------------------------------------------------
 *   STM32 PA2 (USART2_TX)  -->  USB-TTL module RXD
 *   STM32 PA3 (USART2_RX)  <--  USB-TTL module TXD
 *   STM32 GND              <->  USB-TTL module GND
 *   USB-TTL module VCC     ---  leave open (STM32 is powered from ST-Link/DC jack)
 *
 *   *** P5 jumper MUST be removed
 * -----------------------------------------------------------------------------
 *   The Elite board brings PA2 / PA3 out to a 4-pin header called **P5 USART2**
 *   (bottom-left corner of the board, next to the DC power jack), silkscreen 2x2:
 *
 *        +------+------+
 *        | 48T  | 48R  |   <- wired to the on-board SP3485 (RS485 transceiver)
 *        +------+------+
 *        | PA3  | PA2  |   <- STM32 USART2_RX / USART2_TX
 *        +------+------+
 *
 *   The jumper shorts the two rows. **Jumper fitted => PA2/PA3 sit on the SP3485**
 *   (485 mode); the TTL signal is then loaded by the 485 chip, and the symptom is
 *   "nothing is received at all" or "screens full of garbage".
 *   **Jumper removed => PA2/PA3 are cut from the 485 chip, and those two holes are
 *   pure 3.3 V TTL, ready for Dupont wires.**
 *   (Source: ALIENTEK forum thread 311106 "with the P5 jumper removed it can be
 *   used as RS485-to-TTL")
 *
 *   Check the jumper in 30 s: multimeter in continuity mode, one probe on the PA2
 *   hole and the other on the 48R hole -- beep / 0 ohm => jumper still fitted (must
 *   be removed); open => removed (usable).
 *
 *   Also: with TTL only, **PD7 (485 direction enable) needs no attention at all**.
 *
 * Three pitfalls (all hit before, or recorded in other files)
 * -----------------------------------------------------------------------------
 *   1) **Never redirect printf**. Without MicroLIB checked, Keil's printf pulls in
 *      semihosting and hangs the program; this file uses a ~60-line formatter of
 *      its own and never touches <stdio.h>.
 *   2) **USART2 hangs off APB1 = 36 MHz, not APB2's 72 MHz**. HAL computes BRR
 *      from PCLK1 by itself, but the resulting BRR must be printed and checked
 *      (expected 0x0138).
 *   3) **Output must be ASCII only** (handover pitfall 26). The custom formatter
 *      does not support Chinese; non-ASCII comes out as '?'.
 *
 * Why interrupt-driven RX + ring buffer instead of polling
 * -----------------------------------------------------------------------------
 *   Echo is sent blocking, byte by byte; one byte at 115200 takes ~87 us. With
 *   polled RX the bytes arriving during a transmit are lost to hardware ORE, so the
 *   echo is no longer verifiable byte by byte => a measured "byte loss" cannot be
 *   attributed to the wire or to the firmware. Therefore: **RXNE interrupt +
 *   256-byte ring buffer**; the ISR only does "read DR, enqueue", everything else
 *   runs in the main loop and never calls HAL.
 *
 *   At the same time **do not use HAL's HAL_UART_Receive_IT** (its state machine
 *   stops receiving on ORE and has to be re-armed in ErrorCallback, which easily
 *   wedges RX). Instead this file implements USART2_IRQHandler itself and reads
 *   SR/DR directly -- one SR read followed by one DR read clears both RXNE and ORE,
 *   so ORE can never stall us. stm32f1xx_it.c never defined USART2_IRQHandler, so
 *   there is no clash.
 *
 * How to use it (Raspberry Pi side)
 * -----------------------------------------------------------------------------
 *   device: /dev/ttySTM32 (udev rule installed, points at ttyUSB0)
 *   params: 115200 8N1, no flow control
 *
 *   Watch the heartbeat (proves STM32 -> Raspberry Pi direction):
 *     python3 -c "import serial;s=serial.Serial('/dev/ttySTM32',115200,timeout=3);print(s.readline())"
 *
 *   Send one motion command (proves Raspberry Pi -> STM32 direction + protocol):
 *     printf 'V,100,0,0\n' > /dev/ttySTM32
 *     => echoed back locally as `V,100,0,0`, and the STM32 answers one line
 *        `#ACK V 3`
 *
 *   Companion script for this file: pi_setup/test_stm32_link.py (runs the whole
 *   self-test in one go and reports a verdict)
 * =============================================================================
 */

#include "usart2_bench.h"
#include "main.h"
#include "bench.h"        /* only to switch to bench_run() during boot routing */
#include "protocol.h"     /* firmware_core, already on the Keil IncludePath */
#include "mecanum.h"      /* firmware_core: mecanum inverse/forward + one-call solve (shadow mode) */
#include "fixedpoint.h"   /* firmware_core: mm/s -> counts per control window (PID setpoint) */
#include <stdarg.h>

/* The USART1 handle is defined in main.c (same style as bench.c) */
extern UART_HandleTypeDef huart1;

/* Overrides the weak default implementation in startup_stm32f103xe.s.
 * stm32f1xx_it.c never defined it, so there is no clash. */
void USART2_IRQHandler(void);

/* ===========================================================================
 * Configuration
 * =========================================================================== */

#define U2_BAUD          115200u   /* frozen by contract, do not change */
#define U2_FIFO_SZ       256u      /* must be a power of two */
#define U2_FIFO_MASK     (U2_FIFO_SZ - 1u)
#define HB_PERIOD_MS     1000u     /* heartbeat period */
#define STAT_PERIOD_MS   5000u     /* status line period */
#define U2_LINE_MAX      160u      /* single line output buffer */

/* number of proto_cmd_t values (NONE/V/M/P/S/RESET/UNKNOWN) */
#define U2_CMD_SLOTS     7u

/* ** Local command prefix. Must be a character outside the protocol charset
 *    (the protocol only uses letters/digits/','/'-'/'\n').
 *    Why '.': it can never appear in a protocol frame, so it can never swallow a
 *    normal command by mistake; even if it leaks into the parser it is only
 *    flagged BAD_CHAR and never pollutes the following frames.
 *    => local commands are written '.?' '.e' '.h' '.a' '.c' '.k' (see u2_handle_byte()) */
#define U2_LOCAL_PREFIX  '.'

/* ===========================================================================
 * State
 * =========================================================================== */

static UART_HandleTypeDef huart2;

/* --- receive ring buffer (ISR writes head, main loop reads tail) --- */
static volatile uint8_t  s_fifo[U2_FIFO_SZ];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;
static volatile uint32_t s_drop;      /* bytes dropped because the queue was full */
static volatile uint32_t s_rxerr;     /* ORE/FE/NE/PE count */

/* --- statistics --- */
static uint32_t s_rx_bytes;           /* bytes received on PA3 */
static uint32_t s_echo_bytes;         /* bytes echoed back */
static uint32_t s_hb_count;           /* heartbeats sent */
static uint32_t s_cmd_count[U2_CMD_SLOTS];

/* --- switches --- */
static uint8_t s_echo_on = 1u;        /* toggled by '.e', on by default */
static uint8_t s_hb_on   = 1u;        /* toggled by '.h', on by default */
static uint8_t s_ack_on  = 1u;        /* toggled by '.a', on by default */

/* Shadow mode: toggled by '.s', **off by default**.
 * While it is off, the **per-frame behaviour** on USART2 is identical to the
 * "11/11 link self-test" baseline (the only difference is two extra lines in the
 * boot banner, which are one-shot and not inspected by the test script).
 * While it is on, every frame gains 5 extra lines -- logf() sends on **both
 * ports** (see send_on/logf), so shadow results are visible on COM9 and on the
 * Raspberry Pi side alike. */
static uint8_t s_shadow_on;

/* --- local command prefix state (set to 1 after U2_LOCAL_PREFIX; the next byte
 *     is the command letter) --- */
static uint8_t s_local_mode;

static proto_parser_t s_parser;
static proto_msg_t    s_msg;

/* ===========================================================================
 * Mini formatter (deliberately no %f / 64-bit / zero padding -- unused here)
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
 * Supported conversions: %s %c %d %i %u %x %X %%
 * Anything else (field width, '0' flag, l/h prefix) is skipped; no padding/alignment.
 * Return value = number of bytes written (excluding the trailing NUL); >= cap means truncated.
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

        /* skip flag characters and field width (this file does not rely on alignment) */
        while (*fmt == '0' || (*fmt >= '1' && *fmt <= '9')) { fmt++; }
        /* skip l / h length prefixes */
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
                /* INT_MIN-safe formulation */
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
            /* unknown conversion: emit it verbatim so typos are visible */
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
 * Output: both ports / USART2 only
 * =========================================================================== */

static void send_on(UART_HandleTypeDef *h, const char *s, int n)
{
    if (n <= 0) { return; }
    (void)HAL_UART_Transmit(h, (const uint8_t *)s, (uint16_t)n, HAL_MAX_DELAY);
}

/** Put the same line on USART1 (PC COM9) and USART2 (Raspberry Pi) for side-by-side comparison */
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

/** USART2 only -- machine-readable lines (starting with '#'), keeps COM9 clean */
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
 * USART2 init
 * =========================================================================== */

static void u2_init(void)
{
    GPIO_InitTypeDef gpio;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    /* PA2 = USART2_TX: alternate-function push-pull. PA2/PA3 are the **default**
     * pins of USART2 and need no remap, so **AFIO is not touched here** (the TIM1
     * full remap done in bench.c does not conflict with it). */
    gpio.Pin   = GPIO_PIN_2;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PA3 = USART2_RX: floating input (HAL wants INPUT + NOPULL for UART RX, same as PA10) */
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

    /* Enable only the RXNE interrupt, do not use HAL's Receive_IT (see file header) */
    HAL_NVIC_SetPriority(USART2_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/**
 * USART2 interrupt service routine.
 * Only two jobs: push received bytes into the ring buffer, and clear the error flags.
 * Never call any HAL transmit function here (HAL_UART_Transmit is not reentrant).
 */
void USART2_IRQHandler(void)
{
    uint32_t sr = USART2->SR;

    if ((sr & USART_SR_RXNE) != 0u)
    {
        uint8_t b = (uint8_t)(USART2->DR & 0xFFu);   /* reading DR clears RXNE / ORE too */
        uint16_t next = (uint16_t)((s_head + 1u) & U2_FIFO_MASK);

        if (next == s_tail)
        {
            s_drop++;                                 /* queue full: drop the new byte, never overwrite the old */
        }
        else
        {
            s_fifo[s_head] = b;
            s_head = next;
        }
    }
    else if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0u)
    {
        (void)USART2->DR;                             /* clear the error flags */
        s_rxerr++;
    }
}

/** Pop one byte from the ring buffer; returns 1 = got one */
static int u2_pop(uint8_t *b)
{
    if (s_tail == s_head) { return 0; }
    *b = s_fifo[s_tail];
    s_tail = (uint16_t)((s_tail + 1u) & U2_FIFO_MASK);
    return 1;
}

/* ===========================================================================
 * Output content
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
    /* machine-readable line: starts with '#', easy to spot on the Raspberry Pi */
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

/** 'k' -- dump a fixed test pattern so the Raspberry Pi can check "byte-for-byte identical" */
static void u2_kick(void)
{
    logf("[U2] kick: sending fixed pattern on USART2\r\n");
    u2f("#KICK 0123456789 ABCDEFGHIJKLMNOPQRSTUVWXYZ abcdefghijklmnopqrstuvwxyz\r\n");
}

/* ---------------------------------------------------------------------------
 * Shadow mode (toggled by '.s', off by default) -- B1
 *
 *   Runs a `V,<vx>,<vy>,<w>` frame through **the same path the final firmware
 *   will take**:
 *
 *       protocol frame (mecanum_twist_t)
 *         -> mecanum_solve()        inverse + saturate normalize + deadzone lift + mount map
 *         -> fixp_mm_s_to_counts()  mm/s -> encoder counts for this control window
 *
 *   Only prints, never drives a motor -- this is a bench: the motors are not
 *   wired yet and the PID is not written yet. The printed `target=` is the
 *   **setpoint** that will later feed the speed-loop PID, i.e. the only link in
 *   the "protocol frame -> wheels" chain that is still invisible.
 *
 *   Output goes through logf(), and logf() sends on **both ports** (USART1/COM9 +
 *   USART2/Raspberry Pi), so shadow results are visible on both sides. Off by
 *   default => **no per-frame effect** on the 11/11 in
 *   pi_setup/test_stm32_link.py; when on, every frame gains 5 lines and may push
 *   a heartbeat out by a dozen milliseconds (T1's tolerance is 30 ms).
 *
 *   Why off by default: this file is first of all the "link self-test baseline",
 *   and a baseline must be reproducible; shadow mode is a downstream consumer of
 *   it and must not change its output.
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
        logf("[SHADOW] geom INVALID -> all four wheels forced to 0 (never emits fake speeds)\r\n");
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

/** A complete protocol message has been decoded */
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

    /* Shadow mode: actually compute the V frame (print only, no drive; see u2_shadow_v)
     * Requires error == NONE and argc >= 3 -- a frame flagged as bad must never
     * produce a "looks usable" target value. */
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

/** One byte received: echo it unconditionally first, then decide whether it is a
 * local command or a protocol frame
 *
 *  Why local commands must carry the '.' prefix and cannot be bare characters:
 *     the first version used bare ? e h a c k, and it collided on real hardware --
 *     the second character of protocol command `reset` is 'e', so when the
 *     Raspberry Pi sent `reset\n`:
 *          r  -> went to the parser (buffer "r")
 *          e  -> swallowed by the local command, which also turned echo off (!!)
 *          s  -> went to the parser
 *          t  -> went to the parser
 *     the parser finally saw only "rst" -> cmd=UNKNOWN. The protocol layer was
 *     entirely correct; this layer swallowed it.
 *     => use '.' as the prefix: '.' is in no protocol command charset, so it can
 *        never swallow a command by mistake; and '.' is neither a digit, a comma
 *        nor a newline, so feeding it to the parser only yields BAD_CHAR and can
 *        never pollute a normal frame.
 * ------------------------------------------------------------------------- */
static void u2_handle_byte(uint8_t b)
{
    s_rx_bytes++;

    /* (1) echo verbatim (loopback criterion: whatever the Raspberry Pi writes must come back) */
    if (s_echo_on != 0u)
    {
        (void)HAL_UART_Transmit(&huart2, &b, 1u, HAL_MAX_DELAY);
        s_echo_bytes++;
    }

    /* (2) prefix active: this byte is the local command letter */
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

    /* (3) the prefix byte itself: only sets the flag, never goes to the parser */
    if (b == (uint8_t)U2_LOCAL_PREFIX)
    {
        s_local_mode = 1u;
        return;
    }

    /* (4) feed the firmware_core protocol parser (the same code the final firmware uses) */
    if (proto_feed(&s_parser, b, &s_msg) == 1)
    {
        u2_on_message(&s_msg);
    }
}

/* ===========================================================================
 * Main entry point
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

        /* ---- RX: drain the queue ---- */
        while (u2_pop(&b) == 1)
        {
            u2_handle_byte(b);
        }

        /* ---- heartbeat: proves the STM32 -> Raspberry Pi direction ---- */
        if (s_hb_on != 0u && (int32_t)(HAL_GetTick() - t_hb) >= 0)
        {
            t_hb += HB_PERIOD_MS;
            u2_heartbeat();
        }

        /* ---- periodic status: printed on both COM9 and the Raspberry Pi ---- */
        if ((int32_t)(HAL_GetTick() - t_stat) >= 0)
        {
            t_stat += STAT_PERIOD_MS;
            u2_status();
        }
    }
}
