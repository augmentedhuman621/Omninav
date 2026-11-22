/*
 * protocol.c — 串口收包状态机实现
 *
 * 纯整数、零依赖：本文件只 include "protocol.h"（后者只 include <stdint.h>）。
 * 无动态内存、无全局可变状态、无递归、无浮点。
 *
 * 容错策略的完整说明见 protocol.h 文件头。
 */

#include "protocol.h"

/* ------------------------------------------------------------------ */
/* 内部类型与工具                                                      */
/* ------------------------------------------------------------------ */

typedef struct {
    int32_t value;
    uint8_t ok;    /* 1 = 是合法十进制整数 */
    uint8_t range; /* 1 = 超出 int32 范围，已饱和 */
} proto_num_t;

static int proto_is_digit(char c)
{
    return (c >= '0' && c <= '9');
}

static int proto_is_space(char c)
{
    return (c == ' ' || c == '\t');
}

static char proto_lower(char c)
{
    if (c >= 'A' && c <= 'Z') {
        return (char)(c - 'A' + 'a');
    }
    return c;
}

/*
 * 解析一个十进制整数（已经是「一个字段」的字节区间，不含逗号）。
 * 只接受可选 '-' 加至少一位数字。超范围时饱和到 int32 的边界并置 range。
 * 不用 strtol（那要 <stdlib.h>，且算法层不许引第三方/标准库解析函数）。
 */
static proto_num_t proto_parse_i32(const char *s, uint16_t n)
{
    proto_num_t r;
    uint16_t i = (uint16_t)0;
    int neg = 0;
    int any = 0;
    int clamped = 0;
    int64_t acc = 0;

    r.value = 0;
    r.ok    = (uint8_t)0;
    r.range = (uint8_t)0;

    if (n == (uint16_t)0) {
        return r;
    }
    if (s[0] == '-') {
        neg = 1;
        i = (uint16_t)1;
    }
    for (; i < n; ++i) {
        int64_t next;
        if (proto_is_digit(s[i]) == 0) {
            return r; /* ok 保持 0 */
        }
        any = 1;
        next = acc * 10 + (int64_t)(s[i] - '0');
        if (next > 2147483648LL) {
            /* 夹在 2^31 上，并记下「确实超出了」——
               只有这样才能区分「-2147483648 这个恰好合法的值」和
               「-99999999999999 这种真溢出」。 */
            next    = 2147483648LL;
            clamped = 1;
        }
        acc = next;
    }
    if (any == 0) {
        return r;
    }
    if (neg != 0) {
        if (clamped != 0) {
            r.range = (uint8_t)1;
            r.value = INT32_MIN;
        } else {
            r.value = (int32_t)(-acc); /* acc <= 2^31，取负安全 */
        }
    } else {
        if (acc > 2147483647LL) {
            r.range = (uint8_t)1;
            r.value = INT32_MAX;
        } else {
            r.value = (int32_t)acc;
        }
    }
    r.ok = (uint8_t)1;
    return r;
}

void proto_msg_init(proto_msg_t *msg)
{
    int i;
    if (msg == 0) {
        return;
    }
    msg->cmd     = PROTO_CMD_NONE;
    msg->argc    = (uint8_t)0;
    msg->error   = (uint16_t)0;
    msg->raw_len = (uint8_t)0;
    for (i = 0; i < PROTO_ARG_MAX; ++i) {
        msg->arg[i] = 0;
    }
    msg->raw[0] = '\0';
}

void proto_init(proto_parser_t *p)
{
    uint16_t i;
    if (p == 0) {
        return;
    }
    for (i = (uint16_t)0; i < (uint16_t)PROTO_LINE_MAX; ++i) {
        p->buf[i] = '\0';
    }
    p->len           = (uint16_t)0;
    p->discarding    = (uint8_t)0;
    p->stat_lines    = 0u;
    p->stat_overflow = 0u;
    p->stat_bad      = 0u;
    p->stat_empty    = 0u;
}

/* ------------------------------------------------------------------ */
/* 一行 <- 一条报文                                                    */
/* ------------------------------------------------------------------ */

static void proto_parse_line(const char *buf, uint16_t n, proto_msg_t *msg)
{
    uint16_t err = (uint16_t)0;
    uint16_t b;
    uint16_t e;
    uint16_t p;
    int argc = 0;
    proto_cmd_t cmd = PROTO_CMD_UNKNOWN;
    uint16_t i;

    proto_msg_init(msg);

    /* 1) 原文存档 + 非法字符扫描（可打印 ASCII 之外一律算非法字节）
     *    ★ 制表符例外：它是被显式允许的分隔空白（见 proto_is_space）。
     *      反过来，只有 ' ' 和 '\t' 算空白；0x0B / 0x0C / 0x85 / 0xA0
     *      这些恰恰是「波特率不对」的症状，必须留下来报错，不能当空白剥掉
     *      —— 所以这里绝不能按「广义控制字符 = 空白」来放行。 */
    for (i = (uint16_t)0; i < n; ++i) {
        uint8_t c = (uint8_t)buf[i];
        msg->raw[i] = buf[i];
        if ((c < (uint8_t)0x20u && c != (uint8_t)'\t') || c > (uint8_t)0x7Eu) {
            err = (uint16_t)(err | PROTO_ERR_BAD_CHAR);
        }
    }
    msg->raw[n] = '\0';
    msg->raw_len = (uint8_t)n;

    /* 2) 去掉首尾空白 */
    b = (uint16_t)0;
    e = n;
    while (b < e && proto_is_space(buf[b]) != 0) {
        b = (uint16_t)(b + 1);
    }
    while (e > b && proto_is_space(buf[e - 1]) != 0) {
        e = (uint16_t)(e - 1);
    }
    if (b >= e) {
        msg->error = err;
        return; /* cmd 保持 NONE：纯空白行，调用者会忽略 */
    }

    /* 3) 命令字 */
    switch (proto_lower(buf[b])) {
    case 'v': cmd = PROTO_CMD_V; break;
    case 'm': cmd = PROTO_CMD_M; break;
    case 'p': cmd = PROTO_CMD_P; break;
    case 's': cmd = PROTO_CMD_S; break;
    case 'r': cmd = PROTO_CMD_RESET; break;
    default: cmd = PROTO_CMD_UNKNOWN; break;
    }
    p = (uint16_t)(b + 1);

    /* 4) 命令字之后：核对剩余部分 */
    if (cmd == PROTO_CMD_RESET) {
        if ((uint16_t)(e - p) != (uint16_t)4) {
            cmd = PROTO_CMD_UNKNOWN;
        } else {
            static const char k_eset[4] = { 'e', 's', 'e', 't' };
            uint16_t k;
            uint8_t ok = (uint8_t)1;
            for (k = (uint16_t)0; k < (uint16_t)4; ++k) {
                if (proto_lower(buf[p + k]) != k_eset[k]) {
                    ok = (uint8_t)0;
                    break;
                }
            }
            if (ok != (uint8_t)0) {
                p = e;
            } else {
                cmd = PROTO_CMD_UNKNOWN;
            }
        }
    } else if (cmd == PROTO_CMD_S) {
        if (p != e) {
            cmd = PROTO_CMD_UNKNOWN; /* S 之后必须直接是行尾 */
        }
    } else if (cmd == PROTO_CMD_V || cmd == PROTO_CMD_M || cmd == PROTO_CMD_P) {
        if (p >= e || buf[p] != ',') {
            cmd = PROTO_CMD_UNKNOWN; /* 命令字后面缺逗号 */
        } else {
            p = (uint16_t)(p + 1);
        }
    }

    /* 5) 逐字段解析（只有 V / M / P 有字段） */
    if (cmd == PROTO_CMD_V || cmd == PROTO_CMD_M || cmd == PROTO_CMD_P) {
        for (;;) {
            uint16_t fs = p;
            uint16_t fe = p;
            proto_num_t r;
            while (fe < e && buf[fe] != ',') {
                fe = (uint16_t)(fe + 1);
            }
            if (argc >= PROTO_ARG_MAX) {
                err = (uint16_t)(err | PROTO_ERR_ARGC);
            } else {
                r = proto_parse_i32(&buf[fs], (uint16_t)(fe - fs));
                if (r.ok == (uint8_t)0) {
                    err = (uint16_t)(err | PROTO_ERR_BAD_NUMBER);
                }
                if (r.range != (uint8_t)0) {
                    err = (uint16_t)(err | PROTO_ERR_RANGE);
                }
                msg->arg[argc] = r.value;
                argc++;
            }
            if (fe >= e) {
                break;
            }
            p = (uint16_t)(fe + 1);
        }
    }

    /* 6) 参数个数核对 + 未知命令 */
    {
        int want = -1;
        switch (cmd) {
        case PROTO_CMD_V: want = 3; break;
        case PROTO_CMD_M: want = 4; break;
        case PROTO_CMD_P: want = 4; break;
        case PROTO_CMD_S: want = 0; break;
        case PROTO_CMD_RESET: want = 0; break;
        default: want = -1; break;
        }
        if (want >= 0 && argc != want) {
            err = (uint16_t)(err | PROTO_ERR_ARGC);
        }
        if (cmd == PROTO_CMD_UNKNOWN) {
            err = (uint16_t)(err | PROTO_ERR_UNKNOWN_CMD);
        }
    }

    msg->cmd   = cmd;
    msg->argc  = (uint8_t)argc;
    msg->error = err;
}

/* ------------------------------------------------------------------ */
/* 逐字节喂入                                                          */
/* ------------------------------------------------------------------ */

int proto_feed(proto_parser_t *p, uint8_t byte, proto_msg_t *out)
{
    if (p == 0) {
        return 0;
    }

    /* 丢弃模式：一路丢到换行为止（超长行的唯一正确收尾方式） */
    if (p->discarding != (uint8_t)0) {
        if (byte == (uint8_t)'\n') {
            p->discarding = (uint8_t)0;
            p->len        = (uint16_t)0;
        }
        return 0;
    }

    if (byte == (uint8_t)'\n') {
        uint16_t n = p->len;
        proto_msg_t tmp;
        p->len = (uint16_t)0;
        if (n == (uint16_t)0) {
            p->stat_empty++;
            return 0;
        }
        proto_parse_line(p->buf, n, &tmp);
        if (tmp.cmd == PROTO_CMD_NONE && tmp.error == (uint16_t)0) {
            p->stat_empty++;
            return 0; /* 纯空白行 */
        }
        if ((tmp.error & PROTO_ERR_BAD_CHAR) != (uint16_t)0) {
            p->stat_bad++;
        }
        p->stat_lines++;
        if (out != 0) {
            *out = tmp;
        }
        return 1;
    }

    if (byte == (uint8_t)'\r') {
        return 0; /* 兼容 CRLF */
    }

    if (p->len >= (uint16_t)(PROTO_LINE_MAX - 1)) {
        /* 本行已经太长：作废，进入丢弃模式，并且必须一直丢到换行，
           否则这一行的尾巴会被当下一条报文的开头 —— 那才是最坏的情况。 */
        p->stat_overflow++;
        p->discarding = (uint8_t)1;
        p->len        = (uint16_t)0;
        return 0;
    }

    p->buf[p->len] = (char)byte;
    p->len         = (uint16_t)(p->len + 1);
    return 0;
}

int proto_feed_bytes(proto_parser_t *p, const uint8_t *buf, uint16_t n,
                     proto_msg_t *out)
{
    int got = 0;
    uint16_t i;
    proto_msg_t tmp;

    if (p == 0 || buf == 0) {
        return 0;
    }
    proto_msg_init(&tmp);
    for (i = (uint16_t)0; i < n; ++i) {
        if (proto_feed(p, buf[i], &tmp) != 0) {
            got = 1;
            if (out != 0) {
                *out = tmp;
            }
            proto_msg_init(&tmp);
        }
    }
    return got;
}

const char *proto_cmd_name(proto_cmd_t cmd)
{
    switch (cmd) {
    case PROTO_CMD_V: return "V";
    case PROTO_CMD_M: return "M";
    case PROTO_CMD_P: return "P";
    case PROTO_CMD_S: return "S";
    case PROTO_CMD_RESET: return "reset";
    case PROTO_CMD_UNKNOWN: return "UNKNOWN";
    default: return "NONE";
    }
}
