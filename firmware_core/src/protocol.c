/*
 * protocol.c -- serial receive state machine implementation
 *
 * Integer-only, zero dependencies: this file only includes "protocol.h" (which only includes <stdint.h>).
 * No dynamic memory, no global mutable state, no recursion, no floating point.
 *
 * See the protocol.h file header for the full fault-tolerance policy.
 */

#include "protocol.h"

/* ------------------------------------------------------------------ */
/* Internal types and helpers                                         */
/* ------------------------------------------------------------------ */

typedef struct {
    int32_t value;
    uint8_t ok;    /* 1 = valid decimal integer */
    uint8_t range; /* 1 = out of int32 range, saturated */
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
 * Parse one decimal integer (the byte range of one field, no comma).
 * Accepts an optional '-' plus at least one digit; out of range saturates to the
 * int32 bound and sets range. No strtol (needs <stdlib.h>; the algorithm layer forbids it).
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
            return r; /* ok stays 0 */
        }
        any = 1;
        next = acc * 10 + (int64_t)(s[i] - '0');
        if (next > 2147483648LL) {
            /* Saturate at 2^31 and remember that it really overflowed -- this is
               the only way to tell the just-legal value -2147483648 apart from
               a genuine overflow like -99999999999999. */
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
            r.value = (int32_t)(-acc); /* acc <= 2^31, negation is safe */
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
/* One line <- one message                                            */
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

    /* 1) Archive the raw bytes + illegal-char scan (anything outside printable
     *    ASCII is illegal). Tab is the one exception: an explicitly allowed
     *      separator space (see proto_is_space). Only ' ' and '\t' count as space;
     *      0x0B / 0x0C / 0x85 / 0xA0 are the very symptoms of a wrong baud rate, so
     *      they must be reported, never stripped -- never treat control chars as space. */
    for (i = (uint16_t)0; i < n; ++i) {
        uint8_t c = (uint8_t)buf[i];
        msg->raw[i] = buf[i];
        if ((c < (uint8_t)0x20u && c != (uint8_t)'\t') || c > (uint8_t)0x7Eu) {
            err = (uint16_t)(err | PROTO_ERR_BAD_CHAR);
        }
    }
    msg->raw[n] = '\0';
    msg->raw_len = (uint8_t)n;

    /* 2) Trim leading and trailing whitespace */
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
        return; /* cmd stays NONE: blank line, caller ignores it */
    }

    /* 3) Command word */
    switch (proto_lower(buf[b])) {
    case 'v': cmd = PROTO_CMD_V; break;
    case 'm': cmd = PROTO_CMD_M; break;
    case 'p': cmd = PROTO_CMD_P; break;
    case 's': cmd = PROTO_CMD_S; break;
    case 'r': cmd = PROTO_CMD_RESET; break;
    default: cmd = PROTO_CMD_UNKNOWN; break;
    }
    p = (uint16_t)(b + 1);

    /* 4) After the command word: verify the rest */
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
            cmd = PROTO_CMD_UNKNOWN; /* end of line must follow S directly */
        }
    } else if (cmd == PROTO_CMD_V || cmd == PROTO_CMD_M || cmd == PROTO_CMD_P) {
        if (p >= e || buf[p] != ',') {
            cmd = PROTO_CMD_UNKNOWN; /* missing comma after the command word */
        } else {
            p = (uint16_t)(p + 1);
        }
    }

    /* 5) Parse field by field (only V / M / P carry fields) */
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

    /* 6) Argument-count check + unknown command */
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
/* Byte-by-byte feed                                                  */
/* ------------------------------------------------------------------ */

int proto_feed(proto_parser_t *p, uint8_t byte, proto_msg_t *out)
{
    if (p == 0) {
        return 0;
    }

    /* Discard mode: drop everything until a newline (the only correct way to end an overlong line) */
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
            return 0; /* blank line */
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
        return 0; /* tolerate CRLF */
    }

    if (p->len >= (uint16_t)(PROTO_LINE_MAX - 1)) {
        /* Line already too long: void it, enter discard mode, and keep discarding
           until a newline -- otherwise this line's tail would lead the next message: worst case. */
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
