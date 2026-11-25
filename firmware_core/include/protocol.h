/*
 * protocol.h — serial receive state machine (fed byte by byte)
 * ---------------------------------------------------------------------------
 *
 * Design constraints (consistent with the firmware_core iron rules)
 *   - Depends only on <stdint.h>. No HAL / ROS2 / third-party / <stdio.h>.
 *   - All-integer zero floating-point, no dynamic memory, no global mutable state, no recursion.
 *   - The parse result goes into the caller-provided proto_msg_t; the parser state lives in proto_parser_t.
 *
 * Frame format (frozen by 11_interface_contract.md §3; this module only does "framing + parsing",
 * semantic validation is left to the caller)
 *
 *   Downlink Pi -> STM32:
 *     V,<vx>,<vy>,<w>\n          body velocity; vx/vy in mm/s, w in mrad/s
 *     M,<p1>,<p2>,<p3>,<p4>\n    debug: drive the four PWM channels directly, -1000 ~ 1000
 *     P,<id>,<kp>,<ki>,<kd>\n    change PID parameters online
 *     S\n                        soft emergency stop
 *     reset\n                    reset
 *
 * Fault-tolerance requirements (the five named in the acceptance criteria, each mapped to the implementation)
 *   partial packet — fed byte by byte; if it is incomplete nothing is produced (returns 0) and no state is lost.
 *   coalesced packets — one proto_feed call emits at most one message; the bytes left in the buffer must be
 *             fed again by the caller (proto_feed is a byte-at-a-time interface, so coalescing falls apart naturally).
 *             ★ Note the semantics: coalesced packets do not mean "many messages per call", but "the caller feeds
 *               bytes back to back and one message comes out per newline".
 *   overlong — once a line exceeds PROTO_LINE_MAX-1 bytes, the line is dropped and the parser enters
 *             the "discard until newline" state; parsing resumes automatically at the newline.
 *             ★ Key point: an overlong line must not poison the next frame — so it must be discarded all the way to the newline.
 *   invalid character — set PROTO_ERR_BAD_CHAR. ★ The message is **still produced** (returns 1),
 *             so the upper layer can send one A,<text> back with the reason instead of swallowing it silently;
 *             the parser state stays usable and the next frame is unaffected.
 *   unknown command — emit PROTO_CMD_UNKNOWN (with PROTO_ERR_UNKNOWN_CMD),
 *             so the upper layer can send one A,<text> back instead of swallowing it silently.
 *
 * Line endings and whitespace convention (★ must match the stm32_bridge side word for word)
 *   A line ends with '\n'; '\r' is ignored (CRLF compatible).
 *   **Only ' ' and '\t' count as whitespace**: a whitespace-only line (any mix of spaces and tabs) is ignored silently,
 *   produces nothing, and is not an error; these two characters are stripped from both ends of a line.
 *   ★ Conversely, 0x0B / 0x0C / 0x85 / 0xA0 — "other whitespace" — are **all treated as invalid characters**:
 *     they are exactly the symptom of a baud-rate mismatch, and wiping away that diagnostic clue is self-defeating.
 *     Embedded control characters (for example the 0x00 in B,118\x00 00,1) must likewise be rejected.
 *   ★ Lesson (both ends share the same source, the bridge already hit this): the order of "invalid character test" and
 *     "stripping whitespace" matters — letting control characters through as "whitespace" first and stripping afterwards
 *     makes a CRLF line ending kill the whole line, and also makes whitespace-only lines come out as a frame.
 *     The approach here: a tab gets its own exception in the invalid-character scan, while stripping only uses
 *     the narrow set { ' ', '\t' }.
 *
 * Number parsing
 *   Only an optional '-' followed by decimal digits is accepted. Empty fields, non-digit characters, and values outside
 *   the int32 range are all flagged as errors (out-of-range values saturate and set PROTO_ERR_RANGE; no undefined behaviour).
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum bytes in one line (including the command character and commas, excluding the trailing '\n').
 * 96 leaves plenty of margin: the longest P,<id>,<kp>,<ki>,<kd>\n is well under 40 bytes. */
#define PROTO_LINE_MAX 96

/* Maximum numeric arguments per message (M frames 4, P frames 4, with headroom for the future) */
#define PROTO_ARG_MAX 6

/* Bit definitions for proto_msg_t.error */
#define PROTO_ERR_NONE        0x0000u
#define PROTO_ERR_OVERFLOW    0x0001u /* line too long, dropped */
#define PROTO_ERR_BAD_CHAR    0x0002u /* invalid character seen */
#define PROTO_ERR_BAD_NUMBER  0x0004u /* field is not a valid decimal integer */
#define PROTO_ERR_RANGE       0x0008u /* value outside int32, saturated */
#define PROTO_ERR_ARGC        0x0010u /* argument count does not match the command */
#define PROTO_ERR_UNKNOWN_CMD 0x0020u /* unrecognized command character */

/* Command types */
typedef enum {
    PROTO_CMD_NONE = 0, /* no complete message assembled yet */
    PROTO_CMD_V,
    PROTO_CMD_M,
    PROTO_CMD_P,
    PROTO_CMD_S,
    PROTO_CMD_RESET,
    PROTO_CMD_UNKNOWN
} proto_cmd_t;

/* One message */
typedef struct {
    proto_cmd_t cmd;
    int32_t     arg[PROTO_ARG_MAX];
    uint8_t     argc;                 /* how many numeric arguments were actually parsed */
    uint16_t    error;                /* bitwise OR of PROTO_ERR_* */
    char        raw[PROTO_LINE_MAX];  /* raw text (without the newline), for acknowledgements / diagnostics */
    uint8_t     raw_len;
} proto_msg_t;

/* Parser state. No dynamic memory, a plain value type, freely copyable / zeroable. */
typedef struct {
    char     buf[PROTO_LINE_MAX];
    uint16_t len;
    uint8_t  discarding; /* 1 = this line is overlong, dropping until the newline */
    uint32_t stat_lines;    /* total complete lines produced */
    uint32_t stat_overflow; /* total overlong lines dropped */
    uint32_t stat_bad;      /* total invalid characters seen */
    uint32_t stat_empty;    /* total ignored empty lines */
} proto_parser_t;

/* Clear the parser (statistics included). p may be 0 (no-op). */
void proto_init(proto_parser_t *p);

/* Clear msg (so the caller never reads leftovers from the previous message). msg may be 0. */
void proto_msg_init(proto_msg_t *msg);

/*
 * Feed one byte.
 *   Returns 1 = a complete message was produced and written to out (if out is non-zero); this call
 *               has handed the message to the caller and the internal buffer is reset.
 *   Returns 0 = not assembled yet (or this line was dropped / ignored).
 *
 * p may be 0 (returns 0, no crash). out may be 0: parser state and statistics are still updated,
 * only the content is not handed to the caller (for "just reset the state" situations).
 */
int proto_feed(proto_parser_t *p, uint8_t byte, proto_msg_t *out);

/*
 * Convenience wrapper: feed a run of bytes and return the **count of the last** message produced meanwhile (0 or 1).
 * Mainly for tests and for "a whole burst arrived at once" situations; in production firmware feed byte by byte.
 * buf may be 0 or n may be 0 (returns 0).
 */
int proto_feed_bytes(proto_parser_t *p, const uint8_t *buf, uint16_t n,
                     proto_msg_t *out);

/* Static string of the command name, for the upper layer to build the A,<text> acknowledgement (no stdio dependency). */
const char *proto_cmd_name(proto_cmd_t cmd);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
