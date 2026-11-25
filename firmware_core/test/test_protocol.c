/*
 * test_protocol.c - test bench for the protocol module
 *
 * Coverage (the five fault-tolerance cases named by the acceptance criteria, mapped one by one):
 *   [1] Complete frame / partial frame
 *   [2] Packet coalescing (several frames in one burst)
 *   [3] ★ Overlong frame (must drop until the newline; never pollute the next frame)
 *   [4] Illegal characters
 *   [5] Unknown command / missing comma / wrong argument count
 *   [6] Negative numbers and int32 boundaries (including the just-legal INT32_MIN and true overflow)
 *   [7] Line endings: CRLF / empty line / whitespace-only line / command only
 *   [8] reset case and spelling
 *   [9] proto_feed_bytes convenience wrapper
 *   [10] Counters
 *   [11] Null-pointer safety
 *
 * Note: this is a test file, so printf is allowed. src/ and include/ stay third-party-dependency-free.
 */

#include <stdio.h>
#include <stdlib.h>

#include "protocol.h"

static int g_checks = 0;
static int g_fails  = 0;

static void ck_true(const char *tag, int cond)
{
    ++g_checks;
    if (cond == 0) {
        ++g_fails;
        if (g_fails <= 40) {
            printf("  [FAIL] %s\n", tag);
        }
    }
}

static void ck_eq32(const char *tag, int32_t got, int32_t want)
{
    ++g_checks;
    if (got != want) {
        ++g_fails;
        if (g_fails <= 40) {
            printf("  [FAIL] %s: got %ld, want %ld\n", tag, (long)got, (long)want);
        }
    }
}

/* Feed a line one byte at a time; return the number of messages produced */
static int feed_str(proto_parser_t *p, const char *s, proto_msg_t *out)
{
    int got = 0;
    size_t i;
    for (i = 0; s[i] != '\0'; ++i) {
        if (proto_feed(p, (uint8_t)s[i], out) != 0) {
            got++;
        }
    }
    return got;
}

/* Feed a line that is "long enough to always overflow" (without the newline) */
static void feed_long(proto_parser_t *p, int n)
{
    int i;
    for (i = 0; i < n; ++i) {
        (void)proto_feed(p, (uint8_t)'x', 0);
    }
}

/* ================================================================== */
/* [1] Complete frame / partial frame                                  */
/* ================================================================== */

static void test_complete_and_half(void)
{
    proto_parser_t p;
    proto_msg_t m;

    printf("[1] complete frame / partial frame\n");

    proto_init(&p);
    ck_eq32("[1] a complete V frame produces 1 message", feed_str(&p, "V,300,-100,250\n", &m), 1);
    ck_true("[1] cmd = V", m.cmd == PROTO_CMD_V);
    ck_true("[1] no errors", m.error == 0u);
    ck_eq32("[1] argc", (int32_t)m.argc, 3);
    ck_eq32("[1] arg0", m.arg[0], 300);
    ck_eq32("[1] arg1", m.arg[1], -100);
    ck_eq32("[1] arg2", m.arg[2], 250);

    /* partial frame: feed byte by byte, nothing may come out before it is complete */
    proto_init(&p);
    {
        static const char frame[] = "V,1,2,3\n";
        size_t i;
        int produced = 0;
        for (i = 0; i + 1 < sizeof(frame) - 1; ++i) {
            produced += proto_feed(&p, (uint8_t)frame[i], &m);
        }
        ck_eq32("[1] nothing produced during a partial frame", produced, 0);
        produced += proto_feed(&p, (uint8_t)frame[sizeof(frame) - 2], &m);
        ck_eq32("[1] produced only after the newline", produced, 1);
        ck_eq32("[1] arg1 reassembled from the partial frame", m.arg[1], 2);
    }

    /* a partial frame without a trailing newline must never be produced */
    proto_init(&p);
    ck_eq32("[1] no output without a newline", feed_str(&p, "M,1,2,3,4", &m), 0);
    ck_eq32("[1] output only once the newline arrives", feed_str(&p, "\n", &m), 1);
}

/* ================================================================== */
/* [2] Packet coalescing                                               */
/* ================================================================== */

static void test_sticky(void)
{
    proto_parser_t p;
    proto_msg_t m;
    int n1;
    int n2;

    printf("[2] packet coalescing\n");

    proto_init(&p);
    /* two frames in one burst: fed byte by byte, each must be produced exactly once */
    n1 = feed_str(&p, "V,1,2,3\nV,4,5,6\n", &m);
    ck_eq32("[2] both frames produce one message each", n1, 2);
    ck_eq32("[2] the last one is the second frame", m.arg[0], 4);

    /* feed them separately: the first frame comes out first, the second frame after */
    proto_init(&p);
    n1 = feed_str(&p, "V,10,0,0\n", &m);
    ck_eq32("[2] first frame arg0", m.arg[0], 10);
    n2 = feed_str(&p, "V,20,0,0\n", &m);
    ck_eq32("[2] first frame produces 1 message", n1, 1);
    ck_eq32("[2] second frame produces 1 message", n2, 1);
    ck_eq32("[2] second frame arg0", m.arg[0], 20);

    /* three frames stuck together */
    proto_init(&p);
    ck_eq32("[2] three frames", feed_str(&p, "S\nV,1,2,3\nreset\n", &m), 3);
    ck_true("[2] the last one is reset", m.cmd == PROTO_CMD_RESET);
}

/* ================================================================== */
/* [3] ★ Overlong frame                                                */
/* ================================================================== */

static void test_overflow(void)
{
    proto_parser_t p;
    proto_msg_t m;

    printf("[3] ★ overlong frame (must drop until the newline)\n");

    proto_init(&p);
    feed_long(&p, PROTO_LINE_MAX + 50);
    feed_str(&p, "\n", &m);
    ck_eq32("[3] an overlong line counts as one dropped frame", (int32_t)p.stat_overflow, 1);
    ck_eq32("[3] an overlong line produces no message", (int32_t)p.stat_lines, 0);

    /* ★ key: a valid frame immediately after an overlong line must still parse intact */
    ck_eq32("[3] the next frame after an overlong one is produced normally", feed_str(&p, "V,7,8,9\n", &m), 1);
    ck_eq32("[3] and its content is correct", m.arg[0], 7);
    ck_eq32("[3] and there is no error bit", (int32_t)m.error, 0);

    /* ★ a harsher case: the overlong line has not reached a newline yet and a whole valid frame follows.
     *   Without "drop until newline" the valid frame's tail would be cut off and its first half merged into
     *   the overlong line, yielding garbage nobody recognizes -- exactly what must be prevented. */
    proto_init(&p);
    feed_long(&p, PROTO_LINE_MAX + 5);
    feed_str(&p, "GARBAGE", &m);         /* still in drop mode, no output */
    ck_eq32("[3] nothing produced while dropping", (int32_t)p.stat_lines, 0);
    feed_str(&p, "\n", &m);              /* end of dropping */
    ck_eq32("[3] parsing recovers after the drop", feed_str(&p, "V,11,22,33\n", &m), 1);
    ck_eq32("[3] the recovered frame has the right content", m.arg[1], 22);
}

/* ================================================================== */
/* [4] Illegal characters                                              */
/* ================================================================== */

static void test_bad_char(void)
{
    proto_parser_t p;
    proto_msg_t m;

    printf("[4] illegal characters\n");

    proto_init(&p);
    /* 0x01 sits inside a numeric field: BAD_CHAR must be set (BAD_NUMBER too) */
    ck_eq32("[4] a frame with a control char is still produced (for the upper-layer ack)",
            feed_str(&p, "V,1\x01,2,3\n", &m), 1);
    ck_true("[4] BAD_CHAR is set", (m.error & PROTO_ERR_BAD_CHAR) != 0u);
    ck_true("[4] the parser counters recorded it", p.stat_bad >= 1u);

    /* high bytes (>= 0x7F) count as illegal too */
    proto_init(&p);
    (void)proto_feed(&p, (uint8_t)'V', &m);
    (void)proto_feed(&p, (uint8_t)',', &m);
    (void)proto_feed(&p, (uint8_t)'1', &m);
    (void)proto_feed(&p, (uint8_t)0x80u, &m);
    (void)proto_feed(&p, (uint8_t)'\n', &m);
    ck_true("[4] 0x80 is illegal too", (m.error & PROTO_ERR_BAD_CHAR) != 0u);

    /* an illegal character must not pollute the next frame */
    ck_eq32("[4] the next frame is unaffected", feed_str(&p, "V,5,6,7\n", &m), 1);
    ck_true("[4] the next frame is clean", m.error == 0u);

    /* ★ reverse guard: after tabs are let through as whitespace, **embedded** control chars must still be rejected.
       Never accept broad control chars wholesale just because "whitespace-only lines must be tolerated" --
       control chars other than CRLF are exactly the symptom of a wrong baud rate, the most valuable diagnostic clue.
       (same name and meaning as test_decoder_embedded_control_char_still_rejected on the stm32_bridge side) */
    proto_init(&p);
    (void)proto_feed(&p, (uint8_t)'B', &m);
    (void)proto_feed(&p, (uint8_t)',', &m);
    (void)proto_feed(&p, (uint8_t)'1', &m);
    (void)proto_feed(&p, (uint8_t)'1', &m);
    (void)proto_feed(&p, (uint8_t)'8', &m);
    (void)proto_feed(&p, (uint8_t)0x00u, &m);
    (void)proto_feed(&p, (uint8_t)'0', &m);
    (void)proto_feed(&p, (uint8_t)'0', &m);
    (void)proto_feed(&p, (uint8_t)',', &m);
    (void)proto_feed(&p, (uint8_t)'1', &m);
    (void)proto_feed(&p, (uint8_t)'\n', &m);
    ck_true("[4] an embedded 0x00 is still rejected", (m.error & PROTO_ERR_BAD_CHAR) != 0u);

    /* the tab is the only control char let through; 0x0B / 0x0C are not whitespace and must be rejected */
    proto_init(&p);
    ck_eq32("[4] a tab-only line counts as whitespace (no output)", feed_str(&p, "\t\t\n", &m), 0);
    ck_eq32("[4] 0x0B is not whitespace (still produced, with an error)", feed_str(&p, "V,1,\x0b" "2,3\n", &m), 1);
    ck_true("[4] 0x0B sets BAD_CHAR", (m.error & PROTO_ERR_BAD_CHAR) != 0u);
}

/* ================================================================== */
/* [5] Unknown command / missing comma / wrong argc                        */
/* ================================================================== */

static void test_bad_frames(void)
{
    proto_parser_t p;
    proto_msg_t m;

    printf("[5] unknown command / missing comma / wrong argc\n");

    proto_init(&p);
    ck_eq32("[5] an unknown command is still produced", feed_str(&p, "Z,1,2\n", &m), 1);
    ck_true("[5] cmd = UNKNOWN", m.cmd == PROTO_CMD_UNKNOWN);
    ck_true("[5] UNKNOWN_CMD is set", (m.error & PROTO_ERR_UNKNOWN_CMD) != 0u);
    ck_true("[5] the command name is printable", proto_cmd_name(m.cmd)[0] == 'U');

    proto_init(&p);
    (void)feed_str(&p, "V300,0,0\n", &m);
    ck_true("[5] missing comma after the command word -> UNKNOWN", m.cmd == PROTO_CMD_UNKNOWN);

    proto_init(&p);
    (void)feed_str(&p, "V,1,2\n", &m);
    ck_true("[5] V with only 2 arguments -> ARGC", (m.error & PROTO_ERR_ARGC) != 0u);
    ck_eq32("[5] the arguments parsed so far are still carried out", m.arg[1], 2);

    proto_init(&p);
    (void)feed_str(&p, "V,1,2,3,4\n", &m);
    ck_true("[5] V with too many arguments -> ARGC", (m.error & PROTO_ERR_ARGC) != 0u);

    proto_init(&p);
    (void)feed_str(&p, "M,1,2,3\n", &m);
    ck_true("[5] M with too few arguments -> ARGC", (m.error & PROTO_ERR_ARGC) != 0u);

    proto_init(&p);
    (void)feed_str(&p, "S\n", &m);
    ck_true("[5] S is valid and error-free", m.cmd == PROTO_CMD_S && m.error == 0u);
    ck_eq32("[5] S has 0 arguments", (int32_t)m.argc, 0);

    proto_init(&p);
    (void)feed_str(&p, "Sx\n", &m);
    ck_true("[5] trailing junk after S -> UNKNOWN", m.cmd == PROTO_CMD_UNKNOWN);

    proto_init(&p);
    (void)feed_str(&p, "S,1\n", &m);
    ck_true("[5] S with arguments -> UNKNOWN or ARGC",
            m.cmd == PROTO_CMD_UNKNOWN || (m.error & PROTO_ERR_ARGC) != 0u);
}

/* ================================================================== */
/* [6] Numeric boundaries                                              */
/* ================================================================== */

static void test_numbers(void)
{
    proto_parser_t p;
    proto_msg_t m;

    printf("[6] negative numbers and int32 boundaries\n");

    proto_init(&p);
    (void)feed_str(&p, "M,-1000,1000,-500,0\n", &m);
    ck_true("[6] negative fields are error-free", m.error == 0u);
    ck_eq32("[6] arg0 = -1000", m.arg[0], -1000);
    ck_eq32("[6] arg1 = 1000", m.arg[1], 1000);
    ck_eq32("[6] arg2 = -500", m.arg[2], -500);

    /* the just-legal INT32_MIN and INT32_MAX must not be flagged as overflow */
    proto_init(&p);
    (void)feed_str(&p, "M,-2147483648,2147483647,0,0\n", &m);
    ck_true("[6] INT32_MIN/MAX do not count as overflow", (m.error & PROTO_ERR_RANGE) == 0u);
    ck_true("[6] INT32_MIN parses correctly", m.arg[0] == INT32_MIN);
    ck_true("[6] INT32_MAX parses correctly", m.arg[1] == INT32_MAX);

    /* true overflow: saturate + set RANGE, with no undefined behavior */
    proto_init(&p);
    (void)feed_str(&p, "V,99999999999999,0,0\n", &m);
    ck_true("[6] true overflow sets RANGE", (m.error & PROTO_ERR_RANGE) != 0u);
    ck_true("[6] saturates to INT32_MAX", m.arg[0] == INT32_MAX);

    proto_init(&p);
    (void)feed_str(&p, "V,-99999999999999,0,0\n", &m);
    ck_true("[6] negative overflow sets RANGE", (m.error & PROTO_ERR_RANGE) != 0u);
    ck_true("[6] saturates to INT32_MIN", m.arg[0] == INT32_MIN);

    /* empty field / non-numeric field */
    proto_init(&p);
    (void)feed_str(&p, "V,,,\n", &m);
    ck_true("[6] empty field -> BAD_NUMBER", (m.error & PROTO_ERR_BAD_NUMBER) != 0u);
    ck_eq32("[6] an empty field still takes a slot", (int32_t)m.argc, 3);

    proto_init(&p);
    (void)feed_str(&p, "V,1a2,3,4\n", &m);
    ck_true("[6] letters inside a field -> BAD_NUMBER",
            (m.error & PROTO_ERR_BAD_NUMBER) != 0u);

    proto_init(&p);
    (void)feed_str(&p, "V,-,3,4\n", &m);
    ck_true("[6] a lone minus sign -> BAD_NUMBER",
            (m.error & PROTO_ERR_BAD_NUMBER) != 0u);

    proto_init(&p);
    (void)feed_str(&p, "V,0,0,0\n", &m);
    ck_true("[6] all zeroes are legal", m.error == 0u);
}

/* ================================================================== */
/* [7] Line endings and empty lines                                    */
/* ================================================================== */

static void test_line_endings(void)
{
    proto_parser_t p;
    proto_msg_t m;

    printf("[7] line endings and empty lines\n");

    proto_init(&p);
    ck_eq32("[7] CRLF parses", feed_str(&p, "V,300,0,0\r\n", &m), 1);
    ck_eq32("[7] CRLF arg0", m.arg[0], 300);
    ck_eq32("[7] a lone \\r produces nothing", feed_str(&p, "\r", &m), 0);

    proto_init(&p);
    ck_eq32("[7] empty lines produce nothing", feed_str(&p, "\n\n\n", &m), 0);
    ck_eq32("[7] empty-line counter", (int32_t)p.stat_empty, 3);

    proto_init(&p);
    ck_eq32("[7] a whitespace-only line produces nothing", feed_str(&p, "   \t \n", &m), 0);

    proto_init(&p);
    ck_eq32("[7] leading/trailing whitespace is eaten", feed_str(&p, "  V,1,2,3  \n", &m), 1);
    ck_eq32("[7] content is correct", m.arg[2], 3);
    ck_true("[7] no errors", m.error == 0u);

    /* raw keeps the original text (including extra whitespace) for the ack */
    ck_true("[7] raw keeps the original text", m.raw_len >= (uint8_t)9);
}

/* ================================================================== */
/* [8] reset                                                           */
/* ================================================================== */

static void test_reset_cmd(void)
{
    proto_parser_t p;
    proto_msg_t m;

    printf("[8] reset case and spelling\n");

    proto_init(&p);
    (void)feed_str(&p, "reset\n", &m);
    ck_true("[8] all lowercase reset", m.cmd == PROTO_CMD_RESET);

    proto_init(&p);
    (void)feed_str(&p, "RESET\r\n", &m);
    ck_true("[8] all uppercase RESET", m.cmd == PROTO_CMD_RESET);

    proto_init(&p);
    (void)feed_str(&p, "Reset\n", &m);
    ck_true("[8] mixed-case Reset", m.cmd == PROTO_CMD_RESET);

    proto_init(&p);
    (void)feed_str(&p, "resett\n", &m);
    ck_true("[8] misspelled -> UNKNOWN", m.cmd == PROTO_CMD_UNKNOWN);

    proto_init(&p);
    (void)feed_str(&p, "res\n", &m);
    ck_true("[8] truncated -> UNKNOWN", m.cmd == PROTO_CMD_UNKNOWN);

    proto_init(&p);
    (void)feed_str(&p, "reset,1\n", &m);
    ck_true("[8] reset with arguments -> UNKNOWN", m.cmd == PROTO_CMD_UNKNOWN);
}

/* ================================================================== */
/* [9] proto_feed_bytes                                                */
/* ================================================================== */

static void test_feed_bytes(void)
{
    proto_parser_t p;
    proto_msg_t m;
    static const uint8_t two[] = { 'V', ',', '1', ',', '2', ',', '3', '\n',
                                   'S', '\n' };

    printf("[9] proto_feed_bytes convenience wrapper\n");

    proto_init(&p);
    ck_eq32("[9] feeding two at once returns 1 (the last one)",
            proto_feed_bytes(&p, two, (uint16_t)sizeof(two), &m), 1);
    ck_true("[9] the last one is S", m.cmd == PROTO_CMD_S);
    ck_eq32("[9] both were produced", (int32_t)p.stat_lines, 2);

    proto_init(&p);
    ck_eq32("[9] empty input returns 0", proto_feed_bytes(&p, two, (uint16_t)0, &m), 0);
    ck_eq32("[9] NULL input returns 0", proto_feed_bytes(&p, 0, (uint16_t)8, &m), 0);
}

/* ================================================================== */
/* [10] Counters and null pointers                                      */
/* ================================================================== */

static void test_stats_and_null(void)
{
    proto_parser_t p;
    proto_msg_t m;
    uint32_t before;

    printf("[10] counters and null-pointer safety\n");

    proto_init(&p);
    ck_eq32("[10] lines starts at 0", (int32_t)p.stat_lines, 0);
    (void)feed_str(&p, "V,1,2,3\nV,4,5,6\nS\n", &m);
    ck_eq32("[10] lines counter", (int32_t)p.stat_lines, 3);

    /* out = NULL: counters must still update and it must not crash */
    proto_init(&p);
    before = p.stat_lines;
    ck_eq32("[10] out=NULL still returns 1", proto_feed(&p, (uint8_t)'\n', 0), 0); /* empty line */
    (void)feed_str(&p, "V,1,2,3\n", 0);
    ck_true("[10] counters still tick with out=NULL", p.stat_lines == before + 1u);

    /* p = NULL: must return 0 without crashing */
    ck_eq32("[10] p=NULL returns 0", proto_feed(0, (uint8_t)'V', &m), 0);
    ck_eq32("[10] feed_bytes with p=NULL returns 0",
            proto_feed_bytes(0, (const uint8_t *)"V,1,2,3\n", (uint16_t)8, &m), 0);

    proto_init(0);      /* must not crash */
    proto_msg_init(0);  /* must not crash */
    ck_true("[10] proto_init(0)/proto_msg_init(0) are safe", 1);

    ck_true("[10] the command-name table is complete",
            proto_cmd_name(PROTO_CMD_V)[0] == 'V' &&
            proto_cmd_name(PROTO_CMD_M)[0] == 'M' &&
            proto_cmd_name(PROTO_CMD_P)[0] == 'P' &&
            proto_cmd_name(PROTO_CMD_S)[0] == 'S' &&
            proto_cmd_name(PROTO_CMD_RESET)[0] == 'r' &&
            proto_cmd_name(PROTO_CMD_NONE)[0] == 'N');
}

/* ================================================================== */

int main(void)
{
    printf("=== protocol serial receive state-machine tests ===\n");

    test_complete_and_half();
    test_sticky();
    test_overflow();
    test_bad_char();
    test_bad_frames();
    test_numbers();
    test_line_endings();
    test_reset_cmd();
    test_feed_bytes();
    test_stats_and_null();

    printf("--------------------------------------------------\n");
    printf("checks = %d, failures = %d\n", g_checks, g_fails);
    if (g_fails == 0) {
        printf("RESULT: PASSED\n");
        return EXIT_SUCCESS;
    }
    printf("RESULT: FAILED\n");
    return EXIT_FAILURE;
}
