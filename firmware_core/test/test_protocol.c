/*
 * test_protocol.c — protocol 模块的测试台
 *
 * 覆盖（验收标准点名的五种容错，逐条对应）：
 *   [1] 完整帧 / 半包
 *   [2] 粘包（连续多帧）
 *   [3] ★ 超长帧（必须丢到换行为止，绝不许污染下一帧）
 *   [4] 非法字符
 *   [5] 未知命令 / 缺逗号 / 参数个数不符
 *   [6] 负数与 int32 边界（含恰好合法的 INT32_MIN 与真溢出）
 *   [7] 行尾：CRLF / 空行 / 纯空白行 / 只有命令字
 *   [8] reset 的大小写与拼写
 *   [9] proto_feed_bytes 便捷封装
 *   [10] 统计计数
 *   [11] 空指针安全
 *
 * 说明：本文件是测试，允许用 printf。src/ 与 include/ 依旧零第三方依赖。
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

/* 把一行字符串逐字节喂进去，返回产出条数 */
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

/* 喂一个「长到必然超限」的行（不含换行） */
static void feed_long(proto_parser_t *p, int n)
{
    int i;
    for (i = 0; i < n; ++i) {
        (void)proto_feed(p, (uint8_t)'x', 0);
    }
}

/* ================================================================== */
/* [1] 完整帧 / 半包                                                   */
/* ================================================================== */

static void test_complete_and_half(void)
{
    proto_parser_t p;
    proto_msg_t m;

    printf("[1] 完整帧 / 半包\n");

    proto_init(&p);
    ck_eq32("[1] 完整 V 帧产出 1 条", feed_str(&p, "V,300,-100,250\n", &m), 1);
    ck_true("[1] cmd = V", m.cmd == PROTO_CMD_V);
    ck_true("[1] 无错误", m.error == 0u);
    ck_eq32("[1] argc", (int32_t)m.argc, 3);
    ck_eq32("[1] arg0", m.arg[0], 300);
    ck_eq32("[1] arg1", m.arg[1], -100);
    ck_eq32("[1] arg2", m.arg[2], 250);

    /* 半包：逐个字节喂，前面都不许产出 */
    proto_init(&p);
    {
        static const char frame[] = "V,1,2,3\n";
        size_t i;
        int produced = 0;
        for (i = 0; i + 1 < sizeof(frame) - 1; ++i) {
            produced += proto_feed(&p, (uint8_t)frame[i], &m);
        }
        ck_eq32("[1] 半包期间不产出", produced, 0);
        produced += proto_feed(&p, (uint8_t)frame[sizeof(frame) - 2], &m);
        ck_eq32("[1] 换行后才产出", produced, 1);
        ck_eq32("[1] 半包拼出来的 arg1", m.arg[1], 2);
    }

    /* 没有换行结尾的半包必须一直不产出 */
    proto_init(&p);
    ck_eq32("[1] 缺换行不产出", feed_str(&p, "M,1,2,3,4", &m), 0);
    ck_eq32("[1] 补上换行才产出", feed_str(&p, "\n", &m), 1);
}

/* ================================================================== */
/* [2] 粘包                                                            */
/* ================================================================== */

static void test_sticky(void)
{
    proto_parser_t p;
    proto_msg_t m;
    int n1;
    int n2;

    printf("[2] 粘包\n");

    proto_init(&p);
    /* 一坨字节里有两帧：逐字节喂，必须各自产出一次 */
    n1 = feed_str(&p, "V,1,2,3\nV,4,5,6\n", &m);
    ck_eq32("[2] 两帧各产出一条", n1, 2);
    ck_eq32("[2] 最后一条是第二帧", m.arg[0], 4);

    /* 分开喂：第一帧先出来，第二帧后出来 */
    proto_init(&p);
    n1 = feed_str(&p, "V,10,0,0\n", &m);
    ck_eq32("[2] 第一帧 arg0", m.arg[0], 10);
    n2 = feed_str(&p, "V,20,0,0\n", &m);
    ck_eq32("[2] 第一帧产出 1 条", n1, 1);
    ck_eq32("[2] 第二帧产出 1 条", n2, 1);
    ck_eq32("[2] 第二帧 arg0", m.arg[0], 20);

    /* 三条粘在一起 */
    proto_init(&p);
    ck_eq32("[2] 三帧", feed_str(&p, "S\nV,1,2,3\nreset\n", &m), 3);
    ck_true("[2] 最后一条是 reset", m.cmd == PROTO_CMD_RESET);
}

/* ================================================================== */
/* [3] ★ 超长帧                                                        */
/* ================================================================== */

static void test_overflow(void)
{
    proto_parser_t p;
    proto_msg_t m;

    printf("[3] ★ 超长帧（必须丢到换行为止）\n");

    proto_init(&p);
    feed_long(&p, PROTO_LINE_MAX + 50);
    feed_str(&p, "\n", &m);
    ck_eq32("[3] 超长行计一次丢帧", (int32_t)p.stat_overflow, 1);
    ck_eq32("[3] 超长行不产出报文", (int32_t)p.stat_lines, 0);

    /* ★ 关键：超长之后紧跟的合法帧必须完好无损地解析出来 */
    ck_eq32("[3] 超长后下一帧正常产出", feed_str(&p, "V,7,8,9\n", &m), 1);
    ck_eq32("[3] 且内容正确", m.arg[0], 7);
    ck_eq32("[3] 且无错误位", (int32_t)m.error, 0);

    /* ★ 更狠的一种：超长行还没遇到换行，后面紧跟一整条合法帧。
     *   如果不「丢到换行」，这条合法帧的尾巴会被切掉、前半截被并进超长行，
     *   结果是一条谁都不认识的垃圾 —— 这正是要防的。 */
    proto_init(&p);
    feed_long(&p, PROTO_LINE_MAX + 5);
    feed_str(&p, "GARBAGE", &m);         /* 仍在丢弃模式，不产出 */
    ck_eq32("[3] 丢弃期间不产出", (int32_t)p.stat_lines, 0);
    feed_str(&p, "\n", &m);              /* 结束丢弃 */
    ck_eq32("[3] 丢弃结束后恢复正常解析", feed_str(&p, "V,11,22,33\n", &m), 1);
    ck_eq32("[3] 恢复后的帧内容正确", m.arg[1], 22);
}

/* ================================================================== */
/* [4] 非法字符                                                        */
/* ================================================================== */

static void test_bad_char(void)
{
    proto_parser_t p;
    proto_msg_t m;

    printf("[4] 非法字符\n");

    proto_init(&p);
    /* 0x01 落在数值字段里：必须置 BAD_CHAR（同时也会置 BAD_NUMBER） */
    ck_eq32("[4] 含控制字符的帧仍产出（供上层回执）",
            feed_str(&p, "V,1\x01,2,3\n", &m), 1);
    ck_true("[4] 置 BAD_CHAR", (m.error & PROTO_ERR_BAD_CHAR) != 0u);
    ck_true("[4] 解析器统计记了一笔", p.stat_bad >= 1u);

    /* 高字节（>= 0x7F）同样算非法 */
    proto_init(&p);
    (void)proto_feed(&p, (uint8_t)'V', &m);
    (void)proto_feed(&p, (uint8_t)',', &m);
    (void)proto_feed(&p, (uint8_t)'1', &m);
    (void)proto_feed(&p, (uint8_t)0x80u, &m);
    (void)proto_feed(&p, (uint8_t)'\n', &m);
    ck_true("[4] 0x80 也算非法", (m.error & PROTO_ERR_BAD_CHAR) != 0u);

    /* 非法字符不能污染下一帧 */
    ck_eq32("[4] 下一帧不受影响", feed_str(&p, "V,5,6,7\n", &m), 1);
    ck_true("[4] 下一帧干净", m.error == 0u);

    /* ★ 反向保险：制表符被当成空白放行之后，**内嵌的**控制字符必须仍然被拒。
       绝不因为「要容忍纯空白行」就顺手把广义控制字符整体放行 ——
       CRLF 之外的控制字符恰恰是波特率不对的症状，是最值钱的诊断线索。
       （与 stm32_bridge 侧的 test_decoder_embedded_control_char_still_rejected 同名同义） */
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
    ck_true("[4] 内嵌 0x00 仍被拒", (m.error & PROTO_ERR_BAD_CHAR) != 0u);

    /* 制表符是唯一被放行的控制字符；0x0B / 0x0C 不算空白，必须被拒 */
    proto_init(&p);
    ck_eq32("[4] 纯制表符行被当空白（不产出）", feed_str(&p, "\t\t\n", &m), 0);
    ck_eq32("[4] 0x0B 不是空白（仍产出并报错）", feed_str(&p, "V,1,\x0b" "2,3\n", &m), 1);
    ck_true("[4] 0x0B 置 BAD_CHAR", (m.error & PROTO_ERR_BAD_CHAR) != 0u);
}

/* ================================================================== */
/* [5] 未知命令 / 缺逗号 / 参数个数                                        */
/* ================================================================== */

static void test_bad_frames(void)
{
    proto_parser_t p;
    proto_msg_t m;

    printf("[5] 未知命令 / 缺逗号 / 参数个数\n");

    proto_init(&p);
    ck_eq32("[5] 未知命令仍产出", feed_str(&p, "Z,1,2\n", &m), 1);
    ck_true("[5] cmd = UNKNOWN", m.cmd == PROTO_CMD_UNKNOWN);
    ck_true("[5] 置 UNKNOWN_CMD", (m.error & PROTO_ERR_UNKNOWN_CMD) != 0u);
    ck_true("[5] 命令名可打印", proto_cmd_name(m.cmd)[0] == 'U');

    proto_init(&p);
    (void)feed_str(&p, "V300,0,0\n", &m);
    ck_true("[5] 命令字后缺逗号 -> UNKNOWN", m.cmd == PROTO_CMD_UNKNOWN);

    proto_init(&p);
    (void)feed_str(&p, "V,1,2\n", &m);
    ck_true("[5] V 只给 2 个参数 -> ARGC", (m.error & PROTO_ERR_ARGC) != 0u);
    ck_eq32("[5] 仍然把已解析到的参数带出来", m.arg[1], 2);

    proto_init(&p);
    (void)feed_str(&p, "V,1,2,3,4\n", &m);
    ck_true("[5] V 多给参数 -> ARGC", (m.error & PROTO_ERR_ARGC) != 0u);

    proto_init(&p);
    (void)feed_str(&p, "M,1,2,3\n", &m);
    ck_true("[5] M 参数不足 -> ARGC", (m.error & PROTO_ERR_ARGC) != 0u);

    proto_init(&p);
    (void)feed_str(&p, "S\n", &m);
    ck_true("[5] S 合法且无错误", m.cmd == PROTO_CMD_S && m.error == 0u);
    ck_eq32("[5] S 参数个数 0", (int32_t)m.argc, 0);

    proto_init(&p);
    (void)feed_str(&p, "Sx\n", &m);
    ck_true("[5] S 后面有杂字符 -> UNKNOWN", m.cmd == PROTO_CMD_UNKNOWN);

    proto_init(&p);
    (void)feed_str(&p, "S,1\n", &m);
    ck_true("[5] S 带参数 -> UNKNOWN 或 ARGC",
            m.cmd == PROTO_CMD_UNKNOWN || (m.error & PROTO_ERR_ARGC) != 0u);
}

/* ================================================================== */
/* [6] 数值边界                                                        */
/* ================================================================== */

static void test_numbers(void)
{
    proto_parser_t p;
    proto_msg_t m;

    printf("[6] 负数与 int32 边界\n");

    proto_init(&p);
    (void)feed_str(&p, "M,-1000,1000,-500,0\n", &m);
    ck_true("[6] 负数字段无错误", m.error == 0u);
    ck_eq32("[6] arg0 = -1000", m.arg[0], -1000);
    ck_eq32("[6] arg1 = 1000", m.arg[1], 1000);
    ck_eq32("[6] arg2 = -500", m.arg[2], -500);

    /* 恰好合法的 INT32_MIN 与 INT32_MAX 不许被判成溢出 */
    proto_init(&p);
    (void)feed_str(&p, "M,-2147483648,2147483647,0,0\n", &m);
    ck_true("[6] INT32_MIN/MAX 不算溢出", (m.error & PROTO_ERR_RANGE) == 0u);
    ck_true("[6] INT32_MIN 解析正确", m.arg[0] == INT32_MIN);
    ck_true("[6] INT32_MAX 解析正确", m.arg[1] == INT32_MAX);

    /* 真溢出：饱和 + 置 RANGE，不许有未定义行为 */
    proto_init(&p);
    (void)feed_str(&p, "V,99999999999999,0,0\n", &m);
    ck_true("[6] 真溢出置 RANGE", (m.error & PROTO_ERR_RANGE) != 0u);
    ck_true("[6] 饱和到 INT32_MAX", m.arg[0] == INT32_MAX);

    proto_init(&p);
    (void)feed_str(&p, "V,-99999999999999,0,0\n", &m);
    ck_true("[6] 负溢出置 RANGE", (m.error & PROTO_ERR_RANGE) != 0u);
    ck_true("[6] 饱和到 INT32_MIN", m.arg[0] == INT32_MIN);

    /* 空字段 / 非数字字段 */
    proto_init(&p);
    (void)feed_str(&p, "V,,,\n", &m);
    ck_true("[6] 空字段 -> BAD_NUMBER", (m.error & PROTO_ERR_BAD_NUMBER) != 0u);
    ck_eq32("[6] 空字段仍占位", (int32_t)m.argc, 3);

    proto_init(&p);
    (void)feed_str(&p, "V,1a2,3,4\n", &m);
    ck_true("[6] 字段内混字母 -> BAD_NUMBER",
            (m.error & PROTO_ERR_BAD_NUMBER) != 0u);

    proto_init(&p);
    (void)feed_str(&p, "V,-,3,4\n", &m);
    ck_true("[6] 只有一个负号 -> BAD_NUMBER",
            (m.error & PROTO_ERR_BAD_NUMBER) != 0u);

    proto_init(&p);
    (void)feed_str(&p, "V,0,0,0\n", &m);
    ck_true("[6] 全零合法", m.error == 0u);
}

/* ================================================================== */
/* [7] 行尾与空行                                                      */
/* ================================================================== */

static void test_line_endings(void)
{
    proto_parser_t p;
    proto_msg_t m;

    printf("[7] 行尾与空行\n");

    proto_init(&p);
    ck_eq32("[7] CRLF 可解析", feed_str(&p, "V,300,0,0\r\n", &m), 1);
    ck_eq32("[7] CRLF 的 arg0", m.arg[0], 300);
    ck_eq32("[7] 单独一个 \\r 不产出", feed_str(&p, "\r", &m), 0);

    proto_init(&p);
    ck_eq32("[7] 空行不产出", feed_str(&p, "\n\n\n", &m), 0);
    ck_eq32("[7] 空行计数", (int32_t)p.stat_empty, 3);

    proto_init(&p);
    ck_eq32("[7] 纯空白行不产出", feed_str(&p, "   \t \n", &m), 0);

    proto_init(&p);
    ck_eq32("[7] 帧首尾空白被吃掉", feed_str(&p, "  V,1,2,3  \n", &m), 1);
    ck_eq32("[7] 内容正确", m.arg[2], 3);
    ck_true("[7] 无错误", m.error == 0u);

    /* raw 里保留的是原文（含多余空白），方便回执 */
    ck_true("[7] raw 保留原文", m.raw_len >= (uint8_t)9);
}

/* ================================================================== */
/* [8] reset                                                           */
/* ================================================================== */

static void test_reset_cmd(void)
{
    proto_parser_t p;
    proto_msg_t m;

    printf("[8] reset 的大小写与拼写\n");

    proto_init(&p);
    (void)feed_str(&p, "reset\n", &m);
    ck_true("[8] 全小写 reset", m.cmd == PROTO_CMD_RESET);

    proto_init(&p);
    (void)feed_str(&p, "RESET\r\n", &m);
    ck_true("[8] 全大写 RESET", m.cmd == PROTO_CMD_RESET);

    proto_init(&p);
    (void)feed_str(&p, "Reset\n", &m);
    ck_true("[8] 混合大小写 Reset", m.cmd == PROTO_CMD_RESET);

    proto_init(&p);
    (void)feed_str(&p, "resett\n", &m);
    ck_true("[8] 拼错 -> UNKNOWN", m.cmd == PROTO_CMD_UNKNOWN);

    proto_init(&p);
    (void)feed_str(&p, "res\n", &m);
    ck_true("[8] 截断 -> UNKNOWN", m.cmd == PROTO_CMD_UNKNOWN);

    proto_init(&p);
    (void)feed_str(&p, "reset,1\n", &m);
    ck_true("[8] reset 带参数 -> UNKNOWN", m.cmd == PROTO_CMD_UNKNOWN);
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

    printf("[9] proto_feed_bytes 便捷封装\n");

    proto_init(&p);
    ck_eq32("[9] 一次喂两条返回 1（返回的是最后一条）",
            proto_feed_bytes(&p, two, (uint16_t)sizeof(two), &m), 1);
    ck_true("[9] 最后一条是 S", m.cmd == PROTO_CMD_S);
    ck_eq32("[9] 两条都产出过", (int32_t)p.stat_lines, 2);

    proto_init(&p);
    ck_eq32("[9] 空输入返回 0", proto_feed_bytes(&p, two, (uint16_t)0, &m), 0);
    ck_eq32("[9] NULL 输入返回 0", proto_feed_bytes(&p, 0, (uint16_t)8, &m), 0);
}

/* ================================================================== */
/* [10] 统计与空指针                                                    */
/* ================================================================== */

static void test_stats_and_null(void)
{
    proto_parser_t p;
    proto_msg_t m;
    uint32_t before;

    printf("[10] 统计与空指针安全\n");

    proto_init(&p);
    ck_eq32("[10] 初始 lines 为 0", (int32_t)p.stat_lines, 0);
    (void)feed_str(&p, "V,1,2,3\nV,4,5,6\nS\n", &m);
    ck_eq32("[10] lines 统计", (int32_t)p.stat_lines, 3);

    /* out = NULL：仍要更新统计，且不崩 */
    proto_init(&p);
    before = p.stat_lines;
    ck_eq32("[10] out=NULL 仍返回 1", proto_feed(&p, (uint8_t)'\n', 0), 0); /* 空行 */
    (void)feed_str(&p, "V,1,2,3\n", 0);
    ck_true("[10] out=NULL 时统计照记", p.stat_lines == before + 1u);

    /* p = NULL：必须返回 0，不崩 */
    ck_eq32("[10] p=NULL 返回 0", proto_feed(0, (uint8_t)'V', &m), 0);
    ck_eq32("[10] p=NULL 的 feed_bytes 返回 0",
            proto_feed_bytes(0, (const uint8_t *)"V,1,2,3\n", (uint16_t)8, &m), 0);

    proto_init(0);      /* 必须不崩 */
    proto_msg_init(0);  /* 必须不崩 */
    ck_true("[10] proto_init(0)/proto_msg_init(0) 安全", 1);

    ck_true("[10] 命令名表齐全",
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
    printf("=== protocol 串口收包状态机测试 ===\n");

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
