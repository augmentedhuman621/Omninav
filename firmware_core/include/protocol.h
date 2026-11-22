/*
 * protocol.h — 串口收包状态机（逐字节喂入）
 * ---------------------------------------------------------------------------
 *
 * 设计约束（与 firmware_core 铁律一致）
 *   - 只 include <stdint.h>。不许 HAL / ROS2 / 第三方 / <stdio.h>。
 *   - 全整数零浮点、无动态内存、无全局可变状态、无递归。
 *   - 语法分析结果放在调用者给的 proto_msg_t 里；解析器状态放在 proto_parser_t 里。
 *
 * 帧格式（照 11_接口契约.md §3 冻结，本模块只做「拆帧 + 语法分析」，
 * 语义校验留给调用者）
 *
 *   下行 Pi -> STM32：
 *     V,<vx>,<vy>,<w>\n          机体速度；vx/vy 单位 mm/s，w 单位 mrad/s
 *     M,<p1>,<p2>,<p3>,<p4>\n    调试：直接给四路 PWM，-1000 ~ 1000
 *     P,<id>,<kp>,<ki>,<kd>\n    在线改 PID 参数
 *     S\n                        软急停
 *     reset\n                    复位
 *
 * 容错要求（验收标准里点名的五种，逐条对应到实现）
 *   半包   —— 逐字节喂入，凑不齐就不产出（返回 0），状态不丢。
 *   粘包   —— 一次 proto_feed 只吐一条；缓冲区里剩下的字节由调用者
 *             继续喂（proto_feed 是逐字节接口，粘包天然被拆开）。
 *             ★ 注意语义：粘包不是「一次调用吐多条」，而是「调用者按字节
 *               连续喂，每次换行吐一条」。
 *   超长   —— 一行超过 PROTO_LINE_MAX-1 个字节后，本行作废并进入
 *             「丢弃直到换行」状态；遇到换行后自动恢复正常解析。
 *             ★ 关键：超长不能污染下一帧 —— 所以必须一直丢到换行。
 *   非法字符 —— 标记 PROTO_ERR_BAD_CHAR。★ 该报文**仍然产出**（返回 1），
 *             让上层能回一条 A,<text> 把原因报出去，而不是静默吃掉；
 *             解析器状态保持可用，不影响下一帧。
 *   未知命令 —— 产出 PROTO_CMD_UNKNOWN（带 PROTO_ERR_UNKNOWN_CMD），
 *             让上层能回一条 A,<text> 而不是静默吃掉。
 *
 * 行尾与空白约定（★ 与 stm32_bridge 侧必须逐字对齐）
 *   以 '\n' 结束一行；'\r' 被忽略（兼容 CRLF）。
 *   **只把 ' ' 和 '\t' 当空白**：纯空白行（空格/制表符任意组合）静默忽略，
 *   不产出、不计错；行首尾的这两种空白会被剥掉。
 *   ★ 反过来，0x0B / 0x0C / 0x85 / 0xA0 这些「别的空白」**一律当非法字符**：
 *     它们恰恰是波特率不匹配的症状，把诊断线索当空白擦掉是自毁行为。
 *     内嵌的控制字符（例如 B,118\x00 00,1 里的 0x00）同样必须被拒。
 *   ★ 教训（两端同源，桥那边踩过）：判「非法字符」与「剥空白」的顺序很关键 ——
 *     先按「控制字符 = 空白」整体放行、再 strip，会让 CRLF 行尾把整行判死，
 *     也会让纯空白行被当成一帧吐出来。这里的做法是：制表符在非法字符扫描里
 *     单独开一个例外，而 strip 只用窄集合 { ' ', '\t' }。
 *
 * 数值解析
 *   只接受可选 '-' 加十进制数字。空字段、非数字字符、超出 int32 范围
 *   都被标记为错误（超范围时做饱和并置 PROTO_ERR_RANGE，不产生未定义行为）。
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 一行最多这么多字节（含命令字与逗号，不含结尾的 '\n'）。
 * 定 96 是留足余量：最长的 P,<id>,<kp>,<ki>,<kd>\n 也远不到 40 字节。 */
#define PROTO_LINE_MAX 96

/* 一条报文最多带多少个数值参数（M 帧 4 个、P 帧 4 个，留余量给未来） */
#define PROTO_ARG_MAX 6

/* proto_msg_t.error 的位定义 */
#define PROTO_ERR_NONE        0x0000u
#define PROTO_ERR_OVERFLOW    0x0001u /* 行太长，已丢弃 */
#define PROTO_ERR_BAD_CHAR    0x0002u /* 出现非法字符 */
#define PROTO_ERR_BAD_NUMBER  0x0004u /* 字段不是合法十进制整数 */
#define PROTO_ERR_RANGE       0x0008u /* 数值超出 int32，已饱和 */
#define PROTO_ERR_ARGC        0x0010u /* 参数个数与命令不符 */
#define PROTO_ERR_UNKNOWN_CMD 0x0020u /* 命令字不认识 */

/* 命令类型 */
typedef enum {
    PROTO_CMD_NONE = 0, /* 还没凑齐一条完整报文 */
    PROTO_CMD_V,
    PROTO_CMD_M,
    PROTO_CMD_P,
    PROTO_CMD_S,
    PROTO_CMD_RESET,
    PROTO_CMD_UNKNOWN
} proto_cmd_t;

/* 一条报文 */
typedef struct {
    proto_cmd_t cmd;
    int32_t     arg[PROTO_ARG_MAX];
    uint8_t     argc;                 /* 实际解析到几个数值参数 */
    uint16_t    error;                /* PROTO_ERR_* 位或 */
    char        raw[PROTO_LINE_MAX];  /* 原文（不含换行），供回执/诊断 */
    uint8_t     raw_len;
} proto_msg_t;

/* 解析器状态。无动态内存，纯值类型，可随意拷贝/清零。 */
typedef struct {
    char     buf[PROTO_LINE_MAX];
    uint16_t len;
    uint8_t  discarding; /* 1 = 本行已超长，正在丢到换行为止 */
    uint32_t stat_lines;    /* 累计产出过的完整行数 */
    uint32_t stat_overflow; /* 累计超长丢弃行数 */
    uint32_t stat_bad;      /* 累计非法字符数 */
    uint32_t stat_empty;    /* 累计被忽略的空行数 */
} proto_parser_t;

/* 清零解析器（含统计）。允许 p 为 0（空操作）。 */
void proto_init(proto_parser_t *p);

/* 把 msg 清零（避免调用者读到上一次的残留）。允许 msg 为 0。 */
void proto_msg_init(proto_msg_t *msg);

/*
 * 喂一个字节。
 *   返回 1 = 产出了一条完整报文，内容写在 out（若 out 非 0）；本次调用
 *            已经把这条报文交给调用者，内部缓冲已复位。
 *   返回 0 = 还没凑齐（或者本行被丢弃/忽略）。
 *
 * 允许 p 为 0（返回 0，不崩）。out 允许为 0：此时仍然会更新解析器状态
 * 与统计，只是不把内容交给调用者（用于「只想复位状态」的场合）。
 */
int proto_feed(proto_parser_t *p, uint8_t byte, proto_msg_t *out);

/*
 * 便捷封装：把一串字节喂进去，返回期间产出的**最后一条**报文数（0 或 1）。
 * 主要用于测试与「一次收到一坨」的场景；正式固件里建议逐字节喂。
 * 允许 buf 为 0 或 n 为 0（返回 0）。
 */
int proto_feed_bytes(proto_parser_t *p, const uint8_t *buf, uint16_t n,
                     proto_msg_t *out);

/* 命令名的静态字符串，供上层拼 A,<text> 回执（不依赖 stdio）。 */
const char *proto_cmd_name(proto_cmd_t cmd);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
