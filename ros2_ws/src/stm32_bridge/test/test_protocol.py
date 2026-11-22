#!/usr/bin/env python3
"""protocol.py 的单测 —— **不依赖 pytest、不依赖 ROS2**。

跑法（在没装 ROS2 的 Windows 上也能跑）::

    python ros2_ws/src/stm32_bridge/test/test_protocol.py

全绿退出码 0，任何一条失败退出码 1（并打印是哪一条）。

为什么要自带一个极小测试框架：T2 任务卡要求「不依赖 pytest / ROS2，
``python test_protocol.py`` 直接跑」—— 树莓派上现在没有外网，装不了 pytest。
"""

from __future__ import annotations

import ast
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(_HERE, "..")))

from stm32_bridge import protocol as P  # noqa: E402  (必须在 sys.path 调整之后)

# --------------------------------------------------------------------------
# 极简测试框架
# --------------------------------------------------------------------------

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


class _Counter:
    checks = 0


def check(condition, message):
    _Counter.checks += 1
    if not condition:
        raise AssertionError(message)


def eq(actual, expected, message=""):
    check(
        actual == expected,
        "%s：期望 %r，实际 %r" % (message or "取值不符", expected, actual),
    )


# --------------------------------------------------------------------------
# 1) 取整：必须与 C 的 round() 一致，不是 Python 的银行家舍入
# --------------------------------------------------------------------------


@case
def test_rounding_half_away_from_zero():
    r = P.round_half_away_from_zero
    eq(r(0.5), 1, "0.5 应远离零")
    eq(r(-0.5), -1, "-0.5 应远离零")
    eq(r(1.5), 2, "1.5")
    eq(r(2.5), 3, "2.5")
    eq(r(-2.5), -3, "-2.5")
    eq(r(0.4999), 0, "0.4999")
    eq(r(-0.4999), 0, "-0.4999")
    eq(r(0.0), 0, "0.0")
    eq(r(-0.0), 0, "-0.0")
    eq(r(1000.5), 1001, "1000.5")


@case
def test_rounding_differs_from_python_builtin_round():
    """★ 这条是**故意**记录陷阱的：Python 的 round() 是银行家舍入，C 的不是。

    契约 §4 要求「四舍五入，不要 int() 截断（负速度会偏一个 LSB）」——
    如果这里用 Python 内建 round()，2.5 -> 2 而 C 的 lround(2.5) -> 3，
    半个 LSB 的差就是系统性偏差，长期累积会表现出来。
    """
    r = P.round_half_away_from_zero
    check(round(2.5) == 2, "前提变了：Python round(2.5) 居然不是 2")
    eq(r(2.5), 3, "我们的实现必须与 C 一致，不是与 Python 内建一致")
    check(r(2.5) != round(2.5), "两种取整在 .5 上必须不同，否则这条测试没有意义")


@case
def test_unit_conversions():
    eq(P.m_s_to_mm_s(0.2), 200, "0.2 m/s")
    eq(P.m_s_to_mm_s(-0.2), -200, "-0.2 m/s")
    eq(P.m_s_to_mm_s(0.0005), 1, "+0.5 mm/s 要进位")
    eq(P.m_s_to_mm_s(-0.0005), -1, "-0.5 mm/s 要进位")
    eq(P.rad_s_to_mrad_s(0.5), 500, "0.5 rad/s")
    eq(P.rad_s_to_mrad_s(-1.2345), -1235, "-1.2345 rad/s 四舍五入")
    eq(P.mm_s_to_m_s(200), 0.2, "回程")
    eq(P.mrad_s_to_rad_s(-1500), -1.5, "回程")


@case
def test_rounding_rejects_nan_inf():
    for bad in (float("nan"), float("inf"), float("-inf")):
        try:
            P.round_half_away_from_zero(bad)
        except ValueError:
            continue
        raise AssertionError("非有限数 %r 必须抛 ValueError" % (bad,))


# --------------------------------------------------------------------------
# 2) 下行编码
# --------------------------------------------------------------------------


@case
def test_encode_velocity_exact_bytes():
    eq(P.encode_velocity(200, 0, 0), b"V,200,0,0\n", "直行")
    eq(P.encode_velocity(-200, 150, -300), b"V,-200,150,-300\n", "斜行+自转")
    eq(P.encode_velocity(0, 0, 0), P.ZERO_VELOCITY_FRAME, "停车帧常量必须一致")


@case
def test_encode_cmd_vel_from_ros2_twist():
    eq(P.encode_cmd_vel(0.2, 0.0, 0.0), b"V,200,0,0\n", "ROS2 0.2 m/s")
    eq(P.encode_cmd_vel(-0.35, 0.12, -0.5), b"V,-350,120,-500\n", "三分量都要换算")


@case
def test_encode_velocity_rejects_bad_input():
    for bad in (1.5, "200", None, True):
        try:
            P.encode_velocity(bad, 0, 0)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            continue
        raise AssertionError("非整数 %r 必须被拒绝" % (bad,))
    try:
        P.encode_velocity(2 ** 31, 0, 0)
    except ValueError:
        pass
    else:
        raise AssertionError("超出 int32 必须被拒绝")


@case
def test_encode_motor():
    eq(P.encode_motor(0, 0, 0, 0), b"M,0,0,0,0\n", "全零")
    eq(P.encode_motor(-1000, 1000, 500, -500), b"M,-1000,1000,500,-500\n", "边界值合法")
    try:
        P.encode_motor(1001, 0, 0, 0)
    except ValueError:
        pass
    else:
        raise AssertionError("PWM 1001 必须被拒绝")


@case
def test_encode_pid():
    eq(P.encode_pid(0, 10, 2, 1), b"P,0,10,2,1\n", "通道 0")
    eq(P.encode_pid(3, 0, 0, 0), b"P,3,0,0,0\n", "通道 3")
    for bad in (-1, 4):
        try:
            P.encode_pid(bad, 0, 0, 0)
        except ValueError:
            continue
        raise AssertionError("pid_id=%d 必须被拒绝" % bad)


@case
def test_frozen_constants():
    """这些值在 `11_接口契约.md` §3 里冻结了 —— 改这里就等于改协议。"""
    eq(P.ZERO_VELOCITY_FRAME, b"V,0,0,0\n", "停车帧")
    eq(P.SOFT_ESTOP_FRAME, b"S\n", "软急停帧")
    eq(P.RESET_FRAME, b"reset\n", "复位帧")
    eq(P.RESET_BURST_COUNT, 3, "reset 连发次数（契约 §3）")
    eq(P.RESET_BURST_PERIOD_S, 0.3, "reset 连发间隔（契约 §3）")
    eq(P.WHEEL_COUNT, 4, "轮数")
    eq(P.MAX_LINE_BYTES, 256, "单行上限")


# --------------------------------------------------------------------------
# 3) 单行解析
# --------------------------------------------------------------------------


@case
def test_parse_encoder_frame():
    frame = P.parse_line("E,100,101,102,103,1000,2000,3000,4000")
    check(isinstance(frame, P.EncoderFrame), "应解析成 EncoderFrame")
    eq(frame.v_mm_s, (100, 101, 102, 103), "四轮轮速")
    eq(frame.d_mm, (1000, 2000, 3000, 4000), "四轮累计里程")
    eq(frame.kind, "E", "kind")


@case
def test_encoder_wheel_order_is_fl_fr_rl_rr():
    """契约 §3：轮序恒为 FL, FR, RL, RR（不是 ROS2 的 FL,FR,RR,RL）。"""
    frame = P.parse_line("E,1,2,3,4,11,22,33,44")
    eq(frame.v_mm_s, (1, 2, 3, 4), "第 3 个字段是 RL，不能与 RR 互换")
    eq(frame.d_mm, (11, 22, 33, 44), "里程同序")


@case
def test_parse_encoder_negative_values():
    frame = P.parse_line("E,-100,-101,-102,-103,-1000,-2000,-3000,-4000")
    eq(frame.v_mm_s, (-100, -101, -102, -103), "负轮速必须支持")
    eq(frame.d_mm, (-1000, -2000, -3000, -4000), "负里程必须支持")


@case
def test_parse_battery_frame():
    frame = P.parse_line("B,11800,3")
    check(isinstance(frame, P.BatteryFrame), "应解析成 BatteryFrame")
    eq(frame.millivolts, 11800, "mV")
    eq(frame.flags, 3, "flags")
    eq(frame.volts, 11.8, "volts 属性")


@case
def test_parse_ack_keeps_commas():
    """A 帧文本里允许含逗号 —— 只按**第一个**逗号切分。"""
    frame = P.parse_line("A,OK reset, wheels=0")
    check(isinstance(frame, P.AckFrame), "应解析成 AckFrame")
    eq(frame.text, "OK reset, wheels=0", "逗号必须保留")


@case
def test_parse_ack_requires_comma():
    """裸 "A"（没有逗号）是畸形行 —— 与 E / B 的处理保持一致，别当成"空回执"。"""
    frame = P.parse_line("A")
    check(isinstance(frame, P.UnknownFrame), "裸 A 必须是 UnknownFrame，实际 %r" % (frame,))
    eq(frame.reason, "缺失逗号分隔符", "记录原因")
    empty = P.parse_line("A,")
    check(isinstance(empty, P.AckFrame), "A, 是合法的空回执")
    eq(empty.text, "", "空回执文本是空串")


@case
def test_parse_empty_and_blank():
    eq(P.parse_line(""), None, "空串 -> None（不是错误）")


@case
def test_parse_unknown_prefix():
    frame = P.parse_line("Z,1,2,3")
    check(isinstance(frame, P.UnknownFrame), "未知前缀 -> UnknownFrame")
    eq(frame.prefix, "Z", "记录前缀")
    eq(frame.reason, "未知前缀", "记录原因")


@case
def test_parse_encoder_wrong_arity():
    for bad in ("E,1,2,3", "E,1,2,3,4", "E,1,2,3,4,5,6,7,8,9", "E", "E,"):
        frame = P.parse_line(bad)
        check(
            isinstance(frame, P.UnknownFrame),
            "%r 字段数不对，必须是 UnknownFrame，实际 %r" % (bad, frame),
        )


@case
def test_parse_encoder_non_integer_fields():
    for bad in (
        "E,1.5,2,3,4,5,6,7,8",
        "E,1,2,3,4,5,6,7,8.0",
        "E,1e3,2,3,4,5,6,7,8",
        "E,0x10,2,3,4,5,6,7,8",
        "E, 1,2,3,4,5,6,7,8",
        "E,1_0,2,3,4,5,6,7,8",
        "E,abc,2,3,4,5,6,7,8",
        "E,,,,,,,, ",
    ):
        frame = P.parse_line(bad)
        check(
            isinstance(frame, P.UnknownFrame),
            "%r 含非法整数，必须是 UnknownFrame，实际 %r" % (bad, frame),
        )


@case
def test_parse_encoder_out_of_int32():
    frame = P.parse_line("E,99999999999999,2,3,4,5,6,7,8")
    check(isinstance(frame, P.UnknownFrame), "超 int32 必须被拒绝")


@case
def test_parse_battery_wrong_arity():
    for bad in ("B", "B,11800", "B,11800,3,7"):
        frame = P.parse_line(bad)
        check(isinstance(frame, P.UnknownFrame), "%r 必须是 UnknownFrame" % bad)


# --------------------------------------------------------------------------
# 4) 流式解码器：半包 / 粘包 / 垃圾 / 超长
# --------------------------------------------------------------------------


@case
def test_decoder_half_packet():
    """半包：\\n 没到之前不允许吐出任何帧。

    ★ 这三段的拼接必须**恰好**是
    ``E,100,101,102,103,1000,2000,3000,4000``（E + 4 个轮速 + 4 个累计里程 = 8 个字段）。
    这个用例第一版把第三段写成 ``b"000,4000\\n"``，于是拼出来是
    ``...3000,4000,4000`` —— **9 个字段**，被正确判成畸形行（UnknownFrame）。
    那是测试自己的算术错误，不是 decoder 的锅：decoder 拒绝 9 字段的 E 帧是**对的**。
    修的是输入，不是断言。
    """
    decoder = P.LineDecoder()
    eq(decoder.feed(b"E,100,101,102,103,10"), [], "半包不能出帧")
    eq(decoder.feed(b"00,2000,3000,"), [], "还是半包（且正好切在最后一个 token 中间）")
    frames = decoder.feed(b"4000\n")
    eq(len(frames), 1, "补齐换行后出 1 帧")
    check(isinstance(frames[0], P.EncoderFrame), "必须是 EncoderFrame，实际 %r" % (frames[0],))
    eq(frames[0].d_mm, (1000, 2000, 3000, 4000), "内容正确")
    eq(frames[0].v_mm_s, (100, 101, 102, 103), "轮速顺序正确")


@case
def test_decoder_split_mid_token():
    decoder = P.LineDecoder()
    out = []
    for chunk in (b"B,1", b"18", b"00,", b"3", b"\n"):
        out.extend(decoder.feed(chunk))
    eq(len(out), 1, "逐字节喂也要出 1 帧")
    eq(out[0].millivolts, 11800, "跨 chunk 的 token 必须拼回去")


@case
def test_decoder_sticky_packets():
    """粘包：一次收到三帧，必须全部解出来且顺序不变。"""
    decoder = P.LineDecoder()
    frames = decoder.feed(b"E,1,2,3,4,5,6,7,8\nB,11800,0\nA,OK\n")
    eq(len(frames), 3, "三帧")
    check(isinstance(frames[0], P.EncoderFrame), "第 1 帧 E")
    check(isinstance(frames[1], P.BatteryFrame), "第 2 帧 B")
    check(isinstance(frames[2], P.AckFrame), "第 3 帧 A")


@case
def test_decoder_crlf_tolerated():
    """★ 这条是真 bug 的回归测试（Lead 代跑时抓到的）。

    第一版的 ``_decode_line`` 先查「有没有 < 0x20 的字符」再 strip，
    于是 CRLF 行尾的 ``\\r`` 被判成"含非 ASCII / 控制字符"，**整行丢掉**：
    ``B,12000,1`` 解析失败掉进 UnknownFrame。STM32 侧只要改用 ``\\r\\n``，
    桥就会**静默丢掉所有帧**且不报任何错 —— 所以这里把计数也一起钉死。
    """
    decoder = P.LineDecoder()
    frames = decoder.feed(b"E,1,2,3,4,5,6,7,8\r\nB,12000,1\r\n")
    eq(len(frames), 2, "CRLF 也要能解")
    check(isinstance(frames[0], P.EncoderFrame), "第 1 帧是 E，实际 %r" % (frames[0],))
    check(isinstance(frames[1], P.BatteryFrame), "第 2 帧是 B，实际 %r" % (frames[1],))
    eq(frames[1].millivolts, 12000, "\\r 不能混进字段")
    eq(decoder.stats.nonascii_lines, 0, "\\r 不是「非 ASCII 错误」")
    eq(decoder.stats.dropped_lines, 0, "CRLF 不该产生任何畸形帧")


@case
def test_decoder_embedded_control_char_still_rejected():
    """★ 反向保险：剥 ``\\r`` 不能把**真的**控制字符也一起放过。"""
    decoder = P.LineDecoder()
    frames = decoder.feed(b"B,118\x0000,1\nB,12000,1\n")
    eq(len(frames), 2, "坏行仍出 1 条 UnknownFrame，好行照出")
    check(isinstance(frames[0], P.UnknownFrame), "内嵌 \\x00 必须被拒，实际 %r" % (frames[0],))
    check(isinstance(frames[1], P.BatteryFrame), "下一行必须恢复")
    eq(decoder.stats.nonascii_lines, 1, "非 ASCII 计数")
    eq(decoder.stats.frames_ok, 1, "好帧计数")


@case
def test_decoder_blank_lines_are_not_errors():
    decoder = P.LineDecoder()
    frames = decoder.feed(b"\n\r\n\nE,1,2,3,4,5,6,7,8\n")
    eq(len(frames), 1, "空行不出帧")
    eq(decoder.stats.blank_lines, 3, "空行计数")
    eq(decoder.stats.dropped_lines, 0, "空行不算错误")


@case
def test_decoder_garbage_recovery():
    """二进制垃圾（波特率不对的典型症状）之后必须能恢复。"""
    decoder = P.LineDecoder()
    frames = decoder.feed(b"\x00\xff\xfe\x01\x02 garbage \x03\nE,1,2,3,4,5,6,7,8\n")
    eq(len(frames), 2, "垃圾行出一条 UnknownFrame，再出真帧")
    check(isinstance(frames[0], P.UnknownFrame), "第 1 条是 UnknownFrame")
    check(isinstance(frames[1], P.EncoderFrame), "第 2 条是真帧 —— 说明已重新同步")
    eq(decoder.stats.nonascii_lines, 1, "非 ASCII 计数")
    eq(decoder.stats.frames_ok, 1, "好帧计数")


@case
def test_decoder_malformed_line_then_good_line():
    decoder = P.LineDecoder()
    frames = decoder.feed(b"E,1,2,3\nE,1,2,3,4,5,6,7,8\n")
    eq(len(frames), 2, "畸形帧 + 好帧")
    check(isinstance(frames[0], P.UnknownFrame), "第 1 条畸形")
    check(isinstance(frames[1], P.EncoderFrame), "第 2 条正常")
    eq(decoder.stats.dropped_lines, 1, "畸形计数 = 1")


@case
def test_decoder_overlong_line_resyncs():
    """超长行（> MAX_LINE_BYTES 仍没有 \\n）必须丢弃并重新同步，不能无限吃内存。"""
    decoder = P.LineDecoder()
    junk = b"X" * 5000
    frames = decoder.feed(junk + b"\nE,1,2,3,4,5,6,7,8\n")
    eq(len(frames), 1, "只应出超长行之后的那一帧")
    check(isinstance(frames[0], P.EncoderFrame), "是那一帧 E")
    eq(decoder.stats.overlong_lines, 1, "超长计数 = 1")
    check(
        len(decoder._buf) <= decoder.max_line_bytes,  # noqa: SLF001 - 单测直接看内部缓冲
        "内部缓冲不得无限增长，实际 %d 字节" % len(decoder._buf),
    )


@case
def test_decoder_overlong_then_more_overlong():
    decoder = P.LineDecoder()
    frames = decoder.feed(b"Y" * 300 + b"\n" + b"Z" * 400 + b"\nE,9,9,9,9,9,9,9,9\n")
    eq(decoder.stats.overlong_lines, 2, "两次超长")
    eq(len(frames), 1, "最后出一帧")
    eq(frames[0].d_mm, (9, 9, 9, 9), "内容正确")


@case
def test_decoder_reset_discards_partial_line():
    decoder = P.LineDecoder()
    decoder.feed(b"E,1,2,3")
    decoder.reset()
    frames = decoder.feed(b"4,5,6,7,8\n")
    eq(len(frames), 1, "reset 之后的残片被丢掉，剩下的一小段不足以成帧")
    check(isinstance(frames[0], P.UnknownFrame), "残片只能是 UnknownFrame")


@case
def test_decoder_stats_as_dict():
    decoder = P.LineDecoder()
    decoder.feed(b"\nE,1,2,3,4,5,6,7,8\nbad\n" + b"Q" * 300 + b"\n")
    stats = decoder.stats.as_dict()
    eq(stats["frames_ok"], 1, "好帧")
    eq(stats["blank_lines"], 1, "空行")
    eq(stats["dropped_lines"], 1, "畸形行")
    eq(stats["overlong_lines"], 1, "超长行")
    eq(
        set(stats),
        {"frames_ok", "dropped_lines", "nonascii_lines", "overlong_lines", "blank_lines"},
        "计数器名字集合",
    )


@case
def test_decoder_max_line_bytes_guard():
    try:
        P.LineDecoder(max_line_bytes=4)
    except ValueError:
        pass
    else:
        raise AssertionError("max_line_bytes 太小必须报错")


@case
def test_decode_all_helper():
    frames = P.decode_all(b"E,1,2,3,4,5,6,7,8\nA,OK\n")
    eq(len(frames), 2, "便捷函数")


# --------------------------------------------------------------------------
# 5) 结构性约束：本模块必须能在没有 ROS2 的机器上 import
# --------------------------------------------------------------------------


@case
def test_rclpy_not_imported():
    """★ 验收标准第 1 条的核心：protocol 不许把 rclpy 拖进来。"""
    check(
        "rclpy" not in sys.modules,
        "import stm32_bridge.protocol 之后 sys.modules 里出现了 rclpy —— "
        "说明 __init__.py 或 protocol.py 里 import 了 ROS2 的东西",
    )


@case
def test_protocol_only_imports_stdlib():
    """用 AST 检查 protocol.py 的顶层 import 全是标准库。"""
    allowed = {"__future__", "math", "re", "dataclasses", "typing"}
    path = os.path.join(_HERE, "..", "stm32_bridge", "protocol.py")
    with open(path, "r", encoding="utf-8") as handle:
        tree = ast.parse(handle.read(), filename=path)
    roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                roots.add(alias.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom):
            if node.module and node.level == 0:
                roots.add(node.module.split(".")[0])
    extra = roots - allowed
    check(not extra, "protocol.py 出现了非标准库 import：%s" % sorted(extra))


# --------------------------------------------------------------------------
# 12) 限速器 + twist 归一化的名义周期
#    ★ 这两条是树莓派实机跑 run_e2e.py 暴露出来的：
#      ① twist 用实测 dt 归一化会被串口的块交付压到 ~0，把速度放大 20 倍；
#      ② /diagnostics 与下行 V 帧不限速会淹没 2 GB 的树莓派和 115200 的链路。
# --------------------------------------------------------------------------


@case
def test_nominal_encoder_period_is_frozen_50hz():
    """twist 归一化用的周期必须是**协议常量** 1/50 s，不是测出来的。

    冻结依据：``odom_update()`` 的签名里没有 dt，原理上产不出「每秒」量纲；
    而 ``06`` §7 把 E 帧冻结为 50 Hz ⇒ 名义周期 0.020 s。
    """
    eq(P.ENCODER_NOMINAL_HZ, 50.0, "E 帧名义频率（06 §7 冻结值）")
    check(
        abs(P.NOMINAL_ENCODER_PERIOD_S - 0.02) < 1e-12,
        "名义周期应是 1/50 = 0.02 s，实际 %r" % (P.NOMINAL_ENCODER_PERIOD_S,),
    )
    # 反证：如果拿实测 dt 当分母，4 mm 一帧会算出 4 m/s（实测踩到的 max=4.0000）
    eq(
        P.mm_s_to_m_s(4) / P.NOMINAL_ENCODER_PERIOD_S,
        0.2,
        "4 mm/帧 @ 名义 0.02 s 应还原成 0.2 m/s",
    )


@case
def test_rate_limiter_first_call_always_allowed():
    limiter = P.RateLimiter(0.05)
    eq(limiter.min_period_s, 0.05, "最小间隔")
    eq(limiter.last_emit_s, None, "还没发过")
    check(limiter.should_emit(1000.0), "第一次必须放行（没有『还没发过』这个阻塞态）")
    eq(limiter.last_emit_s, 1000.0, "记账")


@case
def test_rate_limiter_blocks_within_period_and_allows_after():
    limiter = P.RateLimiter(0.05)  # 20 Hz
    check(limiter.should_emit(0.0), "t=0 放行")
    check(not limiter.should_emit(0.001), "1 ms 后必须挡住")
    check(not limiter.should_emit(0.049), "49 ms 后仍挡住")
    check(limiter.should_emit(0.050), "整 50 ms 边界要放行")
    check(not limiter.should_emit(0.099), "下一次要重新计时")
    check(limiter.should_emit(0.150), "第二个周期")


@case
def test_rate_limiter_force_bypasses_and_rearms():
    """安全帧（断流补发的 V,0,0,0）必须能绕过限速立刻发。"""
    limiter = P.RateLimiter(0.05)
    check(limiter.should_emit(0.0), "t=0 放行")
    check(limiter.should_emit(0.001, force=True), "force 必须无视最小间隔")
    eq(limiter.last_emit_s, 0.001, "force 也要重新计时，否则紧跟着又会放行一次")
    check(not limiter.should_emit(0.02), "force 之后仍受正常限速约束")
    limiter.reset()
    eq(limiter.last_emit_s, None, "reset 回到未发过")
    check(limiter.should_emit(0.02), "reset 之后立刻可发")


@case
def test_rate_limiter_zero_period_means_unlimited():
    limiter = P.RateLimiter(0.0)
    for now in (0.0, 0.0, 1e-9):
        check(limiter.should_emit(now), "0 间隔 = 不限速")


@case
def test_rate_limiter_rejects_bad_period():
    for bad in (-1.0, float("nan"), float("inf")):
        try:
            P.RateLimiter(bad)
        except ValueError:
            continue
        raise AssertionError("min_period_s=%r 应该被拒绝" % (bad,))


@case
def test_rate_limiter_clock_going_backwards_does_not_leak():
    """单调钟理论上不回退，但真回退了也不能变成"每帧都放行"。"""
    limiter = P.RateLimiter(0.05)
    check(limiter.should_emit(10.0), "t=10 放行")
    check(not limiter.should_emit(9.0), "时钟倒退不该放行")
    check(limiter.should_emit(10.05), "回到正轨后正常放行")


@case
def test_init_does_not_import_bridge_node():
    """__init__.py 里不能 import bridge_node，否则 import protocol 就会拉 rclpy。"""
    path = os.path.join(_HERE, "..", "stm32_bridge", "__init__.py")
    with open(path, "r", encoding="utf-8") as handle:
        tree = ast.parse(handle.read(), filename=path)
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module == "bridge_node":
            raise AssertionError("__init__.py 里 import 了 bridge_node")
        if isinstance(node, ast.ImportFrom) and node.module == "kinematics":
            raise AssertionError("__init__.py 里 import 了 kinematics（会在没 .so 的机器上炸）")


# --------------------------------------------------------------------------
# 跑
# --------------------------------------------------------------------------


def main() -> int:
    print("stm32_bridge.protocol 单测 —— 共 %d 个用例" % len(CASES))
    print("Python %s on %s" % (sys.version.split()[0], sys.platform))
    print("-" * 72)
    failed = []
    for fn in CASES:
        before = _Counter.checks
        try:
            fn()
        except Exception as exc:  # noqa: BLE001 - 测试框架要抓住一切
            failed.append((fn.__name__, exc))
            print("FAIL  %-52s %s" % (fn.__name__, exc))
        else:
            print("ok    %-52s (%d 项断言)" % (fn.__name__, _Counter.checks - before))
    print("-" * 72)
    print(
        "结果：%d 个用例，%d 个失败，共 %d 项断言"
        % (len(CASES), len(failed), _Counter.checks)
    )
    if failed:
        print("\n失败清单：")
        for name, exc in failed:
            print("  * %s: %s" % (name, exc))
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
