#!/usr/bin/env python3
"""protocol.py unit tests — **no pytest, no ROS2 required**.

How to run (also works on Windows without ROS2)::

    python ros2_ws/src/stm32_bridge/test/test_protocol.py

Exit code 0 when everything is green, 1 if any case fails (and which one is printed).

Why ship a tiny test framework: the T2 task card requires "no pytest / ROS2,
just run ``python test_protocol.py``" — the Pi currently has no internet access, so pytest cannot be installed.
"""

from __future__ import annotations

import ast
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(_HERE, "..")))

from stm32_bridge import protocol as P  # noqa: E402  (must come after the sys.path adjustment)

# --------------------------------------------------------------------------
# Minimal test framework
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
        "%s: expected %r, actual %r" % (message or "value mismatch", expected, actual),
    )


# --------------------------------------------------------------------------
# 1) Rounding: must match C's round(), not Python's banker's rounding
# --------------------------------------------------------------------------


@case
def test_rounding_half_away_from_zero():
    r = P.round_half_away_from_zero
    eq(r(0.5), 1, "0.5 should round away from zero")
    eq(r(-0.5), -1, "-0.5 should round away from zero")
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
    """★ This case deliberately records the trap: Python's round() is banker's rounding, C's is not.

    Contract §4 requires "round half away from zero, not int() truncation (negative speeds would be off by one LSB)" —
    using Python's built-in round() here gives 2.5 -> 2, while C's lround(2.5) -> 3;
    a half-LSB difference is a systematic bias that shows up once accumulated.
    """
    r = P.round_half_away_from_zero
    check(round(2.5) == 2, "precondition changed: Python round(2.5) is not 2")
    eq(r(2.5), 3, "our implementation must match C, not Python's built-in")
    check(r(2.5) != round(2.5), "the two roundings must differ at .5, otherwise this test is meaningless")


@case
def test_unit_conversions():
    eq(P.m_s_to_mm_s(0.2), 200, "0.2 m/s")
    eq(P.m_s_to_mm_s(-0.2), -200, "-0.2 m/s")
    eq(P.m_s_to_mm_s(0.0005), 1, "+0.5 mm/s must round up")
    eq(P.m_s_to_mm_s(-0.0005), -1, "-0.5 mm/s must round away from zero")
    eq(P.rad_s_to_mrad_s(0.5), 500, "0.5 rad/s")
    eq(P.rad_s_to_mrad_s(-1.2345), -1235, "-1.2345 rad/s rounded half away from zero")
    eq(P.mm_s_to_m_s(200), 0.2, "round trip")
    eq(P.mrad_s_to_rad_s(-1500), -1.5, "round trip")


@case
def test_rounding_rejects_nan_inf():
    for bad in (float("nan"), float("inf"), float("-inf")):
        try:
            P.round_half_away_from_zero(bad)
        except ValueError:
            continue
        raise AssertionError("non-finite value %r must raise ValueError" % (bad,))


# --------------------------------------------------------------------------
# 2) Downlink encoding
# --------------------------------------------------------------------------


@case
def test_encode_velocity_exact_bytes():
    eq(P.encode_velocity(200, 0, 0), b"V,200,0,0\n", "straight ahead")
    eq(P.encode_velocity(-200, 150, -300), b"V,-200,150,-300\n", "diagonal + rotation")
    eq(P.encode_velocity(0, 0, 0), P.ZERO_VELOCITY_FRAME, "stop-frame constant must match")


@case
def test_encode_cmd_vel_from_ros2_twist():
    eq(P.encode_cmd_vel(0.2, 0.0, 0.0), b"V,200,0,0\n", "ROS2 0.2 m/s")
    eq(P.encode_cmd_vel(-0.35, 0.12, -0.5), b"V,-350,120,-500\n", "all three components must be converted")


@case
def test_encode_velocity_rejects_bad_input():
    for bad in (1.5, "200", None, True):
        try:
            P.encode_velocity(bad, 0, 0)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            continue
        raise AssertionError("non-integer %r must be rejected" % (bad,))
    try:
        P.encode_velocity(2 ** 31, 0, 0)
    except ValueError:
        pass
    else:
        raise AssertionError("out-of-range int32 must be rejected")


@case
def test_encode_motor():
    eq(P.encode_motor(0, 0, 0, 0), b"M,0,0,0,0\n", "all zeros")
    eq(P.encode_motor(-1000, 1000, 500, -500), b"M,-1000,1000,500,-500\n", "boundary values are valid")
    try:
        P.encode_motor(1001, 0, 0, 0)
    except ValueError:
        pass
    else:
        raise AssertionError("PWM 1001 must be rejected")


@case
def test_encode_pid():
    eq(P.encode_pid(0, 10, 2, 1), b"P,0,10,2,1\n", "channel 0")
    eq(P.encode_pid(3, 0, 0, 0), b"P,3,0,0,0\n", "channel 3")
    for bad in (-1, 4):
        try:
            P.encode_pid(bad, 0, 0, 0)
        except ValueError:
            continue
        raise AssertionError("pid_id=%d must be rejected" % bad)


@case
def test_frozen_constants():
    """These values are frozen in `11_interface_contract.md` §3 — changing them changes the protocol."""
    eq(P.ZERO_VELOCITY_FRAME, b"V,0,0,0\n", "stop frame")
    eq(P.SOFT_ESTOP_FRAME, b"S\n", "soft e-stop frame")
    eq(P.RESET_FRAME, b"reset\n", "reset frame")
    eq(P.RESET_BURST_COUNT, 3, "reset burst count (contract §3)")
    eq(P.RESET_BURST_PERIOD_S, 0.3, "reset burst period (contract §3)")
    eq(P.WHEEL_COUNT, 4, "wheel count")
    eq(P.MAX_LINE_BYTES, 256, "max line length")


# --------------------------------------------------------------------------
# 3) Single-line parsing
# --------------------------------------------------------------------------


@case
def test_parse_encoder_frame():
    frame = P.parse_line("E,100,101,102,103,1000,2000,3000,4000")
    check(isinstance(frame, P.EncoderFrame), "must parse as EncoderFrame")
    eq(frame.v_mm_s, (100, 101, 102, 103), "four wheel speeds")
    eq(frame.d_mm, (1000, 2000, 3000, 4000), "four accumulated distances")
    eq(frame.kind, "E", "kind")


@case
def test_encoder_wheel_order_is_fl_fr_rl_rr():
    """Contract §3: wheel order is always FL, FR, RL, RR (not ROS2's FL,FR,RR,RL)."""
    frame = P.parse_line("E,1,2,3,4,11,22,33,44")
    eq(frame.v_mm_s, (1, 2, 3, 4), "field 3 is RL and must not be swapped with RR")
    eq(frame.d_mm, (11, 22, 33, 44), "distances in the same order")


@case
def test_parse_encoder_negative_values():
    frame = P.parse_line("E,-100,-101,-102,-103,-1000,-2000,-3000,-4000")
    eq(frame.v_mm_s, (-100, -101, -102, -103), "negative wheel speeds must be supported")
    eq(frame.d_mm, (-1000, -2000, -3000, -4000), "negative distances must be supported")


@case
def test_parse_battery_frame():
    frame = P.parse_line("B,11800,3")
    check(isinstance(frame, P.BatteryFrame), "must parse as BatteryFrame")
    eq(frame.millivolts, 11800, "mV")
    eq(frame.flags, 3, "flags")
    eq(frame.volts, 11.8, "volts property")


@case
def test_parse_ack_keeps_commas():
    """A-frame text may contain commas — split on the **first** comma only."""
    frame = P.parse_line("A,OK reset, wheels=0")
    check(isinstance(frame, P.AckFrame), "must parse as AckFrame")
    eq(frame.text, "OK reset, wheels=0", "commas must be preserved")


@case
def test_parse_ack_requires_comma():
    """A bare "A" (no comma) is a malformed line — consistent with E / B handling; do not treat it as an "empty acknowledgement"."""
    frame = P.parse_line("A")
    check(isinstance(frame, P.UnknownFrame), "bare A must be UnknownFrame, actual %r" % (frame,))
    eq(frame.reason, "missing comma separator", "record the reason")
    empty = P.parse_line("A,")
    check(isinstance(empty, P.AckFrame), "A, is a valid empty acknowledgement")
    eq(empty.text, "", "empty acknowledgement text is an empty string")


@case
def test_parse_empty_and_blank():
    eq(P.parse_line(""), None, "empty string -> None (not an error)")


@case
def test_parse_unknown_prefix():
    frame = P.parse_line("Z,1,2,3")
    check(isinstance(frame, P.UnknownFrame), "unknown prefix -> UnknownFrame")
    eq(frame.prefix, "Z", "record the prefix")
    eq(frame.reason, "unknown prefix", "record the reason")


@case
def test_parse_encoder_wrong_arity():
    for bad in ("E,1,2,3", "E,1,2,3,4", "E,1,2,3,4,5,6,7,8,9", "E", "E,"):
        frame = P.parse_line(bad)
        check(
            isinstance(frame, P.UnknownFrame),
            "%r has the wrong field count, must be UnknownFrame, actual %r" % (bad, frame),
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
            "%r contains an invalid integer, must be UnknownFrame, actual %r" % (bad, frame),
        )


@case
def test_parse_encoder_out_of_int32():
    frame = P.parse_line("E,99999999999999,2,3,4,5,6,7,8")
    check(isinstance(frame, P.UnknownFrame), "out-of-int32 must be rejected")


@case
def test_parse_battery_wrong_arity():
    for bad in ("B", "B,11800", "B,11800,3,7"):
        frame = P.parse_line(bad)
        check(isinstance(frame, P.UnknownFrame), "%r must be UnknownFrame" % bad)


# --------------------------------------------------------------------------
# 4) Streaming decoder: partial packets / sticky packets / garbage / overlong
# --------------------------------------------------------------------------


@case
def test_decoder_half_packet():
    """Partial packet: no frame may be emitted before \\n arrives.

    ★ Concatenating the three chunks must be **exactly**
    ``E,100,101,102,103,1000,2000,3000,4000`` (E + 4 wheel speeds + 4 accumulated distances = 8 fields).
    The first version of this case wrote the third chunk as ``b"000,4000\\n"``, producing
    ``...3000,4000,4000`` — **9 fields** — correctly flagged as a malformed line (UnknownFrame).
    That was an arithmetic error in the test, not the decoder's fault: rejecting a 9-field E frame is **correct**.
    The input was fixed, not the assertion.
    """
    decoder = P.LineDecoder()
    eq(decoder.feed(b"E,100,101,102,103,10"), [], "no frame before the newline")
    eq(decoder.feed(b"00,2000,3000,"), [], "still partial (and split right in the middle of the last token)")
    frames = decoder.feed(b"4000\n")
    eq(len(frames), 1, "1 frame once the newline completes it")
    check(isinstance(frames[0], P.EncoderFrame), "must be EncoderFrame, actual %r" % (frames[0],))
    eq(frames[0].d_mm, (1000, 2000, 3000, 4000), "content correct")
    eq(frames[0].v_mm_s, (100, 101, 102, 103), "wheel-speed order correct")


@case
def test_decoder_split_mid_token():
    decoder = P.LineDecoder()
    out = []
    for chunk in (b"B,1", b"18", b"00,", b"3", b"\n"):
        out.extend(decoder.feed(chunk))
    eq(len(out), 1, "feeding byte by byte must still emit 1 frame")
    eq(out[0].millivolts, 11800, "a token split across chunks must be reassembled")


@case
def test_decoder_sticky_packets():
    """Sticky packets: three frames in one read must all be decoded, in order."""
    decoder = P.LineDecoder()
    frames = decoder.feed(b"E,1,2,3,4,5,6,7,8\nB,11800,0\nA,OK\n")
    eq(len(frames), 3, "three frames")
    check(isinstance(frames[0], P.EncoderFrame), "frame 1 is E")
    check(isinstance(frames[1], P.BatteryFrame), "frame 2 is B")
    check(isinstance(frames[2], P.AckFrame), "frame 3 is A")


@case
def test_decoder_crlf_tolerated():
    """★★ Regression test for a real bug (caught when the Lead ran it).

    The first version of ``_decode_line`` checked for "any character < 0x20" before stripping,
    so a CRLF line ending's ``\\r`` was judged "contains non-ASCII / control characters" and the **whole line was dropped**:
    ``B,12000,1`` failed to parse and fell into UnknownFrame. The moment the STM32 side switched to ``\\r\\n``,
    the bridge would **silently drop every frame** with no error — so the counters are pinned down here too.
    """
    decoder = P.LineDecoder()
    frames = decoder.feed(b"E,1,2,3,4,5,6,7,8\r\nB,12000,1\r\n")
    eq(len(frames), 2, "CRLF must also decode")
    check(isinstance(frames[0], P.EncoderFrame), "frame 1 is E, actual %r" % (frames[0],))
    check(isinstance(frames[1], P.BatteryFrame), "frame 2 is B, actual %r" % (frames[1],))
    eq(frames[1].millivolts, 12000, "\\r must not leak into fields")
    eq(decoder.stats.nonascii_lines, 0, "\\r is not a non-ASCII error")
    eq(decoder.stats.dropped_lines, 0, "CRLF must not produce any malformed frames")


@case
def test_decoder_embedded_control_char_still_rejected():
    """★ Reverse check: stripping ``\\r`` must not also let **genuine** control characters through."""
    decoder = P.LineDecoder()
    frames = decoder.feed(b"B,118\x0000,1\nB,12000,1\n")
    eq(len(frames), 2, "the bad line still yields 1 UnknownFrame while the good line comes through")
    check(isinstance(frames[0], P.UnknownFrame), "embedded \\x00 must be rejected, actual %r" % (frames[0],))
    check(isinstance(frames[1], P.BatteryFrame), "the next line must recover")
    eq(decoder.stats.nonascii_lines, 1, "non-ASCII count")
    eq(decoder.stats.frames_ok, 1, "good-frame count")


@case
def test_decoder_blank_lines_are_not_errors():
    decoder = P.LineDecoder()
    frames = decoder.feed(b"\n\r\n\nE,1,2,3,4,5,6,7,8\n")
    eq(len(frames), 1, "blank lines emit no frame")
    eq(decoder.stats.blank_lines, 3, "blank-line count")
    eq(decoder.stats.dropped_lines, 0, "blank lines are not errors")


@case
def test_decoder_garbage_recovery():
    """Must recover after binary garbage (the classic symptom of a wrong baud rate)."""
    decoder = P.LineDecoder()
    frames = decoder.feed(b"\x00\xff\xfe\x01\x02 garbage \x03\nE,1,2,3,4,5,6,7,8\n")
    eq(len(frames), 2, "the garbage line yields one UnknownFrame, then a real frame")
    check(isinstance(frames[0], P.UnknownFrame), "entry 1 is UnknownFrame")
    check(isinstance(frames[1], P.EncoderFrame), "entry 2 is a real frame — so the decoder resynchronized")
    eq(decoder.stats.nonascii_lines, 1, "non-ASCII count")
    eq(decoder.stats.frames_ok, 1, "good-frame count")


@case
def test_decoder_malformed_line_then_good_line():
    decoder = P.LineDecoder()
    frames = decoder.feed(b"E,1,2,3\nE,1,2,3,4,5,6,7,8\n")
    eq(len(frames), 2, "malformed frame + good frame")
    check(isinstance(frames[0], P.UnknownFrame), "entry 1 is malformed")
    check(isinstance(frames[1], P.EncoderFrame), "entry 2 is normal")
    eq(decoder.stats.dropped_lines, 1, "malformed count = 1")


@case
def test_decoder_overlong_line_resyncs():
    """An overlong line (> MAX_LINE_BYTES with still no \\n) must be dropped and the decoder resynchronized; memory must not grow without bound."""
    decoder = P.LineDecoder()
    junk = b"X" * 5000
    frames = decoder.feed(junk + b"\nE,1,2,3,4,5,6,7,8\n")
    eq(len(frames), 1, "only the frame after the overlong line should come out")
    check(isinstance(frames[0], P.EncoderFrame), "that E frame")
    eq(decoder.stats.overlong_lines, 1, "overlong count = 1")
    check(
        len(decoder._buf) <= decoder.max_line_bytes,  # noqa: SLF001 - the unit test inspects the internal buffer directly
        "the internal buffer must not grow without bound, actual %d bytes" % len(decoder._buf),
    )


@case
def test_decoder_overlong_then_more_overlong():
    decoder = P.LineDecoder()
    frames = decoder.feed(b"Y" * 300 + b"\n" + b"Z" * 400 + b"\nE,9,9,9,9,9,9,9,9\n")
    eq(decoder.stats.overlong_lines, 2, "two overlong lines")
    eq(len(frames), 1, "one frame at the end")
    eq(frames[0].d_mm, (9, 9, 9, 9), "content correct")


@case
def test_decoder_reset_discards_partial_line():
    decoder = P.LineDecoder()
    decoder.feed(b"E,1,2,3")
    decoder.reset()
    frames = decoder.feed(b"4,5,6,7,8\n")
    eq(len(frames), 1, "the fragment after reset is dropped; the remaining piece is too short to form a frame")
    check(isinstance(frames[0], P.UnknownFrame), "the fragment can only be UnknownFrame")


@case
def test_decoder_stats_as_dict():
    decoder = P.LineDecoder()
    decoder.feed(b"\nE,1,2,3,4,5,6,7,8\nbad\n" + b"Q" * 300 + b"\n")
    stats = decoder.stats.as_dict()
    eq(stats["frames_ok"], 1, "good frames")
    eq(stats["blank_lines"], 1, "blank lines")
    eq(stats["dropped_lines"], 1, "malformed lines")
    eq(stats["overlong_lines"], 1, "overlong lines")
    eq(
        set(stats),
        {"frames_ok", "dropped_lines", "nonascii_lines", "overlong_lines", "blank_lines"},
        "counter-name set",
    )


@case
def test_decoder_max_line_bytes_guard():
    try:
        P.LineDecoder(max_line_bytes=4)
    except ValueError:
        pass
    else:
        raise AssertionError("a too-small max_line_bytes must raise")


@case
def test_decode_all_helper():
    frames = P.decode_all(b"E,1,2,3,4,5,6,7,8\nA,OK\n")
    eq(len(frames), 2, "convenience helper")


# --------------------------------------------------------------------------
# 5) Structural constraints: this module must import on machines without ROS2
# --------------------------------------------------------------------------


@case
def test_rclpy_not_imported():
    """★ Core of acceptance criterion 1: protocol must not drag rclpy in."""
    check(
        "rclpy" not in sys.modules,
        "after importing stm32_bridge.protocol, rclpy appeared in sys.modules — "
        "meaning __init__.py or protocol.py imported something from ROS2",
    )


@case
def test_protocol_only_imports_stdlib():
    """Use AST to check that every top-level import in protocol.py is a standard library."""
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
    check(not extra, "protocol.py has a non-stdlib import: %s" % sorted(extra))


# --------------------------------------------------------------------------
# 12) Rate limiter + the nominal period for twist normalization
#    ★ These two were exposed by running run_e2e.py on the real Pi:
#      (1) normalizing twist by the measured dt is compressed to ~0 by serial block delivery, inflating the speed 20x;
#      (2) unthrottled /diagnostics and downlink V frames swamp a 2 GB Pi and a 115200 link.
# --------------------------------------------------------------------------


@case
def test_nominal_encoder_period_is_frozen_50hz():
    """The period used for twist normalization must be the **protocol constant** 1/50 s, not a measured value.

    Rationale: ``odom_update()``'s signature has no dt, so it cannot produce a "per second" quantity;
    and ``06`` §7 freezes E frames at 50 Hz ⇒ nominal period 0.020 s.
    """
    eq(P.ENCODER_NOMINAL_HZ, 50.0, "nominal E-frame rate (frozen value from 06 §7)")
    check(
        abs(P.NOMINAL_ENCODER_PERIOD_S - 0.02) < 1e-12,
        "nominal period should be 1/50 = 0.02 s, actual %r" % (P.NOMINAL_ENCODER_PERIOD_S,),
    )
    # Counter-check: using the measured dt as the divisor turns 4 mm per frame into 4 m/s (max=4.0000 was observed)
    eq(
        P.mm_s_to_m_s(4) / P.NOMINAL_ENCODER_PERIOD_S,
        0.2,
        "4 mm/frame at the nominal 0.02 s should restore 0.2 m/s",
    )


@case
def test_rate_limiter_first_call_always_allowed():
    limiter = P.RateLimiter(0.05)
    eq(limiter.min_period_s, 0.05, "minimum period")
    eq(limiter.last_emit_s, None, "never emitted")
    check(limiter.should_emit(1000.0), "the first call must be allowed (there is no \'never emitted\' blocking state)")
    eq(limiter.last_emit_s, 1000.0, "accounting")


@case
def test_rate_limiter_blocks_within_period_and_allows_after():
    limiter = P.RateLimiter(0.05)  # 20 Hz
    check(limiter.should_emit(0.0), "t=0 allowed")
    check(not limiter.should_emit(0.001), "blocked after 1 ms")
    check(not limiter.should_emit(0.049), "still blocked after 49 ms")
    check(limiter.should_emit(0.050), "allowed at the exact 50 ms boundary")
    check(not limiter.should_emit(0.099), "the next one restarts the timer")
    check(limiter.should_emit(0.150), "second period")


@case
def test_rate_limiter_force_bypasses_and_rearms():
    """Safety frames (the timeout V,0,0,0) must bypass the rate limit and go out immediately."""
    limiter = P.RateLimiter(0.05)
    check(limiter.should_emit(0.0), "t=0 allowed")
    check(limiter.should_emit(0.001, force=True), "force must ignore the minimum period")
    eq(limiter.last_emit_s, 0.001, "force must also restart the timer, otherwise the next call is allowed immediately")
    check(not limiter.should_emit(0.02), "normal rate limiting applies again after force")
    limiter.reset()
    eq(limiter.last_emit_s, None, "reset returns to never-emitted")
    check(limiter.should_emit(0.02), "allowed immediately after reset")


@case
def test_rate_limiter_zero_period_means_unlimited():
    limiter = P.RateLimiter(0.0)
    for now in (0.0, 0.0, 1e-9):
        check(limiter.should_emit(now), "a 0 period = no rate limit")


@case
def test_rate_limiter_rejects_bad_period():
    for bad in (-1.0, float("nan"), float("inf")):
        try:
            P.RateLimiter(bad)
        except ValueError:
            continue
        raise AssertionError("min_period_s=%r must be rejected" % (bad,))


@case
def test_rate_limiter_clock_going_backwards_does_not_leak():
    """A monotonic clock should never go backwards, but if it does, it must not become "allow every frame"."""
    limiter = P.RateLimiter(0.05)
    check(limiter.should_emit(10.0), "t=10 allowed")
    check(not limiter.should_emit(9.0), "a backwards clock must not allow")
    check(limiter.should_emit(10.05), "allowed normally once back on track")


@case
def test_init_does_not_import_bridge_node():
    """__init__.py must not import bridge_node, otherwise importing protocol pulls in rclpy."""
    path = os.path.join(_HERE, "..", "stm32_bridge", "__init__.py")
    with open(path, "r", encoding="utf-8") as handle:
        tree = ast.parse(handle.read(), filename=path)
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module == "bridge_node":
            raise AssertionError("__init__.py imports bridge_node")
        if isinstance(node, ast.ImportFrom) and node.module == "kinematics":
            raise AssertionError("__init__.py imports kinematics (which would blow up on machines without the .so)")


# --------------------------------------------------------------------------
# Run
# --------------------------------------------------------------------------


def main() -> int:
    print("stm32_bridge.protocol unit tests — %d cases" % len(CASES))
    print("Python %s on %s" % (sys.version.split()[0], sys.platform))
    print("-" * 72)
    failed = []
    for fn in CASES:
        before = _Counter.checks
        try:
            fn()
        except Exception as exc:  # noqa: BLE001 - the test framework must catch everything
            failed.append((fn.__name__, exc))
            print("FAIL  %-52s %s" % (fn.__name__, exc))
        else:
            print("ok    %-52s (%d assertions)" % (fn.__name__, _Counter.checks - before))
    print("-" * 72)
    print(
        "Result: %d cases, %d failed, %d assertions total"
        % (len(CASES), len(failed), _Counter.checks)
    )
    if failed:
        print("\nFailure list:")
        for name, exc in failed:
            print("  * %s: %s" % (name, exc))
        return 1
    print("All passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
