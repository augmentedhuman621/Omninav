"""stm32_bridge.protocol —— STM32 <-> 树莓派 串口帧的纯编码 / 解析。

设计硬约束（照 ``11_接口契约.md`` §3 / §4 与 T2 任务卡）：

* 本模块**不许 import rclpy**，也不 import 任何第三方库。
  这样在没装 ROS2 的 Windows 上也能 ``python test_protocol.py`` 跑单测。
* 只 import 标准库（``math`` / ``re`` / ``dataclasses`` / ``typing``）。
* 解析必须容忍**半包 / 粘包 / 校验错 / 超长 / 非法字符**，并且能恢复。
* 单位：串口侧恒为整数（mm/s、mrad/s）；浮点只出现在 ROS2 边界上。
* 本模块**不含任何运动学 / 里程计数学** —— 那只有一份，在 ``firmware_core`` 的 C 代码里。

帧格式（**冻结**，见 ``11_接口契约.md`` §3）：

下行 Pi -> STM32::

    V,<vx>,<vy>,<w>\\n          机体速度指令；vx/vy 单位 mm/s，w 单位 mrad/s，全部整数
    M,<p1>,<p2>,<p3>,<p4>\\n    调试：直接给四路 PWM，范围 -1000 ~ 1000
    P,<id>,<kp>,<ki>,<kd>\\n    在线改 PID 参数
    S\\n                        软急停
    reset\\n                    复位（上电时连发 3 次，间隔 0.3 s）

上行 STM32 -> Pi::

    E,<v1>,<v2>,<v3>,<v4>,<d1>,<d2>,<d3>,<d4>\\n   轮速 mm/s ×4 + 累计里程 mm ×4，50 Hz
    B,<mv>,<flags>\\n                               电池电压 mV + 状态位，10 Hz
    A,<text>\\n                                     命令回执 / 错误

轮序恒为 ``FL, FR, RL, RR``（= C 侧 ``MECANUM_FL/FR/RL/RR``），
**不是** ROS2 ``mecanum_drive_controller`` 的 ``FL,FR,RR,RL``。
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import ClassVar, Dict, List, Optional, Sequence, Tuple

__all__ = [
    "MAX_LINE_BYTES",
    "WHEEL_COUNT",
    "ZERO_VELOCITY_FRAME",
    "SOFT_ESTOP_FRAME",
    "RESET_FRAME",
    "ENCODER_NOMINAL_HZ",
    "NOMINAL_ENCODER_PERIOD_S",
    "RateLimiter",
    "round_half_away_from_zero",
    "m_s_to_mm_s",
    "rad_s_to_mrad_s",
    "mm_s_to_m_s",
    "mrad_s_to_rad_s",
    "encode_velocity",
    "encode_cmd_vel",
    "encode_motor",
    "encode_pid",
    "EncoderFrame",
    "BatteryFrame",
    "AckFrame",
    "UnknownFrame",
    "Frame",
    "DecoderStats",
    "LineDecoder",
    "parse_line",
    "decode_all",
]

# --------------------------------------------------------------------------
# 常量
# --------------------------------------------------------------------------

#: 单行最大字节数。超过就整行丢弃并重新同步（防串口半路接上时灌进来一整块垃圾）。
MAX_LINE_BYTES = 256

#: 轮数（FL, FR, RL, RR）。
WHEEL_COUNT = 4

INT32_MIN = -(2 ** 31)
INT32_MAX = 2 ** 31 - 1

#: PWM 调试帧的取值范围（契约 §3）。
PWM_MIN = -1000
PWM_MAX = 1000

#: PID 通道号范围（4 路电机）。
PID_ID_MIN = 0
PID_ID_MAX = 3

#: 键盘 / 断流时补发的停车帧。
ZERO_VELOCITY_FRAME = b"V,0,0,0\n"
#: 软急停。
SOFT_ESTOP_FRAME = b"S\n"
#: 复位。契约 §3 要求上电时连发 3 次、间隔 0.3 s。
RESET_FRAME = b"reset\n"
#: 复位连发次数与间隔（契约 §3，冻结值）。
RESET_BURST_COUNT = 3
RESET_BURST_PERIOD_S = 0.3

#: ★★ E 帧的**名义**频率。``06`` §7 把上行 E 帧冻结为 50 Hz（契约 §5.1 修订）。
ENCODER_NOMINAL_HZ = 50.0

#: ★★ 归一化 ``/odom`` 的 twist 时使用的**协议名义周期**，不是实测帧间隔。
#:
#: ``odom_update()`` 的 ``twist_out`` 是**本帧位移增量**（mm / mrad），
#: 它的签名里根本没有 ``dt`` 参数，所以它在原理上产不出「每秒」量纲。
#: 契约 §4 又要求 ``/odom`` 的 twist 是 m/s、rad/s ⇒ 桥必须自己除一个时间。
#:
#: **这个时间恒取 1/50 s = 0.020 s，不许用每帧实测间隔。**
#: 原因（2026-xx 树莓派实测踩到的）：串口 / pty 按**块**交付，一次 ``read()``
#: 常常返回好几条 E 帧；逐帧去测「到达间隔」会退化到 ~0，
#: 于是 ``Δd / dt`` 被放大 20 倍 —— 实测就出现了 ``twist = 4.0 m/s`` 的尖峰
#: （``Δd = 4 mm``、``dt`` 被钳到 1 ms）。位置积分本身没问题，只有这个派生量被除歪。
#: 实测帧间隔依然有用，但它**只能进诊断**（判断发送端是否停摆 / 掉帧）。
NOMINAL_ENCODER_PERIOD_S = 1.0 / ENCODER_NOMINAL_HZ

_INT_TOKEN = re.compile(r"\A[+-]?[0-9]+\Z")

#: 原始行留作证据时的最大长度（够日志用，且不会把一整块垃圾写进日志）。
_RAW_KEEP = 80


# --------------------------------------------------------------------------
# 限速器（纯 stdlib，可脱开 ROS2 单测）
# --------------------------------------------------------------------------


class RateLimiter:
    """按最小间隔放行的节流器。

    ★ 放在 ``protocol.py`` 里是**故意的**：这两个限速（下行 ``V`` 20 Hz、
    ``/diagnostics`` 1 Hz）是纯逻辑，放在这里就能在没有 ROS2 的机器上单测，
    不必等树莓派。

    时钟由调用方传入（``time.monotonic()``），所以单测可以喂假时间。
    """

    __slots__ = ("_min_period_s", "_last_emit_s")

    def __init__(self, min_period_s: float) -> None:
        value = float(min_period_s)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("min_period_s 必须是非负有限数，收到 %r" % (min_period_s,))
        self._min_period_s = value
        self._last_emit_s: Optional[float] = None

    @property
    def min_period_s(self) -> float:
        return self._min_period_s

    @property
    def last_emit_s(self) -> Optional[float]:
        return self._last_emit_s

    def should_emit(self, now_s: float, force: bool = False) -> bool:
        """现在能不能放行。``force=True`` 一定放行（安全帧走这条路）。"""
        if force:
            self._last_emit_s = now_s
            return True
        if self._last_emit_s is None:
            self._last_emit_s = now_s
            return True
        if (now_s - self._last_emit_s) >= self._min_period_s:
            self._last_emit_s = now_s
            return True
        return False

    def reset(self) -> None:
        self._last_emit_s = None


# --------------------------------------------------------------------------
# 单位换算（契约 §4 的两行公式，仅此一处）
# --------------------------------------------------------------------------


def round_half_away_from_zero(value: float) -> int:
    """四舍五入，且**负数也远离零** —— 与 C 的 ``round()`` / ``lround()`` 一致。

    ★★ 不要用 Python 内建的 ``round()``：它是「银行家舍入」（round-half-to-even），
    ``round(0.5) == 0`` 而 ``round(1.5) == 2``；C 的 ``round(0.5) == 1``。
    契约 §4 明确要求「取整用四舍五入，不要 ``int()`` 截断（负速度会偏一个 LSB，
    长期累积成系统性偏差）」—— 半个 LSB 的银行家舍入同样是系统性偏差，
    所以这里自己实现，把行为钉死在 C 的语义上。

    >>> round_half_away_from_zero(0.5)
    1
    >>> round_half_away_from_zero(-0.5)
    -1
    >>> round_half_away_from_zero(0.4999)
    0
    """
    if not math.isfinite(value):
        raise ValueError("取整输入必须是有限数，拿到 %r" % (value,))
    if value >= 0.0:
        return int(math.floor(value + 0.5))
    return int(math.ceil(value - 0.5))


def m_s_to_mm_s(value: float) -> int:
    """ROS2 线速度 m/s -> 串口 mm/s。"""
    return round_half_away_from_zero(value * 1000.0)


def rad_s_to_mrad_s(value: float) -> int:
    """ROS2 角速度 rad/s -> 串口 mrad/s。"""
    return round_half_away_from_zero(value * 1000.0)


def mm_s_to_m_s(value: int) -> float:
    """串口 mm/s -> ROS2 线速度 m/s。"""
    return value / 1000.0


def mrad_s_to_rad_s(value: int) -> float:
    """串口 mrad/s -> ROS2 角速度 rad/s。"""
    return value / 1000.0


# --------------------------------------------------------------------------
# 下行编码
# --------------------------------------------------------------------------


def _require_int32(name: str, value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError("%s 必须是 int，拿到 %r" % (name, type(value).__name__))
    if not INT32_MIN <= value <= INT32_MAX:
        raise ValueError("%s=%d 超出 int32 范围" % (name, value))
    return value


def encode_velocity(vx_mm_s: int, vy_mm_s: int, omega_mrad_s: int) -> bytes:
    """编码 ``V`` 速度帧。三个参数都必须是**整数** mm/s / mrad/s。"""
    return b"V,%d,%d,%d\n" % (
        _require_int32("vx_mm_s", vx_mm_s),
        _require_int32("vy_mm_s", vy_mm_s),
        _require_int32("omega_mrad_s", omega_mrad_s),
    )


def encode_cmd_vel(linear_x: float, linear_y: float, angular_z: float) -> bytes:
    """把 ROS2 ``Twist`` 的三个浮点分量编码成 ``V`` 帧（契约 §4 的换算表）。

    这是**唯一**允许出现 ``m/s -> mm/s`` 换算的地方。
    """
    return encode_velocity(
        m_s_to_mm_s(linear_x), m_s_to_mm_s(linear_y), rad_s_to_mrad_s(angular_z)
    )


def encode_motor(p1: int, p2: int, p3: int, p4: int) -> bytes:
    """编码 ``M`` 调试帧（直接给四路 PWM，-1000 ~ 1000）。"""
    values = []
    for index, raw in enumerate((p1, p2, p3, p4)):
        value = _require_int32("p%d" % (index + 1), raw)
        if not PWM_MIN <= value <= PWM_MAX:
            raise ValueError("p%d=%d 超出 PWM 范围 [%d, %d]" % (index + 1, value, PWM_MIN, PWM_MAX))
        values.append(value)
    return b"M,%d,%d,%d,%d\n" % tuple(values)


def encode_pid(pid_id: int, kp: int, ki: int, kd: int) -> bytes:
    """编码 ``P`` 在线改 PID 帧。"""
    channel = _require_int32("pid_id", pid_id)
    if not PID_ID_MIN <= channel <= PID_ID_MAX:
        raise ValueError("pid_id=%d 超出 [%d, %d]" % (channel, PID_ID_MIN, PID_ID_MAX))
    return b"P,%d,%d,%d,%d\n" % (
        channel,
        _require_int32("kp", kp),
        _require_int32("ki", ki),
        _require_int32("kd", kd),
    )


# --------------------------------------------------------------------------
# 上行帧的数据类型
# --------------------------------------------------------------------------


def _clip_raw(text: str) -> str:
    return text if len(text) <= _RAW_KEEP else text[:_RAW_KEEP] + "..."


@dataclass(frozen=True)
class EncoderFrame:
    """``E`` 帧：四轮轮速（mm/s）+ 四轮累计里程（mm），顺序 FL, FR, RL, RR。"""

    kind: ClassVar[str] = "E"
    v_mm_s: Tuple[int, int, int, int] = (0, 0, 0, 0)
    d_mm: Tuple[int, int, int, int] = (0, 0, 0, 0)
    raw: str = ""

    def __len__(self) -> int:
        return WHEEL_COUNT


@dataclass(frozen=True)
class BatteryFrame:
    """``B`` 帧：电池电压（mV）+ 状态位。状态位的位定义在 STM32 固件里，本契约未冻结。"""

    kind: ClassVar[str] = "B"
    millivolts: int = 0
    flags: int = 0
    raw: str = ""

    @property
    def volts(self) -> float:
        return self.millivolts / 1000.0


@dataclass(frozen=True)
class AckFrame:
    """``A`` 帧：命令回执 / 错误文本。文本里**允许含逗号**，所以只按第一个逗号切分。"""

    kind: ClassVar[str] = "A"
    text: str = ""
    raw: str = ""


@dataclass(frozen=True)
class UnknownFrame:
    """无法识别 / 畸形的行。**不静默丢掉** —— 交给上层计数并记日志。"""

    kind: ClassVar[str] = "?"
    prefix: str = ""
    reason: str = ""
    raw: str = ""


Frame = object  # 上层只需 isinstance 判断具体类型；这里只为 typing 可读性留个名字。


@dataclass
class DecoderStats:
    """解析健康度计数器。放在 ``/diagnostics`` 里比"猜为什么没数据"便宜得多。"""

    frames_ok: int = 0
    dropped_lines: int = 0
    nonascii_lines: int = 0
    overlong_lines: int = 0
    blank_lines: int = 0

    def as_dict(self) -> Dict[str, int]:
        return {
            "frames_ok": self.frames_ok,
            "dropped_lines": self.dropped_lines,
            "nonascii_lines": self.nonascii_lines,
            "overlong_lines": self.overlong_lines,
            "blank_lines": self.blank_lines,
        }


# --------------------------------------------------------------------------
# 单行解析
# --------------------------------------------------------------------------


def _parse_int32_token(token: str):
    """把 token 解析成 int32；不合法返回 ``None``。

    ★ 只接受 ``^[+-]?[0-9]+$``：``1.5`` / ``1e3`` / ``0x10`` / ``" 1"`` 全部拒绝。
    用 ``int()`` 直接试会放过 ``"  1  "`` 和 ``"1_0"``（Python 的下划线字面量规则），
    所以先过正则再转换。
    """
    if not _INT_TOKEN.match(token):
        return None
    try:
        value = int(token, 10)
    except ValueError:  # pragma: no cover - 正则已经挡住
        return None
    if not INT32_MIN <= value <= INT32_MAX:
        return None
    return value


def parse_line(text: str) -> Optional[Frame]:
    """解析**一行**（不含行尾 ``\\n``，首尾空白已由调用方去掉）。

    返回 ``None`` 表示这一行是空的（正常，不算错误）。
    返回 ``UnknownFrame`` 表示畸形 / 未知前缀 —— 调用方应当计数并记日志。
    """
    if not text:
        return None

    head, sep, rest = text.partition(",")

    if head == "E":
        if not sep:
            return UnknownFrame(prefix="E", reason="缺失逗号分隔符", raw=_clip_raw(text))
        tokens = rest.split(",")
        if len(tokens) != 2 * WHEEL_COUNT:
            return UnknownFrame(
                prefix="E",
                reason="需要 %d 个字段，实际 %d 个" % (2 * WHEEL_COUNT, len(tokens)),
                raw=_clip_raw(text),
            )
        values: List[int] = []
        for index, token in enumerate(tokens):
            value = _parse_int32_token(token)
            if value is None:
                return UnknownFrame(
                    prefix="E",
                    reason="第 %d 个字段不是 int32：%r" % (index + 1, _clip_raw(token)),
                    raw=_clip_raw(text),
                )
            values.append(value)
        return EncoderFrame(
            v_mm_s=(values[0], values[1], values[2], values[3]),
            d_mm=(values[4], values[5], values[6], values[7]),
            raw=_clip_raw(text),
        )

    if head == "B":
        if not sep:
            return UnknownFrame(prefix="B", reason="缺失逗号分隔符", raw=_clip_raw(text))
        tokens = rest.split(",")
        if len(tokens) != 2:
            return UnknownFrame(
                prefix="B", reason="需要 2 个字段，实际 %d 个" % len(tokens), raw=_clip_raw(text)
            )
        millivolts = _parse_int32_token(tokens[0])
        flags = _parse_int32_token(tokens[1])
        if millivolts is None or flags is None:
            return UnknownFrame(prefix="B", reason="字段不是 int32", raw=_clip_raw(text))
        return BatteryFrame(millivolts=millivolts, flags=flags, raw=_clip_raw(text))

    if head == "A":
        if not sep:
            # ★ 与 E / B 保持一致：裸 "A"（没有逗号）是畸形行，不能当成"空回执"。
            #   否则一行噪声 "A" 会被解析成一条合法回执，把 /diagnostics 带偏。
            return UnknownFrame(prefix="A", reason="缺失逗号分隔符", raw=_clip_raw(text))
        # 文本里允许含逗号，所以 rest 原样收下，不再切分。
        return AckFrame(text=rest, raw=_clip_raw(text))

    return UnknownFrame(prefix=_clip_raw(head), reason="未知前缀", raw=_clip_raw(text))


# --------------------------------------------------------------------------
# 流式解码器：吃 byte 流，吐帧
# --------------------------------------------------------------------------


class LineDecoder:
    """把串口字节流切成帧，容忍半包 / 粘包 / 超长 / 非法字符。

    与 ``pyserial`` 的 ``readline()`` 的区别（为什么不用它）：

    * ``readline()`` 遇到非法字节或超长行会**一直长到把内存吃光**；
      本实现超过 :data:`MAX_LINE_BYTES` 就丢弃并重新同步。
    * ``readline()`` 的 ``\r\n`` 处理要额外 strip；本实现按 ``\\n`` 切，
      再统一 strip 空白。
    * 本实现顺带给出健康度计数（:class:`DecoderStats`）。
    """

    def __init__(self, max_line_bytes: int = MAX_LINE_BYTES) -> None:
        if max_line_bytes < 8:
            raise ValueError("max_line_bytes 太小：%r" % (max_line_bytes,))
        self.max_line_bytes = max_line_bytes
        self.stats = DecoderStats()
        self._buf = bytearray()
        self._skipping = False

    def reset(self) -> None:
        """丢掉半个包（例如串口刚被重新打开）。"""
        self._buf.clear()
        self._skipping = False

    def feed(self, data: bytes) -> List[Frame]:
        """喂一段字节，返回**这一批**解出来的帧（顺序与到达顺序一致）。"""
        frames: List[Frame] = []
        for byte in data:
            if byte == 0x0A:  # \n
                if self._skipping:
                    # 超长行的收尾：丢掉整行，从这一行之后重新同步。
                    self._skipping = False
                    self._buf.clear()
                    self.stats.overlong_lines += 1
                    continue
                line = bytes(self._buf)
                self._buf.clear()
                frame = self._decode_line(line)
                if frame is None:
                    self.stats.blank_lines += 1
                elif isinstance(frame, UnknownFrame):
                    self.stats.dropped_lines += 1
                    frames.append(frame)
                else:
                    self.stats.frames_ok += 1
                    frames.append(frame)
                continue

            if self._skipping:
                continue

            self._buf.append(byte)
            if len(self._buf) > self.max_line_bytes:
                # ★ 关键：不要无限增长。丢弃并等下一个 \n 重新同步。
                self._skipping = True
                self._buf.clear()
        return frames

    def _decode_line(self, line: bytes):
        text = line.decode("ascii", errors="replace")
        # ★★ 顺序很要命：必须**先剥掉行尾的 CR / 空白，再查可打印性**。
        #    反过来的话，CRLF 行尾那个 \r（0x0D < 0x20）会被当成"控制字符"整行丢掉：
        #      * `B,12000,1\r` 会解析失败，掉进 UnknownFrame；
        #      * 一个孤零零的 `\r\n`（空行）会变成一条**假错误**，污染 nonascii/dropped 计数。
        #    STM32 侧将来只要改用 `\r\n`，桥就会静默丢掉**所有**帧 —— 这是个不报错的静默失效。
        #    只剥 " \t\r\n" 而**不用** text.strip()：后者按 Unicode 空白剥，会把
        #    \x0b \x0c \x1c-\x1f \x85 \xa0 也一起吞掉，而那些恰恰是波特率不对的症状。
        stripped = text.strip(" \t\r\n")
        if not stripped:
            return None  # 空行 / 裸 \r\n —— 不算错误
        for char in stripped:
            code = ord(char)
            if code < 0x20 or code > 0x7E:
                # 非可打印 ASCII：波特率不对时的典型症状，整行丢弃。
                self.stats.nonascii_lines += 1
                return UnknownFrame(
                    prefix="", reason="含非 ASCII / 控制字符", raw=_clip_raw(stripped)
                )
        return parse_line(stripped)


def decode_all(data: bytes, max_line_bytes: int = MAX_LINE_BYTES) -> List[Frame]:
    """一次性把整块 bytes 解成帧（给单测和离线日志分析用）。"""
    return LineDecoder(max_line_bytes=max_line_bytes).feed(data)
