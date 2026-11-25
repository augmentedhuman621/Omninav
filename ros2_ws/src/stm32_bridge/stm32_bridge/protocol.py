"""stm32_bridge.protocol — pure encoding / parsing of STM32 <-> Raspberry Pi serial frames.

Hard design constraints (per ``11_interface_contract.md`` §3 / §4 and the T2 task card):

* This module **must not import rclpy**, nor any third-party library.
  This way the unit tests still run as ``python test_protocol.py`` on Windows without ROS2 installed.
* Imports only the standard library (``math`` / ``re`` / ``dataclasses`` / ``typing``).
* Parsing must tolerate **partial packets / coalesced packets / checksum errors / overlong lines / invalid characters** and still recover.
* Units: the serial side is always integer (mm/s, mrad/s); floats appear only at the ROS2 boundary.
* This module **contains no kinematics / odometry math** — that exists in exactly one place, the C code in ``firmware_core``.

Frame format (**frozen**, see ``11_interface_contract.md`` §3):

Downlink Pi -> STM32::

    V,<vx>,<vy>,<w>\\n          body velocity command; vx/vy in mm/s, w in mrad/s, all integers
    M,<p1>,<p2>,<p3>,<p4>\\n    debug: drive the four PWM channels directly, range -1000 ~ 1000
    P,<id>,<kp>,<ki>,<kd>\\n    change PID parameters online
    S\\n                        soft emergency stop
    reset\\n                    reset (sent 3 times at startup, 0.3 s apart)

Uplink STM32 -> Pi::

    E,<v1>,<v2>,<v3>,<v4>,<d1>,<d2>,<d3>,<d4>\\n   wheel speeds mm/s x4 + accumulated distance mm x4, 50 Hz
    B,<mv>,<flags>\\n                               battery voltage mV + status flags, 10 Hz
    A,<text>\\n                                     command acknowledgement / error

The wheel order is always ``FL, FR, RL, RR`` (= C side ``MECANUM_FL/FR/RL/RR``),
**not** the ROS2 ``mecanum_drive_controller`` order ``FL,FR,RR,RL``.
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
# Constants
# --------------------------------------------------------------------------

#: Maximum bytes per line. Longer lines are dropped and the decoder resynchronizes (prevents a whole block of garbage when the serial port is attached mid-stream).
MAX_LINE_BYTES = 256

#: Number of wheels (FL, FR, RL, RR).
WHEEL_COUNT = 4

INT32_MIN = -(2 ** 31)
INT32_MAX = 2 ** 31 - 1

#: Valid range of the PWM debug frame (contract §3).
PWM_MIN = -1000
PWM_MAX = 1000

#: PID channel index range (4 motors).
PID_ID_MIN = 0
PID_ID_MAX = 3

#: Stop frame sent on keyboard stop / command timeout.
ZERO_VELOCITY_FRAME = b"V,0,0,0\n"
#: Soft emergency stop.
SOFT_ESTOP_FRAME = b"S\n"
#: Reset. Contract §3 requires 3 sends at startup, 0.3 s apart.
RESET_FRAME = b"reset\n"
#: Reset burst count and interval (contract §3, frozen values).
RESET_BURST_COUNT = 3
RESET_BURST_PERIOD_S = 0.3

#: ★★ **Nominal** frequency of E frames. ``06`` §7 freezes the uplink E frame at 50 Hz (contract §5.1 revision).
ENCODER_NOMINAL_HZ = 50.0

#: ★★ The **protocol nominal period** used to normalize the ``/odom`` twist, not the measured frame interval.
#:
#: ``odom_update()`` returns ``twist_out`` as a **per-frame displacement increment** (mm / mrad);
#: its signature has no ``dt`` parameter at all, so it cannot produce a "per second" quantity in principle.
#: Contract §4 nonetheless requires the ``/odom`` twist in m/s and rad/s ⇒ the bridge must divide by a time itself.
#:
#: **This time is always 1/50 s = 0.020 s; the measured per-frame interval must not be used.**
#: Why (hit during Raspberry Pi measurements in 2026-xx): serial / pty delivers data in **blocks**, so one ``read()`` 
#: often returns several E frames; measuring the "arrival interval" frame by frame degenerates to ~0,
#: so ``Δd / dt`` is inflated 20x — measurements actually showed ``twist = 4.0 m/s`` spikes
#: (``Δd = 4 mm`` and ``dt`` clamped to 1 ms). Position integration itself was fine; only this derived quantity was divided wrong.
#: The measured frame interval is still useful, but it **may only feed diagnostics** (detecting a stalled sender / dropped frames).
NOMINAL_ENCODER_PERIOD_S = 1.0 / ENCODER_NOMINAL_HZ

_INT_TOKEN = re.compile(r"\A[+-]?[0-9]+\Z")

#: Maximum length of a raw line kept as evidence (enough for logging without dumping a whole block of garbage into the log).
_RAW_KEEP = 80


# --------------------------------------------------------------------------
# Rate limiter (pure stdlib, unit-testable without ROS2)
# --------------------------------------------------------------------------


class RateLimiter:
    """A throttle that lets events pass no more often than a minimum interval.

    ★ It is **deliberate** that this lives in ``protocol.py``: the two rate limits (downlink ``V`` at 20 Hz,
    ``/diagnostics`` at 1 Hz) are pure logic, so placing them here keeps them unit-testable on a machine without ROS2,
    without waiting for the Raspberry Pi.

    The caller supplies the clock (``time.monotonic()``), so unit tests can feed fake time.
    """

    __slots__ = ("_min_period_s", "_last_emit_s")

    def __init__(self, min_period_s: float) -> None:
        value = float(min_period_s)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("min_period_s must be a non-negative finite number, got %r" % (min_period_s,))
        self._min_period_s = value
        self._last_emit_s: Optional[float] = None

    @property
    def min_period_s(self) -> float:
        return self._min_period_s

    @property
    def last_emit_s(self) -> Optional[float]:
        return self._last_emit_s

    def should_emit(self, now_s: float, force: bool = False) -> bool:
        """Whether an event may pass now. ``force=True`` always passes (safety frames take this path)."""
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
# Unit conversion (the two-line formula of contract §4, only here)
# --------------------------------------------------------------------------


def round_half_away_from_zero(value: float) -> int:
    """Round half away from zero, and **negatives also away from zero** — matching C ``round()`` / ``lround()``.

    ★★ Do not use Python's built-in ``round()``: it does "banker's rounding" (round-half-to-even),
    so ``round(0.5) == 0`` while ``round(1.5) == 2``; C gives ``round(0.5) == 1``.
    Contract §4 explicitly requires "round to nearest, do not truncate with ``int()`` (negative speeds would be off by one LSB,
    which accumulates into a systematic drift over time)" — half-an-LSB banker's rounding is a systematic bias just the same,
    so this is implemented locally and the behavior is pinned to C semantics.

    >>> round_half_away_from_zero(0.5)
    1
    >>> round_half_away_from_zero(-0.5)
    -1
    >>> round_half_away_from_zero(0.4999)
    0
    """
    if not math.isfinite(value):
        raise ValueError("Rounding input must be a finite number, got %r" % (value,))
    if value >= 0.0:
        return int(math.floor(value + 0.5))
    return int(math.ceil(value - 0.5))


def m_s_to_mm_s(value: float) -> int:
    """ROS2 linear velocity m/s -> serial mm/s."""
    return round_half_away_from_zero(value * 1000.0)


def rad_s_to_mrad_s(value: float) -> int:
    """ROS2 angular velocity rad/s -> serial mrad/s."""
    return round_half_away_from_zero(value * 1000.0)


def mm_s_to_m_s(value: int) -> float:
    """Serial mm/s -> ROS2 linear velocity m/s."""
    return value / 1000.0


def mrad_s_to_rad_s(value: int) -> float:
    """Serial mrad/s -> ROS2 angular velocity rad/s."""
    return value / 1000.0


# --------------------------------------------------------------------------
# Downlink encoding
# --------------------------------------------------------------------------


def _require_int32(name: str, value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError("%s must be an int, got %r" % (name, type(value).__name__))
    if not INT32_MIN <= value <= INT32_MAX:
        raise ValueError("%s=%d is outside the int32 range" % (name, value))
    return value


def encode_velocity(vx_mm_s: int, vy_mm_s: int, omega_mrad_s: int) -> bytes:
    """Encode a ``V`` velocity frame. All three arguments must be **integers** in mm/s / mrad/s."""
    return b"V,%d,%d,%d\n" % (
        _require_int32("vx_mm_s", vx_mm_s),
        _require_int32("vy_mm_s", vy_mm_s),
        _require_int32("omega_mrad_s", omega_mrad_s),
    )


def encode_cmd_vel(linear_x: float, linear_y: float, angular_z: float) -> bytes:
    """Encode the three float components of a ROS2 ``Twist`` into a ``V`` frame (contract §4 conversion table).

    This is the **only** place where ``m/s -> mm/s`` conversion is allowed.
    """
    return encode_velocity(
        m_s_to_mm_s(linear_x), m_s_to_mm_s(linear_y), rad_s_to_mrad_s(angular_z)
    )


def encode_motor(p1: int, p2: int, p3: int, p4: int) -> bytes:
    """Encode an ``M`` debug frame (drive the four PWM channels directly, -1000 ~ 1000)."""
    values = []
    for index, raw in enumerate((p1, p2, p3, p4)):
        value = _require_int32("p%d" % (index + 1), raw)
        if not PWM_MIN <= value <= PWM_MAX:
            raise ValueError("p%d=%d is outside the PWM range [%d, %d]" % (index + 1, value, PWM_MIN, PWM_MAX))
        values.append(value)
    return b"M,%d,%d,%d,%d\n" % tuple(values)


def encode_pid(pid_id: int, kp: int, ki: int, kd: int) -> bytes:
    """Encode a ``P`` online PID-update frame."""
    channel = _require_int32("pid_id", pid_id)
    if not PID_ID_MIN <= channel <= PID_ID_MAX:
        raise ValueError("pid_id=%d is outside [%d, %d]" % (channel, PID_ID_MIN, PID_ID_MAX))
    return b"P,%d,%d,%d,%d\n" % (
        channel,
        _require_int32("kp", kp),
        _require_int32("ki", ki),
        _require_int32("kd", kd),
    )


# --------------------------------------------------------------------------
# Uplink frame data types
# --------------------------------------------------------------------------


def _clip_raw(text: str) -> str:
    return text if len(text) <= _RAW_KEEP else text[:_RAW_KEEP] + "..."


@dataclass(frozen=True)
class EncoderFrame:
    """``E`` frame: four wheel speeds (mm/s) + four accumulated distances (mm), order FL, FR, RL, RR."""

    kind: ClassVar[str] = "E"
    v_mm_s: Tuple[int, int, int, int] = (0, 0, 0, 0)
    d_mm: Tuple[int, int, int, int] = (0, 0, 0, 0)
    raw: str = ""

    def __len__(self) -> int:
        return WHEEL_COUNT


@dataclass(frozen=True)
class BatteryFrame:
    """``B`` frame: battery voltage (mV) + status flags. The flag bit definitions live in the STM32 firmware and are not frozen by this contract."""

    kind: ClassVar[str] = "B"
    millivolts: int = 0
    flags: int = 0
    raw: str = ""

    @property
    def volts(self) -> float:
        return self.millivolts / 1000.0


@dataclass(frozen=True)
class AckFrame:
    """``A`` frame: command acknowledgement / error text. The text **may contain commas**, so split only on the first comma."""

    kind: ClassVar[str] = "A"
    text: str = ""
    raw: str = ""


@dataclass(frozen=True)
class UnknownFrame:
    """Unrecognized / malformed line. **Not silently dropped** — passed up for counting and logging."""

    kind: ClassVar[str] = "?"
    prefix: str = ""
    reason: str = ""
    raw: str = ""


Frame = object  # upper layers only need isinstance checks on concrete types; this name exists only for typing readability.


@dataclass
class DecoderStats:
    """Parser health counters. Putting them in ``/diagnostics`` is far cheaper than guessing why no data arrived."""

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
# Single-line parsing
# --------------------------------------------------------------------------


def _parse_int32_token(token: str):
    """Parse a token as int32; returns ``None`` if invalid.

    ★ Only ``^[+-]?[0-9]+$`` is accepted: ``1.5`` / ``1e3`` / ``0x10`` / ``" 1"`` are all rejected.
    Trying ``int()`` directly would accept ``"  1  "`` and ``"1_0"`` (Python's underscore literal rule),
    so a regex check comes first, then the conversion.
    """
    if not _INT_TOKEN.match(token):
        return None
    try:
        value = int(token, 10)
    except ValueError:  # pragma: no cover - the regex already rejects it
        return None
    if not INT32_MIN <= value <= INT32_MAX:
        return None
    return value


def parse_line(text: str) -> Optional[Frame]:
    """Parse **one line** (without the trailing ``\\n``; leading/trailing whitespace already stripped by the caller).

    Returning ``None`` means the line was empty (normal, not an error).
    Returning ``UnknownFrame`` means malformed / unknown prefix — the caller should count it and log.
    """
    if not text:
        return None

    head, sep, rest = text.partition(",")

    if head == "E":
        if not sep:
            return UnknownFrame(prefix="E", reason="missing comma separator", raw=_clip_raw(text))
        tokens = rest.split(",")
        if len(tokens) != 2 * WHEEL_COUNT:
            return UnknownFrame(
                prefix="E",
                reason="expected %d fields, got %d" % (2 * WHEEL_COUNT, len(tokens)),
                raw=_clip_raw(text),
            )
        values: List[int] = []
        for index, token in enumerate(tokens):
            value = _parse_int32_token(token)
            if value is None:
                return UnknownFrame(
                    prefix="E",
                    reason="field %d is not an int32: %r" % (index + 1, _clip_raw(token)),
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
            return UnknownFrame(prefix="B", reason="missing comma separator", raw=_clip_raw(text))
        tokens = rest.split(",")
        if len(tokens) != 2:
            return UnknownFrame(
                prefix="B", reason="expected 2 fields, got %d" % len(tokens), raw=_clip_raw(text)
            )
        millivolts = _parse_int32_token(tokens[0])
        flags = _parse_int32_token(tokens[1])
        if millivolts is None or flags is None:
            return UnknownFrame(prefix="B", reason="field is not an int32", raw=_clip_raw(text))
        return BatteryFrame(millivolts=millivolts, flags=flags, raw=_clip_raw(text))

    if head == "A":
        if not sep:
            # ★ Keep this consistent with E / B: a bare "A" (no comma) is a malformed line and must not be treated as an "empty acknowledgement".
            #   otherwise a single line of noise "A" would parse as a valid acknowledgement and skew /diagnostics.
            return UnknownFrame(prefix="A", reason="missing comma separator", raw=_clip_raw(text))
        # The text may contain commas, so take rest verbatim without further splitting.
        return AckFrame(text=rest, raw=_clip_raw(text))

    return UnknownFrame(prefix=_clip_raw(head), reason="unknown prefix", raw=_clip_raw(text))


# --------------------------------------------------------------------------
# Streaming decoder: consumes a byte stream, emits frames
# --------------------------------------------------------------------------


class LineDecoder:
    """Split a serial byte stream into frames, tolerating partial packets / coalesced packets / overlong lines / invalid characters.

    Differences from ``pyserial``'s ``readline()`` (and why it is not used):

    * ``readline()`` keeps growing until memory is exhausted on invalid bytes or an overlong line;
      this implementation drops the line and resynchronizes once :data:`MAX_LINE_BYTES` is exceeded.
    * ``readline()`` needs extra stripping for ``\\r\\n``; this implementation splits on ``\\n``
      and then strips whitespace uniformly.
    * This implementation also provides health counters (:class:`DecoderStats`).
    """

    def __init__(self, max_line_bytes: int = MAX_LINE_BYTES) -> None:
        if max_line_bytes < 8:
            raise ValueError("max_line_bytes is too small: %r" % (max_line_bytes,))
        self.max_line_bytes = max_line_bytes
        self.stats = DecoderStats()
        self._buf = bytearray()
        self._skipping = False

    def reset(self) -> None:
        """Drop a partial packet (e.g. after the serial port was just reopened)."""
        self._buf.clear()
        self._skipping = False

    def feed(self, data: bytes) -> List[Frame]:
        """Feed a chunk of bytes and return the frames decoded **in this batch** (in arrival order)."""
        frames: List[Frame] = []
        for byte in data:
            if byte == 0x0A:  # \n
                if self._skipping:
                    # End of an overlong line: drop the whole line and resync after it.
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
                # ★ Key point: never grow without bound. Drop it and wait for the next \n to resync.
                self._skipping = True
                self._buf.clear()
        return frames

    def _decode_line(self, line: bytes):
        text = line.decode("ascii", errors="replace")
        # ★★ Order matters a lot: **strip the trailing CR / whitespace first, then check printability**.
        #    If reversed, the \r (0x0D < 0x20) of a CRLF line ending is treated as a "control character" and the whole line is dropped:
        #      * `B,12000,1\r` would fail to parse and fall into UnknownFrame;
        #      * a lone `\r\n` (blank line) would become a **fake error**, polluting the nonascii/dropped counters.
        #    If the STM32 side ever switches to `\r\n`, the bridge would silently drop **all** frames — a silent failure that reports nothing.
        #    Strip only " \t\r\n" and **do not** use text.strip(): the latter strips per Unicode whitespace and would also swallow
        #    \x0b \x0c \x1c-\x1f \x85 \xa0, which are exactly the symptoms of a wrong baud rate.
        stripped = text.strip(" \t\r\n")
        if not stripped:
            return None  # blank line / bare \r\n — not an error
        for char in stripped:
            code = ord(char)
            if code < 0x20 or code > 0x7E:
                # Non-printable ASCII: a typical symptom of a wrong baud rate; drop the whole line.
                self.stats.nonascii_lines += 1
                return UnknownFrame(
                    prefix="", reason="contains non-ASCII / control characters", raw=_clip_raw(stripped)
                )
        return parse_line(stripped)


def decode_all(data: bytes, max_line_bytes: int = MAX_LINE_BYTES) -> List[Frame]:
    """Decode a whole block of bytes into frames in one call (for unit tests and offline log analysis)."""
    return LineDecoder(max_line_bytes=max_line_bytes).feed(data)
