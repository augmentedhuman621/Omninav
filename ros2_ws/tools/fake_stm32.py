#!/usr/bin/env python3
"""Fake STM32 — run the whole serial-bridge chain with zero hardware.

What it does (following the frame format in ``11_interface_contract.md`` §3):

===  =========================================================================
 recv ``V,<vx>,<vy>,<w>``  -> record it and integrate into four-wheel odometry via C's ``mecanum_inverse()``
 recv ``M`` / ``P`` / ``S`` / ``reset`` -> reply ``A,OK ...``; ``S`` zeroes the velocity; ``reset`` clears odometry
 send ``E,<v1..v4>,<d1..d4>``  50 Hz; the four accumulated distances are **integer mm** and only increase (or only decrease), never reset
     ↑ each E frame advances odometry by a **fixed 20 ms step** (not wall-clock accumulation), matching the bridge's normalization
       period (the 0.020 s protocol constant in ``11_interface_contract.md`` §3) ⇒ Δd is strictly proportional
       to the commanded speed, and local scheduling jitter cannot inflate the twist
 send ``B,<mv>,<flags>``       10 Hz, simulating a slowly draining battery
===  =========================================================================

★★ Iron rule: this fake STM32 **must not implement mecanum formulas itself**. Wheel speeds always come from ``mecanum_inverse()``
(loaded from ``libomninav_core.so`` via ``stm32_bridge.kinematics``). So it uses the
**same math** as the bridge, so the end-to-end test really verifies "the two sides agree" rather than "the two sides are wrong the same way".

Two ways to use it:

1. Imported by ``run_e2e.py`` and run on the fake serial port created by ``os.openpty()``;
2. Standalone (★ very useful for manual bring-up on the Pi)::

       python3 tools/fake_stm32.py --pty
       # prints PTY_SLAVE=/dev/pts/3, then open another terminal:
       # ros2 run stm32_bridge stm32_bridge --ros-args -p serial_port:=/dev/pts/3
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from typing import List, Optional, Sequence, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(_HERE, "..", "src", "stm32_bridge")))

from stm32_bridge import kinematics  # noqa: E402
from stm32_bridge.protocol import (  # noqa: E402
    WHEEL_COUNT,
    AckFrame,
    BatteryFrame,
    EncoderFrame,
    LineDecoder,
    UnknownFrame,
)


def _ack(text: str) -> bytes:
    return b"A,OK " + text.encode("ascii", "replace") + b"\n"


class FakeStm32:
    """A faithful but entirely software STM32 peer."""

    def __init__(
        self,
        transport,
        core: Optional[kinematics.Core] = None,
        geom=None,
        encoder_hz: float = 50.0,
        battery_hz: float = 10.0,
        start_millivolts: int = 11800,
        battery_drain_mv: int = 1,
    ) -> None:
        self._transport = transport
        self._core = core if core is not None else kinematics.default_core()
        self._geom = geom if geom is not None else self._core.default_geom()
        if not self._core.geom_is_valid(self._geom):
            raise ValueError("the fake STM32 got invalid geometry; fix the .so or the parameters first")

        self._encoder_period = 1.0 / max(1.0, float(encoder_hz))
        self._battery_period = 1.0 / max(0.1, float(battery_hz))
        self._millivolts = int(start_millivolts)
        self._drain = int(battery_drain_mv)

        self._decoder = LineDecoder()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Motion state: current command + four-wheel accumulated distance
        self._twist = (0, 0, 0)
        # ★ Integrate with an integer "mm * ms" accumulator to avoid floating-point drift:
        #   accum += wheel_mm_s * step_ms  ->  d_mm = accum / 1000 (truncate toward zero)
        self._accum = [0] * WHEEL_COUNT
        # ★ Fixed step: every E frame advances one **nominal** encoder period (20 ms by default),
        #   matching the bridge's normalization period (0.020 s, see contract §3); the wall clock only decides "when to send
        #   the next frame" and no longer participates in the odometry integration. If the bridge changes the nominal period, this must change too.
        self._step_ms = max(1, int(round(self._encoder_period * 1000.0)))
        self._estop = False

        # Observations (for run_e2e / humans)
        self.v_frames: List[Tuple[int, int, int]] = []
        self.lines_received: List[str] = []
        self.acks_sent: List[str] = []
        self.e_frames_sent = 0
        self.b_frames_sent = 0
        self.last_error: Optional[str] = None

    # ------------------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._loop, name="fake-stm32", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def join(self, timeout: float = 1.0) -> None:
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    @property
    def current_twist(self) -> Tuple[int, int, int]:
        with self._lock:
            return self._twist

    @property
    def wheel_mm(self) -> List[int]:
        with self._lock:
            # ★ Use int(v / 1000) to truncate toward zero, **not** v // 1000 (floor division):
            #   C's integer division truncates toward zero; the two differ by 1 mm when reversing, and the end-to-end
            #   comparison on Δd would occasionally go falsely red. float division is exact for |v| < 2^53.
            return [int(value / 1000) for value in self._accum]

    # ------------------------------------------------------------------

    def _loop(self) -> None:
        next_encoder = time.monotonic()
        next_battery = next_encoder + self._battery_period
        while not self._stop.is_set():
            try:
                data = self._transport.read(256)
            except Exception as exc:  # the peer closed the fd
                self.last_error = "%s: %s" % (type(exc).__name__, exc)
                return
            if data:
                self._consume(data)

            now = time.monotonic()
            if now >= next_encoder:
                next_encoder = now + self._encoder_period
                self._emit_encoder()
            if now >= next_battery:
                next_battery = now + self._battery_period
                self._emit_battery()

            time.sleep(0.002)

    # ------------------------------------------------------------------

    def _consume(self, data: bytes) -> None:
        for frame in self._decoder.feed(data):
            if isinstance(frame, EncoderFrame):
                continue  # uplink frame; the fake STM32 should never receive it
            if isinstance(frame, (BatteryFrame,)):
                continue
            if isinstance(frame, AckFrame):
                continue
            if isinstance(frame, UnknownFrame):
                text = (frame.raw or "").strip()
                if not text:
                    continue
                self.lines_received.append(text)
                self._handle_command(text)

    def _handle_command(self, text: str) -> None:
        head = text.split(",", 1)[0].strip()
        if head == "V":
            parts = text.split(",")
            if len(parts) != 4:
                self._send(_ack("V arity"))
                return
            try:
                vx, vy, omega = (int(part) for part in parts[1:])
            except ValueError:
                self._send(_ack("V parse"))
                return
            with self._lock:
                self._twist = (vx, vy, omega)
                self.v_frames.append((vx, vy, omega))
            self._send(_ack("V"))
        elif head == "M":
            self._send(_ack("M"))
        elif head == "P":
            self._send(_ack("P"))
        elif head == "S":
            with self._lock:
                self._twist = (0, 0, 0)
                self._estop = True
            self._send(_ack("EMERGENCY_STOP"))
        elif head == "reset":
            with self._lock:
                self._twist = (0, 0, 0)
                self._estop = False
                self._accum = [0] * WHEEL_COUNT
            self._send(_ack("reset"))
        else:
            self._send(_ack("unknown"))

    # ------------------------------------------------------------------

    def _emit_encoder(self) -> None:
        with self._lock:
            wheels = self._core.inverse(
                self._geom,
                kinematics.MecanumTwist(
                    vx_mm_s=self._twist[0],
                    vy_mm_s=self._twist[1],
                    omega_mrad_s=self._twist[2],
                ),
            )
            speeds = [int(wheels.wheel_mm_s[i]) for i in range(WHEEL_COUNT)]
            # ★ Advance odometry by the fixed nominal step first, then send the accumulated value: this way the
            #   Δd between adjacent E frames (integer mm) is exactly speeds × step_ms, and normalizing by 0.020 s
            #   in the bridge restores the commanded speed exactly, independent of local read/write jitter.
            if not self._estop:
                for index in range(WHEEL_COUNT):
                    self._accum[index] += speeds[index] * self._step_ms
            # ★ Same as wheel_mm: truncate toward zero, do not use // (see the note above).
            distances = [int(self._accum[i] / 1000) for i in range(WHEEL_COUNT)]
        payload = "E," + ",".join(str(v) for v in speeds + distances) + "\n"
        if self._send(payload.encode("ascii")):
            self.e_frames_sent += 1

    def _emit_battery(self) -> None:
        with self._lock:
            self._millivolts = max(9000, self._millivolts - self._drain)
            millivolts = self._millivolts
        if self._send(("B,%d,0\n" % millivolts).encode("ascii")):
            self.b_frames_sent += 1

    def _send(self, payload: bytes) -> bool:
        try:
            self._transport.write(payload)
        except Exception as exc:
            self.last_error = "%s: %s" % (type(exc).__name__, exc)
            return False
        return True


# --------------------------------------------------------------------------
# Transport: raw fd (pty) or pyserial
# --------------------------------------------------------------------------


class FdTransport:
    """Raw file descriptor (the master end returned by ``os.openpty()``). Non-blocking reads."""

    def __init__(self, fd: int) -> None:
        self._fd = fd
        os.set_blocking(fd, False)

    def read(self, size: int) -> bytes:
        try:
            return os.read(self._fd, size)
        except BlockingIOError:
            return b""
        except OSError:
            # After the peer (pyserial) closes the slave, reads get EIO — treat that as "no data"
            # and let the caller finish by its own timeout.
            return b""

    def write(self, payload: bytes) -> int:
        return os.write(self._fd, payload)

    def close(self) -> None:
        try:
            os.close(self._fd)
        except OSError:
            pass


class SerialTransport:
    """Talk to a real serial port (during bring-up, connect the fake STM32 to the other end of the USB-TTL)."""

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        import serial

        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )

    def read(self, size: int) -> bytes:
        return self._serial.read(size)

    def write(self, payload: bytes) -> int:
        return self._serial.write(payload)

    def close(self) -> None:
        self._serial.close()


# --------------------------------------------------------------------------
# Standalone
# --------------------------------------------------------------------------


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="fake STM32 (zero-hardware bring-up)")
    parser.add_argument("--pty", action="store_true", help="create a pseudo-terminal and print the slave path (default)")
    parser.add_argument("--port", default=None, help="use a real serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=0.0, help="run for this many seconds then exit; 0 = run forever")
    parser.add_argument("--hz", type=float, default=50.0, help="E-frame rate")
    args = parser.parse_args(argv)

    transport = None
    if args.port:
        transport = SerialTransport(args.port, args.baudrate)
        print("acting as the STM32 on %s @ %d" % (args.port, args.baudrate))
    else:
        if not sys.platform.startswith("linux"):
            print("pseudo-terminals exist only on Linux; on this platform point --port at a real serial port", file=sys.stderr)
            return 2
        import tty

        master_fd, slave_fd = os.openpty()
        tty.setraw(slave_fd)
        slave_name = os.ttyname(slave_fd)
        transport = FdTransport(master_fd)
        print("PTY_SLAVE=%s" % slave_name)
        print("Run this in another terminal:")
        print("  ros2 run stm32_bridge stm32_bridge --ros-args -p serial_port:=%s" % slave_name)
        sys.stdout.flush()

    fake = FakeStm32(transport, encoder_hz=args.hz)
    fake.start()
    deadline = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        while deadline is None or time.monotonic() < deadline:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        fake.stop()
        fake.join()
        print(
            "Shutdown: %d E frames, %d B frames, %d V commands received"
            % (fake.e_frames_sent, fake.b_frames_sent, len(fake.v_frames))
        )
        if fake.last_error:
            print("last transport error: %s" % fake.last_error)
    return 0


if __name__ == "__main__":
    sys.exit(main())
