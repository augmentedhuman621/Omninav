#!/usr/bin/env python3
"""假 STM32 —— 零硬件就能把串口桥的整条链路跑起来。

它做的事（照 ``11_接口契约.md`` §3 的帧格式）：

===  =========================================================================
 收   ``V,<vx>,<vy>,<w>``  -> 记下来，并按 C 的 ``mecanum_inverse()`` 积分成四轮里程
 收   ``M`` / ``P`` / ``S`` / ``reset`` -> 回 ``A,OK ...``；``S`` 把速度清零；``reset`` 清里程
 发   ``E,<v1..v4>,<d1..d4>``  50 Hz，四个累计里程是**整数 mm**，d 只增（或只减）不重置
     ↑ 每发一帧 E 就推进**固定 20 ms 步长**的里程（不按墙钟累加），与桥的归一化
       周期（``11_接口契约.md`` §3 的协议常量 0.020 s）对齐 ⇒ Δd 与指令速度严格
       成比例，本机调度抖动不会把 twist 放大
 发   ``B,<mv>,<flags>``       10 Hz，模拟电池缓慢掉压
===  =========================================================================

★★ 铁律：这个假 STM32 **不许自己写麦轮公式**。四轮速度一律调 ``mecanum_inverse()``
（经 ``stm32_bridge.kinematics`` 加载 ``libomninav_core.so``）。所以它和桥用的是
**同一份数学**，端到端测试才真的在验证"两边对得上"，而不是"两边错得一样"。

两种用法：

1. 被 ``run_e2e.py`` import，在 ``os.openpty()`` 造出来的假串口上跑；
2. 单独跑（★ 在树莓派上手工联调时很有用）::

       python3 tools/fake_stm32.py --pty
       # 打印 PTY_SLAVE=/dev/pts/3，然后另开一个终端：
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
    """一个够真、但完全是软件的 STM32 对端。"""

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
            raise ValueError("假 STM32 拿到的几何参数不合法，先修 .so 或参数")

        self._encoder_period = 1.0 / max(1.0, float(encoder_hz))
        self._battery_period = 1.0 / max(0.1, float(battery_hz))
        self._millivolts = int(start_millivolts)
        self._drain = int(battery_drain_mv)

        self._decoder = LineDecoder()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # 运动状态：当前指令 + 四轮累计里程
        self._twist = (0, 0, 0)
        # ★ 用「mm * ms」的整数累加器积分，避免浮点漂移：
        #   accum += wheel_mm_s * step_ms  ->  d_mm = accum / 1000（向零截断）
        self._accum = [0] * WHEEL_COUNT
        # ★ 固定步进：每发一帧 E 推进一个**名义**编码器周期（默认 20 ms），
        #   与桥的归一化周期（0.020 s，见契约 §3）一致；墙钟只用来决定"何时发
        #   下一帧"，不再参与里程积分。桥若改了名义周期，这里必须同步改。
        self._step_ms = max(1, int(round(self._encoder_period * 1000.0)))
        self._estop = False

        # 观测（给 run_e2e / 人工看）
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
            # ★ 用 int(v / 1000) 向零截断，**不要**用 v // 1000（向下取整）：
            #   C 侧的整数除法是向零截断，后退时两者差 1 mm，端到端对 Δd 严格
            #   比对时会偶发假红。|v| < 2^53 之前 float 除法是精确的。
            return [int(value / 1000) for value in self._accum]

    # ------------------------------------------------------------------

    def _loop(self) -> None:
        next_encoder = time.monotonic()
        next_battery = next_encoder + self._battery_period
        while not self._stop.is_set():
            try:
                data = self._transport.read(256)
            except Exception as exc:  # 对端关掉了 fd
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
                continue  # 上行帧，假 STM32 不该收到
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
            # ★ 先按固定名义步长推进里程，再取累计值发出去：这样相邻两帧 E 的
            #   差值 Δd（整数 mm）恰好等于 speeds × step_ms，桥按 0.020 s 归一化
            #   后 twist 精确还原成指令速度，不受本机读写抖动影响。
            if not self._estop:
                for index in range(WHEEL_COUNT):
                    self._accum[index] += speeds[index] * self._step_ms
            # ★ 同 wheel_mm：向零截断，别用 //（见上面那段注释）。
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
# 传输层：裸 fd（pty）或 pyserial
# --------------------------------------------------------------------------


class FdTransport:
    """裸文件描述符（``os.openpty()`` 返回的 master 端）。非阻塞读。"""

    def __init__(self, fd: int) -> None:
        self._fd = fd
        os.set_blocking(fd, False)

    def read(self, size: int) -> bytes:
        try:
            return os.read(self._fd, size)
        except BlockingIOError:
            return b""
        except OSError:
            # 对端（pyserial）关掉 slave 后会得到 EIO —— 当成"没有数据"，
            # 让上层自己按超时结束。
            return b""

    def write(self, payload: bytes) -> int:
        return os.write(self._fd, payload)

    def close(self) -> None:
        try:
            os.close(self._fd)
        except OSError:
            pass


class SerialTransport:
    """接真实串口（联调时把假 STM32 接在 USB-TTL 的另一头）。"""

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
# 单独运行
# --------------------------------------------------------------------------


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="假 STM32（零硬件联调）")
    parser.add_argument("--pty", action="store_true", help="自己造一个伪终端并打印从端路径（默认）")
    parser.add_argument("--port", default=None, help="用真实串口，例如 /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=0.0, help="跑多少秒后退出；0 = 一直跑")
    parser.add_argument("--hz", type=float, default=50.0, help="E 帧频率")
    args = parser.parse_args(argv)

    transport = None
    if args.port:
        transport = SerialTransport(args.port, args.baudrate)
        print("在 %s @ %d 上扮演 STM32" % (args.port, args.baudrate))
    else:
        if not sys.platform.startswith("linux"):
            print("伪终端只在 Linux 上有；这个平台请用 --port 指向真实串口", file=sys.stderr)
            return 2
        import tty

        master_fd, slave_fd = os.openpty()
        tty.setraw(slave_fd)
        slave_name = os.ttyname(slave_fd)
        transport = FdTransport(master_fd)
        print("PTY_SLAVE=%s" % slave_name)
        print("另开一个终端跑：")
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
            "收尾：E 帧 %d 条、B 帧 %d 条、收到 V 指令 %d 条"
            % (fake.e_frames_sent, fake.b_frames_sent, len(fake.v_frames))
        )
        if fake.last_error:
            print("传输层最后一次错误：%s" % fake.last_error)
    return 0


if __name__ == "__main__":
    sys.exit(main())
