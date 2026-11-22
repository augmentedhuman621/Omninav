"""stm32_bridge.bridge_node —— 串口桥的 ``rclpy`` 节点（本包**唯一** import ROS2 的地方）。

行为照 ``11_接口契约.md`` §3 / §4 与 T2 任务卡的 7 条行为表：

===  ==========================================================================
 1   订阅 ``/cmd_vel``（``geometry_msgs/Twist``）-> 发 ``V,<vx>,<vy>,<w>\\n``，四舍五入取整
 2   读 ``E,...``（50 Hz）-> 取 Δd -> 调 ``odom_update()`` -> 发 ``/odom`` + TF ``odom->base_link``
 3   读 ``B,...``（10 Hz）-> 发 ``/battery_state``
 4   读 ``A,<text>`` -> 发 ``/diagnostics``
 5   ``/cmd_vel`` 断流 > 300 ms -> 主动补发 ``V,0,0,0``
 6   **只**发 ``odom->base_link``；绝不发 ``map->odom``，绝不发 ``base_link->laser``
 7   串口设备名默认 ``/dev/ttySTM32``（ROS2 参数可覆盖）；**严禁** ``/dev/ttyUSB0``
===  ==========================================================================

★ 本节点**不含任何运动学 / 里程计数学**：那只有一份，在 ``firmware_core`` 的 C 代码里，
  通过 :mod:`stm32_bridge.kinematics` 的 ``ctypes`` 绑定调用。找不到 ``.so`` 就启动失败 ——
  不降级、不自己算。
"""

from __future__ import annotations

import math
import queue
import sys
import threading
import time
from typing import List, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster

from . import kinematics
from .kinematics import MecanumGeom, MecanumTwist, OdomPose, OdomTotal
from .protocol import (
    ENCODER_NOMINAL_HZ,
    RESET_BURST_COUNT,
    RESET_BURST_PERIOD_S,
    RESET_FRAME,
    ZERO_VELOCITY_FRAME,
    AckFrame,
    BatteryFrame,
    EncoderFrame,
    LineDecoder,
    RateLimiter,
    UnknownFrame,
    encode_cmd_vel,
)

try:
    import serial
except ImportError as _exc:  # pragma: no cover - 只在缺依赖时触发
    raise ImportError(
        "stm32_bridge 需要 pyserial。装法：\n"
        "  树莓派：sudo apt install python3-serial\n"
        "  （树莓派已能经 Windows 的 USB 网卡共享（192.168.137.x）上外网，apt 直面装即可）"
    ) from _exc

#: 契约 §2：设备节点名只许用这三个，由 ``pi_setup/`` 的 udev 规则创建。
DEFAULT_SERIAL_PORT = "/dev/ttySTM32"

#: 契约 §2：**严禁**在代码里写 ttyUSBx —— 两个 CH340 同型号、by-id 撞名，
#: 插拔顺序一变雷达和串口桥就会互换。这里做一次硬拦截。
FORBIDDEN_PORT_SUBSTRINGS = ("ttyUSB", "ttyACM")

#: 契约 §4：``/cmd_vel`` 断流超过这个秒数就补发停车帧。
DEFAULT_CMD_VEL_TIMEOUT_S = 0.3

#: 契约 §3：115200 8N1、无流控。
DEFAULT_BAUDRATE = 115200

#: 清空接收队列时的单次上限（防止一次 tick 里处理上万帧把节点卡住）。
MAX_FRAMES_PER_TICK = 200

#: ★★ 归一化 ``/odom`` 的 twist 时，**不使用**每帧实测间隔，恒定用协议名义周期
#: ``NOMINAL_ENCODER_PERIOD_S``（= 1/50 s = 0.020 s，契约 §5.1 修订）。
#:
#: 为什么（树莓派实测踩到的真 bug）：``odom_update()`` 的 ``twist_out`` 是
#: **本帧位移增量**（mm / mrad）—— 它的签名里根本没有 dt，原理上产不出「每秒」量纲。
#: 而契约 §4 要求 ``/odom`` 的 twist 是 m/s / rad/s，所以桥必须自己除一个时间。
#: 但那个时间**不能是每帧实测间隔**：串口 / pty 按块交付，一次 ``read()`` 常返回好几条
#: E 帧，逐帧去测「到达间隔」会退化到 ~0，于是除法把 twist 放大 20 倍 ——
#: 实测出现过 ``twist = 4.0 m/s`` 的尖峰（``Δd = 4 mm``、dt 被钳到 1 ms）。
#: 位置积分当时一直是对的（x 单调、0.294 m ≈ 0.196 m/s），只有这个派生量被除歪。
#:
#: 下面的 MIN/MAX 依然有用，但它们现在**只服务于诊断**：实测间隔用来判断
#: 发送端是否停摆 / 掉帧，以及把离群情况记日志。**绝不进速度计算。**
MIN_ENCODER_DT_S = 1e-3
MAX_ENCODER_DT_S = 1.0

#: 实测帧间隔偏离名义周期多少倍就记一条警告（只用于诊断）。
ENCODER_DT_WARN_RATIO = 3.0

#: 下行 ``V`` 帧的默认限速（Hz）。50 Hz 上报 / 20 Hz 下行是这类底盘的常规配比：
#: 115200 的链路每帧 ~15 B，不限速的话串口带宽会被吃掉约 30%，
#: 而且 STM32 侧收包状态机被无谓地压满。
DEFAULT_CMD_VEL_RATE_HZ = 20.0

#: ``/diagnostics`` 的默认限速（Hz）。它是给 rqt_robot_monitor / Foxglove 看的
#: **低频**话题；树莓派只有 2 GB，不能让每条 A 帧都发一次。
DEFAULT_DIAGNOSTICS_RATE_HZ = 1.0


def _now_msg(node: Node):
    return node.get_clock().now().to_msg()


class Stm32Bridge(Node):
    """STM32 <-> ROS2 的双向桥。"""

    def __init__(self, parameter_overrides=None) -> None:
        # parameter_overrides 是给 tools/run_e2e.py 用的：端到端脚本要在同一进程里
        # 把 serial_port 指到 os.openpty() 造出来的假串口上。用参数覆盖比依赖
        # `--ros-args -p` 全局参数更可靠，而且命令行入口 `main()` 仍是不带参调用。
        super().__init__("stm32_bridge", parameter_overrides=parameter_overrides)

        # ---------------- 参数 ----------------
        # ★ 几何参数的默认值是**哨兵**（-1 / 0），含义是"用 C 侧的 mecanum_default_geom()"。
        #   这样 Python 里不会出现第二份 150 / 10505 之类的数字（契约 §6 硬要求）。
        self.declare_parameter("serial_port", DEFAULT_SERIAL_PORT)
        self.declare_parameter("baudrate", DEFAULT_BAUDRATE)
        self.declare_parameter("cmd_vel_timeout", DEFAULT_CMD_VEL_TIMEOUT_S)
        self.declare_parameter("cmd_vel_rate_hz", DEFAULT_CMD_VEL_RATE_HZ)
        self.declare_parameter("diagnostics_rate_hz", DEFAULT_DIAGNOSTICS_RATE_HZ)
        # ★ E 帧名义频率：只用来把 twist_out（本帧位移）换算成 m/s。
        #   契约 §5.1 修订把它冻结为 50 Hz（06 §7）。**不要**改成"实测值"。
        self.declare_parameter("encoder_nominal_hz", ENCODER_NOMINAL_HZ)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("send_reset_on_start", True)
        self.declare_parameter("poll_period", 0.01)
        self.declare_parameter("link_watchdog_s", 2.0)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("battery_topic", "battery_state")
        self.declare_parameter("diagnostics_topic", "diagnostics")
        self.declare_parameter("half_wheelbase_mm", -1)
        self.declare_parameter("half_track_mm", -1)
        self.declare_parameter("vy_polarity", 0)
        self.declare_parameter("omega_polarity", 0)

        def p(name: str):
            return self.get_parameter(name).value

        self._port: str = str(p("serial_port"))
        self._baudrate: int = int(p("baudrate"))
        self._cmd_vel_timeout: float = float(p("cmd_vel_timeout"))
        self._publish_tf: bool = bool(p("publish_tf"))
        self._poll_period: float = max(0.001, float(p("poll_period")))
        self._link_watchdog_s: float = float(p("link_watchdog_s"))
        self._odom_frame_id: str = str(p("odom_frame_id"))
        self._base_frame_id: str = str(p("base_frame_id"))

        # ★ twist 归一化用的名义周期（契约 §5.1 修订）：恒为 1/名义频率，不用实测 dt。
        nominal_hz = float(p("encoder_nominal_hz"))
        if not math.isfinite(nominal_hz) or nominal_hz <= 0.0:
            raise ValueError("encoder_nominal_hz 必须是正的有限数，收到 %r" % (nominal_hz,))
        self._nominal_period_s: float = 1.0 / nominal_hz

        # 下行 V 限速（Hz -> 最小间隔）。安全帧（断流补发的 V,0,0,0 / S / reset）绕过它。
        cmd_vel_hz = float(p("cmd_vel_rate_hz"))
        if not math.isfinite(cmd_vel_hz) or cmd_vel_hz <= 0.0:
            raise ValueError("cmd_vel_rate_hz 必须是正的有限数，收到 %r" % (cmd_vel_hz,))
        self._cmd_vel_limiter = RateLimiter(1.0 / cmd_vel_hz)

        diag_hz = float(p("diagnostics_rate_hz"))
        if not math.isfinite(diag_hz) or diag_hz < 0.0:
            raise ValueError("diagnostics_rate_hz 必须是非负有限数，收到 %r" % (diag_hz,))
        # diag_hz == 0 当作"不限速"。
        self._diag_limiter = RateLimiter(0.0 if diag_hz <= 0.0 else 1.0 / diag_hz)
        #: 每个诊断项的**最新**状态，按 name 去重 —— /diagnostics 发的是快照而不是流水。
        self._diag_state: "dict" = {}

        self._reject_forbidden_port()

        # ---------------- C 核心（数学的唯一来源） ----------------
        self._core = kinematics.default_core()
        self.get_logger().info("已加载 C 核心：%s" % self._core.describe())
        self._geom: MecanumGeom = self._build_geom(p)

        # ---------------- 状态 ----------------
        self._decoder = LineDecoder()
        self._rx_queue: "queue.Queue" = queue.Queue()
        self._stop = threading.Event()
        self._reader: Optional[threading.Thread] = None
        self._reader_error: Optional[str] = None
        self._write_lock = threading.Lock()

        self._pose: OdomPose
        self._total: OdomTotal
        self._pose, self._total = self._core.odom_reset()
        self._d_prev = [0, 0, 0, 0]
        self._have_encoder = False
        # 上一帧 E 的到达时刻与实测间隔 —— 见 MIN_ENCODER_DT_S 上面那段注释。
        self._last_encoder_t = time.monotonic()
        self._last_encoder_dt = 0.0
        # 收线程用：上一段 chunk 的到达时刻 + 推出来的名义帧间隔
        self._last_chunk_t = time.monotonic()
        self._nominal_dt = 0.02  # 上电默认按 50 Hz 猜，第一帧之后就会被实测值替换

        self._have_cmd = False
        self._last_vel_frame: Optional[bytes] = None
        self._last_cmd_vel_t = 0.0
        #: 限速挡下来的最新一条 V 帧，等下一个 tick 或下一次放行时补发。
        #: ★ 存的是**最新值**而不是排队：速度指令天然是"覆盖式"的，
        #:   排队会累积延迟，覆盖只会丢掉中间过时的值。
        self._pending_vel_frame: Optional[bytes] = None
        #: 实测帧间隔偏离名义周期太多次数（只诊断，不进速度计算）。
        self._dt_outliers = 0

        self._last_frame_t = time.monotonic()
        self._link_ok = True
        self._reset_sent = 0
        self._reset_timer = None

        # ---------------- 发布者 / 订阅者 ----------------
        # ★ /odom 用 RELIABLE：reliable 发布者既能喂 reliable 订阅者，也能喂
        #   best-effort 订阅者（AMCL 在 Jazzy 里用 SensorDataQoS）。
        #   反过来 best-effort 发布者会喂不饱 reliable 订阅者 —— 所以别改成 best-effort。
        odom_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._odom_pub = self.create_publisher(Odometry, str(p("odom_topic")), odom_qos)
        self._battery_pub = self.create_publisher(BatteryState, str(p("battery_topic")), 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, str(p("diagnostics_topic")), 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        self.create_subscription(Twist, str(p("cmd_vel_topic")), self._on_cmd_vel, 10)

        # ---------------- 串口 ----------------
        self._serial = self._open_serial()

        self._reader = threading.Thread(target=self._reader_loop, name="stm32-rx", daemon=True)
        self._reader.start()

        self._timer = self.create_timer(self._poll_period, self._on_timer)

        if self._reset_timer_enabled():
            # 契约 §3：上电时 reset 连发 3 次、间隔 0.3 s。
            self._reset_timer = self.create_timer(RESET_BURST_PERIOD_S, self._on_reset_timer)

        self.get_logger().info(
            "串口桥就绪：%s @ %d 8N1，L=%d mm，cmd_vel 断流阈值 %.3f s，"
            "twist 归一化周期 %.1f ms（名义 %.0f Hz），下行 V 限速 %.0f Hz，/diagnostics 限速 %.0f Hz"
            % (
                self._port,
                self._baudrate,
                self._core.l_mm(self._geom),
                self._cmd_vel_timeout,
                self._nominal_period_s * 1000.0,
                nominal_hz,
                cmd_vel_hz,
                diag_hz,
            )
        )

    # ------------------------------------------------------------------
    # 启动期校验 / 构造
    # ------------------------------------------------------------------

    def _reject_forbidden_port(self) -> None:
        for bad in FORBIDDEN_PORT_SUBSTRINGS:
            if bad in self._port:
                raise ValueError(
                    "拒绝使用串口 %s：契约 §2 明确禁止 ttyUSBx / ttyACMx。\n"
                    "原因：雷达与视觉模块都是 CH340（1a86:7522），/dev/serial/by-id 只有一条，"
                    "插拔顺序一变设备名就会互换 —— 桥会收到雷达的二进制乱码。\n"
                    "正确做法：让 pi_setup/ 的 udev 规则把 STM32 的 USB-TTL 固定成 %s，"
                    "再用 serial_port 参数指向它。" % (self._port, DEFAULT_SERIAL_PORT)
                )

    def _build_geom(self, p) -> MecanumGeom:
        """几何只有两个来源：C 侧 ``mecanum_default_geom()``，或 ROS2 参数覆盖（契约 §6）。"""
        base = self._core.default_geom()
        overrides = {
            "half_wheelbase_mm": p("half_wheelbase_mm"),
            "half_track_mm": p("half_track_mm"),
            "vy_polarity": p("vy_polarity"),
            "omega_polarity": p("omega_polarity"),
        }
        sentinel = {
            "half_wheelbase_mm": -1,
            "half_track_mm": -1,
            "vy_polarity": 0,
            "omega_polarity": 0,
        }
        geom = MecanumGeom(
            half_wheelbase_mm=(
                base.half_wheelbase_mm
                if overrides["half_wheelbase_mm"] == sentinel["half_wheelbase_mm"]
                else int(overrides["half_wheelbase_mm"])
            ),
            half_track_mm=(
                base.half_track_mm
                if overrides["half_track_mm"] == sentinel["half_track_mm"]
                else int(overrides["half_track_mm"])
            ),
            vy_polarity=(
                base.vy_polarity
                if overrides["vy_polarity"] == sentinel["vy_polarity"]
                else int(overrides["vy_polarity"])
            ),
            omega_polarity=(
                base.omega_polarity
                if overrides["omega_polarity"] == sentinel["omega_polarity"]
                else int(overrides["omega_polarity"])
            ),
        )
        if not self._core.geom_is_valid(geom):
            raise ValueError(
                "几何参数不合法：half_wheelbase_mm=%d half_track_mm=%d "
                "vy_polarity=%d omega_polarity=%d"
                % (
                    geom.half_wheelbase_mm,
                    geom.half_track_mm,
                    geom.vy_polarity,
                    geom.omega_polarity,
                )
            )
        if self._core.l_mm(geom) < 1:
            raise ValueError("L = half_wheelbase + half_track < 1 mm，正解会除零")
        return geom

    def _reset_timer_enabled(self) -> bool:
        return bool(self.get_parameter("send_reset_on_start").value)

    def _open_serial(self):
        try:
            # ★ rtscts/dsrdtr/xonxoff 全部显式关掉：契约要求「无流控」，
            #   而且 DTR/RTS 一旦被拉，接了 EN/IO0 的 CH340 会把对端芯片拽进下载模式
            #   （本项目的视觉模块通信板就是这么被刷出 `key_long_down!!` 的）。
            return serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
                write_timeout=0.2,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
        except Exception as exc:  # serial.SerialException 及其它
            raise RuntimeError(
                "打不开串口 %s @ %d：%s\n"
                "排查：① udev 规则建了 %s 没有；② yjc 在不在 dialout 组"
                "（`id` 里要有 dialout，加完要重新登录）；③ 设备真的插着没有。"
                % (self._port, self._baudrate, exc, DEFAULT_SERIAL_PORT)
            ) from exc

    # ------------------------------------------------------------------
    # 串口收线程
    # ------------------------------------------------------------------

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._serial.read(64)
            except Exception as exc:  # 拔线 / 权限 / 设备消失
                self._reader_error = "%s: %s" % (type(exc).__name__, exc)
                self.get_logger().error("串口读失败，收线程退出：%s" % self._reader_error)
                return
            if not data:
                continue
            arrival = time.monotonic()
            frames = self._decoder.feed(data)
            if not frames:
                continue
            # ★ 时间戳打在**收线程**里，不是打在主 tick 里。
            #   主 tick 每 poll_period（默认 10 ms）才排空一次队列，而 E 帧是 20 ms 一帧；
            #   用 tick 时刻会让 dt 抖动 ±10 ms（速度估计抖 ±50%），
            #   更糟的是万一两帧落进同一个 tick，dt≈0 会让速度估计炸成几十倍。
            #   同一 chunk 里的第 2..n 帧没有各自的到达时刻，就按上一段实测到的
            #   平均间隔往后推 —— 宁可略保守，也不要出尖峰。
            # ★★ 2026-10 修 bug：``observed`` 是**整段 chunk** 的到达间隔，而一次
            #    read(64) 常常一次带回 2~4 条 E 帧。原先直接拿它当"单帧间隔"，
            #    会把 dt 放大 len(frames) 倍；链路卡一下之后 _nominal_dt 还会被钳到
            #    1.0 s，于是同一个 chunk 里后续帧的时间戳能漂到未来几十秒。
            #    单帧间隔 = observed / 本段帧数。
            if arrival > self._last_chunk_t:
                observed = arrival - self._last_chunk_t
                self._last_chunk_t = arrival
                self._nominal_dt = min(
                    max(observed / float(len(frames)), MIN_ENCODER_DT_S),
                    MAX_ENCODER_DT_S,
                )
            for index, frame in enumerate(frames):
                self._rx_queue.put((arrival + index * self._nominal_dt, frame))

    def _write(self, payload: bytes) -> bool:
        try:
            with self._write_lock:
                self._serial.write(payload)
        except Exception as exc:
            # ★★ 2026-10 修 bug：这条日志**必须节流**。链路一断，断流看门狗会按主
            #    tick（默认 10 ms = 100 Hz）反复重试写 V,0,0,0，每次失败都打一条
            #    error，几秒钟就能把 2 GB 树莓派的日志和 CPU 打满。
            self.get_logger().error("串口写失败：%s" % exc, throttle_duration_sec=1.0)
            return False
        return True

    # ------------------------------------------------------------------
    # 下行：/cmd_vel
    # ------------------------------------------------------------------

    def _on_cmd_vel(self, msg: Twist) -> None:
        try:
            frame = encode_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)
        except (TypeError, ValueError) as exc:
            # NaN / inf 的 cmd_vel：绝不能编码成垃圾发给 STM32。
            self.get_logger().warn("丢弃非法 /cmd_vel：%s" % exc, throttle_duration_sec=1.0)
            return
        # ★ 先记下"最近一次收到指令的时刻"，再考虑发不发 —— 断流看门狗看的是
        #   有没有**新指令进来**，而不是我们发得快不快。
        self._last_cmd_vel_t = time.monotonic()
        self._have_cmd = True
        self._pending_vel_frame = frame
        self._flush_pending_velocity()

    def _flush_pending_velocity(self) -> None:
        """按 20 Hz 的下行限速把最新一条 V 帧发出去。

        ★ 限速只作用于**常规速度指令**。安全帧（断流补发的 ``V,0,0,0``、
        ``S``、``reset``）一律绕过这里直接 ``_write()``，见
        :meth:`_check_cmd_vel_watchdog` 与 :meth:`_on_reset_timer`。
        """
        frame = self._pending_vel_frame
        if frame is None:
            return
        if not self._cmd_vel_limiter.should_emit(time.monotonic()):
            return  # 下一个 tick 再试
        self._pending_vel_frame = None
        if self._write(frame):
            self._last_vel_frame = frame
        else:
            # ★ 2026-10 修 bug：限速器已经为这一帧记账了，写失败就直接丢等于让
            #   一次瞬时错误白吃一个下行周期。放回去，下个 tick 或下一次放行重发。
            self._pending_vel_frame = frame

    # ------------------------------------------------------------------
    # 主 tick：排空接收队列 + 看门狗
    # ------------------------------------------------------------------

    def _on_timer(self) -> None:
        for _ in range(MAX_FRAMES_PER_TICK):
            try:
                arrival, frame = self._rx_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_frame(frame, arrival)

        # 限速挡下来的那条 V 帧在这里补发（50 Hz 的上报 tick 足以喂饱 20 Hz 的下行）。
        self._flush_pending_velocity()
        self._check_cmd_vel_watchdog()
        self._check_link_watchdog()

    def _handle_frame(self, frame, arrival: float) -> None:
        self._last_frame_t = time.monotonic()
        if not self._link_ok:
            self._link_ok = True
            self._publish_link_status(DiagnosticStatus.OK, "串口链路恢复")

        if isinstance(frame, EncoderFrame):
            self._on_encoder_frame(frame, arrival)
        elif isinstance(frame, BatteryFrame):
            self._on_battery_frame(frame)
        elif isinstance(frame, AckFrame):
            self._on_ack_frame(frame)
        elif isinstance(frame, UnknownFrame):
            self.get_logger().debug("丢弃畸形行：%s" % frame)

    # -- E 帧：唯一会推进里程计的地方 -----------------------------------

    def _on_encoder_frame(self, frame: EncoderFrame, arrival: float) -> None:
        if not self._have_encoder:
            # 第一帧只用来定基线：没有 d_prev，算 Δd 会凭空得到一段假位移。
            self._d_prev = list(frame.d_mm)
            self._have_encoder = True
            self._last_encoder_t = arrival
            return

        dt = arrival - self._last_encoder_t
        if dt <= 0.0:
            # 时钟没走 / 同一时刻两帧 —— 用上一帧的间隔，绝不除零。
            dt = self._last_encoder_dt
        dt = min(max(dt, MIN_ENCODER_DT_S), MAX_ENCODER_DT_S)

        # ★ 这个实测 dt **只用于诊断**：偏离名义周期太多说明发送端停摆或掉帧。
        #   它绝不参与速度计算 —— 见 MIN_ENCODER_DT_S 上面那段注释。
        if dt > self._nominal_period_s * ENCODER_DT_WARN_RATIO:
            self._dt_outliers += 1
            self.get_logger().warn(
                "E 帧实测间隔 %.1f ms 远大于名义周期 %.1f ms（第 %d 次）"
                % (dt * 1000.0, self._nominal_period_s * 1000.0, self._dt_outliers),
                throttle_duration_sec=5.0,
            )

        d_now = list(frame.d_mm)
        twist = self._core.odom_update(
            self._pose, self._total, self._geom, d_now, self._d_prev
        )
        if twist is None:
            # C 返回 0 = 几何无效 / 指针为空（器质性错误）。
            # ★ 关键：**不推进 d_prev** —— 否则这段里程被静默吞掉。
            self.get_logger().error(
                "odom_update() 返回 0（几何无效？），本帧里程被丢弃且不推进基线",
                throttle_duration_sec=2.0,
            )
            return

        self._d_prev = d_now
        self._last_encoder_t = arrival
        self._last_encoder_dt = dt
        # ★ 归一化用**协议名义周期**，不是 dt（契约 §5.1 修订）。
        self._publish_odom(twist, self._nominal_period_s)

    def _publish_odom(self, twist: MecanumTwist, dt_s: float) -> None:
        """把位姿与 twist 发出去。

        ★ ``twist`` 是 ``odom_update()`` 给的**本帧位移**（mm / mrad），
        必须除以时间才是 m/s。但 ``dt_s`` 传进来的恒是 **协议名义周期**
        ``1/50 s``，**不是**每帧实测间隔 —— 原因见 ``NOMINAL_ENCODER_PERIOD_S``
        与 ``MIN_ENCODER_DT_S`` 上面的注释（实测 dt 会被串口的块交付压到 ~0，
        把 twist 放大 20 倍）。
        """
        stamp = _now_msg(self)
        yaw_rad = self._pose.theta_mrad / 1000.0

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame_id
        msg.child_frame_id = self._base_frame_id
        msg.pose.pose.position.x = self._pose.x_mm / 1000.0
        msg.pose.pose.position.y = self._pose.y_mm / 1000.0
        msg.pose.pose.position.z = 0.0
        # 平面车只绕 z 转：四元数退化成 (0, 0, sin(θ/2), cos(θ/2))。
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        # ★ 协方差故意留 0：契约没有冻结协方差模型，编一个数字出来比留 0 更有害。
        #   等实车标定出编码器/轮径的噪声水平再填（见 README「未验证项」）。
        msg.twist.twist.linear.x = twist.vx_mm_s / dt_s / 1000.0
        msg.twist.twist.linear.y = twist.vy_mm_s / dt_s / 1000.0
        msg.twist.twist.angular.z = twist.omega_mrad_s / dt_s / 1000.0
        self._odom_pub.publish(msg)

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._odom_frame_id
            tf.child_frame_id = self._base_frame_id
            tf.transform.translation.x = msg.pose.pose.position.x
            tf.transform.translation.y = msg.pose.pose.position.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = msg.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf)

    # -- B 帧 ------------------------------------------------------------

    def _on_battery_frame(self, frame: BatteryFrame) -> None:
        msg = BatteryState()
        msg.header.stamp = _now_msg(self)
        msg.voltage = frame.millivolts / 1000.0
        msg.present = True
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        # ★ percentage 留 NaN：契约没冻结电池曲线，编一个充满/放空电压出来
        #   会让上层拿到一个看起来很确定、其实是假的百分比。
        msg.percentage = float("nan")
        # flags 的位定义在 STM32 固件里（06 §7），本契约未冻结 —— 原样透出，别猜。
        msg.location = "stm32_flags=%d" % frame.flags
        self._battery_pub.publish(msg)

    # -- A 帧 ------------------------------------------------------------

    def _on_ack_frame(self, frame: AckFrame) -> None:
        text = frame.text.strip()
        level = DiagnosticStatus.OK if text.upper().startswith("OK") else DiagnosticStatus.WARN
        status = DiagnosticStatus()
        status.name = "stm32_bridge/stm32_ack"
        status.hardware_id = self._port
        status.level = level
        status.message = text or "(空回执)"
        self._publish_diagnostics([status])

    # ------------------------------------------------------------------
    # 看门狗
    # ------------------------------------------------------------------

    def _check_cmd_vel_watchdog(self) -> None:
        """契约 §4：``/cmd_vel`` 断流 > 300 ms 就补发 ``V,0,0,0``（与 STM32 自己的超时双保险）。

        ★ 安全帧**绕过下行限速**，立刻直接写。
        """
        if not self._have_cmd:
            return
        if self._last_vel_frame == ZERO_VELOCITY_FRAME:
            return
        if (time.monotonic() - self._last_cmd_vel_t) <= self._cmd_vel_timeout:
            return
        # 断流了：任何还挂着的速度指令都已过时，丢掉 —— 绝不能让它排在停车帧后面。
        self._pending_vel_frame = None
        if self._write(ZERO_VELOCITY_FRAME):
            self._last_vel_frame = ZERO_VELOCITY_FRAME
            self.get_logger().warn(
                "/cmd_vel 断流 > %.3f s，已补发 V,0,0,0" % self._cmd_vel_timeout,
                throttle_duration_sec=2.0,
            )

    def _check_link_watchdog(self) -> None:
        """E 帧是 50 Hz 常发的 —— 超过阈值没有**任何**帧，说明链路断了。"""
        if self._reader_error is not None:
            return  # 已经在收线程里报过错了
        if not self._link_ok:
            return
        if (time.monotonic() - self._last_frame_t) > self._link_watchdog_s:
            self._link_ok = False
            self._publish_link_status(
                DiagnosticStatus.ERROR,
                "%.1f s 内没有收到任何串口帧（STM32 的 E 帧是 50 Hz 常发）"
                % self._link_watchdog_s,
            )

    def _publish_link_status(self, level: int, message: str) -> None:
        status = DiagnosticStatus()
        status.name = "stm32_bridge/serial_link"
        status.hardware_id = self._port
        status.level = level
        status.message = message
        status.values = [
            KeyValue(key="decoder_frames_ok", value=str(self._decoder.stats.frames_ok)),
            KeyValue(key="decoder_dropped_lines", value=str(self._decoder.stats.dropped_lines)),
            KeyValue(key="decoder_nonascii_lines", value=str(self._decoder.stats.nonascii_lines)),
            KeyValue(key="decoder_overlong_lines", value=str(self._decoder.stats.overlong_lines)),
            KeyValue(key="core_library", value=self._core.path),
            KeyValue(key="twist_nominal_period_ms", value="%.1f" % (self._nominal_period_s * 1000.0)),
            KeyValue(key="encoder_dt_outliers", value=str(self._dt_outliers)),
        ]
        text = "链路诊断 level=%d：%s" % (level, message)
        if level == DiagnosticStatus.OK:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)
        self._publish_diagnostics([status])

    def _publish_diagnostics(self, statuses: List[DiagnosticStatus], force: bool = False) -> None:
        """发一次 ``/diagnostics``，但**默认限速 1 Hz**（``diagnostics_rate_hz``）。

        ★ 两个理由：
        1. 它是给 rqt_robot_monitor / Foxglove 看的**低频**话题。树莓派只有 2 GB，
           实测里每条 A 帧都发一次，2.4 s 里发了 1475 条 —— 比 E 帧多 8 倍。
        2. ``/diagnostics`` 的正确语义是**状态快照**，不是事件流水：
           同名项只保留最新的一条，重复的 ``A,OK`` 不该各占一条消息。

        所以这里做两件事：① 按 ``name`` 覆盖式维护快照；② 限速放行，
        但**等级发生变化时立刻发**（OK <-> WARN <-> ERROR），不让告警被限速压住。

        ★ 注意：只有 **level 变化**才会插队，**message 变化不会**。
        这是故意的 —— 真机固件的 A 帧很可能是 ``A,OK v=200`` 这种**带数值**的回执，
        那样"文本变了就立刻发"等于没限速（每个值都不一样）。等级不变时
        最长延迟 ``1 / diagnostics_rate_hz`` 秒，对 snapshot 语义完全够用。
        """
        changed = False
        for status in statuses:
            previous = self._diag_state.get(status.name)
            if previous is None or previous.level != status.level:
                changed = True
            self._diag_state[status.name] = status

        if not self._diag_limiter.should_emit(time.monotonic(), force=force or changed):
            return

        msg = DiagnosticArray()
        msg.header.stamp = _now_msg(self)
        msg.status = list(self._diag_state.values())
        self._diag_pub.publish(msg)

    # ------------------------------------------------------------------
    # 上电 reset 连发
    # ------------------------------------------------------------------

    def _on_reset_timer(self) -> None:
        if self._reset_sent >= RESET_BURST_COUNT:
            if self._reset_timer is not None:
                self._reset_timer.cancel()
                self.destroy_timer(self._reset_timer)
                self._reset_timer = None
            return
        if self._write(RESET_FRAME):
            # ★★ 2026-10 修 bug：STM32 收到 reset 会把累计里程清零，而桥这边的基线
            #    _d_prev 还是清零前的值 —— 下一帧 E 会算出一个大负数 Δd，/odom 在
            #    开机头 0.9 s 内向后跳一段。以前 tools/run_e2e.py 只能靠"多等 1.2 s
            #    把 reset 连发全部吃完"绕开它。正确做法：每成功发一条 reset，
            #    就把里程基线一起作废，让下一帧 E 重新定基线。
            self._have_encoder = False
            self._pose, self._total = self._core.odom_reset()
        self._reset_sent += 1
        self.get_logger().info("已发 reset（%d/%d）" % (self._reset_sent, RESET_BURST_COUNT))

    # ------------------------------------------------------------------
    # 收尾
    # ------------------------------------------------------------------

    def shutdown(self) -> None:
        self._stop.set()
        if self._reader is not None:
            self._reader.join(timeout=1.0)
        try:
            self._serial.close()
        except Exception:
            pass


def main(args=None) -> int:
    rclpy.init(args=args)
    node: Optional[Stm32Bridge] = None
    try:
        node = Stm32Bridge()
    except Exception as exc:
        # 契约 §5.3：找不到 .so / 几何不合法 / 串口打不开 —— 一律明确失败，不降级。
        print("stm32_bridge 启动失败：%s" % exc, file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
