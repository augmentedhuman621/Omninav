"""stm32_bridge.bridge_node — the serial bridge's ``rclpy`` node (the **only** place in this package that imports ROS2).

Behavior follows ``11_interface_contract.md`` §3 / §4 and the 7-row behavior table of the T2 task card:

===  ==========================================================================
 1   Subscribe to ``/cmd_vel`` (``geometry_msgs/Twist``) -> send ``V,<vx>,<vy>,<w>\\n``, rounding half away from zero
 2   Read ``E,...`` (50 Hz) -> take Δd -> call ``odom_update()`` -> publish ``/odom`` + TF ``odom->base_link``
 3   Read ``B,...`` (10 Hz) -> publish ``/battery_state``
 4   Read ``A,<text>`` -> publish ``/diagnostics``
 5   ``/cmd_vel`` silent > 300 ms -> actively send ``V,0,0,0``
 6   **Only** publish ``odom->base_link``; never publish ``map->odom``, never publish ``base_link->laser``
 7   Serial device defaults to ``/dev/ttySTM32`` (overridable by a ROS2 parameter); ``/dev/ttyUSB0`` is **forbidden**
===  ==========================================================================

★ This node **contains no kinematics / odometry math**: that exists in exactly one place, the C code in ``firmware_core``,
  and is called through the ``ctypes`` bindings of :mod:`stm32_bridge.kinematics`. A missing ``.so`` means startup failure —
  no fallback, no local math.
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
except ImportError as _exc:  # pragma: no cover - only triggered when a dependency is missing
    raise ImportError(
        "stm32_bridge requires pyserial. Install it with:\n"
        "  Raspberry Pi: sudo apt install python3-serial\n"
        "  (the Pi can already reach the internet through the Windows USB NIC sharing (192.168.137.x), so apt installs directly)"
    ) from _exc

#: Contract §2: only these three device node names are allowed, created by the udev rules in ``pi_setup/``.
DEFAULT_SERIAL_PORT = "/dev/ttySTM32"

#: Contract §2: **never** hardcode ttyUSBx in code — the two CH340 devices are the same model and collide in by-id,
#: so a change in plug order swaps the lidar and the serial bridge. Hard-reject that here.
FORBIDDEN_PORT_SUBSTRINGS = ("ttyUSB", "ttyACM")

#: Contract §4: send a stop frame when ``/cmd_vel`` has been silent longer than this.
DEFAULT_CMD_VEL_TIMEOUT_S = 0.3

#: Contract §3: 115200 8N1, no flow control.
DEFAULT_BAUDRATE = 115200

#: Per-tick cap when draining the receive queue (prevents one tick from processing tens of thousands of frames and stalling the node).
MAX_FRAMES_PER_TICK = 200

#: ★★ When normalizing the ``/odom`` twist, **do not use** the measured per-frame interval; always use the protocol nominal period
#: ``NOMINAL_ENCODER_PERIOD_S`` (= 1/50 s = 0.020 s, contract §5.1 revision).
#:
#: Why (a real bug hit during Raspberry Pi measurements): ``odom_update()``'s ``twist_out`` is
#: **a per-frame displacement increment** (mm / mrad) — its signature has no dt at all, so it cannot produce a "per second" quantity.
#: Contract §4 requires the ``/odom`` twist in m/s / rad/s, so the bridge must divide by a time itself.
#: But that time **must not be the measured per-frame interval**: serial / pty delivers in blocks, and one ``read()`` often returns several
#: E frames; measuring the "arrival interval" frame by frame degenerates to ~0, so the division inflates the twist 20x —
#: measurements showed ``twist = 4.0 m/s`` spikes (``Δd = 4 mm`` with dt clamped to 1 ms).
#: Position integration was correct all along (x monotonic, 0.294 m ≈ 0.196 m/s); only this derived quantity was divided wrong.
#:
#: The MIN/MAX values below are still useful, but now **serve diagnostics only**: the measured interval is used to tell
#: whether the sender stalled / dropped frames, and to log outliers. **Never used in velocity computation.**
MIN_ENCODER_DT_S = 1e-3
MAX_ENCODER_DT_S = 1.0

#: Ratio by which the measured frame interval may deviate from the nominal period before a warning is logged (diagnostics only).
ENCODER_DT_WARN_RATIO = 3.0

#: Default rate limit (Hz) for downlink ``V`` frames. 50 Hz telemetry / 20 Hz downlink is the usual ratio for this kind of chassis:
#: each frame is ~15 B on a 115200 link, so without a limit the serial bandwidth would be ~30% consumed,
#: and the STM32 receive state machine would be needlessly saturated.
DEFAULT_CMD_VEL_RATE_HZ = 20.0

#: Default rate limit (Hz) for ``/diagnostics``. It is a **low-rate** topic for rqt_robot_monitor / Foxglove,
#: and the Pi has only 2 GB, so not every A frame may be published.
DEFAULT_DIAGNOSTICS_RATE_HZ = 1.0


def _now_msg(node: Node):
    return node.get_clock().now().to_msg()


class Stm32Bridge(Node):
    """Bidirectional bridge between STM32 and ROS2."""

    def __init__(self, parameter_overrides=None) -> None:
        # parameter_overrides is for tools/run_e2e.py: the end-to-end script needs to point serial_port
        # at the fake serial port created by os.openpty() in the same process. Parameter overrides are more reliable than relying
        # on the global `--ros-args -p` arguments, and the command-line entry `main()` still calls without arguments.
        super().__init__("stm32_bridge", parameter_overrides=parameter_overrides)

        # ---------------- Parameters ----------------
        # ★ Geometry parameter defaults are **sentinels** (-1 / 0), meaning "use the C-side mecanum_default_geom()".
        #   This keeps a second set of numbers like 150 / 10505 out of Python (contract §6 hard requirement).
        self.declare_parameter("serial_port", DEFAULT_SERIAL_PORT)
        self.declare_parameter("baudrate", DEFAULT_BAUDRATE)
        self.declare_parameter("cmd_vel_timeout", DEFAULT_CMD_VEL_TIMEOUT_S)
        self.declare_parameter("cmd_vel_rate_hz", DEFAULT_CMD_VEL_RATE_HZ)
        self.declare_parameter("diagnostics_rate_hz", DEFAULT_DIAGNOSTICS_RATE_HZ)
        # ★ Nominal E-frame rate: only used to convert twist_out (per-frame displacement) into m/s.
        #   The contract §5.1 revision freezes it at 50 Hz (06 §7). Do **not** change it to a "measured value".
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

        # ★ Nominal period for twist normalization (contract §5.1 revision): always 1/nominal rate, never the measured dt.
        nominal_hz = float(p("encoder_nominal_hz"))
        if not math.isfinite(nominal_hz) or nominal_hz <= 0.0:
            raise ValueError("encoder_nominal_hz must be a positive finite number, got %r" % (nominal_hz,))
        self._nominal_period_s: float = 1.0 / nominal_hz

        # Downlink V rate limit (Hz -> minimum interval). Safety frames (timeout V,0,0,0 / S / reset) bypass it.
        cmd_vel_hz = float(p("cmd_vel_rate_hz"))
        if not math.isfinite(cmd_vel_hz) or cmd_vel_hz <= 0.0:
            raise ValueError("cmd_vel_rate_hz must be a positive finite number, got %r" % (cmd_vel_hz,))
        self._cmd_vel_limiter = RateLimiter(1.0 / cmd_vel_hz)

        diag_hz = float(p("diagnostics_rate_hz"))
        if not math.isfinite(diag_hz) or diag_hz < 0.0:
            raise ValueError("diagnostics_rate_hz must be a non-negative finite number, got %r" % (diag_hz,))
        # diag_hz == 0 means "no rate limit".
        self._diag_limiter = RateLimiter(0.0 if diag_hz <= 0.0 else 1.0 / diag_hz)
        #: **Latest** state per diagnostic item, deduplicated by name — /diagnostics publishes a snapshot, not a stream.
        self._diag_state: "dict" = {}

        self._reject_forbidden_port()

        # ---------------- C core (single source of math) ----------------
        self._core = kinematics.default_core()
        self.get_logger().info("Loaded C core: %s" % self._core.describe())
        self._geom: MecanumGeom = self._build_geom(p)

        # ---------------- State ----------------
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
        # Arrival time and measured interval of the previous E frame — see the comment above MIN_ENCODER_DT_S.
        self._last_encoder_t = time.monotonic()
        self._last_encoder_dt = 0.0
        # For the receive thread: arrival time of the previous chunk + inferred nominal frame interval
        self._last_chunk_t = time.monotonic()
        self._nominal_dt = 0.02  # default to 50 Hz at startup; replaced by the measured value after the first frame

        self._have_cmd = False
        self._last_vel_frame: Optional[bytes] = None
        self._last_cmd_vel_t = 0.0
        #: The newest V frame held back by the rate limiter, resent on the next tick or next allowed slot.
        #: ★ The **newest value** is stored instead of queueing: velocity commands are inherently "overwrite" style;
        #:   queueing accumulates latency, while overwriting only discards intermediate stale values.
        self._pending_vel_frame: Optional[bytes] = None
        #: Count of measured intervals deviating too far from the nominal period (diagnostics only, never used in velocity).
        self._dt_outliers = 0

        self._last_frame_t = time.monotonic()
        self._link_ok = True
        self._reset_sent = 0
        self._reset_timer = None

        # ---------------- Publishers / subscribers ----------------
        # ★ /odom uses RELIABLE: a reliable publisher can feed both reliable subscribers and
        #   best-effort subscribers (AMCL uses SensorDataQoS on Jazzy).
        #   Conversely a best-effort publisher cannot feed a reliable subscriber — so do not switch to best-effort.
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

        # ---------------- Serial port ----------------
        self._serial = self._open_serial()

        self._reader = threading.Thread(target=self._reader_loop, name="stm32-rx", daemon=True)
        self._reader.start()

        self._timer = self.create_timer(self._poll_period, self._on_timer)

        if self._reset_timer_enabled():
            # Contract §3: send reset 3 times at startup, 0.3 s apart.
            self._reset_timer = self.create_timer(RESET_BURST_PERIOD_S, self._on_reset_timer)

        self.get_logger().info(
            "Serial bridge ready: %s @ %d 8N1, L=%d mm, cmd_vel timeout %.3f s, "
            "twist nominal period %.1f ms (nominal %.0f Hz), downlink V limit %.0f Hz, /diagnostics limit %.0f Hz"
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
    # Startup validation / construction
    # ------------------------------------------------------------------

    def _reject_forbidden_port(self) -> None:
        for bad in FORBIDDEN_PORT_SUBSTRINGS:
            if bad in self._port:
                raise ValueError(
                    "Refusing serial port %s: contract §2 explicitly forbids ttyUSBx / ttyACMx.\n"
                    "Reason: both the lidar and the vision module are CH340 (1a86:7522) and /dev/serial/by-id has only one entry,"
                    "so a change in plug order swaps the device names — the bridge would receive the lidar's binary garbage.\n"
                    "Correct approach: let the udev rules in pi_setup/ pin the STM32 USB-TTL to %s,"
                    "then point the serial_port parameter at it." % (self._port, DEFAULT_SERIAL_PORT)
                )

    def _build_geom(self, p) -> MecanumGeom:
        """Geometry has only two sources: the C-side ``mecanum_default_geom()``, or ROS2 parameter overrides (contract §6)."""
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
                "Invalid geometry: half_wheelbase_mm=%d half_track_mm=%d "
                "vy_polarity=%d omega_polarity=%d"
                % (
                    geom.half_wheelbase_mm,
                    geom.half_track_mm,
                    geom.vy_polarity,
                    geom.omega_polarity,
                )
            )
        if self._core.l_mm(geom) < 1:
            raise ValueError("L = half_wheelbase + half_track < 1 mm; forward kinematics would divide by zero")
        return geom

    def _reset_timer_enabled(self) -> bool:
        return bool(self.get_parameter("send_reset_on_start").value)

    def _open_serial(self):
        try:
            # ★ rtscts/dsrdtr/xonxoff are all explicitly disabled: the contract requires "no flow control",
            #   and once DTR/RTS are asserted, a CH340 wired to EN/IO0 pulls the peer chip into download mode
            #   (that is how this project's vision-module comm board got bricked into `key_long_down!!`).
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
        except Exception as exc:  # serial.SerialException and others
            raise RuntimeError(
                "Cannot open serial port %s @ %d: %s\n"
                "Check: (1) whether the udev rule created %s; (2) whether yjc is in the dialout group"
                "(`id` must list dialout, and re-login after adding it); (3) whether the device is actually plugged in."
                % (self._port, self._baudrate, exc, DEFAULT_SERIAL_PORT)
            ) from exc

    # ------------------------------------------------------------------
    # Serial receive thread
    # ------------------------------------------------------------------

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._serial.read(64)
            except Exception as exc:  # cable unplugged / permissions / device gone
                self._reader_error = "%s: %s" % (type(exc).__name__, exc)
                self.get_logger().error("Serial read failed, receive thread exiting: %s" % self._reader_error)
                return
            if not data:
                continue
            arrival = time.monotonic()
            frames = self._decoder.feed(data)
            if not frames:
                continue
            # ★ Timestamps are taken in the **receive thread**, not in the main tick.
            #   The main tick only drains the queue every poll_period (10 ms by default), while E frames arrive every 20 ms;
            #   using the tick time would make dt jitter by ±10 ms (velocity estimate jitter of ±50%),
            #   and worse, if two frames land in the same tick, dt≈0 would blow the velocity estimate up tens of times.
            #   The 2nd..nth frames in the same chunk have no individual arrival time, so they are extrapolated using the
            #   average interval measured in the previous chunk — better slightly conservative than spiky.
            # ★★ 2026-10 bug fix: ``observed`` is the arrival interval of the **whole chunk**, while one
            #    read(64) often brings back 2~4 E frames at once. Originally it was used directly as the "single-frame interval",
            #    which multiplied dt by len(frames); after a link hiccup _nominal_dt would also be clamped to
            #    1.0 s, so later frames in the same chunk could drift tens of seconds into the future.
            #    Single-frame interval = observed / number of frames in this chunk.
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
            # ★★ 2026-10 bug fix: this log **must be throttled**. Once the link drops, the timeout watchdog retries on every
            #    main tick (10 ms = 100 Hz) writing V,0,0,0, and each failure logs one
            #    error — a few seconds would saturate the logs and CPU of a 2 GB Raspberry Pi.
            self.get_logger().error("Serial write failed: %s" % exc, throttle_duration_sec=1.0)
            return False
        return True

    # ------------------------------------------------------------------
    # Downlink: /cmd_vel
    # ------------------------------------------------------------------

    def _on_cmd_vel(self, msg: Twist) -> None:
        try:
            frame = encode_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)
        except (TypeError, ValueError) as exc:
            # NaN / inf cmd_vel: never encode garbage and send it to the STM32.
            self.get_logger().warn("Dropping invalid /cmd_vel: %s" % exc, throttle_duration_sec=1.0)
            return
        # ★ Record the time of the last received command first, then decide whether to send — the timeout watchdog looks at
        #   whether **a new command came in**, not at how fast we send.
        self._last_cmd_vel_t = time.monotonic()
        self._have_cmd = True
        self._pending_vel_frame = frame
        self._flush_pending_velocity()

    def _flush_pending_velocity(self) -> None:
        """Send the newest V frame subject to the 20 Hz downlink rate limit.

        ★ The rate limit applies only to **regular velocity commands**. Safety frames (the timeout ``V,0,0,0``,
        ``S``, ``reset``) always bypass this and go straight to ``_write()``, see
        :meth:`_check_cmd_vel_watchdog` and :meth:`_on_reset_timer`.
        """
        frame = self._pending_vel_frame
        if frame is None:
            return
        if not self._cmd_vel_limiter.should_emit(time.monotonic()):
            return  # retry on the next tick
        self._pending_vel_frame = None
        if self._write(frame):
            self._last_vel_frame = frame
        else:
            # ★ 2026-10 bug fix: the rate limiter has already accounted for this frame, so dropping it on a write failure would let
            #   a transient error waste a whole downlink slot. Put it back and resend on the next tick or next allowed slot.
            self._pending_vel_frame = frame

    # ------------------------------------------------------------------
    # Main tick: drain the receive queue + watchdogs
    # ------------------------------------------------------------------

    def _on_timer(self) -> None:
        for _ in range(MAX_FRAMES_PER_TICK):
            try:
                arrival, frame = self._rx_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_frame(frame, arrival)

        # The V frame held back by the rate limit is resent here (the 50 Hz tick comfortably feeds the 20 Hz downlink).
        self._flush_pending_velocity()
        self._check_cmd_vel_watchdog()
        self._check_link_watchdog()

    def _handle_frame(self, frame, arrival: float) -> None:
        self._last_frame_t = time.monotonic()
        if not self._link_ok:
            self._link_ok = True
            self._publish_link_status(DiagnosticStatus.OK, "serial link recovered")

        if isinstance(frame, EncoderFrame):
            self._on_encoder_frame(frame, arrival)
        elif isinstance(frame, BatteryFrame):
            self._on_battery_frame(frame)
        elif isinstance(frame, AckFrame):
            self._on_ack_frame(frame)
        elif isinstance(frame, UnknownFrame):
            self.get_logger().debug("Dropping malformed line: %s" % frame)

    # -- E frames: the only place that advances odometry --------------------

    def _on_encoder_frame(self, frame: EncoderFrame, arrival: float) -> None:
        if not self._have_encoder:
            # The first frame only establishes the baseline: without d_prev, Δd would produce a phantom displacement.
            self._d_prev = list(frame.d_mm)
            self._have_encoder = True
            self._last_encoder_t = arrival
            return

        dt = arrival - self._last_encoder_t
        if dt <= 0.0:
            # Clock did not advance / two frames at the same instant — reuse the previous interval, never divide by zero.
            dt = self._last_encoder_dt
        dt = min(max(dt, MIN_ENCODER_DT_S), MAX_ENCODER_DT_S)

        # ★ This measured dt is **for diagnostics only**: deviating far from the nominal period means the sender stalled or dropped frames.
        #   It never participates in the velocity computation — see the comment above MIN_ENCODER_DT_S.
        if dt > self._nominal_period_s * ENCODER_DT_WARN_RATIO:
            self._dt_outliers += 1
            self.get_logger().warn(
                "measured E-frame interval %.1f ms far exceeds the nominal period %.1f ms (occurrence %d)"
                % (dt * 1000.0, self._nominal_period_s * 1000.0, self._dt_outliers),
                throttle_duration_sec=5.0,
            )

        d_now = list(frame.d_mm)
        twist = self._core.odom_update(
            self._pose, self._total, self._geom, d_now, self._d_prev
        )
        if twist is None:
            # C returned 0 = invalid geometry / null pointer (structural error).
            # ★ Key point: **do not advance d_prev** — otherwise this chunk of odometry is silently swallowed.
            self.get_logger().error(
                "odom_update() returned 0 (invalid geometry?), this frame's odometry is dropped and the baseline is not advanced",
                throttle_duration_sec=2.0,
            )
            return

        self._d_prev = d_now
        self._last_encoder_t = arrival
        self._last_encoder_dt = dt
        # ★ Normalize with the **protocol nominal period**, not dt (contract §5.1 revision).
        self._publish_odom(twist, self._nominal_period_s)

    def _publish_odom(self, twist: MecanumTwist, dt_s: float) -> None:
        """Publish the pose and twist.

        ★ ``twist`` is the **per-frame displacement** (mm / mrad) returned by ``odom_update()``,
        so it must be divided by a time to become m/s. But ``dt_s`` is always the **protocol nominal period**
        ``1/50 s``, **not** the measured per-frame interval — see the comment above ``NOMINAL_ENCODER_PERIOD_S``
        and ``MIN_ENCODER_DT_S`` (the measured dt is compressed to ~0 by the serial block delivery,
        inflating the twist 20x).
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
        # A planar robot only rotates about z: the quaternion degrades to (0, 0, sin(θ/2), cos(θ/2)).
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        # ★ Covariance is deliberately left at 0: the contract does not freeze a covariance model, and inventing numbers is more harmful than leaving 0.
        #   Fill it in once the encoder/wheel-radius noise level has been calibrated on the real robot (see "unverified items" in the README).
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

    # -- B frames --------------------------------------------------------

    def _on_battery_frame(self, frame: BatteryFrame) -> None:
        msg = BatteryState()
        msg.header.stamp = _now_msg(self)
        msg.voltage = frame.millivolts / 1000.0
        msg.present = True
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        # ★ percentage stays NaN: the contract does not freeze a battery curve, and inventing full/empty voltages
        #   would hand upper layers a percentage that looks certain but is fake.
        msg.percentage = float("nan")
        # The flags bit definitions live in the STM32 firmware (06 §7) and are not frozen by this contract — pass them through as-is, don't guess.
        msg.location = "stm32_flags=%d" % frame.flags
        self._battery_pub.publish(msg)

    # -- A frames --------------------------------------------------------

    def _on_ack_frame(self, frame: AckFrame) -> None:
        text = frame.text.strip()
        level = DiagnosticStatus.OK if text.upper().startswith("OK") else DiagnosticStatus.WARN
        status = DiagnosticStatus()
        status.name = "stm32_bridge/stm32_ack"
        status.hardware_id = self._port
        status.level = level
        status.message = text or "(empty acknowledgement)"
        self._publish_diagnostics([status])

    # ------------------------------------------------------------------
    # Watchdogs
    # ------------------------------------------------------------------

    def _check_cmd_vel_watchdog(self) -> None:
        """Contract §4: send ``V,0,0,0`` when ``/cmd_vel`` has been silent for > 300 ms (double safety with the STM32's own timeout).

        ★ Safety frames **bypass the downlink rate limit** and are written immediately.
        """
        if not self._have_cmd:
            return
        if self._last_vel_frame == ZERO_VELOCITY_FRAME:
            return
        if (time.monotonic() - self._last_cmd_vel_t) <= self._cmd_vel_timeout:
            return
        # Timed out: any pending velocity command is stale, drop it — it must never end up queued after the stop frame.
        self._pending_vel_frame = None
        if self._write(ZERO_VELOCITY_FRAME):
            self._last_vel_frame = ZERO_VELOCITY_FRAME
            self.get_logger().warn(
                "/cmd_vel timeout > %.3f s, sent V,0,0,0" % self._cmd_vel_timeout,
                throttle_duration_sec=2.0,
            )

    def _check_link_watchdog(self) -> None:
        """E frames arrive at a steady 50 Hz — no frames at all beyond the threshold means the link is down."""
        if self._reader_error is not None:
            return  # already reported in the receive thread
        if not self._link_ok:
            return
        if (time.monotonic() - self._last_frame_t) > self._link_watchdog_s:
            self._link_ok = False
            self._publish_link_status(
                DiagnosticStatus.ERROR,
                "no serial frames received for %.1f s (the STM32 sends E frames at a steady 50 Hz)"
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
        text = "Link diagnostics level=%d: %s" % (level, message)
        if level == DiagnosticStatus.OK:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)
        self._publish_diagnostics([status])

    def _publish_diagnostics(self, statuses: List[DiagnosticStatus], force: bool = False) -> None:
        """Publish ``/diagnostics`` once, but **rate-limited to 1 Hz by default** (``diagnostics_rate_hz``).

        ★ Two reasons:
        1. It is a **low-rate** topic for rqt_robot_monitor / Foxglove. The Pi has only 2 GB, and
           measurements showed one message per A frame: 1475 messages in 2.4 s — 8x more than E frames.
        2. The correct semantics of ``/diagnostics`` is a **state snapshot**, not an event stream:
           items with the same name keep only the latest entry; repeated ``A,OK`` must not each take a message.

        So two things happen here: (1) maintain the snapshot by overwriting per ``name``; (2) rate-limit the output,
        but **send immediately when the level changes** (OK <-> WARN <-> ERROR) so alarms are not suppressed by the rate limit.

        ★ Note: only a **level change** jumps the queue, **a message change does not**.
        This is deliberate — real firmware A frames are likely acknowledgements **with values** such as ``A,OK v=200``,
        and "send immediately when the text changes" would mean no rate limit at all (every value differs). While the level is unchanged
        the worst-case delay is ``1 / diagnostics_rate_hz`` seconds, which is entirely adequate for snapshot semantics.
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
    # Startup reset burst
    # ------------------------------------------------------------------

    def _on_reset_timer(self) -> None:
        if self._reset_sent >= RESET_BURST_COUNT:
            if self._reset_timer is not None:
                self._reset_timer.cancel()
                self.destroy_timer(self._reset_timer)
                self._reset_timer = None
            return
        if self._write(RESET_FRAME):
            # ★★ 2026-10 bug fix: on receiving reset the STM32 clears its accumulated distance, while the bridge's baseline
            #    _d_prev still holds the pre-clear value — the next E frame computes a large negative Δd and /odom jumps
            #    backwards during the first 0.9 s after boot. Previously tools/run_e2e.py could only "wait an extra 1.2 s
            #    to consume the whole reset burst" to work around it. The correct fix: for every reset sent successfully,
            #    invalidate the odometry baseline too, so the next E frame re-establishes it.
            self._have_encoder = False
            self._pose, self._total = self._core.odom_reset()
        self._reset_sent += 1
        self.get_logger().info("Sent reset (%d/%d)" % (self._reset_sent, RESET_BURST_COUNT))

    # ------------------------------------------------------------------
    # Shutdown
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
        # Contract §5.3: missing .so / invalid geometry / serial port cannot open — always fail explicitly, never fall back.
        print("stm32_bridge failed to start: %s" % exc, file=sys.stderr)
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
