#!/usr/bin/env python3
"""Zero-hardware end-to-end: ``os.openpty()`` fake serial port + fake STM32 + real bridge + real rclpy.

It verifies the **whole chain**, not just the individual unit tests:

===  ==========================================================================
 1   E frames really come in, and ``/odom`` and TF ``odom->base_link`` are really published
 2   Publish one ``/cmd_vel`` and the fake STM32 really receives ``V,200,0,0``
 3   After the robot "drives", ``/odom``'s ``position.x`` grows monotonically and ``twist.linear.x`` ≈ 0.2 m/s
 4   TF contains **only** ``odom->base_link``, no ``map->odom`` and no ``base_link->laser``
 5   After ``/cmd_vel`` is silent for > 300 ms the bridge really sends ``V,0,0,0``
 6   ``B`` frames -> ``/battery_state``; ``A`` frames -> ``/diagnostics``
===  ==========================================================================

★ Why ``os.openpty()`` instead of socat: the Pi currently has **no default route / no internet access**,
so socat cannot be installed. A pseudo-terminal is built into the kernel: zero dependencies.

Exit codes: ``0`` all passed, ``1`` failures, ``77`` skipped (not Linux / no ROS2 / no .so).
77 is deliberate — "environment unavailable" and "test failed" must be distinguishable, otherwise a run would pretend to have passed.

How to run (on the Pi, inside ``ros2_ws/`` at the workspace root)::

    source /opt/ros/jazzy/setup.bash
    colcon build --packages-select stm32_bridge
    source install/setup.bash
    python3 tools/run_e2e.py

For laptop / WSL development (colcon is optional as long as rclpy imports)::

    export OMNINAV_CORE_SO=/absolute/path/firmware_core/build/libomninav_core.so
    python3 tools/run_e2e.py
"""

from __future__ import annotations

import math
import os
import sys
import time
from typing import List, Optional, Sequence

SKIP = 77

#: Contract §4: timeout threshold 0.3 s. Passed explicitly here, not guessed from the node default.
CMD_VEL_TIMEOUT_S = 0.3

#: Fake STM32 frame rate (consistent with contract §3).
ENCODER_HZ = 50.0

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(_HERE, "..", "src", "stm32_bridge")))
sys.path.insert(0, os.path.abspath(_HERE))


class Checks:
    """Minimal assertion collector: prints every conclusion together with its evidence."""

    def __init__(self) -> None:
        self.rows = []

    def check(self, name: str, ok: bool, detail: str = "") -> bool:
        self.rows.append((name, bool(ok), detail))
        return bool(ok)

    @property
    def failed(self):
        return [row for row in self.rows if not row[1]]

    def report(self) -> int:
        print("")
        print("-" * 96)
        for name, ok, detail in self.rows:
            print("%-4s %-52s %s" % ("ok" if ok else "FAIL", name, detail))
        print("-" * 96)
        print("%d checks, %d failed" % (len(self.rows), len(self.failed)))
        if self.failed:
            print("End-to-end: FAILED")
            return 1
        print("End-to-end: all passed")
        return 0


def _skip(reason: str, *hints: str) -> int:
    print("Skipping end-to-end test: %s" % reason)
    for hint in hints:
        print("  %s" % hint)
    return SKIP


def main(argv: Optional[Sequence[str]] = None) -> int:
    if not sys.platform.startswith("linux"):
        return _skip(
            "pseudo-terminals (os.openpty) exist only on Linux; current platform is %s" % sys.platform,
            "run it on the Raspberry Pi: python3 tools/run_e2e.py",
        )

    try:
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from sensor_msgs.msg import BatteryState
        from tf2_msgs.msg import TFMessage
    except ImportError as exc:
        return _skip(
            "no ROS2 on this machine (%s: %s)" % (type(exc).__name__, exc),
            "the end-to-end test must run on a Pi with ROS2 Jazzy installed (the laptop not having ROS2 is a project hard constraint).",
            "to test only protocol parsing use: python3 src/stm32_bridge/test/test_protocol.py",
        )

    try:
        from stm32_bridge.kinematics import default_core
        core = default_core()
    except Exception as exc:  # CoreLibraryNotFound / CoreSymbolMissing / ...
        return _skip(
            "failed to load libomninav_core.so: %s" % exc,
            "first have T1 produce it: cd firmware_core && make lib (output build/libomninav_core.so)",
            "then: OMNINAV_CORE_SO=/absolute/path/libomninav_core.so python3 tools/run_e2e.py",
        )

    import tty

    from fake_stm32 import FakeStm32, FdTransport
    from stm32_bridge.bridge_node import Stm32Bridge

    checks = Checks()
    print("C core: %s" % core.describe())

    master_fd, slave_fd = os.openpty()
    tty.setraw(slave_fd)
    slave_name = os.ttyname(slave_fd)
    print("pseudo-terminal: master_fd=%d  slave=%s" % (master_fd, slave_name))

    transport = FdTransport(master_fd)
    fake = FakeStm32(transport, core=core, encoder_hz=ENCODER_HZ)
    fake.start()

    rclpy.init(args=list(argv) if argv else None)
    bridge = None
    probe = None
    executor = None
    try:
        try:
            bridge = Stm32Bridge(
                parameter_overrides=[
                    Parameter("serial_port", value=slave_name),
                    Parameter("cmd_vel_timeout", value=CMD_VEL_TIMEOUT_S),
                ]
            )
        except Exception as exc:
            print("bridge failed to start: %s" % exc)
            return 1

        probe = Node("e2e_probe")
        cmd_pub = probe.create_publisher(Twist, "cmd_vel", 10)

        odoms: List[Odometry] = []
        tfs: List[TFMessage] = []
        batteries: List[BatteryState] = []
        diags: List[DiagnosticArray] = []
        probe.create_subscription(Odometry, "odom", odoms.append, 10)
        probe.create_subscription(TFMessage, "tf", tfs.append, 100)
        probe.create_subscription(BatteryState, "battery_state", batteries.append, 10)
        probe.create_subscription(DiagnosticArray, "diagnostics", diags.append, 10)

        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(probe)

        def pump(seconds: float) -> None:
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=0.02)

        # ---- 1) Link bootstrap: E frames -> /odom + TF --------------------------
        # ★ Pump for 1.2 s here instead of 1.0 s: the bridge defaults to send_reset_on_start=True and
        #   contract §3 requires 3 resets at startup, 0.3 s apart, so there is still one at 0.9 s.
        #   The fake STM32 clears accumulated distance on reset — if that one lands inside the driving phase,
        #   Δd suddenly turns negative and the "position.x is monotonically non-decreasing" assertion goes red for no obvious reason.
        #   Waiting the extra 0.3 s consumes the whole reset burst, which is what makes the test deterministic.
        pump(1.2)
        checks.check(
            "the fake STM32 is sending E frames continuously",
            fake.e_frames_sent >= 10,
            "e_frames_sent=%d" % fake.e_frames_sent,
        )
        checks.check(
            "E frames are parsed and published as /odom",
            len(odoms) >= 5,
            "received %d /odom messages" % len(odoms),
        )
        checks.check(
            "TF odom->base_link is published",
            len(tfs) >= 5,
            "received %d /tf messages" % len(tfs),
        )
        checks.check(
            "/odom frame_id / child_frame_id are correct",
            all(m.header.frame_id == "odom" and m.child_frame_id == "base_link" for m in odoms),
            "odom=%s base=%s"
            % (
                odoms[0].header.frame_id if odoms else "(none)",
                odoms[0].child_frame_id if odoms else "(none)",
            ),
        )

        # ---- 2) /cmd_vel -> V frames ---------------------------------------
        baseline = len(odoms)
        command = Twist()
        command.linear.x = 0.2  # 200 mm/s
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            cmd_pub.publish(command)
            executor.spin_once(timeout_sec=0.05)
        checks.check(
            "/cmd_vel 0.2 m/s is encoded as V,200,0,0",
            (200, 0, 0) in fake.v_frames,
            "V frames the fake STM32 received (first 5): %s" % (fake.v_frames[:5],),
        )

        # ---- 3) Odometry really advances --------------------------------------
        driven = odoms[baseline:]
        checks.check(
            "/odom keeps being published while driving",
            len(driven) >= 20,
            "%d /odom messages in the driving phase" % len(driven),
        )
        if driven:
            xs = [m.pose.pose.position.x for m in driven]
            monotonic = all(xs[i + 1] >= xs[i] - 1e-6 for i in range(len(xs) - 1))
            checks.check(
                "/odom position.x is monotonically non-decreasing",
                monotonic,
                "x: %.4f -> %.4f m" % (xs[0], xs[-1]),
            )
            checks.check(
                "after 1.5 s @ 200 mm/s, x is at least 0.1 m",
                xs[-1] >= 0.1,
                "x=%.4f m (theory ≈0.3 m, minus command latency at the start)" % xs[-1],
            )
            twists = [m.twist.twist.linear.x for m in driven]
            mean_twist = sum(twists) / len(twists)
            checks.check(
                "/odom twist.linear.x mean ≈ 0.2 m/s",
                abs(mean_twist - 0.2) <= 0.03,
                "mean %.4f m/s (%d samples, min %.4f max %.4f)"
                % (mean_twist, len(twists), min(twists), max(twists)),
            )
            # ★ Regression guard: the Pi measurements showed a max = 4.0000 m/s spike; root cause was
            #   "normalizing by the measured per-frame dt" — serial delivers in blocks, so when one read() returns several E frames
            #   the measured dt degenerates to ~0 and the division inflates the twist 20x. Now the protocol nominal period is used
            #   (NOMINAL_ENCODER_PERIOD_S = 1/50 s), so the **maximum must also stay close to 0.2**.
            #   Checking only the mean cannot catch this bug (the mean was 0.4185 at the time, only 2x too high).
            checks.check(
                "/odom twist.linear.x max does not exceed 0.25 m/s (guards against dt-degeneration inflation)",
                max(twists) <= 0.25,
                "max %.4f m/s (a multiple of ≈ 0.2 means dt was compressed)" % max(twists),
            )

        # ---- 4) TF boundary: only odom->base_link is allowed --------------------------
        pairs = set()
        for message in tfs:
            for transform in message.transforms:
                pairs.add((transform.header.frame_id, transform.child_frame_id))
        checks.check(
            "TF contains odom->base_link",
            ("odom", "base_link") in pairs,
            "TF pairs seen: %s" % sorted(pairs),
        )
        checks.check(
            "map->odom is not published (that is slam_toolbox / AMCL's job)",
            ("map", "odom") not in pairs,
            "explicitly forbidden by contract §4",
        )
        checks.check(
            "base_link->laser is not published (that is URDF / robot_state_publisher's job)",
            ("base_link", "laser") not in pairs,
            "explicitly forbidden by contract §4",
        )

        # ---- 5) cmd_vel timeout watchdog -------------------------------------
        before_watchdog = len(fake.v_frames)
        pump(CMD_VEL_TIMEOUT_S + 0.5)
        after = fake.v_frames[before_watchdog:]
        checks.check(
            "V,0,0,0 is resent after > %.1f s of silence" % CMD_VEL_TIMEOUT_S,
            bool(after) and after[-1] == (0, 0, 0),
            "new V frames during the silence: %s" % (after[:5],),
        )

        # ---- 6) B frames -> /battery_state, A frames -> /diagnostics -----------
        checks.check(
            "B frames are published as /battery_state",
            len(batteries) >= 1,
            "received %d" % len(batteries),
        )
        if batteries:
            checks.check(
                "/battery_state voltage ≈ 11.8 V",
                abs(batteries[-1].voltage - 11.8) <= 0.3,
                "voltage=%.3f V  location=%s" % (batteries[-1].voltage, batteries[-1].location),
            )
            checks.check(
                "percentage is deliberately NaN (the contract does not freeze a battery curve)",
                math.isnan(batteries[-1].percentage),
                "percentage=%r" % (batteries[-1].percentage,),
            )
        ack_names = {
            status.name for message in diags for status in message.status
        }
        checks.check(
            "A frames are published as /diagnostics",
            "stm32_bridge/stm32_ack" in ack_names,
            "diagnostic items: %s" % sorted(ack_names),
        )
        # ★ Regression guard (Pi measurement: 1475 /diagnostics in 2.4 s, 8x more than E frames).
        #   Now throttled to 1 Hz with one snapshot per name, so only single digits are expected for the whole case.
        checks.check(
            "/diagnostics is throttled (<= 30 messages for the whole case)",
            len(diags) <= 30,
            "received %d (1475 measured before throttling)" % len(diags),
        )
        # ★ Regression guard (Pi measurement: 2369 V frames unthrottled, ~30% of the 115200 bandwidth).
        #   The effective downlink time here is ~2.5 s at 20 Hz ⇒ ~50 frames; 120 leaves plenty of margin.
        checks.check(
            "downlink V frames are throttled (<= 120)",
            len(fake.v_frames) <= 120,
            "total %d (2369 measured before throttling)" % len(fake.v_frames),
        )
        checks.check(
            "the serial link reported no errors",
            fake.last_error is None,
            "fake.last_error=%r" % (fake.last_error,),
        )

        print(
            "\nTraffic: E frames %d, B frames %d, V commands %d, /odom %d, /tf %d, "
            "/battery_state %d, /diagnostics %d"
            % (
                fake.e_frames_sent,
                fake.b_frames_sent,
                len(fake.v_frames),
                len(odoms),
                len(tfs),
                len(batteries),
                len(diags),
            )
        )
        return checks.report()
    finally:
        if bridge is not None:
            try:
                bridge.shutdown()
            except Exception:
                pass
            try:
                bridge.destroy_node()
            except Exception:
                pass
        if probe is not None:
            try:
                probe.destroy_node()
            except Exception:
                pass
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        fake.stop()
        fake.join()
        transport.close()
        try:
            os.close(slave_fd)
        except OSError:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
