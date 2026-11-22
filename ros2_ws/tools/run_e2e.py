#!/usr/bin/env python3
"""零硬件端到端：``os.openpty()`` 假串口 + 假 STM32 + 真桥 + 真 rclpy。

它验证的是**整条链**，而不只是各自单测：

===  ==========================================================================
 1   E 帧真的能进来，``/odom`` 与 TF ``odom->base_link`` 真的被发出来
 2   发一条 ``/cmd_vel``，假 STM32 那边真的收到 ``V,200,0,0``
 3   车"走"了之后，``/odom`` 的 ``position.x`` 单调增，``twist.linear.x`` ≈ 0.2 m/s
 4   TF 里**只有** ``odom->base_link``，没有 ``map->odom``，也没有 ``base_link->laser``
 5   ``/cmd_vel`` 断流 > 300 ms 后，桥真的补发了 ``V,0,0,0``
 6   ``B`` 帧 -> ``/battery_state``；``A`` 帧 -> ``/diagnostics``
===  ==========================================================================

★ 为什么用 ``os.openpty()`` 而不是 socat：树莓派现在**没有默认路由 / 上不了外网**，
装不了 socat。伪终端是内核自带的，零依赖。

退出码：``0`` 全过、``1`` 有失败、``77`` 跳过（不是 Linux / 没装 ROS2 / 没有 .so）。
77 是刻意选的 —— 「环境不具备」和「测试失败」必须分得开，否则会假装跑过了。

跑法（树莓派上，在工作区根的 ``ros2_ws/`` 里）::

    source /opt/ros/jazzy/setup.bash
    colcon build --packages-select stm32_bridge
    source install/setup.bash
    python3 tools/run_e2e.py

笔记本 / WSL 上开发时（还没装 colcon 也可以，只要 rclpy 能 import）::

    export OMNINAV_CORE_SO=/绝对路径/firmware_core/build/libomninav_core.so
    python3 tools/run_e2e.py
"""

from __future__ import annotations

import math
import os
import sys
import time
from typing import List, Optional, Sequence

SKIP = 77

#: 契约 §4：断流阈值 0.3 s。测试里显式传入，不去猜节点默认值。
CMD_VEL_TIMEOUT_S = 0.3

#: 假 STM32 的发帧频率（与契约 §3 一致）。
ENCODER_HZ = 50.0

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(_HERE, "..", "src", "stm32_bridge")))
sys.path.insert(0, os.path.abspath(_HERE))


class Checks:
    """极简断言收集器：把每条结论连同证据一起打出来。"""

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
        print("共 %d 条检查，%d 条失败" % (len(self.rows), len(self.failed)))
        if self.failed:
            print("端到端：失败")
            return 1
        print("端到端：全部通过")
        return 0


def _skip(reason: str, *hints: str) -> int:
    print("跳过端到端测试：%s" % reason)
    for hint in hints:
        print("  %s" % hint)
    return SKIP


def main(argv: Optional[Sequence[str]] = None) -> int:
    if not sys.platform.startswith("linux"):
        return _skip(
            "伪终端（os.openpty）只在 Linux 上有，当前平台是 %s" % sys.platform,
            "请在树莓派上跑：python3 tools/run_e2e.py",
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
            "这台机器上没有 ROS2（%s: %s）" % (type(exc).__name__, exc),
            "端到端测试必须在装了 ROS2 Jazzy 的树莓派上跑（笔记本不装 ROS2 是项目硬约束）。",
            "只测纯协议解析的话用：python3 src/stm32_bridge/test/test_protocol.py",
        )

    try:
        from stm32_bridge.kinematics import default_core
        core = default_core()
    except Exception as exc:  # CoreLibraryNotFound / CoreSymbolMissing / ...
        return _skip(
            "加载 libomninav_core.so 失败：%s" % exc,
            "先让 T1 产出它：cd firmware_core && make lib（产物 build/libomninav_core.so）",
            "然后：OMNINAV_CORE_SO=/绝对路径/libomninav_core.so python3 tools/run_e2e.py",
        )

    import tty

    from fake_stm32 import FakeStm32, FdTransport
    from stm32_bridge.bridge_node import Stm32Bridge

    checks = Checks()
    print("C 核心：%s" % core.describe())

    master_fd, slave_fd = os.openpty()
    tty.setraw(slave_fd)
    slave_name = os.ttyname(slave_fd)
    print("伪终端：master_fd=%d  slave=%s" % (master_fd, slave_name))

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
            print("桥启动失败：%s" % exc)
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

        # ---- 1) 链路自举：E 帧 -> /odom + TF --------------------------
        # ★ 这里要 pump 到 1.2 s 而不是 1.0 s：桥默认 send_reset_on_start=True，
        #   契约 §3 要求上电时 reset 连发 3 次、间隔 0.3 s，也就是在 0.9 s 还有一发。
        #   假 STM32 收到 reset 会把累计里程清零 —— 若那一发落在驱动阶段里，
        #   Δd 会突然变成负数，"position.x 单调不减" 那条断言就会莫名其妙地红。
        #   多等 0.3 s 把 reset 连发全部吃完，测试才谈得上确定性。
        pump(1.2)
        checks.check(
            "假 STM32 已连续发 E 帧",
            fake.e_frames_sent >= 10,
            "e_frames_sent=%d" % fake.e_frames_sent,
        )
        checks.check(
            "E 帧被解析并发成 /odom",
            len(odoms) >= 5,
            "收到 %d 条 /odom" % len(odoms),
        )
        checks.check(
            "TF odom->base_link 已发布",
            len(tfs) >= 5,
            "收到 %d 条 /tf" % len(tfs),
        )
        checks.check(
            "/odom 的 frame_id / child_frame_id 正确",
            all(m.header.frame_id == "odom" and m.child_frame_id == "base_link" for m in odoms),
            "odom=%s base=%s"
            % (
                odoms[0].header.frame_id if odoms else "(无)",
                odoms[0].child_frame_id if odoms else "(无)",
            ),
        )

        # ---- 2) /cmd_vel -> V 帧 ---------------------------------------
        baseline = len(odoms)
        command = Twist()
        command.linear.x = 0.2  # 200 mm/s
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            cmd_pub.publish(command)
            executor.spin_once(timeout_sec=0.05)
        checks.check(
            "/cmd_vel 0.2 m/s 被编码成 V,200,0,0",
            (200, 0, 0) in fake.v_frames,
            "假 STM32 收到的 V 帧（前 5 条）：%s" % (fake.v_frames[:5],),
        )

        # ---- 3) 里程计真的在推进 --------------------------------------
        driven = odoms[baseline:]
        checks.check(
            "走的过程中 /odom 持续发布",
            len(driven) >= 20,
            "驱动段收到 %d 条 /odom" % len(driven),
        )
        if driven:
            xs = [m.pose.pose.position.x for m in driven]
            monotonic = all(xs[i + 1] >= xs[i] - 1e-6 for i in range(len(xs) - 1))
            checks.check(
                "/odom position.x 单调不减",
                monotonic,
                "x: %.4f -> %.4f m" % (xs[0], xs[-1]),
            )
            checks.check(
                "1.5 s @ 200 mm/s 之后 x 至少 0.1 m",
                xs[-1] >= 0.1,
                "x=%.4f m（理论 ≈0.3 m，扣掉起步阶段的指令延迟）" % xs[-1],
            )
            twists = [m.twist.twist.linear.x for m in driven]
            mean_twist = sum(twists) / len(twists)
            checks.check(
                "/odom twist.linear.x 平均值 ≈ 0.2 m/s",
                abs(mean_twist - 0.2) <= 0.03,
                "均值 %.4f m/s（%d 个样本，min %.4f max %.4f）"
                % (mean_twist, len(twists), min(twists), max(twists)),
            )
            # ★ 回归护栏：树莓派实测出现过 max = 4.0000 m/s 的尖峰，根因是
            #   「用每帧实测 dt 归一化」——串口按块交付，一次 read() 返回好几条 E 帧时
            #   实测 dt 退化到 ~0，除法把 twist 放大 20 倍。现在改用协议名义周期
            #   （NOMINAL_ENCODER_PERIOD_S = 1/50 s），所以**最大值也必须贴着 0.2**。
            #   只查均值是抓不到这个 bug 的（当时均值 0.4185，只偏高 2 倍）。
            checks.check(
                "/odom twist.linear.x 最大值不超 0.25 m/s（防 dt 退化放大）",
                max(twists) <= 0.25,
                "max %.4f m/s（若 ≈ 0.2 的整数倍，说明 dt 被压小了）" % max(twists),
            )

        # ---- 4) TF 边界：只许 odom->base_link --------------------------
        pairs = set()
        for message in tfs:
            for transform in message.transforms:
                pairs.add((transform.header.frame_id, transform.child_frame_id))
        checks.check(
            "TF 里出现 odom->base_link",
            ("odom", "base_link") in pairs,
            "实到的 TF 对：%s" % sorted(pairs),
        )
        checks.check(
            "没有发 map->odom（那是 slam_toolbox / AMCL 的活）",
            ("map", "odom") not in pairs,
            "契约 §4 明确禁止",
        )
        checks.check(
            "没有发 base_link->laser（那是 URDF / robot_state_publisher 的活）",
            ("base_link", "laser") not in pairs,
            "契约 §4 明确禁止",
        )

        # ---- 5) cmd_vel 断流看门狗 -------------------------------------
        before_watchdog = len(fake.v_frames)
        pump(CMD_VEL_TIMEOUT_S + 0.5)
        after = fake.v_frames[before_watchdog:]
        checks.check(
            "断流 > %.1f s 后补发 V,0,0,0" % CMD_VEL_TIMEOUT_S,
            bool(after) and after[-1] == (0, 0, 0),
            "断流期间新增的 V 帧：%s" % (after[:5],),
        )

        # ---- 6) B 帧 -> /battery_state，A 帧 -> /diagnostics -----------
        checks.check(
            "B 帧发成了 /battery_state",
            len(batteries) >= 1,
            "收到 %d 条" % len(batteries),
        )
        if batteries:
            checks.check(
                "/battery_state 电压 ≈ 11.8 V",
                abs(batteries[-1].voltage - 11.8) <= 0.3,
                "voltage=%.3f V  location=%s" % (batteries[-1].voltage, batteries[-1].location),
            )
            checks.check(
                "percentage 故意是 NaN（契约没冻结电池曲线）",
                math.isnan(batteries[-1].percentage),
                "percentage=%r" % (batteries[-1].percentage,),
            )
        ack_names = {
            status.name for message in diags for status in message.status
        }
        checks.check(
            "A 帧发成了 /diagnostics",
            "stm32_bridge/stm32_ack" in ack_names,
            "诊断项：%s" % sorted(ack_names),
        )
        # ★ 回归护栏（树莓派实测：2.4 s 里发了 1475 条 /diagnostics，比 E 帧多 8 倍）。
        #   现在限速到 1 Hz 且同名项只留快照，整个用例期间应该只有个位数。
        checks.check(
            "/diagnostics 已限速（整个用例 <= 30 条）",
            len(diags) <= 30,
            "收到 %d 条（限速前实测 1475 条）" % len(diags),
        )
        # ★ 回归护栏（树莓派实测：不限速时下行 V 帧 2369 条，吃掉约 30% 的 115200 带宽）。
        #   本用例的有效下行时间约 2.5 s，20 Hz ⇒ 约 50 条；给到 120 留足余量。
        checks.check(
            "下行 V 帧已限速（<= 120 条）",
            len(fake.v_frames) <= 120,
            "共 %d 条（限速前实测 2369 条）" % len(fake.v_frames),
        )
        checks.check(
            "串口链路没有报错",
            fake.last_error is None,
            "fake.last_error=%r" % (fake.last_error,),
        )

        print(
            "\n流量统计：E 帧 %d、B 帧 %d、V 指令 %d、/odom %d、/tf %d、"
            "/battery_state %d、/diagnostics %d"
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
