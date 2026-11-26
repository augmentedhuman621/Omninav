"""``stm32_bridge.wheel_joint_states`` -- publish ``/joint_states`` so RViz2 shows spinning wheels.

Display-only companion of the bridge simulation:

  ``/cmd_vel``  ->  C core ``mecanum_inverse()``  ->  wheel angular rates  ->  integrated angles
                                                                      |
                                                              ``/joint_states`` (50 Hz)

``robot_state_publisher`` turns ``/joint_states`` into the four wheel TFs, so RViz2 animates the
wheels while Gazebo keeps moving the body with the ``planar_move`` plug-in.

It never publishes ``/odom`` and never touches the serial link: the bridge stays the single owner
of odometry, and this node only makes the picture look right. When ``/cmd_vel`` goes quiet for
``cmd_vel_timeout`` seconds the wheels stop, mirroring the bridge's own watchdog.

Run it (inside the bridge simulation):

    ros2 run stm32_bridge wheel_joint_states
    # optional: --ros-args -p cmd_vel_timeout:=0.5 -p rate_hz:=50.0
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

from stm32_bridge.wheel_angles import DEFAULT_WHEEL_RADIUS_M, WHEEL_JOINTS, WheelAngleIntegrator

#: Defaults, matching the contract constants used by the bridge (cmd_vel watchdog 0.300 s).
DEFAULT_CMD_VEL_TIMEOUT_S = 0.3
DEFAULT_RATE_HZ = 50.0
#: Longest integration step we accept; anything bigger means the node was stalled and a
#: large jump would just make the wheels flicker.
MAX_STEP_S = 0.5


class WheelJointStates(Node):
    """Convert ``/cmd_vel`` into wheel joint angles and publish ``/joint_states``."""

    def __init__(self) -> None:
        super().__init__("wheel_joint_states")

        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("rate_hz", DEFAULT_RATE_HZ)
        self.declare_parameter("wheel_radius_m", DEFAULT_WHEEL_RADIUS_M)
        self.declare_parameter("cmd_vel_timeout", DEFAULT_CMD_VEL_TIMEOUT_S)

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        radius = float(self.get_parameter("wheel_radius_m").value)
        self._timeout_s = float(self.get_parameter("cmd_vel_timeout").value)

        if not rate_hz > 0.0:
            raise ValueError("rate_hz must be > 0, got %r" % (rate_hz,))

        self._integrator = WheelAngleIntegrator(wheel_radius_m=radius)
        self._twist_mm = (0, 0, 0)
        self._last_cmd_ns: Optional[int] = None
        self._last_tick_ns: Optional[int] = None

        self._publisher = self.create_publisher(JointState, joint_states_topic, 10)
        self._subscription = self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            "wheel animation: %s -> %s, radius %.6f m, L = %d mm (core: %s)"
            % (cmd_vel_topic, joint_states_topic, self._integrator.wheel_radius_m,
               self._integrator.l_mm, self._integrator.core_description)
        )

    # ------------------------------------------------------------------

    def _on_cmd_vel(self, msg: Twist) -> None:
        # m/s -> mm/s and rad/s -> mrad/s, the integer units frozen by the contract.
        vx_mm_s = int(round(float(msg.linear.x) * 1000.0))
        vy_mm_s = int(round(float(msg.linear.y) * 1000.0))
        omega_mrad_s = int(round(float(msg.angular.z) * 1000.0))
        self._twist_mm = (vx_mm_s, vy_mm_s, omega_mrad_s)
        self._last_cmd_ns = self.get_clock().now().nanoseconds

    def _current_twist(self, now_ns: int) -> Sequence[int]:
        if self._last_cmd_ns is None:
            return (0, 0, 0)
        if (now_ns - self._last_cmd_ns) / 1e9 > self._timeout_s:
            return (0, 0, 0)  # same watchdog behaviour as the bridge
        return self._twist_mm

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_tick_ns is None:
            dt_s = 0.0
        else:
            dt_s = (now_ns - self._last_tick_ns) / 1e9
            if not math.isfinite(dt_s) or dt_s > MAX_STEP_S:
                dt_s = 0.0
        self._last_tick_ns = now_ns

        vx_mm_s, vy_mm_s, omega_mrad_s = self._current_twist(now_ns)
        angles: List[float] = self._integrator.step(vx_mm_s, vy_mm_s, omega_mrad_s, dt_s)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(WHEEL_JOINTS)
        msg.position = angles
        msg.velocity = self._integrator.rates
        self._publisher.publish(msg)


def main(argv: Optional[Sequence[str]] = None) -> int:
    rclpy.init(args=list(argv) if argv is not None else None)
    node = None
    try:
        node = WheelJointStates()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
