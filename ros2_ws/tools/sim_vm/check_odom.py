#!/usr/bin/env python3
"""Count /odom messages and print the last pose (no ROS CLI / daemon needed)."""

import sys
import time

import rclpy
from nav_msgs.msg import Odometry

WINDOW_S = 4.0


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("odom_check")
    msgs = []
    node.create_subscription(Odometry, "/odom", msgs.append, 10)

    t0 = time.monotonic()
    while time.monotonic() - t0 < WINDOW_S:
        rclpy.spin_once(node, timeout_sec=0.1)

    names = node.get_node_names()
    print("nodes=%s" % ",".join(sorted(names)))
    topics = [name for name, _types in node.get_topic_names_and_types()]
    wanted = [t for t in sorted(topics) if any(k in t for k in ("odom", "cmd_vel", "scan", "clock"))]
    print("topics_of_interest=%s" % ",".join(wanted))
    print("odom_count=%d window=%.1fs" % (len(msgs), WINDOW_S))
    if msgs:
        pose = msgs[-1].pose.pose
        twist = msgs[-1].twist.twist
        print(
            "last_pose x=%.4f y=%.4f z=%.4f  yaw_quat z=%.4f w=%.4f"
            % (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.z,
                pose.orientation.w,
            )
        )
        print(
            "last_twist vx=%.4f vy=%.4f wz=%.4f"
            % (twist.linear.x, twist.linear.y, twist.angular.z)
        )
        stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in msgs]
        if len(stamps) > 1 and stamps[-1] > stamps[0]:
            print("header_rate_hz=%.2f" % ((len(stamps) - 1) / (stamps[-1] - stamps[0])))

    node.destroy_node()
    rclpy.shutdown()
    return 0 if msgs else 1


if __name__ == "__main__":
    sys.exit(main())
