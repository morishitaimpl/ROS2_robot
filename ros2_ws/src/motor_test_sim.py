#!/usr/bin/env python3
"""
Simulation helper for this project.

Raspbot (mecanum) in Gazebo is controlled via:
  /model/raspbot/cmd_vel  (geometry_msgs/msg/Twist)

This script publishes cmd_vel for N seconds and then stops.

Examples:
  # forward for 1.0s
  python3 motor_test_sim.py --seconds 1.0 --vx 0.2

  # strafe left for 2.0s (mecanum)
  python3 motor_test_sim.py --seconds 2.0 --vy 0.2

  # rotate in place
  python3 motor_test_sim.py --seconds 1.5 --wz 0.8

  # compatibility-ish: use --speed/--direction like your HW test
  python3 motor_test_sim.py --seconds 1.0 --speed 120 --direction 0
"""

from __future__ import annotations

import argparse
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist


CMD_VEL_TOPIC = "/model/raspbot/cmd_vel"


def _speed_to_vx(speed: int) -> float:
    # Map [0..255] roughly to [0..0.6] m/s (matches our model limits)
    s = max(0, min(255, int(speed)))
    return 0.6 * (s / 255.0)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=1.0, help="Duration to drive")
    parser.add_argument("--rate", type=float, default=10.0, help="Publish rate (Hz)")
    parser.add_argument(
        "--topic",
        type=str,
        default=CMD_VEL_TOPIC,
        help="cmd_vel topic (default matches raspbot_sim bridge)",
    )
    parser.add_argument(
        "--wait-subscriber",
        type=float,
        default=2.0,
        help="Wait up to N seconds for a subscriber (bridge) to appear; 0 disables",
    )

    # Direct Twist
    parser.add_argument("--vx", type=float, default=0.0, help="linear.x (m/s)")
    parser.add_argument("--vy", type=float, default=0.0, help="linear.y (m/s)")
    parser.add_argument("--wz", type=float, default=0.0, help="angular.z (rad/s)")

    # Hardware-style args (optional)
    parser.add_argument("--speed", type=int, default=None, help="0-255 (mapped to vx)")
    parser.add_argument(
        "--direction",
        type=int,
        default=None,
        help="0=fwd, 1=back, 2=left(strafe), 3=right(strafe), 4=ccw, 5=cw",
    )
    args = parser.parse_args(argv)

    vx, vy, wz = args.vx, args.vy, args.wz

    # If direction/speed specified, override vx/vy/wz
    if args.speed is not None or args.direction is not None:
        base_v = _speed_to_vx(args.speed or 0)
        d = int(args.direction or 0)
        vx = vy = wz = 0.0
        if d == 0:
            vx = base_v
        elif d == 1:
            vx = -base_v
        elif d == 2:
            vy = base_v
        elif d == 3:
            vy = -base_v
        elif d == 4:
            wz = abs(base_v) * 3.0
        elif d == 5:
            wz = -abs(base_v) * 3.0

    rclpy.init(args=None)
    node = rclpy.create_node("motor_test_sim")
    topic = str(args.topic)
    pub = node.create_publisher(Twist, topic, 10)

    msg = Twist()
    msg.linear.x = float(vx)
    msg.linear.y = float(vy)
    msg.angular.z = float(wz)

    period = 1.0 / max(1e-6, float(args.rate))
    t0 = time.monotonic()

    node.get_logger().info(
        f"Publishing {topic}: vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, wz={msg.angular.z:.3f} for {args.seconds:.2f}s"
    )

    # Helpful diagnostics: if the ROS<->Gazebo bridge isn't up, there may be 0 subscribers.
    if float(args.wait_subscriber) > 0:
        deadline = time.monotonic() + float(args.wait_subscriber)
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if pub.get_subscription_count() == 0:
            node.get_logger().warning(
                "No subscribers detected for cmd_vel. "
                "Is `ros2 launch raspbot_sim sim.launch.py` running (including ros_gz_bridge)?"
            )

    try:
        while time.monotonic() - t0 < float(args.seconds):
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    finally:
        stop = Twist()
        pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.0)
        node.get_logger().info("Stopped (published zero cmd_vel).")
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

