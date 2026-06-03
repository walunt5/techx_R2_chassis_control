from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelPublisher(Node):
    def __init__(self, vx: float, vy: float, omega: float, rate_hz: float, duration_sec: float) -> None:
        super().__init__("test_cmd_vel_pub")
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self._vx = vx
        self._vy = vy
        self._omega = omega
        self._duration_sec = duration_sec
        self._start_time = time.monotonic()
        self._timer = self.create_timer(1.0 / max(1.0, rate_hz), self._timer_cb)

    def _timer_cb(self) -> None:
        elapsed = time.monotonic() - self._start_time
        if elapsed > self._duration_sec:
            self.get_logger().info("Finished publishing /cmd_vel")
            rclpy.shutdown()
            return
        msg = Twist()
        msg.linear.x = self._vx
        msg.linear.y = self._vy
        msg.angular.z = self._omega
        self._publisher.publish(msg)
        self.get_logger().info(f"Publish /cmd_vel vx={self._vx:.3f} vy={self._vy:.3f} omega={self._omega:.3f}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish test /cmd_vel")
    parser.add_argument("--vx", type=float, default=0.1, help="linear.x in m/s")
    parser.add_argument("--vy", type=float, default=0.0, help="linear.y in m/s")
    parser.add_argument("--omega", type=float, default=0.0, help="angular.z in rad/s")
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--duration-sec", type=float, default=3.0)
    return parser


def main(args=None) -> None:
    parser = build_arg_parser()
    parsed_args, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = CmdVelPublisher(parsed_args.vx, parsed_args.vy, parsed_args.omega, parsed_args.rate_hz, parsed_args.duration_sec)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
