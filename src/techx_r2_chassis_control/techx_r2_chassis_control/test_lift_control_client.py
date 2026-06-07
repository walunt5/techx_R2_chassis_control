from __future__ import annotations

import argparse
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from techx_r2_chassis_interfaces.action import LiftControl


class TestLiftControlClient(Node):
    def __init__(self) -> None:
        super().__init__("test_lift_control_client")
        self._client = ActionClient(self, LiftControl, "/r2_chassis/lift_control")

    def send_goal(self, args) -> None:
        self.get_logger().info("Waiting for /r2_chassis/lift_control action server...")
        self._client.wait_for_server()

        goal = LiftControl.Goal()
        goal.target_h_mm = args.target_h_mm
        goal.mask = args.mask
        goal.timeout_sec = args.timeout_sec

        self.get_logger().info(
            f"Send lift goal target_h={goal.target_h_mm} mask=0x{goal.mask:02X}"
        )

        future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Feedback state={fb.state} "
            f"lift=({fb.lift1_mm},{fb.lift2_mm},{fb.lift3_mm}) "
            f"message={fb.message}"
        )

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        result = future.result().result
        self.get_logger().info(
            f"Result success={result.success} final_state={result.final_state} "
            f"error_code=0x{result.error_code:04X} message={result.message}"
        )
        rclpy.shutdown()


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-h-mm", type=int, required=True)
    parser.add_argument("--mask", type=lambda x: int(x, 0), default=0x07)
    parser.add_argument("--timeout-sec", type=float, default=10.0)

    parsed, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = TestLiftControlClient()
    node.send_goal(parsed)
    rclpy.spin(node)


if __name__ == "__main__":
    main()