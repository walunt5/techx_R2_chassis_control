from __future__ import annotations

import argparse

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from techx_r2_chassis_interfaces.action import StepCommand


class StepCommandClient(Node):
    def __init__(self) -> None:
        super().__init__("test_step_command_client")
        self._client = ActionClient(self, StepCommand, "/r2_chassis/step_command")

    def send_goal(self, args) -> None:
        goal = StepCommand.Goal()
        goal.cmd_type = args.cmd_type
        goal.from_block = args.from_block
        goal.target_block = args.target_block
        goal.current_h = args.current_h
        goal.target_h = args.target_h
        goal.edge_id = args.edge_id
        goal.timeout_sec = args.timeout_sec

        self.get_logger().info("Waiting for /r2_chassis/step_command action server...")
        self._client.wait_for_server()
        self.get_logger().info(
            f"Send goal cmd_type={goal.cmd_type} from={goal.from_block} to={goal.target_block} "
            f"current_h={goal.current_h} target_h={goal.target_h} edge_id={goal.edge_id}"
        )
        future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.get_logger().info(f"Feedback state={fb.state} message={fb.message}")

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


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Test /r2_chassis/step_command Action")
    parser.add_argument("cmd_type", nargs="?", default="CLIMB_200", choices=[
        "MOVE_FLAT", "CLIMB_200", "CLIMB_400", "DESCEND_200", "DESCEND_400"
    ])
    parser.add_argument("--from-block", default="A")
    parser.add_argument("--target-block", default="B")
    parser.add_argument("--current-h", type=int, default=0)
    parser.add_argument("--target-h", type=int, default=200)
    parser.add_argument("--edge-id", default="1")
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    return parser


def main(args=None) -> None:
    parser = build_arg_parser()
    parsed_args, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = StepCommandClient()
    node.send_goal(parsed_args)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
