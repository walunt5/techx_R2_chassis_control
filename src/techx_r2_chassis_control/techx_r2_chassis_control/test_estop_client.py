from __future__ import annotations

import rclpy
from rclpy.node import Node
from techx_r2_chassis_interfaces.srv import EStop


class EStopClient(Node):
    def __init__(self) -> None:
        super().__init__("test_estop_client")
        self._client = self.create_client(EStop, "/r2_chassis/estop")

    def send(self) -> None:
        self._client.wait_for_service()
        req = EStop.Request()
        req.trigger = True
        future = self._client.call_async(req)
        future.add_done_callback(self._done)

    def _done(self, future) -> None:
        resp = future.result()
        self.get_logger().info(f"ESTOP response success={resp.success} message={resp.message}")
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EStopClient()
    node.send()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
