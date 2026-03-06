import ast

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool

from smartfactory_interfaces.action import ExecuteTask


class PickAndPlaceTriggerNode(Node):
    def __init__(self):
        super().__init__("pick_and_place_trigger_node")

        self.declare_parameter("start_topic", "/jetcobot/pick_and_place/start")
        self.declare_parameter("marker_ids", "[4, 5]")
        self.declare_parameter("reset_stack", True)

        self.start_topic = str(self.get_parameter("start_topic").value)
        marker_ids_raw = str(self.get_parameter("marker_ids").value)
        self.marker_ids = self._parse_marker_ids(marker_ids_raw)
        self.reset_stack = bool(self.get_parameter("reset_stack").value)

        self.action_client = ActionClient(self, ExecuteTask, "/task/execute")
        self._sending_goal = False

        self.sub = self.create_subscription(Bool, self.start_topic, self.start_cb, 10)
        self.get_logger().info(
            f"Listening on {self.start_topic}, marker_ids={self.marker_ids}, reset_stack={self.reset_stack}"
        )

    def _parse_marker_ids(self, raw: str):
        try:
            values = ast.literal_eval(raw)
            if isinstance(values, (list, tuple)) and all(isinstance(v, int) for v in values):
                return list(values)
        except (SyntaxError, ValueError):
            pass

        self.get_logger().warn("Invalid marker_ids parameter. Falling back to [4, 5].")
        return [4, 5]

    def start_cb(self, msg: Bool):
        if not msg.data:
            return

        if self._sending_goal:
            self.get_logger().warn("Goal request ignored: another goal is already in progress.")
            return

        self._sending_goal = True
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("ExecuteTask action server not available: /task/execute")
            self._sending_goal = False
            return

        goal_msg = ExecuteTask.Goal()
        goal_msg.marker_ids = self.marker_ids
        goal_msg.reset_stack = self.reset_stack

        self.get_logger().info("Start trigger received. Sending /task/execute goal.")
        send_future = self.action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"Failed to send goal: {exc}")
            self._sending_goal = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by /task/execute.")
            self._sending_goal = False
            return

        self.get_logger().info("Goal accepted by /task/execute.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        try:
            wrapped = future.result()
            result = wrapped.result
            status = wrapped.status
            self.get_logger().info(
                f"Goal completed: status={status}, success={result.success}, message='{result.message}'"
            )
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"Failed to get goal result: {exc}")
        finally:
            self._sending_goal = False


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceTriggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
