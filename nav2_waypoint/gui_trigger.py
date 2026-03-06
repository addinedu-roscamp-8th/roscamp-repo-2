#!/usr/bin/env python3
import os

os.environ.setdefault(
    "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{name}]: {message}"
)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32

class GuiTrigger(Node):
    def __init__(self):
        super().__init__("gui_trigger")
        self._move_pub = self.create_publisher(String, "/move_role", 10)
        self._load_pub = self.create_publisher(Bool, "/load_done", 10)
        self._unload_pub = self.create_publisher(Bool, "/unload_done", 10)
        self.create_subscription(String, "/keyboard_raw", self._raw_cb, 10)
        self.create_subscription(Int32, "/arrived_point", self._arrived_cb, 10)

    def _raw_cb(self, msg):
        cmd = msg.data.strip()
        if not cmd:
            return
        if cmd in ("1", "3", "4", "0"):
            self._move_pub.publish(String(data=cmd))
            self.get_logger().info(f"Move role sent: {cmd}")
            return
        if cmd == "SPACE":
            self._load_pub.publish(Bool(data=True))
            self.get_logger().info("Load done sent: True")
            return
        if cmd.upper() == "U":
            self._unload_pub.publish(Bool(data=True))
            self.get_logger().info("Unload done sent: True")
            return

    def _arrived_cb(self, msg):
        point_id = msg.data
        self.get_logger().info(f"Arrived point received: {point_id}")


def main():
    rclpy.init()
    node = GuiTrigger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
