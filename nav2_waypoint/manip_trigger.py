#!/usr/bin/env python3
import os

os.environ.setdefault(
    "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{name}]: {message}"
)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32


class ManipTrigger(Node):
    def __init__(self):
        super().__init__("manip_trigger")
        self._manip_pub = self.create_publisher(Bool, "/pick_and_place/done", 10)
        self.create_subscription(String, "/keyboard_raw", self._raw_cb, 10)
        self.create_subscription(Int32, "/arrived_point", self._arrived_cb, 10)

    def _raw_cb(self, msg):
        cmd = msg.data.strip()
        if not cmd:
            return
        if cmd.lower() == "q":
            self._manip_pub.publish(Bool(data=True))
            self.get_logger().info("Manipulator done sent: True")

    def _arrived_cb(self, msg):
        point_id = msg.data
        self.get_logger().info(f"Arrived point received: {point_id}")


def main():
    rclpy.init()
    node = ManipTrigger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
