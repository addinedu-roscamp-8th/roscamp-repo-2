#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from pinkylib import Battery


class BatteryCmdPublisher(Node):
    def __init__(self):
        super().__init__("battery_cmd_publisher")
        self.declare_parameter("topic", "/battery_state")
        self.declare_parameter("rate_hz", 1.0)

        topic = self.get_parameter("topic").value
        self._pub = self.create_publisher(BatteryState, topic, 10)
        self._battery = Battery()

        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._tick)

    def _tick(self):
        try:
            pct = float(self._battery.battery_percentage()) / 100.0
            voltage = float(self._battery.get_voltage())
        except Exception as exc:
            self.get_logger().warn(f"battery read failed: {exc}")
            return
        msg = BatteryState()
        msg.voltage = voltage
        msg.percentage = pct
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = BatteryCmdPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
