#!/usr/bin/env python3
import sys
import threading
import termios
import tty
from select import select

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

os.environ.setdefault(
    "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{name}]: {message}"
)


class KeyboardTrigger(Node):
    def __init__(self):
        super().__init__("keyboard_trigger")
        self._raw_pub = self.create_publisher(String, "/keyboard_raw", 10)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_keys, daemon=True)
        self._thread.start()

    def _read_keys(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            self._ensure_newline()
            while not self._stop.is_set():
                ready, _, _ = select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue
                ch = sys.stdin.read(1)
                if ch in ("1", "3", "4"):
                    self._ensure_newline()
                    self._raw_pub.publish(String(data=ch))
                    self.get_logger().info(f"Raw key sent: {ch}")
                elif ch == " ":
                    self._ensure_newline()
                    self._raw_pub.publish(String(data="SPACE"))
                    self.get_logger().info("Raw key sent: SPACE")
                elif ch.lower() == "w":
                    self._ensure_newline()
                    self._raw_pub.publish(String(data="W"))
                    self.get_logger().info("Raw key sent: W")
                elif ch.lower() == "u":
                    self._ensure_newline()
                    self._raw_pub.publish(String(data="U"))
                    self.get_logger().info("Raw key sent: U")
                elif ch.lower() == "q":
                    self._ensure_newline()
                    self._raw_pub.publish(String(data="Q"))
                    self.get_logger().info("Raw key sent: Q")
                elif ch == "0":
                    self._ensure_newline()
                    self._raw_pub.publish(String(data="0"))
                    self.get_logger().info("Raw key sent: 0")
                elif ch.lower() == "x":
                    self._ensure_newline()
                    self.get_logger().info("Quit requested")
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _ensure_newline(self):
        try:
            sys.stdout.write("\r\n")
            sys.stdout.flush()
        except Exception:
            pass

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main():
    rclpy.init()
    node = KeyboardTrigger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
