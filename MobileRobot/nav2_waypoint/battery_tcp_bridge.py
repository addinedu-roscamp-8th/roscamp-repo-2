#!/usr/bin/env python3
import socket
import threading
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

'''
현재 상태 
    이동 : 좌표값이 변환중임 / 미션을 하는 중임
    대기 : 이동이 없음 좌표값 변화 / 미션을 안함
    충전 : 현재있는 위치가 대기장소

연결 상태 : 온 / 오프

배터리 : 몇프로
'''

class BatteryTcpBridge(Node):
    def __init__(self):
        super().__init__("battery_tcp_bridge")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 9001)
        self.declare_parameter("topic", "/battery_state")
        self.declare_parameter("send_rate_hz", 2.0)

        self._latest = None
        self._tcp_clients = []
        self._lock = threading.Lock()

        topic = self.get_parameter("topic").value
        self.create_subscription(BatteryState, topic, self._battery_cb, 10)

        self._server_thread = threading.Thread(target=self._server_loop, daemon=True)
        self._server_thread.start()

        rate = float(self.get_parameter("send_rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._broadcast_latest)

    def _battery_cb(self, msg: BatteryState):
        self._latest = msg

    def _server_loop(self):
        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen(5)
        self.get_logger().info(f"TCP server listening on {host}:{port}")
        while rclpy.ok():
            try:
                client, addr = srv.accept()
                client.setblocking(True)
                with self._lock:
                    self._tcp_clients.append(client)
                self.get_logger().info(f"Client connected: {addr}")
            except Exception:
                time.sleep(0.1)

    def _broadcast_latest(self):
        if self._latest is None:
            return
        data = {
            "voltage": float(self._latest.voltage),
            "current": float(self._latest.current),
            "percentage": float(self._latest.percentage),
        }
        payload = (json.dumps(data) + "\n").encode("utf-8")
        dead = []
        with self._lock:
            for c in self._tcp_clients:
                try:
                    c.sendall(payload)
                except Exception:
                    dead.append(c)
            for c in dead:
                try:
                    c.close()
                except Exception:
                    pass
                self._tcp_clients.remove(c)


def main():
    rclpy.init()
    node = BatteryTcpBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
