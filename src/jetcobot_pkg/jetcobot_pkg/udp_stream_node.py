#!/usr/bin/env python3
import base64
import socket
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node


class UdpStreamNode(Node):
    def __init__(self):
        super().__init__("udp_stream_node")

        self.declare_parameter("host_ip", "0.0.0.0")
        self.declare_parameter("port", 9999)
        self.declare_parameter("buff_size", 65536)
        self.declare_parameter("camera_device", "/dev/jetcocam0")
        self.declare_parameter("frame_width", 400)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter(
            "intrinsics_path",
            "/home/jetcobot/robot_ws/src/jetcobot_pkg/jetcobot_pkg/calib_data/"
            "camera_calib_upgraded_2026-01-12_16-17-24.npz",
        )
        self.declare_parameter("show_preview", False)
        self.declare_parameter("fps_log_interval_sec", 2.0)

        self.host_ip = str(self.get_parameter("host_ip").value)
        self.port = int(self.get_parameter("port").value)
        self.buff_size = int(self.get_parameter("buff_size").value)
        self.camera_device = str(self.get_parameter("camera_device").value)
        self.frame_width = max(1, int(self.get_parameter("frame_width").value))
        self.jpeg_quality = int(np.clip(int(self.get_parameter("jpeg_quality").value), 1, 100))
        self.intrinsics_path = str(self.get_parameter("intrinsics_path").value)
        self.show_preview = bool(self.get_parameter("show_preview").value)
        self.fps_log_interval_sec = max(0.2, float(self.get_parameter("fps_log_interval_sec").value))

        self.server_socket = None
        self.cap = None
        self.K = None
        self.dist = None

    def _load_intrinsics(self):
        calib = np.load(self.intrinsics_path, allow_pickle=True)
        if "camera_matrix" not in calib or "dist_coeffs" not in calib:
            raise KeyError(
                "Intrinsics file must contain 'camera_matrix' and 'dist_coeffs'. "
                f"Available keys: {calib.files}"
            )

        K = calib["camera_matrix"]
        dist = calib["dist_coeffs"]

        if K.shape != (3, 3):
            raise ValueError(f"camera_matrix shape must be (3, 3), got {K.shape}")

        self.K = K.astype(np.float64)
        self.dist = dist.astype(np.float64)
        self.get_logger().info(f"Loaded intrinsics: {self.intrinsics_path}")

    def _open_udp(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buff_size)
        self.server_socket.bind((self.host_ip, self.port))
        self.server_socket.settimeout(0.5)
        self.get_logger().info(f"UDP listening at {(self.host_ip, self.port)}")

    def _open_camera(self):
        self.cap = cv2.VideoCapture(self.camera_device)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera: {self.camera_device}")
        self.get_logger().info(f"Camera opened: {self.camera_device}")

    def run(self):
        self._load_intrinsics()
        self._open_udp()
        self._open_camera()

        frame_count = 0
        last_fps_t = time.time()
        last_log_t = time.time()

        try:
            while rclpy.ok():
                try:
                    msg, client_addr = self.server_socket.recvfrom(self.buff_size)
                except socket.timeout:
                    continue
                except Exception as e:
                    self.get_logger().error(f"UDP recv failed: {e}")
                    continue

                self.get_logger().info(f"Client connected: {client_addr} msg={msg[:20]}")

                while rclpy.ok() and self.cap.isOpened():
                    ok, frame = self.cap.read()
                    if not ok:
                        self.get_logger().warn("Failed to read frame from camera")
                        break

                    undistorted = cv2.undistort(frame, self.K, self.dist)
                    h, w = undistorted.shape[:2]
                    new_h = max(1, int(h * (float(self.frame_width) / float(w))))
                    resized = cv2.resize(undistorted, (self.frame_width, new_h))

                    encoded, buffer = cv2.imencode(
                        ".jpg",
                        resized,
                        [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                    )
                    if not encoded:
                        self.get_logger().warn("JPEG encoding failed")
                        continue

                    try:
                        payload = base64.b64encode(buffer)
                        self.server_socket.sendto(payload, client_addr)
                    except Exception as e:
                        self.get_logger().warn(f"Send failed to {client_addr}: {e}")
                        break

                    frame_count += 1
                    now = time.time()
                    dt = now - last_fps_t
                    fps = (frame_count / dt) if dt > 1e-6 else 0.0

                    if now - last_log_t >= self.fps_log_interval_sec:
                        self.get_logger().info(f"Streaming to {client_addr}, fps={fps:.1f}")
                        last_log_t = now

                    if self.show_preview:
                        display = cv2.putText(
                            resized.copy(),
                            f"FPS: {fps:.1f}",
                            (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (0, 0, 255),
                            2,
                        )
                        cv2.imshow("UDP STREAM", display)
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord("q"):
                            self.get_logger().info("Preview requested shutdown with key q")
                            return

                    if dt >= 5.0:
                        frame_count = 0
                        last_fps_t = now
        finally:
            self.cleanup()

    def cleanup(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

        if self.server_socket is not None:
            try:
                self.server_socket.close()
            except Exception:
                pass
            self.server_socket = None

        if self.show_preview:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass


def main():
    rclpy.init()
    node = UdpStreamNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
