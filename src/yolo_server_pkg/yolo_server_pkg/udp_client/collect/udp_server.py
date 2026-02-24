import base64
import socket
import time

import cv2
import imutils
import numpy as np

BUFF_SIZE = 65536
HOST_IP = "192.168.0.52"
PORT = 9999
CAMERA_DEVICE = "/dev/jetcocam0"
FRAME_WIDTH = 400
JPEG_QUALITY = 80
INTRINSICS_PATH = "/home/jetcobot/yolo_ws/src/collect/camera_intrinsics/camera_calib_upgraded_2026-01-12_16-17-24.npz"


def create_udp_server(host_ip: str, port: int, buff_size: int = BUFF_SIZE) -> socket.socket:
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, buff_size)
    server_socket.bind((host_ip, port))
    print(f"[INFO] Listening at: {(host_ip, port)}")
    return server_socket


def load_intrinsics(npz_path: str) -> tuple[np.ndarray, np.ndarray]:
    calib = np.load(npz_path, allow_pickle=True)
    if "camera_matrix" not in calib or "dist_coeffs" not in calib:
        raise KeyError(
            "Intrinsics file must contain 'camera_matrix' and 'dist_coeffs'. "
            f"Available keys: {calib.files}"
        )

    camera_matrix = calib["camera_matrix"]
    dist_coeffs = calib["dist_coeffs"]

    if camera_matrix.shape != (3, 3):
        raise ValueError(
            f"camera_matrix shape must be (3, 3), but got {camera_matrix.shape}"
        )

    print(f"[INFO] Loaded intrinsics from: {npz_path}")
    return camera_matrix, dist_coeffs


def stream_video_over_udp(
    server_socket: socket.socket,
    camera_device: str,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> None:
    vid = cv2.VideoCapture(camera_device)
    if not vid.isOpened():
        raise RuntimeError(f"Cannot open camera: {camera_device}")

    fps, frames_to_count, cnt = 0, 20, 0
    st = time.time()

    try:
        while True:
            msg, client_addr = server_socket.recvfrom(BUFF_SIZE)
            print(f"[INFO] Got connection from {client_addr}, msg={msg[:20]}")

            while vid.isOpened():
                ok, frame = vid.read()
                if not ok:
                    print("[WARN] Failed to read frame from camera.")
                    break

                undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
                resized = imutils.resize(undistorted, width=FRAME_WIDTH)

                encoded, buffer = cv2.imencode(
                    ".jpg",
                    resized,
                    [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
                )
                if not encoded:
                    print("[WARN] JPEG encoding failed.")
                    continue

                message = base64.b64encode(buffer)
                server_socket.sendto(message, client_addr)

                display_frame = cv2.putText(
                    resized,
                    f"FPS: {fps}",
                    (10, 40),
                    cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
                cv2.imshow("TRANSMITTING VIDEO (UNDISTORTED)", display_frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    print("[INFO] Shutdown requested by keyboard input.")
                    return

                if cnt == frames_to_count:
                    elapsed = time.time() - st
                    if elapsed > 0:
                        fps = round(frames_to_count / elapsed)
                    st = time.time()
                    cnt = 0
                cnt += 1
    finally:
        vid.release()
        cv2.destroyAllWindows()


def main() -> None:
    print(f"[INFO] Host IP: {HOST_IP}")
    server_socket = create_udp_server(HOST_IP, PORT)
    try:
        camera_matrix, dist_coeffs = load_intrinsics(INTRINSICS_PATH)
        stream_video_over_udp(
            server_socket=server_socket,
            camera_device=CAMERA_DEVICE,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
        )
    finally:
        server_socket.close()
        print("[INFO] Server socket closed.")


if __name__ == "__main__":
    main()
