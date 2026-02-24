import cv2
import socket
import numpy as np
from pathlib import Path

# UDP 설정
UDP_IP = "127.0.0.1"  # 수신자 IP
UDP_PORT = 5005
MAX_DGRAM = 2**16 # UDP 최대 패킷 크기 (65535)
INTRINSIC_NPZ = Path(
    "/home/jetcobot/yolo_ws/src/collect/camera_intrinsics/camera_calib_upgraded_2026-01-12_16-17-24.npz"
)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cap = cv2.VideoCapture("/dev/jetcocam0") # 웹캠
camera_matrix = None
dist_coeffs = None

if INTRINSIC_NPZ.exists():
    data = np.load(INTRINSIC_NPZ, allow_pickle=True)
    if "camera_matrix" in data.files and "dist_coeffs" in data.files:
        camera_matrix = np.asarray(data["camera_matrix"], dtype=np.float32).reshape(3, 3)
        dist_coeffs = np.asarray(data["dist_coeffs"], dtype=np.float32).reshape(-1)
        print(f"[INFO] Loaded intrinsics: {INTRINSIC_NPZ}")
    else:
        print(f"[WARN] Invalid intrinsic npz keys: {INTRINSIC_NPZ}")
else:
    print(f"[WARN] Intrinsic npz not found: {INTRINSIC_NPZ}")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    if camera_matrix is not None and dist_coeffs is not None:
        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
    
    # 이미지 크기 조절 (전송 속도 향상)
    frame = cv2.resize(frame, (640, 480))
    
    # JPEG 압축
    _, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    data = encoded.tobytes()
    
    # 패킷 분할 전송 (이미지가 크면 여러 패킷으로)
    size = len(data)
    for i in range(0, size, MAX_DGRAM - 100):
        # 헤더로 패킷 번호 등을 포함할 수 있으나 여기서는 단순화
        sock.sendto(data[i:i + MAX_DGRAM - 100], (UDP_IP, UDP_PORT))

cap.release()
