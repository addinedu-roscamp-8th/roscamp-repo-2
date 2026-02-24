import cv2
import socket
import numpy as np
from pathlib import Path
from ultralytics import YOLO

# YOLO 모델 로드 (nano 모델이 실시간에 적합)
model = YOLO(str(Path(__file__).resolve().parent / 'bolts-obb.pt'))

# UDP 소켓 설정
UDP_IP = "127.0.0.1" # 전체 인터페이스에서 수신
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("수신 시작...")

data = b''
while True:
    # 데이터 수신
    packet, addr = sock.recvfrom(2**16)
    
    # 실제 환경에서는 프레임의 끝을 알리는 신호(EOF)를 통해
    # 프레임을 조립해야 함. 여기서는 매우 단순화된 수신 방식.
    # 완벽한 패킷 조립을 위해 bytearray 사용 및 헤더 파싱 필요.
    
    # 예시: 임시 조립 (패킷 하나가 하나의 프레임이라 가정시)
    nparr = np.frombuffer(packet, np.uint8)
    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    
    if frame is not None:
        # YOLOv8 추론
        results = model(frame, show=True)
        
        # 결과 화면에 표시
        annotated_frame = results[0].plot()
        cv2.imshow("YOLOv8 UDP Reception", annotated_frame)
    
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
