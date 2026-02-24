import cv2 as cv
import numpy as np
import os
import datetime

save_dir = os.path.expanduser('~/mycobot_ws/camera_capture')
os.makedirs(save_dir, exist_ok=True)

index = 0
cap = cv.VideoCapture(2)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera frame not received")
        break

    cv.imshow('frame', frame)

    key = cv.waitKey(20) & 0xFF

    if key == ord('p'): ##p 누르면 캡처 저장
        now = datetime.datetime.now()
        filename = now.strftime("%Y-%m-%d_%H-%M-%S") + ".jpg"
        filepath = os.path.join(save_dir, filename)
        cv.imwrite(filepath, frame)
        print(f"Saved: {filepath}")

    elif key == ord('q'):  # q눌러 종료
        break

cap.release()
cv.destroyAllWindows()