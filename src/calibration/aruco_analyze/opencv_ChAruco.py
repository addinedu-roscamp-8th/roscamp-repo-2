import cv2
import numpy as np

def main():
    # ============================
    # 1) ChArUco 보드 파라미터 설정
    # ============================
    # DICT_4X4_50 ~ DICT_6X6_250 등 사용 가능
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    # 보드 격자 수 (사각형 개수 기준)
    squares_x = 5
    squares_y = 7

    # 실제 보드의 길이 단위(미터/센티미터/밀리미터 등 아무거나 OK, 일관성만 유지)
    square_length = 0.039  # 3cm
    marker_length = 0.023  # 2cm (square_length보다 작아야 함)

    # CharucoBoard 생성
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y),
        square_length,
        marker_length,
        aruco_dict
    )

    # ============================
    # 2) ArUco Detector 준비 (OpenCV 버전 대응)
    # ============================
    # OpenCV 4.7+ 에서 DetectorParameters / ArucoDetector 사용 가능
    try:
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        use_new_api = True
    except Exception:
        params = cv2.aruco.DetectorParameters_create()
        use_new_api = False

    # ============================
    # 3) 카메라 열기
    # ============================
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 카메라를 열 수 없습니다. (VideoCapture(0) 확인)")
        return

    # (선택) 해상도 지정 - 카메라가 지원하는 범위에서만 적용됨
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("✅ 실행 중... 'q'를 누르면 종료됩니다.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 프레임을 읽지 못했습니다.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ============================
        # 4) ArUco 마커 검출
        # ============================
        if use_new_api:
            corners, ids, rejected = detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

        # 검출 결과를 그릴 표시용 이미지
        vis = frame.copy()

        if ids is not None and len(ids) > 0:
            # 마커 박스 그리기
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)

            # ============================
            # 5) ChArUco 코너 보간(Interpolation)
            # ============================
            # charuco_corners: (N,1,2), charuco_ids: (N,1)
            # 코너 개수가 충분해야 안정적
            retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                markerCorners=corners,
                markerIds=ids,
                image=gray,
                board=board
            )

            if retval is not None and retval > 0 and charuco_corners is not None:
                # ChArUco 코너 그리기 (빨간 점)
                cv2.aruco.drawDetectedCornersCharuco(
                    vis, charuco_corners, charuco_ids, (0, 0, 255)
                )

                # ============================
                # 6) (옵션) 보드 Pose 추정 + 축(axis) 그리기
                # ============================
                # Pose 추정을 하려면 카메라 내부파라미터(cameraMatrix, distCoeffs)가 필요합니다.
                # 아래는 임시로 "포즈 추정 생략" 상태이며, 캘리브레이션 후 넣으면 바로 동작합니다.

                # 예시: 이미 캘리브레이션 된 값이 있다면 아래처럼 사용
                # cameraMatrix = np.array([[fx, 0, cx],
                #                          [0, fy, cy],
                #                          [0,  0,  1]], dtype=np.float64)
                # distCoeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float64)
                
                # ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                #     charuco_corners, charuco_ids, board, cameraMatrix, distCoeffs, None, None
                # )
                # if ok:
                #     cv2.drawFrameAxes(vis, cameraMatrix, distCoeffs, rvec, tvec, 0.05)  # 5cm 축

        cv2.imshow("ChArUco Detection", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

