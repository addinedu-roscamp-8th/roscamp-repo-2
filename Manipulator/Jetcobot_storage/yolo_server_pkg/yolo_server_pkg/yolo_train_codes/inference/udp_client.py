import base64
import socket
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from ultralytics import YOLO

BUFF_SIZE = 65536
HOST_IP = "192.168.0.52"
PORT = 9999
START_MESSAGE = b"Hello"
MODEL_PATH = Path(__file__).resolve().parent / "bolts-obb.pt"
CALIB_PATH = (
    Path(__file__).resolve().parent
    / "camera_calib_upgraded_2026-01-12_16-17-24.npz"
)
MODEL_TASK = "obb"
CONF_THRES = 0.25

# Real object size in meters (update to match your object).
OBJECT_WIDTH_M = 0.06
OBJECT_HEIGHT_M = 0.04

WINDOW_NAME = "UDP OBB CLIENT"
DISPLAY_SCALE = 1.5


def load_intrinsics(calib_path: Path) -> tuple[np.ndarray, np.ndarray]:
    if not calib_path.exists():
        raise FileNotFoundError(f"Calibration file not found: {calib_path}")

    calib = np.load(str(calib_path))
    camera_matrix = calib["camera_matrix"].astype(np.float64)
    dist_coeffs = calib["dist_coeffs"].astype(np.float64)

    if dist_coeffs.ndim == 1:
        dist_coeffs = dist_coeffs.reshape(-1, 1)
    elif dist_coeffs.ndim == 2 and dist_coeffs.shape[0] == 1:
        dist_coeffs = dist_coeffs.reshape(-1, 1)

    return camera_matrix, dist_coeffs


def order_points_tl_tr_br_bl(pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, dtype=np.float64)
    if pts.shape != (4, 2):
        raise ValueError(f"Expected (4,2), got {pts.shape}")

    rect = np.zeros((4, 2), dtype=np.float64)
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]

    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect


def extract_detection_candidates(result_obj: Any, conf_thres: float) -> list[dict[str, Any]]:
    candidates: list[dict[str, Any]] = []
    names = getattr(result_obj, "names", {})

    if getattr(result_obj, "obb", None) is not None and len(result_obj.obb) > 0:
        obb_xyxyxyxy = result_obj.obb.xyxyxyxy.cpu().numpy()
        confs = (
            result_obj.obb.conf.cpu().numpy()
            if getattr(result_obj.obb, "conf", None) is not None
            else np.ones((len(obb_xyxyxyxy),), dtype=np.float64)
        )
        clses = (
            result_obj.obb.cls.cpu().numpy().astype(int)
            if getattr(result_obj.obb, "cls", None) is not None
            else np.full((len(obb_xyxyxyxy),), -1, dtype=int)
        )

        for det_idx, (pts, conf, cls_id) in enumerate(zip(obb_xyxyxyxy, confs, clses)):
            if float(conf) < conf_thres:
                continue
            ordered = order_points_tl_tr_br_bl(pts.reshape(4, 2))
            label = (
                names.get(int(cls_id), str(int(cls_id)))
                if isinstance(names, dict) and cls_id >= 0
                else "unknown"
            )
            candidates.append(
                {
                    "det_idx": int(det_idx),
                    "mode": "obb",
                    "conf": float(conf),
                    "cls_id": int(cls_id),
                    "label": label,
                    "img_points": ordered,
                }
            )
        return candidates

    if getattr(result_obj, "boxes", None) is not None and len(result_obj.boxes) > 0:
        xyxy = result_obj.boxes.xyxy.cpu().numpy()
        confs = (
            result_obj.boxes.conf.cpu().numpy()
            if getattr(result_obj.boxes, "conf", None) is not None
            else np.ones((len(xyxy),), dtype=np.float64)
        )
        clses = (
            result_obj.boxes.cls.cpu().numpy().astype(int)
            if getattr(result_obj.boxes, "cls", None) is not None
            else np.full((len(xyxy),), -1, dtype=int)
        )

        for det_idx, (bbox, conf, cls_id) in enumerate(zip(xyxy, confs, clses)):
            if float(conf) < conf_thres:
                continue
            x1, y1, x2, y2 = bbox.astype(np.float64)
            pts = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float64)
            ordered = order_points_tl_tr_br_bl(pts)
            label = (
                names.get(int(cls_id), str(int(cls_id)))
                if isinstance(names, dict) and cls_id >= 0
                else "unknown"
            )
            candidates.append(
                {
                    "det_idx": int(det_idx),
                    "mode": "bbox",
                    "conf": float(conf),
                    "cls_id": int(cls_id),
                    "label": label,
                    "img_points": ordered,
                }
            )

    return candidates


def estimate_poses(
    detections: list[dict[str, Any]],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> list[dict[str, Any]]:
    w = float(OBJECT_WIDTH_M)
    h = float(OBJECT_HEIGHT_M)
    object_points = np.array(
        [
            [-w / 2, -h / 2, 0.0],
            [w / 2, -h / 2, 0.0],
            [w / 2, h / 2, 0.0],
            [-w / 2, h / 2, 0.0],
        ],
        dtype=np.float64,
    )

    poses: list[dict[str, Any]] = []
    for det in detections:
        image_points = det["img_points"].astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            objectPoints=object_points,
            imagePoints=image_points,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            continue

        tx, ty, tz = tvec.reshape(3)
        poses.append(
            {
                **det,
                "image_points": image_points,
                "rvec": rvec,
                "tvec": tvec,
                "xyz": (float(tx), float(ty), float(tz)),
                "distance": float(np.linalg.norm(tvec)),
            }
        )

    return poses


def draw_pose_overlay(
    frame: np.ndarray,
    poses: list[dict[str, Any]],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> np.ndarray:
    vis = frame.copy()
    axis_len = min(OBJECT_WIDTH_M, OBJECT_HEIGHT_M) * 0.7
    axis_3d = np.array(
        [
            [0.0, 0.0, 0.0],
            [axis_len, 0.0, 0.0],
            [0.0, axis_len, 0.0],
            [0.0, 0.0, -axis_len],
        ],
        dtype=np.float64,
    )
    palette = [
        (0, 255, 255),
        (255, 255, 0),
        (255, 0, 255),
        (0, 255, 0),
        (0, 128, 255),
        (255, 128, 0),
    ]

    for i, pose in enumerate(poses):
        color = palette[i % len(palette)]
        poly = pose["image_points"].astype(np.int32)

        cv2.polylines(vis, [poly], isClosed=True, color=color, thickness=2)
        for p in poly:
            cv2.circle(vis, tuple(p), 3, color, -1)

        proj, _ = cv2.projectPoints(
            axis_3d, pose["rvec"], pose["tvec"], camera_matrix, dist_coeffs
        )
        proj = proj.reshape(-1, 2).astype(np.int32)
        p0, px, py, pz = proj
        cv2.line(vis, tuple(p0), tuple(px), (0, 0, 255), 2)
        cv2.line(vis, tuple(p0), tuple(py), (0, 255, 0), 2)
        cv2.line(vis, tuple(p0), tuple(pz), (255, 0, 0), 2)

        tx, ty, tz = pose["xyz"]
        label = (
            f"id={i} {pose['label']} "
            f"X={tx:.3f} Y={ty:.3f} Z={tz:.3f} "
            f"c={pose['conf']:.2f}"
        )
        anchor = tuple(np.mean(poly, axis=0).astype(int))
        text_pos = (max(0, anchor[0] - 30), max(15, anchor[1] - 12))
        cv2.putText(
            vis,
            label,
            text_pos,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            2,
        )

    return vis


def main() -> None:
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"Model file not found: {MODEL_PATH}")

    camera_matrix, dist_coeffs = load_intrinsics(CALIB_PATH)
    model = YOLO(str(MODEL_PATH))
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BUFF_SIZE)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    client_socket.sendto(START_MESSAGE, (HOST_IP, PORT))
    print(f"[INFO] UDP client started: {HOST_IP}:{PORT}")
    print(f"[INFO] OBB model loaded: {MODEL_PATH}")
    print(f"[INFO] Calibration loaded: {CALIB_PATH}")

    recv_fps, infer_fps = 0, 0
    recv_st, infer_st = time.time(), time.time()
    frames_to_count, recv_cnt, infer_cnt = 20, 0, 0

    try:
        while True:
            packet, _ = client_socket.recvfrom(BUFF_SIZE)

            try:
                data = base64.b64decode(packet, " /")
            except Exception:
                continue

            npdata = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(npdata, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            result = model.predict(
                source=frame, task=MODEL_TASK, conf=CONF_THRES, verbose=False
            )[0]
            annotated_frame = result.plot()

            detections = extract_detection_candidates(result, CONF_THRES)
            poses = estimate_poses(detections, camera_matrix, dist_coeffs)
            monitor_frame = draw_pose_overlay(
                annotated_frame, poses, camera_matrix, dist_coeffs
            )

            cv2.putText(
                monitor_frame,
                f"RECV FPS: {recv_fps}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
            cv2.putText(
                monitor_frame,
                f"INFER FPS: {infer_fps}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
            cv2.putText(
                monitor_frame,
                f"Detections: {len(detections)}  PoseOK: {len(poses)}",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            display_frame = cv2.resize(
                monitor_frame,
                None,
                fx=DISPLAY_SCALE,
                fy=DISPLAY_SCALE,
                interpolation=cv2.INTER_LINEAR,
            )
            cv2.imshow(WINDOW_NAME, display_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            recv_cnt += 1
            infer_cnt += 1

            if recv_cnt >= frames_to_count:
                now = time.time()
                elapsed = now - recv_st
                if elapsed > 0:
                    recv_fps = round(frames_to_count / elapsed)
                recv_st = now
                recv_cnt = 0

            if infer_cnt >= frames_to_count:
                now = time.time()
                elapsed = now - infer_st
                if elapsed > 0:
                    infer_fps = round(frames_to_count / elapsed)
                infer_st = now
                infer_cnt = 0

    finally:
        client_socket.close()
        cv2.destroyAllWindows()
        print("[INFO] UDP client closed")


if __name__ == "__main__":
    main()
