import argparse
from pathlib import Path

import cv2
import numpy as np
from robot_pose import move_robot_to_capture_pose


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Press SPACE to save undistorted camera frames for YOLO dataset collection."
    )
    parser.add_argument(
        "--save-dir",
        type=Path,
        default=Path("data/images"),
        help="Directory where captured images will be saved.",
    )
    parser.add_argument(
        "--camera-port",
        type=str,
        default="/dev/jetcocam0",
        help="Camera device path (default: /dev/jetcocam0).",
    )
    parser.add_argument(
        "--window-name",
        type=str,
        default="YOLO Data Capture",
        help="OpenCV window name.",
    )
    parser.add_argument(
        "--intrinsic-npz",
        type=Path,
        default=Path(
            "src/collect/camera_intrinsics/camera_calib_upgraded_2026-01-12_16-17-24.npz"
        ),
        help="Camera intrinsic npz path. If file exists, this is used first.",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="capture",
        help="Prefix for saved image filenames.",
    )
    parser.add_argument(
        "--start-index",
        type=int,
        default=0,
        help="Starting number for saved filenames.",
    )
    parser.add_argument(
        "--fx",
        type=float,
        default=0.0,
        help="Camera matrix fx. If 0, frame-width is used.",
    )
    parser.add_argument(
        "--fy",
        type=float,
        default=0.0,
        help="Camera matrix fy. If 0, frame-width is used.",
    )
    parser.add_argument(
        "--cx",
        type=float,
        default=-1.0,
        help="Camera matrix cx. If <0, frame-center is used.",
    )
    parser.add_argument(
        "--cy",
        type=float,
        default=-1.0,
        help="Camera matrix cy. If <0, frame-center is used.",
    )
    parser.add_argument("--k1", type=float, default=0.0, help="Distortion k1")
    parser.add_argument("--k2", type=float, default=0.0, help="Distortion k2")
    parser.add_argument("--p1", type=float, default=0.0, help="Distortion p1")
    parser.add_argument("--p2", type=float, default=0.0, help="Distortion p2")
    parser.add_argument("--k3", type=float, default=0.0, help="Distortion k3")
    parser.add_argument("--robot-port", type=str, default="/dev/ttyJETCOBOT")
    parser.add_argument("--robot-baud", type=int, default=1000000)
    parser.add_argument("--robot-speed", type=int, default=30)
    parser.add_argument("--robot-wait-sec", type=float, default=4.0)
    parser.add_argument(
        "--skip-robot",
        action="store_true",
        help="Skip robot movement and start camera capture directly.",
    )
    return parser.parse_args()


def find_next_index(save_dir: Path, prefix: str, start_index: int) -> int:
    existing = sorted(save_dir.glob(f"{prefix}_*.jpg"))
    if not existing:
        return start_index

    max_idx = start_index - 1
    for path in existing:
        parts = path.stem.split("_")
        if not parts:
            continue
        try:
            idx = int(parts[-1])
            max_idx = max(max_idx, idx)
        except ValueError:
            continue

    return max_idx + 1


def build_camera_params(frame, args: argparse.Namespace):
    h, w = frame.shape[:2]

    fx = args.fx if args.fx > 0 else float(w)
    fy = args.fy if args.fy > 0 else float(w)
    cx = args.cx if args.cx >= 0 else (w / 2.0)
    cy = args.cy if args.cy >= 0 else (h / 2.0)

    camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)
    dist_coeffs = np.array([args.k1, args.k2, args.p1, args.p2, args.k3], dtype=np.float32)
    return camera_matrix, dist_coeffs


def load_intrinsics_from_npz(npz_path: Path):
    if not npz_path.exists():
        return None, None

    data = np.load(npz_path, allow_pickle=True)
    if "camera_matrix" not in data.files or "dist_coeffs" not in data.files:
        raise KeyError(
            f"Invalid intrinsic npz: {npz_path}. "
            "Required keys: camera_matrix, dist_coeffs"
        )

    camera_matrix = np.asarray(data["camera_matrix"], dtype=np.float32).reshape(3, 3)
    dist_coeffs = np.asarray(data["dist_coeffs"], dtype=np.float32).reshape(-1)
    return camera_matrix, dist_coeffs


def main() -> None:
    args = parse_args()
    args.save_dir.mkdir(parents=True, exist_ok=True)
    move_robot_to_capture_pose(args)

    cap = cv2.VideoCapture(args.camera_port)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera: {args.camera_port}")

    index = find_next_index(args.save_dir, args.prefix, args.start_index)
    saved_count = len(list(args.save_dir.glob(f"{args.prefix}_*.jpg")))

    print("[INFO] SPACE: save image | q: quit")
    print(f"[INFO] Camera: {args.camera_port}")
    print(f"[INFO] Save directory: {args.save_dir.resolve()}")
    print(f"[INFO] Intrinsic npz: {args.intrinsic_npz}")

    try:
        camera_matrix, dist_coeffs = load_intrinsics_from_npz(args.intrinsic_npz)
        if camera_matrix is not None:
            print("[INFO] Loaded camera intrinsics from npz.")
        else:
            print("[INFO] Intrinsic npz not found. Using CLI distortion parameters.")

        while True:
            ok, frame = cap.read()
            if not ok:
                print("[WARN] Failed to read frame from camera.")
                break

            if camera_matrix is None:
                camera_matrix, dist_coeffs = build_camera_params(frame, args)

            undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
            display_frame = undistorted.copy()

            cv2.putText(
                display_frame,
                f"Saved: {saved_count}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                display_frame,
                "SPACE: save | q: quit",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow(args.window_name, display_frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord(" "):
                filename = f"{args.prefix}_{index:06d}.jpg"
                save_path = args.save_dir / filename
                cv2.imwrite(str(save_path), undistorted)
                print(f"[SAVED] {save_path}")
                index += 1
                saved_count += 1
            elif key == ord("q"):
                print("[INFO] Exit requested.")
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
