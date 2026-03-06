import argparse
import time

from pymycobot.mycobot import MyCobot

# 측정 자세(6축 각도)를 여기에 입력하세요. 예: [0, -20, -30, 0, 45, 0]
CAP_ANGLES = [-90, -30, 0, -60, 0, 45]


def move_robot_to_capture_pose(args: argparse.Namespace) -> None:
    if getattr(args, "skip_robot", False):
        print("[INFO] Robot move skipped by --skip-robot.")
        return

    if len(CAP_ANGLES) != 6:
        raise ValueError(
            "CAP_ANGLES must contain 6 joint angles. "
            f"Current value: {CAP_ANGLES}"
        )

    port = getattr(args, "robot_port", "/dev/ttyJETCOBOT")
    baud = getattr(args, "robot_baud", 1000000)
    speed = getattr(args, "robot_speed", 30)
    wait_sec = getattr(args, "robot_wait_sec", 4.0)

    print(f"[INFO] Connecting robot (port={port}, baud={baud})")
    mc = MyCobot(port, baud)
    time.sleep(1.0)

    mc.power_on()
    time.sleep(0.3)

    print(f"[INFO] Target CAP_ANGLES: {CAP_ANGLES}")
    mc.send_angles(CAP_ANGLES, speed)
    time.sleep(wait_sec)
    print("[INFO] Robot moved to capture pose.")

    print(mc.get_coords())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move MyCobot to CAP_ANGLES capture pose."
    )
    parser.add_argument("--robot-port", type=str, default="/dev/ttyJETCOBOT")
    parser.add_argument("--robot-baud", type=int, default=1000000)
    parser.add_argument("--robot-speed", type=int, default=30)
    parser.add_argument("--robot-wait-sec", type=float, default=4.0)
    parser.add_argument("--skip-robot", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    move_robot_to_capture_pose(args)


if __name__ == "__main__":
    main()
