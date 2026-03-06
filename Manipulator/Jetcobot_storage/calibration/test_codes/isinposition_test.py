#!/usr/bin/env python3
import time
from pymycobot.mycobot280 import MyCobot280

# =========================
# 사용자 설정
# =========================
PORT = "/dev/ttyUSB0"
BAUD = 1000000

MOVE_SPEED = 30           # 1~100
STEP_DEG = 30             # 베이스 조인트를 몇 도씩 움직일지
N_STEPS = 2               # 몇 번 반복할지 (6번이면 180도)
WAIT_TIMEOUT_SEC = 8.0    # 도착 판정 타임아웃
POLL_PERIOD_SEC = 0.1     # is_in_position 체크 주기


def wait_until_in_position(mc: MyCobot280, target_angles, timeout_sec=8.0, period=0.1):
    """
    target_angles: [a1,a2,a3,a4,a5,a6] (deg)
    return: (arrived: bool, last_angles: list|None, elapsed: float)
    """
    t0 = time.time()
    while True:
        # -1: error / 0: not arrived / 1: arrived
        ok = mc.is_in_position(target_angles, 0)

        curr = mc.get_angles()

        elapsed = time.time() - t0
        if curr is not None and len(curr) >= 6:
            # 베이스 조인트 오차만 보기
            err_base = curr[0] - target_angles[0]
            print(
                f"  [poll] elapsed={elapsed:4.1f}s | "
                f"is_in_position={ok} | "
                f"curr_base={curr[0]:7.2f} deg | "
                f"target_base={target_angles[0]:7.2f} deg | "
                f"err={err_base:+7.2f} deg"
            )
        else:
            print(f"  [poll] elapsed={elapsed:4.1f}s | is_in_position={ok} | get_angles failed")

        if ok == 1:
            return True, curr, elapsed
        if ok == -1:
            return False, curr, elapsed

        if elapsed > timeout_sec:
            return False, curr, elapsed

        time.sleep(period)


def main():
    mc = MyCobot280(PORT, BAUD)

    mc.send_angles([0,0,0,0,0,0], 30)

    # 서보 ON
    mc.focus_all_servos()
    time.sleep(0.5)

    # 현재 각도 읽기
    init_angles = mc.get_angles()
    if init_angles is None or len(init_angles) < 6:
        raise RuntimeError("get_angles() failed. Check connection/servos.")

    print("\n=== is_in_position() Base Rotate Test ===")
    print(f"PORT={PORT}, BAUD={BAUD}")
    print(f"STEP={STEP_DEG} deg, N_STEPS={N_STEPS}, SPEED={MOVE_SPEED}")
    print("Press Ctrl+C to stop.\n")
    print("Initial angles:", [round(a, 2) for a in init_angles])

    # 시작각 유지 (다른 관절은 그대로)
    base0 = init_angles[0]
    base_targets = [base0 + STEP_DEG * i for i in range(1, N_STEPS + 1)]

    try:
        for i, base_t in enumerate(base_targets, start=1):
            # 목표 각도 만들기
            target = init_angles[:]  # 6개 복사
            target[0] = base_t

            print(f"\n[{i}/{N_STEPS}] Command base -> {base_t:.2f} deg")
            ret = mc.send_angle(1, base_t, MOVE_SPEED)  # joint index 1 = base
            print(f"  send_angle return: {ret}")

            # is_in_position 체크 (angles, flag=0)
            arrived, last_angles, elapsed = wait_until_in_position(
                mc,
                target_angles=target,
                timeout_sec=WAIT_TIMEOUT_SEC,
                period=POLL_PERIOD_SEC,
            )

            if arrived:
                print(f"✅ Arrived in {elapsed:.2f}s")
            else:
                print(f"❌ Not arrived (timeout or error) after {elapsed:.2f}s")

            # 다음 스텝 전에 잠깐 휴식
            time.sleep(0.5)

        print("\nDone. Releasing servos.")
        mc.release_all_servos()

    except KeyboardInterrupt:
        print("\nInterrupted by user. Releasing servos.")
        mc.release_all_servos()


if __name__ == "__main__":
    main()
