#!/usr/bin/env python3
import time
from pymycobot.mycobot280 import MyCobot280

# =========================
# 설정
# =========================
PORT = "/dev/ttyUSB0"
BAUD = 1000000
SPEED = 30

POLL_DT = 0.05          # 20Hz 폴링
TIMEOUT = 10.0          # 각 동작 타임아웃

HOME = [0, 0, 0, 0, 0, 0]
T30  = [30, 0, 0, 0, 0, 0]
T60  = [60, 0, 0, 0, 0, 0]
REST_AT_30_SEC = 3.0

# 베이스 각도 변화 기반 추정 파라미터
START_EPS_DEG = 0.6      # 이 이상 변하면 "움직임 시작"으로 판단
STABLE_EPS_DEG = 0.15    # 이 이하로 변화가 작으면 "안정"으로 판단
STABLE_SEC = 0.35        # 안정 상태가 이 시간 이상 지속되면 "멈춤"으로 판단


def fmt6(a):
    return "[" + ", ".join(f"{x:6.1f}" for x in a) + "]"


def move_base_and_estimate_delay(mc: MyCobot280, target_angles, timeout=10.0):
    """
    베이스(J1)만 움직일 때,
    각도 변화량으로 moving start/stop 시점을 추정한다.
    """
    # 명령 전송 시각
    send_t = time.time()

    # 초기 베이스 각도
    angles0 = mc.get_angles()
    if angles0 is None or len(angles0) < 6:
        angles0 = [0, 0, 0, 0, 0, 0]
    base0 = float(angles0[0])

    # 추정 타이밍
    start_t = None
    stop_t = None

    # 안정 판정용
    stable_since = None
    last_base = base0

    # 명령 전송 (베이스만)
    mc.send_angle(1, target_angles[0], SPEED)

    # 모니터링 루프
    while True:
        now = time.time()
        elapsed = now - send_t

        curr = mc.get_angles()
        if curr is None or len(curr) < 6:
            curr = angles0

        base = float(curr[0])

        inpos = mc.is_in_position(target_angles, 0)  # 1/0/-1
        moving = mc.is_moving()                      # 1/0/-1

        # ====== start 추정: base가 움직이기 시작했는지 ======
        if start_t is None:
            if abs(base - base0) >= START_EPS_DEG:
                start_t = now

        # ====== stop 추정: base가 안정 상태로 유지되는지 ======
        if start_t is not None and stop_t is None:
            d_base = abs(base - last_base)

            if d_base <= STABLE_EPS_DEG:
                if stable_since is None:
                    stable_since = now
                elif (now - stable_since) >= STABLE_SEC:
                    stop_t = now
            else:
                stable_since = None

        last_base = base

        # ====== 출력 ======
        print(
            f"t={elapsed:5.2f}s | "
            f"curr={fmt6(curr)} | "
            f"target={fmt6(target_angles)} | "
            f"in_position={inpos:2d} | is_moving={moving:2d}"
        )

        # 종료 조건: 도착(in_position=1) && 안정 stop 추정 완료
        if inpos == 1 and stop_t is not None:
            break

        if elapsed > timeout:
            print("!! TIMEOUT")
            break

        time.sleep(POLL_DT)

    # ====== 요약 출력 ======
    print("----- moving delay estimate (base-angle based) -----")
    if start_t is None:
        print(" start_delay: NOT DETECTED (base did not move enough)")
    else:
        print(f" start_delay: {start_t - send_t:.3f} s  (send -> base motion start)")

    if stop_t is None:
        print(" stop_delay:  NOT DETECTED (base not stabilized)")
    else:
        print(f" stop_delay:  {stop_t - send_t:.3f} s  (send -> base stabilized)")
        if start_t is not None:
            print(f" move_duration_est: {stop_t - start_t:.3f} s  (start -> stop)")

    final_inpos = mc.is_in_position(target_angles, 0)
    final_moving = mc.is_moving()
    print(f" final check: in_position={final_inpos}, is_moving={final_moving}")
    print("---------------------------------------------------\n")


def main():
    mc = MyCobot280(PORT, BAUD)

    print("\n=== Base-only moving delay estimate test ===")
    print(f"PORT={PORT}, BAUD={BAUD}, SPEED={SPEED}")
    print("NOTE: This does NOT release servos at the end.\n")

    # 서보 ON
    mc.focus_all_servos()
    time.sleep(0.5)

    # HOME 이동
    print("[HOME] ->", HOME)
    mc.send_angles(HOME, SPEED)
    move_base_and_estimate_delay(mc, HOME, timeout=TIMEOUT)

    # 30도 이동
    print("[MOVE] ->", T30)
    move_base_and_estimate_delay(mc, T30, timeout=TIMEOUT)

    # 30도에서 3초 대기 (상태 출력)
    print(f"[REST] at 30deg for {REST_AT_30_SEC}s")
    t_end = time.time() + REST_AT_30_SEC
    while time.time() < t_end:
        curr = mc.get_angles() or T30
        inpos = mc.is_in_position(T30, 0)
        moving = mc.is_moving()
        print(
            f"REST | curr={fmt6(curr)} | target={fmt6(T30)} | "
            f"in_position={inpos:2d} | is_moving={moving:2d}"
        )
        time.sleep(POLL_DT)
    print()

    # 60도 이동
    print("[MOVE] ->", T60)
    move_base_and_estimate_delay(mc, T60, timeout=TIMEOUT)

    print("Done. (Servos are still ON) ✅")


if __name__ == "__main__":
    main()
