import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot

# =========================
# 1) 경로 설정
# =========================
NPZ_PATH = "/home/addinedu/calibration/hand_eye_calibration/2026-01-13_14-00-51_hand_eye_calibration.npz"
OUT_DIR = os.path.dirname(NPZ_PATH)
OUT_CLEAN = os.path.join(OUT_DIR, os.path.basename(NPZ_PATH).replace(".npz", "_cleaned.npz"))

# =========================
# 2) 유틸 함수
# =========================
def rodrigues_to_rotvec(rvec_xyz):
    rvec = np.array(rvec_xyz, dtype=np.float64).reshape(3, 1)
    R, _ = cv2.Rodrigues(rvec)
    return Rot.from_matrix(R)

def quat_to_rot(q_xyzw):
    return Rot.from_quat(np.array(q_xyzw, dtype=np.float64))

def ang_deg_between(Ra: Rot, Rb: Rot) -> float:
    # Ra^{-1} Rb 의 회전각
    Rrel = Ra.inv() * Rb
    return float(np.degrees(Rrel.magnitude()))

def robust_z(x):
    x = np.asarray(x, dtype=np.float64)
    med = np.median(x)
    mad = np.median(np.abs(x - med)) + 1e-12
    return 0.6745 * (x - med) / mad  # robust z-score

def plot_series(x, title, ylabel):
    plt.figure()
    plt.plot(x, marker='o')
    plt.title(title)
    plt.xlabel("sample index")
    plt.ylabel(ylabel)
    plt.grid(True)

# =========================
# 3) 데이터 로드 + 형태 체크
# =========================
data = np.load(NPZ_PATH, allow_pickle=True)
print("keys:", data.files)

tvecs = data["Target_tvecs"]      # [count, id, tx, ty, tz]  (cam->target, m)
rvecs = data["Target_rvecs"]      # [count, rx, ry, rz]      (cam->target, Rodrigues)
grip  = data["gripper_pose"]      # [x, y, z, qx, qy, qz, qw] (base->gripper, m + quat)

# 빈 데이터 방지
if tvecs.size == 0 or rvecs.size == 0 or grip.size == 0:
    raise RuntimeError("npz 안에 데이터가 비어있습니다. (tvecs/rvecs/gripper_pose)")

# shape 보정 (혹시 1D로 들어가면)
tvecs = np.atleast_2d(tvecs)
rvecs = np.atleast_2d(rvecs)
grip  = np.atleast_2d(grip)

N = min(len(tvecs), len(rvecs), len(grip))
tvecs = tvecs[:N]
rvecs = rvecs[:N]
grip  = grip[:N]

print("N:", N)
print("tvecs:", tvecs.shape, "rvecs:", rvecs.shape, "grip:", grip.shape)

# =========================
# 4) time-series 추출
# =========================
# target (cam->target)
t_t = tvecs[:, 2:5].astype(np.float64)          # (N,3)
r_t = rvecs[:, 1:4].astype(np.float64)          # (N,3) Rodrigues

# gripper (base->gripper)
t_g = grip[:, 0:3].astype(np.float64)           # (N,3)
q_g = grip[:, 3:7].astype(np.float64)           # (N,4) xyzw

# 회전(각도)로 보기 위한 변환
R_t_list = [rodrigues_to_rotvec(r_t[i]) for i in range(N)]
R_g_list = [quat_to_rot(q_g[i]) for i in range(N)]

# =========================
# 5) "튀는 정도" 지표 만들기
#    - step jump: i-1 -> i 변화량
# =========================
# translation jump (mm)
dt_target_mm  = np.zeros(N)
dt_gripper_mm = np.zeros(N)

# rotation jump (deg)
dr_target_deg  = np.zeros(N)
dr_gripper_deg = np.zeros(N)

for i in range(1, N):
    dt_target_mm[i]  = np.linalg.norm(t_t[i] - t_t[i-1]) * 1000.0
    dt_gripper_mm[i] = np.linalg.norm(t_g[i] - t_g[i-1]) * 1000.0
    dr_target_deg[i]  = ang_deg_between(R_t_list[i-1], R_t_list[i])
    dr_gripper_deg[i] = ang_deg_between(R_g_list[i-1], R_g_list[i])

# =========================
# 6) 그래프 시연 (원본)
# =========================
plot_series(t_t[:,0]*1000, "Target tvec X (mm)", "mm")
plot_series(t_t[:,1]*1000, "Target tvec Y (mm)", "mm")
plot_series(t_t[:,2]*1000, "Target tvec Z (mm)", "mm")

plot_series(t_g[:,0]*1000, "Gripper pos X (mm)", "mm")
plot_series(t_g[:,1]*1000, "Gripper pos Y (mm)", "mm")
plot_series(t_g[:,2]*1000, "Gripper pos Z (mm)", "mm")

plot_series(dt_target_mm,  "Target translation step-jump (mm)", "mm")
plot_series(dt_gripper_mm, "Gripper translation step-jump (mm)", "mm")
plot_series(dr_target_deg,  "Target rotation step-jump (deg)", "deg")
plot_series(dr_gripper_deg, "Gripper rotation step-jump (deg)", "deg")

plt.show()

# =========================
# 7) Outlier 자동 탐지 (robust z-score)
#    - translation/rotation jump 둘 다 큰 샘플을 outlier로 간주
# =========================
Z_DT_T = np.abs(robust_z(dt_target_mm))
Z_DT_G = np.abs(robust_z(dt_gripper_mm))
Z_DR_T = np.abs(robust_z(dr_target_deg))
Z_DR_G = np.abs(robust_z(dr_gripper_deg))

# 점수(가중합) - 필요하면 가중치 조절
score = 0.35*Z_DT_T + 0.35*Z_DT_G + 0.15*Z_DR_T + 0.15*Z_DR_G

# threshold: 보수적으로 3.5 추천, 많이 튀면 3.0
TH = 3.5
out_idx = np.where(score > TH)[0].tolist()

print("\n=== Outlier detection ===")
print("threshold:", TH)
print("outlier idx:", out_idx)
print("outlier count:", len(out_idx), "/", N)

# =========================
# 8) 제거 + 저장
# =========================
mask = np.ones(N, dtype=bool)
mask[out_idx] = False

tvecs_c = tvecs[mask]
rvecs_c = rvecs[mask]
grip_c  = grip[mask]

np.savez(
    OUT_CLEAN,
    Target_tvecs=tvecs_c.astype(np.float64),
    Target_rvecs=rvecs_c.astype(np.float64),
    gripper_pose=grip_c.astype(np.float64),
    removed_indices=np.array(out_idx, dtype=np.int32),
    outlier_score=score.astype(np.float64),
    outlier_threshold=float(TH),
)

print("\nSaved cleaned npz:", OUT_CLEAN)
print("Kept:", len(tvecs_c), "Removed:", len(out_idx))

# =========================
# 9) 제거 후 그래프 (필요한 것만 간단히)
# =========================
# cleaned step-jump만 다시 계산해서 비교
t_t2 = tvecs_c[:, 2:5].astype(np.float64)
t_g2 = grip_c[:, 0:3].astype(np.float64)
r_t2 = rvecs_c[:, 1:4].astype(np.float64)
q_g2 = grip_c[:, 3:7].astype(np.float64)
N2 = len(tvecs_c)

R_t2 = [rodrigues_to_rotvec(r_t2[i]) for i in range(N2)]
R_g2 = [quat_to_rot(q_g2[i]) for i in range(N2)]

dt_target2 = np.zeros(N2); dt_gripper2 = np.zeros(N2)
dr_target2 = np.zeros(N2); dr_gripper2 = np.zeros(N2)
for i in range(1, N2):
    dt_target2[i]  = np.linalg.norm(t_t2[i]-t_t2[i-1])*1000.0
    dt_gripper2[i] = np.linalg.norm(t_g2[i]-t_g2[i-1])*1000.0
    dr_target2[i]  = ang_deg_between(R_t2[i-1], R_t2[i])
    dr_gripper2[i] = ang_deg_between(R_g2[i-1], R_g2[i])

plot_series(dt_target_mm,  "BEFORE: Target translation jump (mm)", "mm")
plot_series(dt_target2,    "AFTER:  Target translation jump (mm)", "mm")
plot_series(dr_target_deg, "BEFORE: Target rotation jump (deg)", "deg")
plot_series(dr_target2,    "AFTER:  Target rotation jump (deg)", "deg")
plt.show()
