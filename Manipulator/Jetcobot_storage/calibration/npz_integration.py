import numpy as np
import os
from glob import glob

# ======================================================
# 설정
# ======================================================
NPZ_DIR = "/home/addinedu/calibration/hand_eye_calibration"  # npz 폴더
OUT_NPZ = os.path.join(NPZ_DIR, "merged_layer_v.0.1.npz")

# ======================================================
# npz 파일 목록 불러오기
# ======================================================
npz_files = sorted(glob(os.path.join(NPZ_DIR, "*.npz")))

assert len(npz_files) > 0, "❌ npz 파일이 없습니다."

print("Found npz files:")
for f in npz_files:
    print(" -", os.path.basename(f))

# ======================================================
# 누적 버퍼
# ======================================================
all_tvecs = []
all_rvecs = []
all_gripper = []

meta = {}   # metadata (첫 파일 기준)

# ======================================================
# 파일 순회
# ======================================================
count_global = 1

for fi, path in enumerate(npz_files):
    data = np.load(path)

    tvecs = data["Target_tvecs"]
    rvecs = data["Target_rvecs"]
    grip = data["gripper_pose"]

    assert len(tvecs) == len(rvecs) == len(grip), "❌ 길이 불일치"

    # metadata는 첫 파일에서만
    if fi == 0:
        for k in data.files:
            if k not in ["Target_tvecs", "Target_rvecs", "gripper_pose"]:
                meta[k] = data[k]

    for i in range(len(tvecs)):
        # count 재부여
        all_tvecs.append([
            count_global,
            int(tvecs[i, 1]),
            tvecs[i, 2],
            tvecs[i, 3],
            tvecs[i, 4],
        ])

        all_rvecs.append([
            count_global,
            rvecs[i, 1],
            rvecs[i, 2],
            rvecs[i, 3],
        ])

        all_gripper.append(grip[i].tolist())

        count_global += 1

# ======================================================
# numpy array로 변환
# ======================================================
all_tvecs = np.array(all_tvecs, dtype=np.float64)
all_rvecs = np.array(all_rvecs, dtype=np.float64)
all_gripper = np.array(all_gripper, dtype=np.float64)

# ======================================================
# 저장
# ======================================================
np.savez(
    OUT_NPZ,
    Target_tvecs=all_tvecs,
    Target_rvecs=all_rvecs,
    gripper_pose=all_gripper,
    **meta
)

print("\n✅ Merge complete")
print(f"Total samples: {len(all_tvecs)}")
print(f"Saved to: {OUT_NPZ}")
