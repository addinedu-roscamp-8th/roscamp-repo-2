import numpy as np
import cv2
from scipy.spatial.transform import Rotation as Rot

NPZ_PATH = "/home/addinedu/mycobot_ws/hand_eye_calibration/2026-01-12_18-59-13_hand_eye_calibration.npz"

# ----------------------------
# Helpers
# ----------------------------
def rodrigues_to_R(rvec):
    rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
    R, _ = cv2.Rodrigues(rvec)
    return R

def quat_to_R_xyzw(q):
    q = np.asarray(q, dtype=np.float64).reshape(4,)
    return Rot.from_quat(q).as_matrix()

def invert_T(R, t):
    R = np.asarray(R, dtype=np.float64).reshape(3,3)
    t = np.asarray(t, dtype=np.float64).reshape(3,1)
    R_inv = R.T
    t_inv = -R_inv @ t
    return R_inv, t_inv

def compose_T(R, t):
    T = np.eye(4, dtype=np.float64)
    T[:3,:3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3,)
    return T

def rot_angle_deg(Ra, Rb):
    Rrel = Ra.T @ Rb
    ang = Rot.from_matrix(Rrel).magnitude()
    return np.degrees(ang)

def mean_pose_translation(ts):
    return np.mean(np.stack(ts, axis=0), axis=0)

def robust_center_translation(ts):
    # median center for robustness
    return np.median(np.stack(ts, axis=0), axis=0)

def rotation_geodesic_mean(Rs, iters=30):
    # simple iterative mean on SO(3)
    Rm = Rs[0].copy()
    for _ in range(iters):
        errs = []
        for R in Rs:
            Rerr = Rm.T @ R
            w = Rot.from_matrix(Rerr).as_rotvec()
            errs.append(w)
        w_mean = np.mean(np.stack(errs, axis=0), axis=0)
        if np.linalg.norm(w_mean) < 1e-10:
            break
        Rm = Rm @ Rot.from_rotvec(w_mean).as_matrix()
    return Rm

def compute_Bt_list(R_bg_list, t_bg_list, R_g2c, t_g2c, R_ct_list, t_ct_list):
    # ^bTt(i) = ^bTg(i) * ^gTc * ^cTt(i)
    Bt_Ts = []
    for R_bg, t_bg, R_ct, t_ct in zip(R_bg_list, t_bg_list, R_ct_list, t_ct_list):
        T_bg = compose_T(R_bg, t_bg)
        T_g2c = compose_T(R_g2c, t_g2c)
        T_ct = compose_T(R_ct, t_ct)
        T_bt = T_bg @ T_g2c @ T_ct
        Bt_Ts.append(T_bt)
    return Bt_Ts

def score_solution(Bt_Ts):
    # translation std (mm), rotation std (deg) around mean
    ts = [T[:3,3] for T in Bt_Ts]
    t_center = robust_center_translation(ts)
    t_err = np.linalg.norm(np.stack(ts,0) - t_center.reshape(1,3), axis=1)  # m
    t_std_mm = np.std(t_err) * 1000.0
    t_med_mm = np.median(t_err) * 1000.0
    t_max_mm = np.max(t_err) * 1000.0

    Rs = [T[:3,:3] for T in Bt_Ts]
    R_mean = rotation_geodesic_mean(Rs)
    r_err = np.array([rot_angle_deg(R_mean, R) for R in Rs])  # deg
    r_std = np.std(r_err)
    r_med = np.median(r_err)
    r_max = np.max(r_err)

    return {
        "t_med_mm": float(t_med_mm),
        "t_std_mm": float(t_std_mm),
        "t_max_mm": float(t_max_mm),
        "r_med_deg": float(r_med),
        "r_std_deg": float(r_std),
        "r_max_deg": float(r_max),
        "t_err_mm": t_err * 1000.0,
        "r_err_deg": r_err,
        "t_center_m": t_center,
        "R_mean": R_mean,
    }

def rotation_diversity(R_list):
    # relative rotation angles distribution
    n = len(R_list)
    if n < 2:
        return None
    angs = []
    for i in range(n-1):
        for j in range(i+1, n):
            angs.append(rot_angle_deg(R_list[i], R_list[j]))
    angs = np.array(angs)
    return {
        "pair_mean_deg": float(np.mean(angs)),
        "pair_median_deg": float(np.median(angs)),
        "pair_max_deg": float(np.max(angs)),
        "pair_min_deg": float(np.min(angs)),
    }

# ----------------------------
# Load
# ----------------------------
data = np.load(NPZ_PATH)
for k in ["Target_tvecs", "Target_rvecs", "gripper_pose"]:
    if k not in data.files:
        raise KeyError(f"Missing key '{k}' in npz. keys={data.files}")

tvecs = data["Target_tvecs"]        # [count, id, tx, ty, tz] cam->target (m)
rvecs = data["Target_rvecs"]        # [count, rx, ry, rz] cam->target (Rodrigues, rad)
gpose = data["gripper_pose"]        # [x,y,z,qx,qy,qz,qw] base->gripper (m, quat xyzw)

if tvecs.size == 0 or rvecs.size == 0 or gpose.size == 0:
    raise ValueError("Empty arrays in npz. No samples were saved.")

# enforce 2D
tvecs = np.atleast_2d(tvecs)
rvecs = np.atleast_2d(rvecs)
gpose = np.atleast_2d(gpose)

N = min(len(tvecs), len(rvecs), len(gpose))
tvecs = tvecs[:N]
rvecs = rvecs[:N]
gpose = gpose[:N]

print(f"[Load] N={N}")
print("[Load] keys:", data.files)

# Build lists (as stored)
# cam->target:
R_ct_all = [rodrigues_to_R(rvecs[i, 1:4]) for i in range(N)]
t_ct_all = [tvecs[i, 2:5].astype(np.float64).reshape(3,1) for i in range(N)]
# base->gripper:
R_bg_all = [quat_to_R_xyzw(gpose[i, 3:7]) for i in range(N)]
t_bg_all = [gpose[i, 0:3].astype(np.float64).reshape(3,1) for i in range(N)]

div_bg = rotation_diversity(R_bg_all)
div_ct = rotation_diversity(R_ct_all)
print("\n[Rotation diversity]")
print("  base->gripper pairwise (deg):", div_bg)
print("  cam->target   pairwise (deg):", div_ct)
print("  (Tip) pairwise mean/median이 너무 낮으면(예: <20~30deg) 해가 불안정해질 수 있어요.\n")

# ----------------------------
# Candidate definitions to test
# ----------------------------
# OpenCV calibrateHandEye expects:
#   R_gripper2base, t_gripper2base, R_target2cam, t_target2cam
# depending on your storage, we test combinations.
#
# Stored:
#   base->gripper  (R_bg, t_bg)
#   cam->target    (R_ct, t_ct)
#
# Convert:
#   gripper->base = inv(base->gripper)
#   target->cam   = inv(cam->target)
#
# We'll test 4 combos:
# A: use gripper->base, target->cam   (often correct for eye-in-hand)
# B: use base->gripper, target->cam
# C: use gripper->base, cam->target
# D: use base->gripper, cam->target
#
def build_combo(name):
    if name == "A_g2b__t2c":
        R_g2b, t_g2b = zip(*[invert_T(R_bg_all[i], t_bg_all[i]) for i in range(N)])
        R_t2c, t_t2c = zip(*[invert_T(R_ct_all[i], t_ct_all[i]) for i in range(N)])
        return list(R_g2b), list(t_g2b), list(R_t2c), list(t_t2c)
    if name == "B_b2g__t2c":
        R_t2c, t_t2c = zip(*[invert_T(R_ct_all[i], t_ct_all[i]) for i in range(N)])
        return R_bg_all, t_bg_all, list(R_t2c), list(t_t2c)
    if name == "C_g2b__c2t":
        R_g2b, t_g2b = zip(*[invert_T(R_bg_all[i], t_bg_all[i]) for i in range(N)])
        return list(R_g2b), list(t_g2b), R_ct_all, t_ct_all
    if name == "D_b2g__c2t":
        return R_bg_all, t_bg_all, R_ct_all, t_ct_all
    raise ValueError("unknown combo")

COMBOS = ["A_g2b__t2c", "B_b2g__t2c", "C_g2b__c2t", "D_b2g__c2t"]
METHODS = [
    ("TSAI", cv2.CALIB_HAND_EYE_TSAI),
    ("PARK", cv2.CALIB_HAND_EYE_PARK),
    ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD),
    ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS),
]

# ----------------------------
# Evaluate all candidates
# ----------------------------
results = []

for combo in COMBOS:
    R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list = build_combo(combo)

    for mname, mm in METHODS:
        try:
            R_cam2grip, t_cam2grip = cv2.calibrateHandEye(
                R_g2b_list, t_g2b_list,
                R_t2c_list, t_t2c_list,
                method=mm
            )
        except cv2.error as e:
            print(f"[Skip] {combo} {mname}: OpenCV error:", e)
            continue

        # OpenCV returns cam->gripper (per docs). We often want gripper->cam or g->c.
        # We'll build g->c as inverse of (c->g).
        R_c2g = R_cam2grip
        t_c2g = t_cam2grip.reshape(3,1)

        # g->c
        R_g2c, t_g2c = invert_T(R_c2g, t_c2g)

        # For scoring, we need Bt = bTg * gTc * cTt (using STORED b->g and c->t!)
        Bt_Ts = compute_Bt_list(R_bg_all, t_bg_all, R_g2c, t_g2c, R_ct_all, t_ct_all)
        score = score_solution(Bt_Ts)

        results.append({
            "combo": combo,
            "method": mname,
            "R_c2g": R_c2g,
            "t_c2g": t_c2g,
            "R_g2c": R_g2c,
            "t_g2c": t_g2c,
            **{k: score[k] for k in ["t_med_mm","t_std_mm","t_max_mm","r_med_deg","r_std_deg","r_max_deg"]},
            "t_err_mm": score["t_err_mm"],
            "r_err_deg": score["r_err_deg"],
        })

# Sort by a combined metric (median translation + median rotation)
results_sorted = sorted(results, key=lambda d: (d["t_med_mm"], d["r_med_deg"]))

print("\n[Candidates ranked by (t_med_mm, r_med_deg)]")
for i, r in enumerate(results_sorted[:10], start=1):
    print(f"{i:2d}) {r['combo']:12s} {r['method']:10s} | "
          f"t_med={r['t_med_mm']:.2f}mm t_std={r['t_std_mm']:.2f}mm t_max={r['t_max_mm']:.2f}mm | "
          f"r_med={r['r_med_deg']:.2f}deg r_std={r['r_std_deg']:.2f}deg r_max={r['r_max_deg']:.2f}deg")

best = results_sorted[0]
print("\n[BEST]")
print(" combo:", best["combo"])
print(" method:", best["method"])
print(" t_med_mm:", best["t_med_mm"], " r_med_deg:", best["r_med_deg"])

# ----------------------------
# Outlier removal + recalibration on BEST combo/method
# ----------------------------
def calibrate_and_score_on_indices(idxs, combo, method_flag):
    # build lists for idx subset
    # base->gripper, cam->target always from stored
    R_bg = [R_bg_all[i] for i in idxs]
    t_bg = [t_bg_all[i] for i in idxs]
    R_ct = [R_ct_all[i] for i in idxs]
    t_ct = [t_ct_all[i] for i in idxs]

    # build combo inputs
    if combo == "A_g2b__t2c":
        R_g2b, t_g2b = zip(*[invert_T(R_bg[k], t_bg[k]) for k in range(len(idxs))])
        R_t2c, t_t2c = zip(*[invert_T(R_ct[k], t_ct[k]) for k in range(len(idxs))])
        R_in1, t_in1, R_in2, t_in2 = list(R_g2b), list(t_g2b), list(R_t2c), list(t_t2c)
    elif combo == "B_b2g__t2c":
        R_t2c, t_t2c = zip(*[invert_T(R_ct[k], t_ct[k]) for k in range(len(idxs))])
        R_in1, t_in1, R_in2, t_in2 = R_bg, t_bg, list(R_t2c), list(t_t2c)
    elif combo == "C_g2b__c2t":
        R_g2b, t_g2b = zip(*[invert_T(R_bg[k], t_bg[k]) for k in range(len(idxs))])
        R_in1, t_in1, R_in2, t_in2 = list(R_g2b), list(t_g2b), R_ct, t_ct
    elif combo == "D_b2g__c2t":
        R_in1, t_in1, R_in2, t_in2 = R_bg, t_bg, R_ct, t_ct
    else:
        raise ValueError(combo)

    R_c2g, t_c2g = cv2.calibrateHandEye(R_in1, t_in1, R_in2, t_in2, method=method_flag)
    R_g2c, t_g2c = invert_T(R_c2g, t_c2g.reshape(3,1))

    Bt_Ts = compute_Bt_list(R_bg, t_bg, R_g2c, t_g2c, R_ct, t_ct)
    sc = score_solution(Bt_Ts)

    # residual per sample: mix translation(mm) + rotation(deg)
    # normalize roughly: 1deg ~ 1mm (tunable)
    resid = sc["t_err_mm"] + sc["r_err_deg"]
    return R_c2g, t_c2g.reshape(3,1), R_g2c, t_g2c, sc, resid

# map method name to flag
method_flag = dict(METHODS)[best["method"]]
combo = best["combo"]

idxs = list(range(N))
print("\n[Outlier removal + re-calibration]")
print(f" Start with N={len(idxs)} using combo={combo}, method={best['method']}")

# iterative remove
REMOVE_FRACTION = 0.15     # 한 번에 상위 15% 제거
MIN_KEEP = max(12, int(0.6 * N))  # 최소 유지 샘플 수

for it in range(4):
    R_c2g, t_c2g, R_g2c, t_g2c, sc, resid = calibrate_and_score_on_indices(idxs, combo, method_flag)
    print(f" iter{it}: N={len(idxs)} | "
          f"t_med={sc['t_med_mm']:.2f}mm t_std={sc['t_std_mm']:.2f}mm t_max={sc['t_max_mm']:.2f}mm | "
          f"r_med={sc['r_med_deg']:.2f}deg r_std={sc['r_std_deg']:.2f}deg r_max={sc['r_max_deg']:.2f}deg")

    if len(idxs) <= MIN_KEEP:
        break

    # remove worst samples
    k = int(np.ceil(REMOVE_FRACTION * len(idxs)))
    k = max(1, k)
    order = np.argsort(resid)  # ascending good->bad
    keep = order[:-k]          # drop worst k
    removed_local = order[-k:]
    removed_global = [idxs[i] for i in removed_local]

    # update idxs
    idxs = [idxs[i] for i in keep]

    print(f"   removed {k} worst samples (global indices): {removed_global}")

# final recompute on kept
R_c2g, t_c2g, R_g2c, t_g2c, sc, resid = calibrate_and_score_on_indices(idxs, combo, method_flag)

print("\n[FINAL after outlier removal]")
print(f" kept N={len(idxs)} / original N={N}")
print(f" t_med={sc['t_med_mm']:.2f}mm t_std={sc['t_std_mm']:.2f}mm t_max={sc['t_max_mm']:.2f}mm")
print(f" r_med={sc['r_med_deg']:.2f}deg r_std={sc['r_std_deg']:.2f}deg r_max={sc['r_max_deg']:.2f}deg")

print("\nT_c2g (camera -> gripper):")
print(compose_T(R_c2g, t_c2g))

print("\nT_g2c (gripper -> camera):")
print(compose_T(R_g2c, t_g2c))

# Save refined result
out_refined = NPZ_PATH.replace(".npz", "_refined_result.npz")
np.savez(
    out_refined,
    best_combo=combo,
    best_method=best["method"],
    kept_indices=np.array(idxs, dtype=np.int32),

    R_c2g=R_c2g,
    t_c2g=t_c2g,
    R_g2c=R_g2c,
    t_g2c=t_g2c,

    t_med_mm=sc["t_med_mm"],
    t_std_mm=sc["t_std_mm"],
    t_max_mm=sc["t_max_mm"],
    r_med_deg=sc["r_med_deg"],
    r_std_deg=sc["r_std_deg"],
    r_max_deg=sc["r_max_deg"],
)
print("\nSaved refined result:", out_refined)
