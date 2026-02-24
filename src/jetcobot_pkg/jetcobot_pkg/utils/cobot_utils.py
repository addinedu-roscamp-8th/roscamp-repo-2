from pymycobot.mycobot280 import MyCobot280
import numpy as np
import math

#=============================================================
#좌표 변환 / 수학 유틸 함수
#=============================================================

# mycobot coords --> z 값만 수정하기 
def coords_replace_z(coords, new_z_mm: float):
    """coords: [x,y,z,rx,ry,rz], z = new_z_mm"""
    out = list(coords)
    out[2] = float(new_z_mm)
    return out


# pose msg --> translation / rotation array로 분리하기
def pose_mm_to_xyz_quat(pose_msg):
    x = float(pose_msg.position.x)
    y = float(pose_msg.position.y)
    z = float(pose_msg.position.z)

    qx = float(pose_msg.orientation.x)
    qy = float(pose_msg.orientation.y)
    qz = float(pose_msg.orientation.z)
    qw = float(pose_msg.orientation.w)

    return np.array([x, y, z], dtype=np.float64), np.array([qx, qy, qz, qw], dtype=np.float64)

# 쿼터니언 normalize
def quat_normalize(q):
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q / n

# 쿼터니언 샘플들 평균 구하기
def quat_mean_xyzw(quats):
    if len(quats) == 0:
        return None

    Q = np.stack(quats, axis=0).astype(np.float64)
    ref = Q[0].copy()

    for i in range(Q.shape[0]):
        if np.dot(Q[i], ref) < 0.0:
            Q[i] = -Q[i]

    q_avg = np.mean(Q, axis=0)
    return quat_normalize(q_avg)

# 쿼터니언 회전을 3x3 회전행렬로 변환하기
def quat_to_rotmat(q_xyzw):
    x, y, z, w = q_xyzw.tolist()
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z

    R = np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)],
        [2*(xy + wz),         1 - 2*(xx + zz),   2*(yz - wx)],
        [2*(xz - wy),         2*(yz + wx),       1 - 2*(xx + yy)],
    ], dtype=np.float64)
    return R

# 3x3 회전행렬을 ZYX inrtinsic euler angle 회전으로 변환하기
def rotmat_to_euler_intrinsic_ZYX_deg(Rm):
    sy = math.sqrt(Rm[0, 0]*Rm[0, 0] + Rm[1, 0]*Rm[1, 0])
    singular = sy < 1e-9

    if not singular:
        rz = math.atan2(Rm[1, 0], Rm[0, 0])
        ry = math.atan2(-Rm[2, 0], sy)
        rx = math.atan2(Rm[2, 1], Rm[2, 2])
    else:
        rz = math.atan2(-Rm[0, 1], Rm[1, 1])
        ry = math.atan2(-Rm[2, 0], sy)
        rx = 0.0

    return (rx * 180.0 / math.pi,
            ry * 180.0 / math.pi,
            rz * 180.0 / math.pi)


# ------------------------
# 내부 유틸들 (외부 함수 의존 제거)
# ------------------------
def _deg2rad(d): return d * math.pi / 180.0
def _rad2deg(r): return r * 180.0 / math.pi

def _Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]], dtype=np.float64)

def _Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]], dtype=np.float64)

def _Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]], dtype=np.float64)

def euler_intrinsic_ZYX_deg_to_rotmat(rx_deg, ry_deg, rz_deg):
    # R = Rz(rz) * Ry(ry) * Rx(rx)
    rx, ry, rz = _deg2rad(rx_deg), _deg2rad(ry_deg), _deg2rad(rz_deg)
    return _Rz(rz) @ _Ry(ry) @ _Rx(rx)

def rotmat_to_euler_intrinsic_ZYX_deg(Rm):
    # intrinsic ZYX: yaw(Z), pitch(Y), roll(X)
    sy = math.sqrt(Rm[0, 0]*Rm[0, 0] + Rm[1, 0]*Rm[1, 0])
    singular = sy < 1e-9

    if not singular:
        rz = math.atan2(Rm[1, 0], Rm[0, 0])
        ry = math.atan2(-Rm[2, 0], sy)
        rx = math.atan2(Rm[2, 1], Rm[2, 2])
    else:
        rz = math.atan2(-Rm[0, 1], Rm[1, 1])
        ry = math.atan2(-Rm[2, 0], sy)
        rx = 0.0

    return (_rad2deg(rx), _rad2deg(ry), _rad2deg(rz))

def coords_mm_deg_to_T(coords):
    x, y, z, rx, ry, rz = [float(v) for v in coords]
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = euler_intrinsic_ZYX_deg_to_rotmat(rx, ry, rz)
    T[:3, 3] = np.array([x, y, z], dtype=np.float64)
    return T

def T_to_coords_mm_deg(T):
    x, y, z = T[:3, 3].tolist()
    rx, ry, rz = rotmat_to_euler_intrinsic_ZYX_deg(T[:3, :3])
    return [float(x), float(y), float(z), float(rx), float(ry), float(rz)]

def inv_T(T):
    Rm = T[:3, :3]
    t  = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = Rm.T
    Ti[:3, 3] = -Rm.T @ t
    return Ti


# ⭐⭐ TCP 적용 send_coords 보낼 명령 좌표 변환 ⭐⭐
def gripper_goal_to_ee_cmd_coords_mm_deg(
    gripper_coords_mm_deg,
    gripper_z_offset_deg: float = -45.0,
    gripper_y_offset_mm: float = -10.0,
    gripper_z_offset_mm: float = 100.0,
):
    """
    ✅ 입력: base->Gripper(목표)  [x,y,z,rx,ry,rz] (mm, deg), Euler intrinsic ZYX
    ✅ 출력: base->EE(send_coords) [x,y,z,rx,ry,rz] (mm, deg), Euler intrinsic ZYX

    관계:
      T_b2g = T_b2e * T_e2g  =>  T_b2e = T_b2g * inv(T_e2g)

    여기서 T_e2g:
      - 회전: EE z축 기준 gripper yaw 보정 (예: gripper가 +45° 돌아가 있으면 -45°로 보정)
      - 이동: EE frame 기준 y=-10mm, z=+100mm (gripper TCP 오프셋)
    """

    # ------------------------
    # 본 계산
    # ------------------------
    T_b2g = coords_mm_deg_to_T(gripper_coords_mm_deg)

    # EE -> Gripper (고정 오프셋)
    T_e2g = np.eye(4, dtype=np.float64)
    T_e2g[:3, :3] = _Rz(_deg2rad(gripper_z_offset_deg))  # yaw about EE z
    T_e2g[:3, 3] = np.array([0.0, float(gripper_y_offset_mm), float(gripper_z_offset_mm)], dtype=np.float64)

    # base->EE = base->Gripper * inv(EE->Gripper)
    T_b2e = T_b2g @ inv_T(T_e2g)
    return T_to_coords_mm_deg(T_b2e)


def apply_xyz_offset_to_sendcoords(
    coords_mm_deg,
    dx_mm: float,
    dy_mm: float,
    dz_mm: float,
):
    """
    ✅ 요청하신 동작 그대로:
    - coords(sendcoords) -> T_cmd
    - offset(dx,dy,dz) -> T_off (회전은 I)
    - 두 개 곱해서 T_new 만들기
    - T_new -> sendcoords 반환

    offset_frame:
      - "base"(default): base 좌표계 기준으로 평행이동 적용  => T_new = T_off @ T_cmd
      - "tool": tool(ee) 좌표계 기준으로 평행이동 적용      => T_new = T_cmd @ T_off
    """
    T_cmd = coords_mm_deg_to_T(coords_mm_deg)

    T_off = np.eye(4, dtype=np.float64)
    T_off[:3, 3] = np.array([float(dx_mm), float(dy_mm), float(dz_mm)], dtype=np.float64)
    # 회전은 identity로 유지

    T_new = T_cmd @ T_off

    return T_to_coords_mm_deg(T_new)



