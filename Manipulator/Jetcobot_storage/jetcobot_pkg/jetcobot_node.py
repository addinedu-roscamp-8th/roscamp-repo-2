#!/usr/bin/env python3
import time
import rclpy as rp
import math
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from std_msgs.msg import Bool
from jetcobot_interfaces.srv import CoordsAngles

from jetcobot_interfaces.action import Pick
from jetcobot_interfaces.action import MoveToPose
from jetcobot_interfaces.action import Place

from pymycobot.mycobot280 import MyCobot280

from jetcobot_pkg.utils.cobot_utils import(
    coords_replace_z,
    rotmat_to_euler_intrinsic_ZYX_deg,
    gripper_goal_to_ee_cmd_coords_mm_deg,
    apply_xyz_offset_to_sendcoords,
)

# =========================
# ✅ 전역 설정
# =========================

'''
단위: MM, deg 통일
frame 기준 잘 확인하기!!
'''

# action 이름
ACTION_PICK_NAME = "/pick"
ACTION_MOVETOPOSE_NAME = "/move_to_pose"
ACTION_PLACE_NAME = "/place"

# mycobot Port/Baud
MYCOBOT_PORT = "/dev/ttyJETCOBOT"
MYCOBOT_BAUD = 1000000

# mycobot 움직임 속도(0~100), 모드(0: angular, 1: linear)
MOVE_SPEED = 60         # 평소 속도
PICK_SPEED = 20         # 물체 집으러 내려가고 올라가는 속도
PLACE_SPEED = 30        # 물체를 놓을때 내려가고 올라가는 속도
MOVE_MODE = 0

# 그리퍼 입력값(0~100), 속도(0~100)
GRIP_CLOSE_VAL = 0     # 그리퍼 닫히는 값
GRIP_OPEN_VAL = 100     # 그리퍼 열리는 값
GRIP_SPEED = 50         # 그리퍼 열고 닫히는 속도

# Pick하기 위해 (EE frame)기준 Z축 방향으로 추가로 더 이동하는 좌표
PICK_Z_MM = 5.0

# end effector frame 기준 그리퍼 frame offset --> !!! 그리퍼를 다른거 쓰는 경우가 아니고서는 바꾸지 말것 !!!
GRIPPER_Z_OFFSET_DEG = -45.0
GRIPPER_Y_OFFSET_MM = -10.0
GRIPPER_Z_OFFSET_MM = 100.0

# 홈 포즈(원하면 수정)
HOME_ANGLES = [-90, 45, -90, -20, 0, 45]             # [deg, deg, deg, deg, deg, deg]
HOME_COORDS = [ -62.8 ,  -79.6 ,  257.4 , -164.41,   15.44,  139.1 ]  # [mm, mm, mm, deg, deg, deg](Base frame)

# pick/place 동작 시 안전 lift 높이 (mm) (Target frame)기준
LIFT_MM = 50.0

# 로봇 위치 판단 허용 오차값 - 작을수록 정밀 / backlash 때문에 너무 작은 값 비추천
POS_FLAG_THRESH_MM = 50.0

# 도달 체크 폴링
POLL_DT = 0.05           # 50ms
MOVE_TIMEOUT_SEC = 15.0  # 각 모션 최대 허용 시간(원하면 늘려도 됨)

# 실제 오차값 --> 최정 보정 오차 (오차가 적용된 frame에 알맞게 값 넣기)
OFFSET_FRAME = "Target" # OFFSET을 더해주는 기준 frame --> "Base" / "Target"
SELF_OFFSET_X_MM = 20.0
SELF_OFFSET_Y_MM = 2.0
SELF_OFFSET_Z_MM = 10.0

# 시나리오 맞춤 고정 Z (평면 물체 집기 시나리오)
FIXED_Z = True
BOX1_HEIGHT = 24.5
BASE_HEIGHT = 2.5
FIXED_Z_MM = BOX1_HEIGHT - BASE_HEIGHT

# =========================
# ✅ Jetcobot Node
# =========================
class JetcobotRobotActionServer(Node):
    def __init__(self):
        super().__init__("jetcobot_node")

        self.mc = MyCobot280(MYCOBOT_PORT, MYCOBOT_BAUD)

        # 시작 시 홈
        try:
            self.mc.focus_all_servos()
            self.mc.send_angles(HOME_ANGLES, MOVE_SPEED, MOVE_MODE)
            time.sleep(2.0)
            self.mc.set_color(255,0,0)
        except Exception as e:
            self.get_logger().error(f"Init HOME failed: {e}")

        # =================
        # 📡 ROS 통신 
        # =================  

        self.done_pub = self.create_publisher(Bool, "/action_done", 10)

        # services
        self.srv_mode = self.create_service(CoordsAngles, "/get_coords_angles", self.coords_angles_cb)

        self._pick_server = ActionServer(
            self,
            Pick,
            ACTION_PICK_NAME,
            execute_callback=self.execute_pick_cb,
            goal_callback=self.goal_pick_cb,
            cancel_callback=self.cancel_cb,
        )

        self._moveto_server = ActionServer(
            self,
            MoveToPose,
            ACTION_MOVETOPOSE_NAME,
            execute_callback=self.execute_movetopose_cb,
            goal_callback=self.goal_movetopose_cb,
            cancel_callback=self.cancel_cb,
        )

        self._place_server = ActionServer(
            self,
            Place,
            ACTION_PLACE_NAME,
            execute_callback=self.execute_place_cb,
            goal_callback=self.goal_place_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info("✅ Robot Action Server started")
        self.get_logger().info(f"- pick action name: {ACTION_PICK_NAME}")
        self.get_logger().info(f"- movetopose action name: {ACTION_MOVETOPOSE_NAME}")
        self.get_logger().info(f"- place action name: {ACTION_PLACE_NAME}")
        self.get_logger().info(f"- port: {MYCOBOT_PORT} baud:{MYCOBOT_BAUD}")
    
    # =================
    # 🖨️ Node 함수
    # =================

    # -------------------------
    # Service callback
    # -------------------------
    def coords_angles_cb(self, req: CoordsAngles.Request, res: CoordsAngles.Response):
        if req.type == 0:
            res.coords_angles = self.mc.get_angles()
            res.msg = "angles"
        elif req.type == 1:
            res.coords_angles = self.mc.get_coords()
            res.msg = "coords"
        else:
            res.coords_angles = [0,0,0,0,0,0]
            res.msg = "unknown type"
        return res

    # -------------------------
    # Goal accept/reject
    # -------------------------
    def goal_pick_cb(self, goal_request: Pick.Goal):
        if len(goal_request.pick_coords) != 6:
            self.get_logger().error("❌ Goal rejected: pick coords must be length 6")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def goal_place_cb(self, goal_request: Place.Goal):
        if len(goal_request.place_coords) != 6:
            self.get_logger().error("❌ Goal rejected: place coords must be length 6")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def goal_movetopose_cb(self, goal_request: MoveToPose.Goal):
        # MovetoPose는 send_coords 또는 send_angles 입력 6개 하나
        if len(goal_request.pose) != 6:
            self.get_logger().error("❌ Goal rejected: pose must be length 6")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().warn("⚠️ Cancel requested")
        return CancelResponse.ACCEPT

    # -------------------------
    # Feedback 함수
    # -------------------------
    def send_feedback(self, goal_handle, progress: float, state: str):
        # goal_handle.request 타입에 따라 Feedback 메시지 타입 선택
        if isinstance(goal_handle.request, Pick.Goal):
            fb = Pick.Feedback()
        elif isinstance(goal_handle.request, MoveToPose.Goal):
            fb = MoveToPose.Feedback()
        elif isinstance(goal_handle.request, Place.Goal):
            fb = Place.Feedback()
        else:
            # 예외: 예상 못한 goal 타입
            self.get_logger().error("Unknown action goal type in send_feedback()")
            return

        fb.progress = float(progress)
        fb.state = str(state)
        goal_handle.publish_feedback(fb)


    # -------------------------
    # ⭐⭐ Wait until in position ⭐⭐ (상태판단 함수)
    # -------------------------
    def wait_until_reached(self, goal_handle, target_coords, timeout_sec=MOVE_TIMEOUT_SEC):
        """
        target_coords: [x,y,z,rx,ry,rz] (mm,deg)
        is_in_position(data, flag)
          flag=1 -> coords
        return: (ok:bool, reason:str)
        """
        t0 = time.time()
        while rp.ok():
            if goal_handle.is_cancel_requested:
                return False, "CANCEL"

            done_flag = self.mc.is_moving()  # 1:true, 0:false, -1:error    
            current_coords = self.mc.get_coords()
            pos_flag = 1
            for i in range(0,3):
                if abs(target_coords[i] - current_coords[i]) > POS_FLAG_THRESH_MM: # 로봇 translation 위치만 확인 (회전X -> 특이점 발생)
                    pos_flag = 0
                    # print("value is over Thresh index:",i,", value", target_coords[i] - current_coords[i])

            # print("pos_flag:",pos_flag,", done_flag:",done_flag)
            if done_flag == 0 and pos_flag == 1:
                return True, "REACHED"

            if done_flag == -1:
                return False, "ERROR"

            if time.time() - t0 > timeout_sec:
                return False, str(self.mc.get_error_information())

            time.sleep(POLL_DT)

        return False, "ROS_NOT_OK"

    def sendcoords_flip_z(self, coords_mm_deg) -> np.ndarray:
        """
        입력: send_coords 형식 [x,y,z,rx,ry,rz] (mm, deg)
        - rx,ry,rz 는 intrinsic ZYX euler (deg) 로 해석
        출력: 3x3 Rotation matrix (np.float64)

        정의:
        R = Rz(rz) * Ry(ry) * Rx(rx)
        """
        if coords_mm_deg is None or len(coords_mm_deg) != 6:
            raise ValueError("coords_mm_deg must be length 6: [x,y,z,rx,ry,rz]")

        rx = float(coords_mm_deg[3]) * math.pi / 180.0
        ry = float(coords_mm_deg[4]) * math.pi / 180.0
        rz = float(coords_mm_deg[5]) * math.pi / 180.0


        _Rx = np.array([[1.0, 0.0, 0.0],
                     [0.0,  math.cos(rx), -math.sin(rx)],
                     [0.0,  math.sin(rx),  math.cos(rx)]], dtype=np.float64)
        
        _Ry = np.array([[ math.cos(ry), 0.0, math.sin(ry)],
                     [0.0, 1.0, 0.0],
                     [-math.sin(ry), 0.0, math.cos(ry)]], dtype=np.float64)
        
        _Rz = np.array([[math.cos(rz), -math.sin(rz), 0.0],
                     [math.sin(rz),  math.cos(rz), 0.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)

        R = _Rz @ _Ry @ _Rx

        R_flip_z_180 = np.array([
        [1.0,  0.0,  0.0],
        [0.0, 1.0,  0.0],
        [0.0,  0.0, -1.0],
        ], dtype=np.float64)

        Rm_cmd = R @ R_flip_z_180

        rx, ry, rz = rotmat_to_euler_intrinsic_ZYX_deg(Rm_cmd)

        return [float(coords_mm_deg[0]), float(coords_mm_deg[1]), float(coords_mm_deg[2]),
                    float(rx), float(ry), float(rz)]

    # -------------------------
    # ✅ Pick Action
    # -------------------------
    def execute_pick_cb(self, goal_handle):
        self.get_logger().info("🚀 Executing Pick action (0,1,2,3)")
        result = Pick.Result()
        result.success = False
        result.message = ""
        
        safe_pick = bool(goal_handle.request.safe)

        pick = list(goal_handle.request.pick_coords)

        if OFFSET_FRAME == "Target":
            pick = apply_xyz_offset_to_sendcoords(
                list(goal_handle.request.pick_coords),
                SELF_OFFSET_X_MM,
                SELF_OFFSET_Y_MM,
                SELF_OFFSET_Z_MM
                )
        elif OFFSET_FRAME == "Base":
            pick[0] += SELF_OFFSET_X_MM
            pick[1] += SELF_OFFSET_Y_MM
            pick[2] += SELF_OFFSET_Z_MM
        else:
            result.message = f"Unkown OFFSET_FRAME. Available OFFSET_FRAMEs = Target, Base "
            goal_handle.abort()
            return result
        
        if safe_pick:
            pre_pick = apply_xyz_offset_to_sendcoords(pick, 0.0, 0.0, LIFT_MM)

            # TCP 적용 (gripper 기준 목표 -> EE send_coords)
            pre_pick_coords = gripper_goal_to_ee_cmd_coords_mm_deg(
            self.sendcoords_flip_z(pre_pick),
            gripper_z_offset_deg=GRIPPER_Z_OFFSET_DEG,
            gripper_y_offset_mm=GRIPPER_Y_OFFSET_MM,
            gripper_z_offset_mm=GRIPPER_Z_OFFSET_MM-PICK_Z_MM,
            )

        # TCP 적용 (gripper 기준 목표 -> EE send_coords)
        pick_coords = gripper_goal_to_ee_cmd_coords_mm_deg(
            self.sendcoords_flip_z(pick),
            gripper_z_offset_deg=GRIPPER_Z_OFFSET_DEG,
            gripper_y_offset_mm=GRIPPER_Y_OFFSET_MM,
            gripper_z_offset_mm=GRIPPER_Z_OFFSET_MM-PICK_Z_MM,
        )

        if FIXED_Z:
            pick_coords = gripper_goal_to_ee_cmd_coords_mm_deg(
            coords_replace_z(self.sendcoords_flip_z(pick),FIXED_Z_MM),
            gripper_z_offset_deg=GRIPPER_Z_OFFSET_DEG,
            gripper_y_offset_mm=GRIPPER_Y_OFFSET_MM,
            gripper_z_offset_mm=GRIPPER_Z_OFFSET_MM-PICK_Z_MM,
        )

        self.mc.set_color(0,255,0) #action 수행 시작시 LED 초록색

        # 1) Gripper OPEN
        self.send_feedback(goal_handle, 20, "Gripper OPEN")
        try:
            self.mc.set_gripper_value(GRIP_OPEN_VAL, GRIP_SPEED)
            time.sleep(0.3)
        except Exception as e:
            result.message = f"Gripper open failed: {e}"
            goal_handle.abort()
            return result

        # 2) Safety - PRE-PICK (안전 높이)
        if safe_pick:
            try:
                self.send_feedback(goal_handle, 40, "Move PRE-PICK")
                self.mc.send_coords(pre_pick_coords, MOVE_SPEED, MOVE_MODE)
                ok, why = self.wait_until_reached(goal_handle, pre_pick_coords)
                if not ok:
                    result.message = f"Move PRE-PICK failed: {why}"
                    goal_handle.abort()
                    return result
            except Exception as e:
                result.message = f"Move PRE-PICK failed: {e}"
                goal_handle.abort()
                return result
            

        # 3) PICK (내려가기)
        try:
            self.send_feedback(goal_handle, 60, "Move PICK")
            self.mc.send_coords(pick_coords, PICK_SPEED, MOVE_MODE)
            ok, why = self.wait_until_reached(goal_handle, pick_coords)
            if not ok:
                result.message = f"Move PICK failed: {why}"
                goal_handle.abort()
                return result
        except Exception as e:
            result.message = f"Move PICK failed: {e}"
            goal_handle.abort()
            return result

        # 4) Gripper CLOSE
        self.send_feedback(goal_handle, 80, "Gripper OPEN")
        try:
            self.mc.set_gripper_value(GRIP_CLOSE_VAL, GRIP_SPEED)
            time.sleep(0.4)
        except Exception as e:
            result.message = f"Gripper open failed: {e}"
            goal_handle.abort()
            return result

        # 5) Safety - 다시 PRE-PICK (안전 높이)
        if safe_pick:
            try:
                self.send_feedback(goal_handle, 100, "Move PRE-PICK")
                self.mc.send_coords(pre_pick_coords, PICK_SPEED, MOVE_MODE)
                ok, why = self.wait_until_reached(goal_handle, pre_pick_coords)
                if not ok:
                    result.message = f"Move PRE-PICK failed: {why}"
                    goal_handle.abort()
                    return result
            except Exception as e:
                result.message = f"Move PRE-PICK failed: {e}"
                goal_handle.abort()
                return result

        # DONE
        self.mc.set_color(255,0,0) # action 종료시 LED 빨간색
        self.send_feedback(goal_handle, 100, "Done ✅")
        result.success = True
        result.message = "Pick done (checked)"

        goal_handle.succeed()

        done_msg = Bool()
        done_msg.data = True
        self.done_pub.publish(done_msg)

        return result

    # -------------------------
    # ✅ MovetoPose Action (입력 좌표 하나)
    # -------------------------
    def execute_movetopose_cb(self, goal_handle):
        self.get_logger().info("🚀 Executing MovetoPose action")

        pose = list(goal_handle.request.pose)  # [6]
        use_angles = bool(goal_handle.request.use_angles)

        result = MoveToPose.Result()
        result.success = False
        result.message = ""
        self.mc.set_color(0,255,0) # action 시작시 LED 초록색
        

        # ✅ send_angles 모드
        if use_angles:
            self.send_feedback(goal_handle, 50, "Move ANGLES")
            try:
                self.mc.send_angles(pose, MOVE_SPEED, MOVE_MODE)
                time.sleep(0.1)
            except Exception as e:
                result.message = f"send_angles failed: {e}"
                goal_handle.abort()
                return result
            pose_coords = self.mc.angles_to_coords(pose)

            # =========================================================================
            #  백래쉬로 인해 angles_to_coords 모듈의 결과값 오차가 10cm 이상 나서 해둔 설정
            if pose == HOME_ANGLES:
                pose_coords = HOME_COORDS
            # =========================================================================

            ok, why = self.wait_until_reached(goal_handle, pose_coords)
            if not ok:
                result.message = f"Move COORDS failed: {why}"
                goal_handle.abort()
                return result

        # ✅ send_coords 모드
        else:
            self.send_feedback(goal_handle, 50, "Move COORDS")
            try:
                # TCP 적용 (gripper 기준 목표 -> EE send_coords)
                pose = gripper_goal_to_ee_cmd_coords_mm_deg(
                    pose,
                    gripper_z_offset_deg=GRIPPER_Z_OFFSET_DEG,
                    gripper_y_offset_mm=GRIPPER_Y_OFFSET_MM,
                    gripper_z_offset_mm=GRIPPER_Z_OFFSET_MM-PICK_Z_MM,
                )
                self.mc.send_coords(pose, MOVE_SPEED, MOVE_MODE)
                ok, why = self.wait_until_reached(goal_handle, pose)
                if not ok:
                    result.message = f"Move COORDS failed: {why}"
                    goal_handle.abort()
                    return result
                
            except Exception as e:
                result.message = f"send_coords failed: {e}"
                goal_handle.abort()
                return result

        # DONE
        self.mc.set_color(255,0,0) # action 종료시 LED 빨간색
        self.send_feedback(goal_handle, 100, "Done ✅")
        result.success = True
        result.message = "MovetoPose done (checked)"

        goal_handle.succeed()

        done_msg = Bool()
        done_msg.data = True
        self.done_pub.publish(done_msg)

        return result

    # -------------------------
    # ✅ Place Action 
    # -------------------------
    def execute_place_cb(self, goal_handle):
        self.get_logger().info("🚀 Executing Place action")

        place = list(goal_handle.request.place_coords)
        safe_place = bool(goal_handle.request.safe)

        # TCP 적용 (gripper 기준 목표 -> EE send_coords)
        place_coords = gripper_goal_to_ee_cmd_coords_mm_deg(
            place,
            gripper_z_offset_deg=GRIPPER_Z_OFFSET_DEG,
            gripper_y_offset_mm=GRIPPER_Y_OFFSET_MM,
            gripper_z_offset_mm=GRIPPER_Z_OFFSET_MM-PICK_Z_MM,
        )

        if safe_place:
            pre_pick = apply_xyz_offset_to_sendcoords(place, 0.0, 0.0, -LIFT_MM)

            # TCP 적용 (gripper 기준 목표 -> EE send_coords)
            pre_place_coords = gripper_goal_to_ee_cmd_coords_mm_deg(
            pre_pick,
            gripper_z_offset_deg=GRIPPER_Z_OFFSET_DEG,
            gripper_y_offset_mm=GRIPPER_Y_OFFSET_MM,
            gripper_z_offset_mm=GRIPPER_Z_OFFSET_MM-PICK_Z_MM,
            )
        
        result = Place.Result()
        result.success = False
        result.message = ""
        self.mc.set_color(0,255,0) # action 시작시 LED 초록색

        # 1) Safety - PRE-PLACE
        if safe_place:
            try:
                self.send_feedback(goal_handle, 20, "Move PRE-PLACE")
                self.mc.send_coords(pre_place_coords, MOVE_SPEED, MOVE_MODE)
                ok, why = self.wait_until_reached(goal_handle, pre_place_coords)
                if not ok:
                    result.message = f"Move PRE-PLACE failed: {why}"
                    goal_handle.abort()
                    return result
            except Exception as e:
                result.message = f"Move PRE-PLACE failed: {e}"
                goal_handle.abort()
                return result

        # 2) PLACE (내려가기)
        try:
            self.send_feedback(goal_handle, 40, "Move PLACE")
            self.mc.send_coords(place_coords, PLACE_SPEED, MOVE_MODE)
            ok, why = self.wait_until_reached(goal_handle, place_coords)
            if not ok:
                result.message = f"Move PLACE failed: {why}"
                goal_handle.abort()
                return result
        except Exception as e:
            result.message = f"Move PLACE failed: {e}"
            goal_handle.abort()
            return result

        # 3) Gripper OPEN (release)
        self.send_feedback(goal_handle, 60, "Gripper OPEN (release)")
        try:
            self.mc.set_gripper_value(GRIP_OPEN_VAL, GRIP_SPEED)
            time.sleep(0.4)
        except Exception as e:
            result.message = f"Gripper open(release) failed: {e}"
            goal_handle.abort()
            return result
        
        # 4) Safety - 다시 Pre-PLACE (올라가기)
        if safe_place:
            try:
                self.send_feedback(goal_handle, 80, "Move PRE-PLACE")
                self.mc.send_coords(pre_place_coords, PLACE_SPEED, MOVE_MODE)
                ok, why = self.wait_until_reached(goal_handle, pre_place_coords)
                if not ok:
                    result.message = f"Move PRE-PLACE failed: {why}"
                    goal_handle.abort()
                    return result
            except Exception as e:
                result.message = f"Move PRE-PLACE failed: {e}"
                goal_handle.abort()
                return result
            
        # DONE
        self.mc.set_color(255,0,0) # action 종료시 LED 빨간색
        self.send_feedback(goal_handle, 100, "Done ✅")
        result.success = True
        result.message = "Place done (checked)"

        goal_handle.succeed()

        done_msg = Bool()
        done_msg.data = True
        self.done_pub.publish(done_msg)

        return result


def main(args=None):
    rp.init(args=args)
    node = JetcobotRobotActionServer()
    try:
        rp.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rp.shutdown()


if __name__ == "__main__":
    main()
