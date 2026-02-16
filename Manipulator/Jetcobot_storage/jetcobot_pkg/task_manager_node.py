#!/usr/bin/env python3
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Int32, String

from jetcobot_interfaces.msg import (
    PartArray, 
    SectionResult, 
    StorageRequest, 
    StorageResponse,
    ManualRequest,
    ManualResponse,
)

from jetcobot_pkg.utils.cobot_utils import (
    pose_mm_to_xyz_quat,
    quat_normalize,
    quat_mean_xyzw,
    quat_to_rotmat,
    rotmat_to_euler_intrinsic_ZYX_deg,
)

from jetcobot_pkg.utils.jetcobot_action_client import (
    PickClient,
    MoveToPoseClient,
    PlaceClient
)

# =========================
# ✅ 전역 설정
# =========================
# publish
STORAGE_TOPIC = "/jetcobot/storage/start"
STORAGE_AUTO_REQUEST_TOPIC = "/jetcobot/storage/auto/request"
STORAGE_MANUAL_RESEPONSE_TOPIC = "/jetcobot/storage/manual/response"
DB_UPDATE_TOPIC = "/jetcobot/storage/db_update"
STORAGE_ROBOT_STATUS_TOPIC = "/jetcobot/storage/status"

# subscribe
STORAGE_ROBOT_BOOT_TOPIC = "/jetcobot/storage/boot"
STORAGE_SET_MODE_TOPIC = "/jetcobot/storage/set_mode"
PARTS_TOPIC = "/jetcobot/storage/camera/parts"
ASSEMBLY_TOPIC = "/jetcobot/assembly/start"
STORAGE_AUTO_RESPONSE_TOPIC= "/jetcobot/storage/auto/response"
STORAGE_MANUAL_REQUEST_TOPIC = "/jetcobot/storage/manual/request"

AUTO_STABLE_TIME_SEC = 2.0
SAMPLE_N = 20

WAITING_ANGLES = [90, 90, -90, -50, 0, 45]
HOME_ANGLES = [-90, 45, -90, -20, 0, 45]
SAFE_ANGLES = [0, 90, -90, -50, 0, 45]

BOX_HEIGHT = 24.5
BASE_HEIGHT = 2.5

PLACE_Z_MM = BOX_HEIGHT - BASE_HEIGHT

# x -70 0 70
# y 180 220 260

SECTION_EDGE_X = -70
SECTION_EDGE_Y = 180

SECTION_DIST_X = 70
SECTION_DIST_Y = 40 

SECTION_A_LIST = [
    [ SECTION_EDGE_X+SECTION_DIST_X*2, SECTION_EDGE_Y+SECTION_DIST_Y*2, PLACE_Z_MM, 180.0, 0.0, 180.0], 
    [ SECTION_EDGE_X+SECTION_DIST_X*2, SECTION_EDGE_Y+SECTION_DIST_Y*1, PLACE_Z_MM, 180.0, 0.0, 180.0], 
    [ SECTION_EDGE_X+SECTION_DIST_X*2, SECTION_EDGE_Y+SECTION_DIST_Y*0, PLACE_Z_MM, 180.0, 0.0, 180.0], 
] # id 1

SECTION_B_LIST = [
    [ SECTION_EDGE_X+SECTION_DIST_X*1, SECTION_EDGE_Y+SECTION_DIST_Y*2, PLACE_Z_MM, 180.0, 0.0, 180.0], 
    [ SECTION_EDGE_X+SECTION_DIST_X*1, SECTION_EDGE_Y+SECTION_DIST_Y*1, PLACE_Z_MM, 180.0, 0.0, 180.0], 
    [ SECTION_EDGE_X+SECTION_DIST_X*1, SECTION_EDGE_Y+SECTION_DIST_Y*0, PLACE_Z_MM, 180.0, 0.0, 180.0], 
] # id 2

SECTION_C_LIST = [
    [ SECTION_EDGE_X+SECTION_DIST_X*0, SECTION_EDGE_Y+SECTION_DIST_Y*2, PLACE_Z_MM, 180.0, 0.0, 180.0], 
    [ SECTION_EDGE_X+SECTION_DIST_X*0, SECTION_EDGE_Y+SECTION_DIST_Y*1, PLACE_Z_MM, 180.0, 0.0, 180.0], 
    [ SECTION_EDGE_X+SECTION_DIST_X*0, SECTION_EDGE_Y+SECTION_DIST_Y*0, PLACE_Z_MM, 180.0, 0.0, 180.0], 
] # id 3

PLACE_COORDS_LIST = [
    [ -70.0, 180.0, PLACE_Z_MM, 180.0, 0.0, 180.0],  # id 1
    [ -70.0, 260.0, PLACE_Z_MM, 180.0, 0.0, 180.0],  # id 2
    [ 70.0, 180.0, PLACE_Z_MM, 180.0, 0.0, 180.0],  # id 3
]

SAFE_PLACE_COORDS = [ 200.0, 0.0, PLACE_Z_MM, 180.0, 0.0, 0.0]

TICK_HZ = 20.0

REQUEST_TIMEOUT = 10.0 # sec

AUTO = 0
MANUAL = 1

# =========================
# ✅ 유틸 함수
# =========================
def robust_estimate_coords_mm(pose_mm_list):
    """
    return: [x,y,z,rx,ry,rz] (mm, deg)
      - xyz median
      - quat mean -> base->target rotation
      - target +Z와 반대(-Z) 방향으로 자세 뒤집기(Rx 180)
    """
    if len(pose_mm_list) == 0:
        return None

    xyz_list = []
    q_list = []

    for p in pose_mm_list:
        xyz, q = pose_mm_to_xyz_quat(p)
        xyz_list.append(xyz)
        q_list.append(quat_normalize(q))

    xyz_stack = np.stack(xyz_list, axis=0)
    xyz_med = np.median(xyz_stack, axis=0)

    q_avg = quat_mean_xyzw(q_list)
    if q_avg is None:
        return None

    Rm = quat_to_rotmat(q_avg)

    rx, ry, rz = rotmat_to_euler_intrinsic_ZYX_deg(Rm)

    return [float(xyz_med[0]), float(xyz_med[1]), float(xyz_med[2]),
            float(rx), float(ry), float(rz)]


# =========================
# ✅ Task Manager Node
# =========================
class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager_node")

        # ================= 
        # ✖️ Class 변수 
        # =================
        
        # 상위 상태
        self.state = "IDLE" # timer 총괄 상태: IDLE / SAMPLING / EXECUTING / ...
        self.set_mode = AUTO # auto / manual mode

        # cobot 통신
        self.msg = Bool() # storage cobot 동작 상태(pub)
        self.msg.data = False
        self.assembly_start = False # assembly cobot 동작 상태(sub)

        # DB 통신
        self.str_req_sent = False # DB 에게 보관 위치 요청 확인 트리거
        self.place_failed = False # place 실패

        # 부팅 상태
        self._booted = False # 로봇 부팅 여부
        self.shutdown = None # 로봇 shutdown 트리거
        
        # sample 측정용 변수
        self.parts = {}
        self.candidates = []
        self.selected_id = None
        self.sample_buf = []

        # pick, place coords 저장 변수
        self.str_id = None # 보관 section id
        self.selected_section = None # 보관 section
        self.place_coords = None # 보관 좌표
        self.pick_coords = None # 부품 좌표

        # 기타
        self.sent_time = None # 요청을 보낸 시간 저장
        self.pub_once = None # 단일 publish 확인
        self.cnt = 0 # candidate 파싱 누적 카운트 변수

        # =================
        # 📡 ROS 통신 
        # =================

        #topics
        self.pub_start = self.create_publisher(Bool, STORAGE_TOPIC, 10)
        self.pub_str_req = self.create_publisher(StorageRequest, STORAGE_AUTO_REQUEST_TOPIC, 10)
        self.pub_sec_res = self.create_publisher(SectionResult, DB_UPDATE_TOPIC, 10)
        self.pub_man_res = self.create_publisher(ManualResponse,STORAGE_MANUAL_RESEPONSE_TOPIC , 10)
        self.pub_state = self.create_publisher(String, STORAGE_ROBOT_STATUS_TOPIC, 10)

        # subscribe
        self.sub_parts = self.create_subscription(PartArray, PARTS_TOPIC, self.cb_parts, 10)
        self.sub_assembly = self.create_subscription(Bool, ASSEMBLY_TOPIC, self.cb_cobotcomms, 10)
        self.sub_str_res = self.create_subscription(StorageResponse, STORAGE_AUTO_RESPONSE_TOPIC, self.cb_response, 10)
        self.sub_manual_req = self.create_subscription(ManualRequest,STORAGE_MANUAL_REQUEST_TOPIC, self.cb_manual, 10)
        self.sub_mode = self.create_subscription(Int32, STORAGE_SET_MODE_TOPIC, self.cb_set_mode, 10)
        self.boot = self.create_subscription(Bool, STORAGE_ROBOT_BOOT_TOPIC, self.cb_sub_boot, 10)
        
        # action clients
        self.pick_cli = PickClient(self, action_name="/pick")
        self.move_cli = MoveToPoseClient(self, action_name="/move_to_pose")
        self.place_cli = PlaceClient(self, action_name="/place")

        self.get_logger().info("✅ TaskManagerNode started")
        self.get_logger().info("✅ Default Settings: Mode -> Auto")
        self.get_logger().info("✋ Waiting for boot trigger...")

    # =================
    # 🖨️ Node 함수
    # =================

    # ---------------
    # ✅ 유틸 함수
    # ---------------
    def section_to_placecoords(self, section, id): # 섹션을 좌표로 변환 시켜주는 유틸함수
        if section == 'A':
            place_coords = list(SECTION_A_LIST[id-1])
        elif section == 'B':
            place_coords = list(SECTION_B_LIST[id-1])
        elif section == 'C':
            place_coords = list(SECTION_C_LIST[id-1])
        else:
            place_coords = list(SAFE_PLACE_COORDS)

        return place_coords
    
    def shutdown_postponed(self):
        self._booted = False
        self.shutdown = None
        self.timer.cancel()
        self.get_logger().info("Shutdown received -> Canceled main timer")

    def state_msg(self):
        if self.state == "IDLE":
            return "IDLE"
        else:
            return "BUSY"


    # ---------------
    # ✅ 콜백 함수
    # ---------------
    def cb_sub_boot(self, msg:Bool): 

        if not self._booted and msg.data: # msg -> True, Boot up
            self._booted = True
            self.get_logger().info("BOOT received -> starting main timer")
            self.timer = self.create_timer(1.0 / TICK_HZ, self.tick)

        if self._booted and not msg.data: # msg -> False, Shutdown
            if self.state != "IDLE": 
                self.get_logger().info("Shutdown received -> Robot is Busy, Timer will be canceled when state is IDLE")
                self.shutdown = True
                return
            self._booted = False
            self.timer.cancel()
            self.get_logger().info("Shutdown received -> Canceled main timer")

    def cb_parts(self, msg: PartArray):
        for part in msg.parts:
            self.parts[int(part.id)] = {
                "pose_mm": part.pose_mm,
                "ready": bool(part.ready_to_pick),
                "stable": float(part.stable_time_sec)
            }

        if self.state == "SAMPLING" and self.selected_id is not None:
            if self.selected_id in self.parts:
                self.pub_once = None
                p = self.parts[self.selected_id]["pose_mm"]
                self.sample_buf.append(p)
                if len(self.sample_buf) > SAMPLE_N:
                    self.sample_buf = self.sample_buf[-SAMPLE_N:]
            if self.selected_id not in self.parts:
                if self.set_mode == MANUAL and self.pub_once is None:
                    man_msg = ManualResponse()
                    man_msg.success = False
                    man_msg.msg = "The Requested Part cannot be detected. Please check the working field"
                    self.pub_man_res.publish(man_msg) 
                    self.pub_once = True
                if self.set_mode == AUTO:
                    # error msg
                    return
                
    def cb_cobotcomms(self, msg: Bool):
        # if not self.state == 'EXECUTING_WAIT':
        #     return
        self.assembly_start = bool(msg.data)

    def cb_response(self, msg: StorageResponse):
        str_section = msg.section
        self.str_id = msg.id
        if self.str_id is None:
            self.get_logger().warn("The Storage Area is full, Sampling the next candidate")
            self.cnt += 1
            self.selected_id = self.candidates[self.cnt]
            # self.state = "SAMPLING"
            return
        self.place_coords = self.section_to_placecoords(str_section, self.str_id)

    def cb_set_mode(self, msg: Int32):
        self.set_mode = int(msg.data) # 0:auto, 1:manual
        if self.set_mode == MANUAL:
            self.get_logger().info("Robot is set to Manual Mode.")
        if self.set_mode == AUTO:
            self.get_logger().info("Robot is set to Auto Mode.")

    def cb_manual(self, msg:ManualRequest):
        man_msg = ManualResponse()

        if self.set_mode != MANUAL:
            man_msg.success = False
            man_msg.msg = "Robot is currently in Auto Mode, Please Set to Manual!"
            self.pub_man_res.publish(man_msg) 
            return
        
        if self.state != "IDLE":
            man_msg.success = False
            man_msg.msg = "Robot is currently Busy. Please wait until the current task is done!"
            self.pub_man_res.publish(man_msg)        
            return 

        task_id = int(msg.task_id)
        part_id = int(msg.part_id)
        self.selected_section = str(msg.section)
        self.str_id  = int(msg.section_id)

        if task_id == 1:
            self.selected_id = part_id*1000 + 1
            self.sample_buf = []
            self.state = "SAMPLING"
            self.place_coords = self.section_to_placecoords(self.selected_section, self.str_id)

    # ---------------
    # ✅ 메인 타이머
    # ---------------
    def tick(self):
        # 동기 topic publish
        state = String()
        state.data = self.state_msg()

        self.pub_start.publish(self.msg)
        self.pub_state.publish(state)

        # ✅ IDLE
        if self.state == "IDLE":
            self.msg.data = False
            if self.shutdown:
                self.shutdown_postponed()

            if self.set_mode == AUTO:
                self.candidates = [pid for pid, info in self.parts.items() if info["stable"] >= AUTO_STABLE_TIME_SEC]
                if not self.candidates:
                    return
                self.candidates.sort() # id 빠른 순서로 id 선정
                chosen = self.candidates[0] 
                self.selected_id = chosen
                self.sample_buf = []
                self.state = "SAMPLING"
                return
            
            if self.set_mode == MANUAL:
                if self.pub_once is None:
                    man_msg = ManualResponse()
                    man_msg.success = False
                    man_msg.msg = "Robot is ready for a manual request!"
                    self.pub_man_res.publish(man_msg)
                    self.pub_once = True
                return


        # ✅ SAMPLING
        if self.state == "SAMPLING":
            self.msg.data = False

            if self.selected_id is None:
                self.state = "IDLE"
                return

            if len(self.sample_buf) < SAMPLE_N:
                return
            
            if self.set_mode == AUTO: # auto mode
                if not self.str_req_sent:
                    # publish storage request   
                    req_msg = StorageRequest()
                    
                    if self.selected_id // 1000 == 1:
                        self.selected_section = "A"
                    elif self.selected_id // 1000 == 2:
                        self.selected_section = "B"
                    elif self.selected_id // 1000 == 3:
                        self.selected_section = "C"
                    else:
                        self.get_logger().error("Undefined Object Detected!")
                        self._reset_to_idle()
                        return
                    req_msg.section = self.selected_section
                    req_msg.task_id = 1 # 1: place
                    self.pub_str_req.publish(req_msg) # publish
                    self.get_logger().info("Place Request Sent to Storage Database!")
                    self.str_req_sent = True

                    self.sent_time = time.time()
                elif time.time() - self.sent_time > REQUEST_TIMEOUT:
                    self.str_req_sent = False
                    self.get_logger().warn("Place Request Time Out! Returning to IDLE State")
                    self._reset_to_idle()

                if self.place_coords is None:
                    return
                self.sent_time = None
                

            pick_coords = robust_estimate_coords_mm(self.sample_buf[:SAMPLE_N])
            if pick_coords is None:
                self._reset_to_idle()
                return

            self.pick_coords = pick_coords
            self.safe_pick = True
            self.safe_place = True

            if not self.pick_cli.send_goal(self.pick_coords, self.safe_pick):
                self.get_logger().error("send_goal failed.. Trying Again")
                return
            
            if self.set_mode == MANUAL:
                man_msg = ManualResponse()
                man_msg.success = False
                man_msg.msg = "The Request is Sent and in Action!"
                self.pub_man_res.publish(man_msg)


            self.state = "EXECUTING_PICK"
            return

        # ✅ EXECUTING_PICK
        if self.state == "EXECUTING_PICK":
            self.msg.data = False

            # pick action done 확인
            if not self._is_action_done(self.pick_cli.action_done()):
                return
            
            # move to pose action 보내기
            if not self.move_cli.send_goal_angles(WAITING_ANGLES):
                self.get_logger().error("send_goal failed.. Trying Again")
                return

            self.state = "EXECUTING_WAIT_POSE"
            return

        # ✅ EXECUTING_WAIT_POSE
        if self.state == "EXECUTING_WAIT_POSE":
            self.msg.data = False

            if not self._is_action_done(self.move_cli.action_done()):
                return

            if self.assembly_start == True:
                self.get_logger().info(f"[TASK DONE] Waiting until Assembly Cobot Leaves Storage Area..")
                self.state = "EXECUTING_WAITING"
                return

            if not self.place_cli.send_goal(self.place_coords, self.safe_place):
                self.get_logger().error("send_goal failed.. Trying Again")
                return

            self.msg.data = True
            self.state = "EXECUTING_PLACE"
            return

        # ✅ EXECUTING_WAITING
        if self.state == "EXECUTING_WAITING":
            self.msg.data = False

            if self.assembly_start == False:
                if not self.place_cli.send_goal(self.place_coords, self.safe_place):
                    self.get_logger().error("send_goal failed.. Trying Again")
                    return

                self.msg.data = True
                self.state = "EXECUTING_PLACE"
            return

        # ✅ EXECUTING_PLACE
        if self.state == "EXECUTING_PLACE":
            self.msg.data = True

            if not self._is_action_done(self.place_cli.action_done()):
                return
            
            if not self.place_failed:
                res_msg = SectionResult()
                res_msg.section = self.selected_section
                res_msg.id = self.str_id
                res_msg.occupy = 1
                self.pub_sec_res.publish(res_msg) # publish section_result
                self.get_logger().info("Storage Section Result is Sent to Storage Database!")
                if self.set_mode == MANUAL:
                    man_msg = ManualResponse()
                    man_msg.success = True
                    man_msg.msg = "The Request is successfully done! Wait for a new Request to be done"
                    self.pub_man_res.publish(man_msg)


            if not self.move_cli.send_goal_angles(HOME_ANGLES):
                self.get_logger().error("send_goal failed.. Trying Again")
                return

            self.msg.data = False
            self.state = "EXECUTING_HOME_POSE"
            return

        # ✅ EXECUTING_HOME_POSE
        if self.state == "EXECUTING_HOME_POSE":
            self.msg.data = False

            if not self._is_action_done(self.move_cli.action_done()):
                return

            self._reset_to_idle()
            return
        
        # ✅ EXECUTING_SAFE_MOVE
        if self.state == "EXECUTING_SAFE_MOVE":
            self.msg.data = False

            if not self._is_action_done(self.move_cli.action_done()):
                self.get_logger().error("send_goal failed.. Trying Again")
                return
            
            self.safe_place = True
            if not self.place_cli.send_goal(SAFE_PLACE_COORDS, self.safe_place):
                self.get_logger().error("send_goal failed.. Trying Again")
                return

            self.msg.data = False
            self.state = "EXECUTING_PLACE"
            return
        
    def _is_action_done(self, done):

        if done is None:
            return False
        success, message = done
        if success:
            self.get_logger().info(f"[TASK DONE] success={success} msg={message}")
            return True
        else:
            self.get_logger().error(f"[TASK FAIL] success={success} msg={message}")
            self.place_failed = True
            if self.state == "EXECUTING_PICK":
                self.get_logger().error(f"[TASK FAIL] Object is out of Cobot's Range. Replace Object! <Returning to Scanning State> 💨")
            self._reset_to_idle()
            return False

    def _reset_to_idle(self): # 상태 초기화 --> 만약 작업 중간에 끊기면 원상 복구 작업도 진행

        if self.place_failed:
            res_msg = SectionResult()
            res_msg.section = self.selected_section
            res_msg.id = self.str_id
            res_msg.occupy = 0
            self.pub_sec_res.publish(res_msg) # publish section_result
            self.get_logger().info("Storage Section Result is Sent to Storage Database!")
            if self.set_mode is MANUAL:
                man_msg = ManualResponse()
                man_msg.success = True
                man_msg.msg = "The Request has failed during progress. Wait until to send a new request"
                self.pub_man_res.publish(man_msg)

        if self.state in ("EXECUTING_WAIT_POSE", "EXECUTING_PLACE"):
            self.msg.data = False # action을 실패했기에 False로 변경
            if not self.move_cli.send_goal_angles(SAFE_ANGLES): # SAFE_ANGLES는 실패 안한다고 가정
                self.get_logger().error("🛑 Safety Measures failed.. Breaking System, 👷 Manual Assistance Needed") # 추후 시스템 정지, 경고 보내는 기능 여기에 추가
                # self.state = "BREAK"
            self.state = "EXECUTING_SAFE_MOVE"
            return

        else: 
            self.state = "IDLE"



        # Class 변수 초기화        
        self.selected_id = None
        self.str_id = None
        self.selected_section = None
        self.sample_buf = []
        self.parts = {}  # ✅ 누적 방지: DB 초기화
        self.pick_coords = None
        self.place_coords = None
        self.place_failed = False
        self.str_req_sent = False
        self.pub_once = None
        self.cnt = 0


def main():
    rclpy.init()
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
