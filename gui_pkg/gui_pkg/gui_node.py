import json  # ★ JSON 처리를 위해 추가
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Int32, Float32, Bool
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped

import mysql.connector
from PyQt6.QtCore import QThread, pyqtSignal

# 1. 모바일 로봇 관련
T_MOBILE_PREFIX = "/pinky"

T_SUFFIX_POSE    = "/amcl_pose"    # 위치 정보 (PoseWithCovarianceStamped)
T_SUFFIX_BATTERY = "/battery/present" # 배터리 정보 (Float32)
T_SUFFIX_STATE   = "/state"   # 상태 정보 (String)
T_SUFFIX_CMD     = "/cmd"     # 이동 명령 (String)
T_SUFFIX_LOAD_DONE = "/load_done"   # 상차 완료 (Bool)
T_SUFFIX_UNLOAD_DONE = "/unload_done" # 하차 완료 (Bool)
T_SUFFIX_MOVE_ROLE = "/move_role"   # 이동 역할 (String)

# 네임스페이스 없이 로봇 1대만 테스트할 때 사용
SINGLE_ROBOT_MODE = False
SINGLE_ROBOT_ID = "pinky1"
SINGLE_POSE_TOPIC = "/amcl_pose"
SINGLE_BATTERY_TOPIC = "/battery/present"
SINGLE_STATE_TOPIC = "/state"
SINGLE_CMD_TOPIC = "/cmd"
SINGLE_LOAD_DONE_TOPIC = "/load_done"
SINGLE_UNLOAD_DONE_TOPIC = "/unload_done"
SINGLE_MOVE_ROLE_TOPIC = "/move_role"

# 2. 로봇팔 관련
T_ARM_UNLOAD_SIGNAL = "/warehouse/unload"     # [수신] 로봇팔이 출고 완료했을 때 (Int)
T_ARM_TARGET_SLOT   = "/robot_arm/target_slot" # [송신] 로봇팔에게 "여기다 넣어" 명령 (String)

# 3. Jetcobot 자동 창고 시스템 (String/JSON 통신) -추가
T_JETCO_REQ = "/jetcobot/storage/auto/request"   # [수신] 요청
T_JETCO_RES = "/jetcobot/storage/auto/response"  # [발행] 응답
T_JETCO_UPD = "/jetcobot/db_update"              # [수신] DB 업데이트

# 4. DB 설정
DB_HOST = 'localhost'
DB_USER = 'root'
DB_PASS = '1234'
DB_NAME = 'smart_factory'

# 5. 지도 설정
REAL_MAP_WIDTH_CM = 203
REAL_MAP_HEIGHT_CM = 83
MAP_OFFSET_X_CM = REAL_MAP_WIDTH_CM / 2.0
MAP_OFFSET_Y_CM = REAL_MAP_HEIGHT_CM / 2.0

# ★ [핵심] GUI 노드 (ROS 2 통신 담당)
class GuiNode(QThread):
    robot_update_signal = pyqtSignal(dict)  
    unload_signal = pyqtSignal(int)
    jetco_log_signal = pyqtSignal(str) # Jetcobot 로그용 신호

    def __init__(self):
        super().__init__()
        self.node = None
        self.running = True
        self.cmd_pubs = {} 
        self.load_done_pubs = {}
        self.unload_done_pubs = {}
        self.move_role_pubs = {}
        self.arm_pub = None
        self.robot_role_assignments = {}
        self.jetco_res_pub = None # Response 발행용

    def run(self):
        rclpy.init()
        self.node = Node('smartfactory_gui_node')
        amcl_pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # 1. Pinky 로봇 설정
        if SINGLE_ROBOT_MODE:
            # (기존 SINGLE 모드 코드 생략 없이 유지)
            robot_name = SINGLE_ROBOT_ID
            self.node.create_subscription(PoseWithCovarianceStamped, SINGLE_POSE_TOPIC,
                                          lambda m, r=robot_name: self.pose_callback(m, r), amcl_pose_qos)
            self.node.create_subscription(Float32, SINGLE_BATTERY_TOPIC,
                                          lambda m, r=robot_name: self.battery_callback(m, r), 10)
            self.node.create_subscription(String, SINGLE_STATE_TOPIC,
                                          lambda m, r=robot_name: self.state_callback(m, r), 10)
            self.cmd_pubs[f"/{robot_name}"] = self.node.create_publisher(String, SINGLE_CMD_TOPIC, 10)
            self.load_done_pubs[f"/{robot_name}"] = self.node.create_publisher(Bool, SINGLE_LOAD_DONE_TOPIC, 10)
            self.unload_done_pubs[f"/{robot_name}"] = self.node.create_publisher(Bool, SINGLE_UNLOAD_DONE_TOPIC, 10)
            self.move_role_pubs[f"/{robot_name}"] = self.node.create_publisher(String, SINGLE_MOVE_ROLE_TOPIC, 10)
        else:
            for i in range(1, 4):
                robot_name = f"{T_MOBILE_PREFIX}{i}"
                self.node.create_subscription(PoseWithCovarianceStamped, f"{robot_name}{T_SUFFIX_POSE}",
                                              lambda m, r=robot_name: self.pose_callback(m, r), amcl_pose_qos)
                self.node.create_subscription(Float32, f"{robot_name}{T_SUFFIX_BATTERY}",
                                              lambda m, r=robot_name: self.battery_callback(m, r), 10)
                self.node.create_subscription(String, f"{robot_name}{T_SUFFIX_STATE}",
                                              lambda m, r=robot_name: self.state_callback(m, r), 10)
                self.cmd_pubs[robot_name] = self.node.create_publisher(String, f"{robot_name}{T_SUFFIX_CMD}", 10)
                self.load_done_pubs[robot_name] = self.node.create_publisher(Bool, f"{robot_name}{T_SUFFIX_LOAD_DONE}", 10)
                self.unload_done_pubs[robot_name] = self.node.create_publisher(Bool, f"{robot_name}{T_SUFFIX_UNLOAD_DONE}", 10)
                self.move_role_pubs[robot_name] = self.node.create_publisher(String, f"{robot_name}{T_SUFFIX_MOVE_ROLE}", 10)

        # 2. 로봇팔 (Jetcobot) 설정 [★추가된 부분]
        # Request 구독 (String으로 받아서 내부에서 JSON 파싱)
        self.node.create_subscription(String, T_JETCO_REQ, self.callback_jetco_request, 10)
        # Update 구독
        self.node.create_subscription(String, T_JETCO_UPD, self.callback_jetco_update, 10)
        # Response 발행
        self.jetco_res_pub = self.node.create_publisher(String, T_JETCO_RES, 10)

        # Arm 통신
        self.node.create_subscription(Int32, T_ARM_UNLOAD_SIGNAL, self.unload_callback, 10)
        self.arm_pub = self.node.create_publisher(String, T_ARM_TARGET_SLOT, 10)

        # OpenManipulator Start Publisher Test1
        self.manip_start_pub = self.node.create_publisher(Bool,'/pick_and_place/start',10)
       

        print(f"GUI 노드 시작 (토픽 설정 완료)")
        while self.running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        self.node.destroy_node()
        rclpy.shutdown()

    # -------------------------------------------------------------
    # ★ [기능 1] Request 처리: DB 확인 후 Response 발행
    def callback_jetco_request(self, msg):
        try:
            # 작은따옴표를 큰따옴표로 바꿔서 JSON 호환되게 만듦
            json_str = msg.data.replace("'", '"')
            data = json.loads(json_str)
            
            target_section = data.get('section', '')
            task = int(data.get('task_id', -1)) # 0: pick, 1: place
            found_id = 0 # 0이면 실패 또는 없음

            self.jetco_log_signal.emit(f"📩 [요청] 구역:{target_section}, 작업:{'입고' if task==1 else '출고'}({task})")

            conn = mysql.connector.connect(host=DB_HOST, user=DB_USER, password=DB_PASS, database=DB_NAME)
            cursor = conn.cursor()

            if task == 0: # Pick (출고): 물건이 있는(1) 자리 찾기
                sql = "SELECT current_part_id FROM warehouse_slots WHERE section=%s AND is_occupied=1 LIMIT 1"
                cursor.execute(sql, (target_section,))
                res = cursor.fetchone()
                if res and res[0]: 
                    found_id = res[0] # 꺼낼 물건 ID

            elif task == 1: # Place (입고): 빈(0) 자리 찾기
                sql = "SELECT COUNT(*) FROM warehouse_slots WHERE section=%s AND is_occupied=0"
                cursor.execute(sql, (target_section,))
                cnt = cursor.fetchone()[0]
                # 빈 자리가 있으면 성공 신호로 999 (또는 실제 넣을 ID) 리턴
                found_id = 999 if cnt > 0 else 0

            conn.close()

            # Response 전송 (JSON 문자열)
            res_data = {"section": target_section, "id": found_id}
            self.jetco_res_pub.publish(String(data=json.dumps(res_data)))
            self.jetco_log_signal.emit(f"[응답] {res_data}")

        except Exception as e:
            print(f"Jetcobot Request Error: {e}")

    # -------------------------------------------------------------
    # ★ [기능 2] Update 처리: DB 상태 실제 변경
    def callback_jetco_update(self, msg):
        try:
            json_str = msg.data.replace("'", '"')
            data = json.loads(json_str)
            
            sec = data.get('section', '')
            pid = int(data.get('id', 0))
            occ = int(data.get('occupy', 0)) # 0: 비움, 1: 채움
            
            action_str = "채움" if occ == 1 else "비움"
            self.jetco_log_signal.emit(f"🔄 [DB수정] {sec}구역, ID:{pid} -> {action_str}")

            conn = mysql.connector.connect(host=DB_HOST, user=DB_USER, password=DB_PASS, database=DB_NAME)
            cursor = conn.cursor()

            if occ == 1: # 채우기 (Place 완료)
                # 빈 슬롯 하나를 해당 ID로 채움
                sql = """UPDATE warehouse_slots SET is_occupied=1, current_part_id=%s 
                         WHERE section=%s AND is_occupied=0 LIMIT 1"""
                cursor.execute(sql, (pid, sec))
            else: # 비우기 (Pick 완료)
                # 해당 ID가 있던 슬롯을 비움
                sql = """UPDATE warehouse_slots SET is_occupied=0, current_part_id=NULL 
                         WHERE section=%s AND current_part_id=%s LIMIT 1"""
                cursor.execute(sql, (sec, pid))
            
            conn.commit()
            conn.close()
        except Exception as e:
            print(f"Jetcobot Update Error: {e}")

    # 🔥 GUI 로봇 데이터 update point
    def pose_callback(self, msg, robot_id):
        clean_id = robot_id.replace("/", "")
        x_m = msg.pose.pose.position.x
        y_m = msg.pose.pose.position.y
        x_cm = int(x_m * 100.0 + MAP_OFFSET_X_CM)
        y_cm = int(y_m * 100.0 + MAP_OFFSET_Y_CM)
        data = {"id": clean_id, "location": f"{x_cm},{y_cm}"}
        self.robot_update_signal.emit(data)

    def battery_callback(self, msg, robot_id):
        clean_id = robot_id.replace("/", "")
        data = {"id": clean_id, "battery": msg.data}
        self.robot_update_signal.emit(data)

    def state_callback(self, msg, robot_id):
        clean_id = robot_id.replace("/", "")
        data = {"id": clean_id, "state": msg.data}
        self.robot_update_signal.emit(data)

    def send_manip_start(self):
        if self.manip_start_pub:
            msg = Bool()
            msg.data = True
            self.manip_start_pub.publish(msg)

            print("▶ Published /pick_and_place/start = True")

            data = {
                "id": "jetcobot3",   # openmanipulator id
                "status": "PNP",
                "mode": "작업중"
            }
            self.robot_update_signal.emit(data)

    ##################################################
    def unload_callback(self, msg):
        self.unload_signal.emit(msg.data)

    def send_command(self, robot_id, cmd_str):
        target_key = robot_id if robot_id.startswith("/") else f"/{robot_id}"
        if target_key not in self.cmd_pubs:
            for key in self.cmd_pubs.keys():
                if robot_id in key:
                    target_key = key
                    break
        if target_key in self.cmd_pubs:
            msg = String()
            msg.data = cmd_str
            self.cmd_pubs[target_key].publish(msg)
            print(f"명령 전송 [{target_key}]: {cmd_str}")
        else:
            print(f"❌ 로봇을 찾을 수 없음: {robot_id}")

    def send_load_done(self, robot_id):
        target_key = robot_id if robot_id.startswith("/") else f"/{robot_id}"
        if target_key not in self.load_done_pubs:
            for key in self.load_done_pubs.keys():
                if robot_id in key:
                    target_key = key; break
        if target_key in self.load_done_pubs:
            msg = Bool(); msg.data = True
            self.load_done_pubs[target_key].publish(msg)
            print(f"상차 완료 전송 [{target_key}]: True")
        else: print(f"❌ 로봇을 찾을 수 없음: {robot_id}")

    def send_unload_done(self, robot_id):
        target_key = robot_id if robot_id.startswith("/") else f"/{robot_id}"
        if target_key not in self.unload_done_pubs:
            for key in self.unload_done_pubs.keys():
                if robot_id in key:
                    target_key = key; break
        if target_key in self.unload_done_pubs:
            msg = Bool(); msg.data = True
            self.unload_done_pubs[target_key].publish(msg)
            print(f"하차 완료 전송 [{target_key}]: True")
        else: print(f"❌ 로봇을 찾을 수 없음: {robot_id}")
        
    def send_move_role(self, robot_id, role_id):
        target_key = robot_id if robot_id.startswith("/") else f"/{robot_id}"
        if target_key not in self.move_role_pubs:
            for key in self.move_role_pubs.keys():
                if robot_id in key:
                    target_key = key; break
        if target_key in self.move_role_pubs:
            msg = String(); msg.data = str(role_id)
            self.move_role_pubs[target_key].publish(msg)
            clean_robot_id = target_key.lstrip("/") # 지니 : 로봇아이디 분리
            self.robot_role_assignments[clean_robot_id] = msg.data # 지니 : 로봇별 할당된 업무 저장
            print(f"이동 역할 전송 [{target_key}]: {msg.data}")
        else: print(f"❌ 로봇을 찾을 수 없음: {robot_id}")
            
    #지니 : 업무 랜덤 할당하는 함수
    def assign_random_work_and_move(self): 
        assignments = {}
        if not self.move_role_pubs:
            print("❌ move_role 퍼블리셔가 아직 준비되지 않았습니다.")
            return assignments

        robot_keys = sorted(self.move_role_pubs.keys())
        available_roles = ["1", "3", "4"]
        random.shuffle(available_roles)                                 ################# 🔷 수정필요! 

        for robot_key, role_id in zip(robot_keys, available_roles):
            robot_id = robot_key.lstrip("/")
            self.send_move_role(robot_id, role_id)
            assignments[robot_id] = role_id

        print(f"랜덤 업무 할당 완료: {assignments}")
        return assignments

    def send_arm_target(self, slot_id):
        if self.arm_pub:
            msg = String(); msg.data = slot_id
            self.arm_pub.publish(msg)
            print(f"로봇팔 목표 전송: {slot_id}")
    
    # OpenManipulator Start Publisher Test1
    def send_manip_start(self):
        if self.manip_start_pub:
            msg = Bool()
            msg.data = True
            self.manip_start_pub.publish(msg)
            print("▶ Published /pick_and_place/start = True")

    def stop(self):
        self.running = False; self.wait()


    def trigger_manip_start(self):
        self.ros_thread.send_manip_start()
