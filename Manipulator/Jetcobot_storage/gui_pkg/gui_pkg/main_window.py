import os
import time
import datetime
import cv2
import cv2.aruco as aruco
import mysql.connector
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, 
                             QGroupBox, QPushButton, QLabel, QFrame, 
                             QGridLayout, QTabWidget, QTextEdit, QTableWidget, 
                             QTableWidgetItem, QHeaderView, QSizePolicy, QMessageBox, 
                             QLineEdit, QAbstractItemView)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QPoint, QTimer
from PyQt6.QtGui import QImage, QPixmap, QPainter, QBrush, QColor, QFont, QPen

os.environ["QT_QPA_PLATFORM"] = "xcb"

##############################################################################
# Global Configuration
# DB / Map / Part Mapping
##############################################################################


# 4. DB 설정
DB_HOST = 'localhost'
DB_USER = 'root'
DB_PASS = '1234'
DB_NAME = 'smart_factory'

# 5. 지도 설정
REAL_MAP_WIDTH_CM = 203
REAL_MAP_HEIGHT_CM = 83

# 6. 부품 매핑 (ArUco ID -> 구역)
PART_MAPPING = { 1: 'A', 2: 'B', 3: 'A' }

##############################################################################
# Database System
# DB initialization / CRUD
##############################################################################
def initialize_database():
    """DB 테이블 및 기초 데이터 자동 생성 함수"""
    try:
        conn = mysql.connector.connect(host=DB_HOST, user=DB_USER, password=DB_PASS)
        cursor = conn.cursor()
        cursor.execute(f"CREATE DATABASE IF NOT EXISTS {DB_NAME}")
        conn.database = DB_NAME
        
        tables = [
            """CREATE TABLE IF NOT EXISTS request_list (
                aruco_id INT PRIMARY KEY, name VARCHAR(100), 
                target_qty INT DEFAULT 0, current_qty INT DEFAULT 0)""",
            
            """CREATE TABLE IF NOT EXISTS inventory_history (
                id INT AUTO_INCREMENT PRIMARY KEY, part_id INT, quantity INT, 
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP)""",
            
            """CREATE TABLE IF NOT EXISTS warehouse_slots (
                slot_id VARCHAR(10) PRIMARY KEY, section CHAR(1), 
                is_occupied TINYINT DEFAULT 0, current_part_id INT DEFAULT NULL)""",
            
            """CREATE TABLE IF NOT EXISTS quotes (
                quote_id INT AUTO_INCREMENT PRIMARY KEY, project_name VARCHAR(255), 
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP)""",
            
            """CREATE TABLE IF NOT EXISTS parts (
                part_id INT PRIMARY KEY, part_name VARCHAR(100))""",
            
            """CREATE TABLE IF NOT EXISTS quote_details (
                id INT AUTO_INCREMENT PRIMARY KEY, quote_id INT, part_id INT, req_quantity INT,
                FOREIGN KEY (quote_id) REFERENCES quotes(quote_id),
                FOREIGN KEY (part_id) REFERENCES parts(part_id))"""
        ]
        
        for table_sql in tables:
            cursor.execute(table_sql)
            
        # A, B, C 구역의 1~3번 슬롯이 있는지 확인하고 없으면 넣음
        slots = []
        for section in ['A', 'B', 'C']:
            for i in range(1, 4): # 1, 2, 3
                slots.append((f'{section}-{i}', section))
        
        # 나머지 구역도 혹시 모르니 일단 둠 (D~F)
        slots.extend([('D-1', 'D'), ('E-1', 'E'), ('F-1', 'F')])
        
        cursor.executemany("""
            INSERT IGNORE INTO warehouse_slots (slot_id, section, is_occupied) 
            VALUES (%s, %s, 0)
        """, slots)

        parts = [(1, 'Part A (Red)'), (2, 'Part B (Blue)'), (3, 'Part C (Green)')]
        cursor.executemany("INSERT IGNORE INTO parts (part_id, part_name) VALUES (%s, %s)", parts)
        
        cursor.execute("SELECT COUNT(*) FROM quotes")
        if cursor.fetchone()[0] == 0:
            cursor.execute("INSERT INTO quotes (project_name) VALUES ('Test Project Alpha')")
            quote_id = cursor.lastrowid
            cursor.execute("INSERT INTO quote_details (quote_id, part_id, req_quantity) VALUES (%s, 1, 3)", (quote_id,))
            cursor.execute("INSERT INTO quote_details (quote_id, part_id, req_quantity) VALUES (%s, 2, 2)", (quote_id,))
            print("✅ [DB 자동설정] 기초 데이터(슬롯, 부품, 테스트 주문)가 생성되었습니다.")
        
        conn.commit()
        conn.close()
        print("✅ [DB 자동설정] 데이터베이스 초기화 완료")
        
    except Exception as e:
        print(f"⚠️ [DB 경고] 초기화 중 오류 발생 (무시 가능): {e}")

##############################################################################
# Camera System
# ArUco detection / IP camera / dummy mode
##############################################################################
class CameraThread(QThread):
    changePixmap = pyqtSignal(QImage)
    matchFound = pyqtSignal(int) 
    slotAllocated = pyqtSignal(str) 
    
    def __init__(self):
        super().__init__()
        self.running = True

    def run(self):
        # aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        # parameters = aruco.DetectorParameters()
        
        url = "http://192.168.0.6:5000/video_feed" # 학원
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG) 
        ############## test
        if not cap.isOpened():
            print("❌ IP 카메라 실패 → 로컬카메라 시도")
            cap = cv2.VideoCapture(2)

        if not cap.isOpened():
            print("❌ 카메라 없음 (더미 모드)")
            self.run_dummy_mode()
            return

        while self.running:
            ret, frame = cap.read()
            if not ret:
                continue

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            qimg = QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888)

            self.changePixmap.emit(
                qimg.scaled(500, 350, Qt.AspectRatioMode.KeepAspectRatio)
            )

        cap.release()
        ##############
        # present_ids = set()
        # disappear_start_time = {} 
        
        # while self.running:
        #     ret, frame = cap.read()
        #     if not ret: break
        #     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #     corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        #     detected_now = set()
        #     if ids is not None:
        #         aruco.drawDetectedMarkers(frame, corners, ids)
        #         for marker_id in ids:
        #             id_num = int(marker_id[0])
        #             detected_now.add(id_num)
            
        #     current_time = time.time()
        #     for id_num in detected_now:
        #         if id_num not in present_ids:
        #             print(f"📷 물건 감지됨: ID {id_num}")
        #             self.increase_quantity(id_num)
        #             self.matchFound.emit(id_num)
        #             allocated_slot = self.find_and_fill_empty_slot(id_num)
        #             if allocated_slot:
        #                 print(f"✅ 슬롯 배정 완료: {allocated_slot} -> 로봇 이동 명령 준비")
        #                 self.slotAllocated.emit(allocated_slot)
        #             else: print(f"⚠️ 경고: ID {id_num}을 넣을 빈 슬롯이 없습니다!")
        #             present_ids.add(id_num)
        #         if id_num in disappear_start_time: del disappear_start_time[id_num]
            
        #     missing_ids = present_ids - detected_now
        #     for missing_id in missing_ids:
        #         if missing_id not in disappear_start_time: disappear_start_time[missing_id] = current_time
        #         elif (current_time - disappear_start_time[missing_id]) > 2.0:
        #             present_ids.remove(missing_id); del disappear_start_time[missing_id]
            
        #     rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        #     h, w, ch = rgb.shape
        #     qimg = QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888)
        #     self.changePixmap.emit(qimg.scaled(500, 350, Qt.AspectRatioMode.KeepAspectRatio))
        # cap.release()

    def run_dummy_mode(self):
        import numpy as np
        blank = np.zeros((350, 500, 3), np.uint8)
        cv2.putText(blank, "No Camera", (150, 175), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        while self.running:
            h, w, ch = blank.shape
            qimg = QImage(blank.data, w, h, ch * w, QImage.Format.Format_RGB888)
            self.changePixmap.emit(qimg)
            time.sleep(0.1)

    def increase_quantity(self, aruco_id):
        try:
            conn = mysql.connector.connect(host=DB_HOST, user=DB_USER, password=DB_PASS, database=DB_NAME)
            cursor = conn.cursor()
            sql = """INSERT INTO request_list (aruco_id, name, target_qty, current_qty) 
                     VALUES (%s, 'Detected Item', 0, 1) 
                     ON DUPLICATE KEY UPDATE current_qty = current_qty + 1"""
            cursor.execute(sql, (aruco_id,))
            cursor.execute("INSERT INTO inventory_history (part_id, quantity) VALUES (%s, 1)", (aruco_id,))
            conn.commit(); conn.close()
        except Exception: pass

    def find_and_fill_empty_slot(self, aruco_id):
        target_section = PART_MAPPING.get(aruco_id)
        if not target_section: return None
        conn = None; found_slot = None
        try:
            conn = mysql.connector.connect(host=DB_HOST, user=DB_USER, password=DB_PASS, database=DB_NAME)
            cursor = conn.cursor()
            sql_sel = "SELECT slot_id FROM warehouse_slots WHERE section=%s AND is_occupied=0 ORDER BY slot_id ASC LIMIT 1"
            cursor.execute(sql_sel, (target_section,))
            res = cursor.fetchone()
            if res:
                found_slot = res[0]
                sql_upd = "UPDATE warehouse_slots SET is_occupied=1, current_part_id=%s WHERE slot_id=%s"
                cursor.execute(sql_upd, (aruco_id, found_slot))
                conn.commit()
                print(f"✅ 슬롯 할당: {found_slot}")
        except Exception as e: print(f"DB Error: {e}")
        finally: 
            if conn: conn.close()
        return found_slot

    def stop(self):
        self.running = False; self.wait()

##############################################################################
# UI Widgets
# Map / WarehouseCard / CollapsibleBox
##############################################################################
class SimpleMapWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setStyleSheet("background-color: white; border: 2px solid #555;")
        self.robot_positions = {} 
        map_path = os.path.join(os.path.dirname(__file__), "map.png")
        if os.path.exists(map_path): self.map_img = QPixmap(map_path)
        else: self.map_img = QPixmap()
    
    def update_position(self, name, x, y):
        self.robot_positions[name] = (x, y); self.update() 
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        if not self.map_img.isNull(): painter.drawPixmap(0, 0, w, h, self.map_img)
        else: painter.setBrush(QBrush(Qt.GlobalColor.white)); painter.drawRect(0, 0, w, h)
        painter.setPen(QPen(Qt.GlobalColor.black, 2)); painter.drawRect(0, 0, w, h)
        for name, (rx, ry) in self.robot_positions.items():
            sx = int((rx / REAL_MAP_WIDTH_CM) * w)
            sy = int((ry / REAL_MAP_HEIGHT_CM) * h)
            sx = max(0, min(sx, w)); sy = max(0, min(sy, h))
            color = QColor(255, 100, 100) if "Pinky" in name else QColor(100, 100, 255)
            painter.setBrush(QBrush(color)); painter.drawEllipse(QPoint(sx, sy), 6, 6)
            painter.drawText(sx + 8, sy + 4, name)

# ==========================================
# [창고 카드 위젯]
class WarehouseCard(QFrame):
    def __init__(self, title, slot_id):
        super().__init__()
        self.slot_id = slot_id
        self.setFrameShape(QFrame.Shape.Box)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        layout = QVBoxLayout(); layout.setContentsMargins(2, 2, 2, 2); layout.setSpacing(2)
        
        # 타이틀 (예: 1번 칸)
        self.lbl_title = QLabel(f"{title}"); self.lbl_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_title.setStyleSheet("font-size: 14px; font-weight: bold; color: #333; border: none;")
        
        # 부품 이름 (예: Part A)
        self.lbl_part = QLabel("-"); self.lbl_part.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_part.setStyleSheet("font-size: 12px; color: #666; border: none;")
        
        # 상태 텍스트
        self.lbl_status = QLabel("비어있음"); self.lbl_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_status.setStyleSheet("font-size: 11px; font-weight: bold; color: #BBB; border: none;")
        
        layout.addWidget(self.lbl_title)
        layout.addWidget(self.lbl_part)
        layout.addWidget(self.lbl_status)
        self.setLayout(layout)
        self.update_info("-", 0)

    def update_info(self, part_name, is_occupied):
        self.lbl_part.setText(part_name if is_occupied else "-")
        if is_occupied:
            self.setStyleSheet("QFrame { border-radius: 6px; border: 2px solid #4CAF50; background-color: #E8F5E9; }")
            self.lbl_status.setText("보관중")
            self.lbl_status.setStyleSheet("color: #4CAF50; font-weight: bold; border:none;")
        else:
            self.setStyleSheet("QFrame { border-radius: 6px; border: 1px solid #DDD; background-color: #FAFAFA; }")
            self.lbl_status.setText("비어있음")
            self.lbl_status.setStyleSheet("color: #BBB; border:none;")

# ==========================================
# [UI 컴포넌트]
class CollapsibleBox(QWidget):
    def __init__(self, title="", parent=None):
        super().__init__(parent)
        self.base_title = title
        self.toggle_button = QPushButton(f"▼ {title}")
        self.toggle_button.setCheckable(True)
        self.toggle_button.setStyleSheet("text-align: left; padding: 8px; font-weight: bold; background-color: #E0E0E0;")
        self.toggle_button.toggled.connect(self.on_toggled)
        self.content_area = QFrame(); self.content_layout = QVBoxLayout(self.content_area)
        self.content_area.setVisible(False)
        layout = QVBoxLayout(self); layout.setContentsMargins(0,0,0,0)
        layout.addWidget(self.toggle_button); layout.addWidget(self.content_area)

    def on_toggled(self, checked):
        self.toggle_button.setText(f"{'▲' if checked else '▼'} {self.toggle_button.text()[2:]}")
        self.content_area.setVisible(checked)
    def add_widget(self, widget): self.content_layout.addWidget(widget)

    def set_selected(self, text):
        self.base_title = f"{text}"
        arrow = "▲" if self.toggle_button.isChecked() else "▼"
        self.toggle_button.setText(f"{arrow} {self.base_title}")

##############################################################################
# Main Robot Control System
##############################################################################
class RobotControlSystem(QWidget):
    def __init__(self, ros_thread):
        super().__init__()
        self.robot_data_storage = {} 
        self.current_viewing_robot = None 
        self.warehouse_cards = {} 
        self.ros_thread = ros_thread
        self.robot_name_map = {
            "pinky1": "pinky_c0bd",
            "pinky2": "pinky_b44f",
            "pinky3": "pinky_1542",
            "jetcobot1": "storage_jetcobot",
            "jetcobot2": "assembly_jetcobot",
            "jetcobot3": "Openmanipulator",
        }
        # assembly 전용 토글 사용 변수
        self.selected_module = None
        self.module_map = {
            "모듈1": "MODULE_A",
            "모듈2": "MODULE_B",
            "모듈3": "MODULE_C"
        }

        self.initUI()
        self.ros_thread.robot_update_signal.connect(self.update_ros_data)
        self.ros_thread.unload_signal.connect(self.handle_unload_event)
        
        # Jetcobot 로그 연결 - 추가
        self.ros_thread.jetco_log_signal.connect(self.add_log)
        # Jetcobot 클래스 변수
        self.part_id = None
        self.section = None
        self.section_id = None
        
        self.ros_thread.start()
        ######################################################################

        self.camera_thread = CameraThread()
        self.camera_thread.changePixmap.connect(self.update_camera_image)
        self.camera_thread.matchFound.connect(self.load_verification_table)
        self.camera_thread.matchFound.connect(self.refresh_search_result)
        self.camera_thread.slotAllocated.connect(self.on_slot_allocated)
        self.camera_thread.start()
        
        # DB 화면 갱신 타이머 (2초) - 추가
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.load_verification_table)
        self.timer.start(2000)


    ##############################################################################
    # UI Layout & Widget Setup
    # 버튼 생성 / 시트 구성 / 탭 배치
    ##############################################################################
    def initUI(self):
        self.setWindowTitle('Smart Factory - Integrated System (Parametrized)')
        self.resize(1400, 900) 
        self.setStyleSheet("background-color: #FFFFFF; color: black;") 
        main_layout = QVBoxLayout()
        top_cont = QWidget(); top_cont.setFixedHeight(400)
        top_layout = QHBoxLayout(top_cont)
        self.map_widget = SimpleMapWidget()
        self.top_camera_label = QLabel("Top View Camera"); self.top_camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.top_camera_label.setStyleSheet("background: black; color: white;")
        top_layout.addWidget(self.map_widget, 1); top_layout.addWidget(self.top_camera_label, 1) 
        main_layout.addWidget(top_cont)
        bot_layout = QHBoxLayout()
        self.control_group = QGroupBox("로봇 제어")
        ctl_layout = QVBoxLayout()

        # 🔷 전체 Pinky 모니터 버튼
        ##########################################
        self.main_control_btn = QPushButton("Main Control")
        self.main_control_btn.setMinimumHeight(40)
        self.main_control_btn.setStyleSheet(
            "background-color: #3F51B5; color: white; font-weight: bold;"
        )
        # self.btn_pinky_monitor.clicked.connect(self.show_pinky_monitor)
        # ctl_layout.addWidget(self.btn_pinky_monitor)
        self.main_control_btn.clicked.connect(lambda: self.select_robot("main_control"))
        ctl_layout.addWidget(self.main_control_btn)
        ##########################################

        pinky_box = CollapsibleBox("Pinky 선택")
        for internal_id, display_name in self.robot_name_map.items():
            if "pinky" in internal_id: 
                btn = QPushButton(display_name)
                btn.clicked.connect(lambda _, n=internal_id: self.select_robot(n))
                pinky_box.add_widget(btn)
        ctl_layout.addWidget(pinky_box)
        jet_box = CollapsibleBox("Jetcobot 선택")
        for internal_id, display_name in self.robot_name_map.items():
            if "jetcobot" in internal_id:
                btn = QPushButton(display_name)
                btn.clicked.connect(lambda _, n=internal_id: self.select_robot(n))
                jet_box.add_widget(btn)
        ctl_layout.addWidget(jet_box)
        ctl_layout.addStretch(1)
        self.control_group.setLayout(ctl_layout)

        self.tabs = QTabWidget(); self.tabs.currentChanged.connect(self.on_tab_changed)
        self.tab_status = QWidget(); self.setup_status_tab(); self.tabs.addTab(self.tab_status, "Sheet1 - 상태")
        self.tab_log = QWidget(); self.setup_log_tab(); self.tabs.addTab(self.tab_log, "Sheet2 - Log")
        self.tab_db = QWidget(); self.setup_db_tab_full(); self.tabs.addTab(self.tab_db, "Sheet3 - 창고/검수")
        bot_layout.addWidget(self.control_group, 2); bot_layout.addWidget(self.tabs, 8)
        main_layout.addLayout(bot_layout); self.setLayout(main_layout); self.show()

    ######################################################################
    # UI button 관리
    def setup_status_tab(self):
        layout = QVBoxLayout()
        self.info_label = QLabel("왼쪽 메뉴에서 로봇을 선택해주세요.")
        self.info_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.info_label)
        
        ################################################3
        # 🔷 Main Control 패널
        self.pinky_all_panel = QFrame()
        self.pinky_all_panel.setStyleSheet(
            "background-color: #F0F8FF; border-radius: 8px; border: 1px solid #90CAF9;")
        
        main_layout_pinky = QVBoxLayout(self.pinky_all_panel)

        # 🔷 맨 위 타이틀
        self.lbl_pinky_info = QLabel("Pinky 통합 상태 영역")
        self.lbl_pinky_info.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_pinky_info.setStyleSheet("font-size:16px; font-weight:bold;")
        main_layout_pinky.addWidget(self.lbl_pinky_info)

        layout_pinky = QHBoxLayout()

        # 🔷 상단 로봇 상태 표시 바
        self.robot_status_layout = QGridLayout()
        self.pinky_status_labels = {}  # id별 QLabel 저장
        robots = list(self.robot_name_map.items())
        for index, (rid, display_name) in enumerate(robots):
            row = index // 3   # 3열 기준
            col = index % 3

            lbl = QLabel(f"{display_name} : 대기")
            lbl.setStyleSheet(
                "font-weight: bold; padding: 5px; border: 1px solid #CCC;"
            )
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)

            self.robot_status_layout.addWidget(lbl, row, col)
            self.pinky_status_labels[rid] = lbl

        main_layout_pinky.addLayout(self.robot_status_layout)

        # 🔷 왼쪽 버튼
        left_btn_layout = QVBoxLayout()
        self.pinky_all_btn1 = QPushButton("작업 시작")
        self.pinky_all_btn1.clicked.connect(lambda _, b=self.pinky_all_btn1: self.apply_click_feedback(b))
        self.pinky_all_btn1.clicked.connect(self.on_start_random_assignment) #지니      

        self.pinky_all_btn2 = QPushButton("작업종료")
        self.pinky_all_btn2.clicked.connect(lambda _, b=self.pinky_all_btn2: self.apply_click_feedback(b))
        self.pinky_all_btn2.clicked.connect(self.on_stop_all_assignments) #지니

        self.pinky_all_btn3 = QPushButton("test_ bnt")
        self.pinky_all_btn3.clicked.connect(lambda _, b=self.pinky_all_btn3: self.apply_click_feedback(b))
        self.pinky_all_btn3.clicked.connect(self.on_load_complete) ## 연결필요!!!


        for btn in [self.pinky_all_btn1, self.pinky_all_btn2, self.pinky_all_btn3]:
            self.setup_primary_button(btn, left_btn_layout)

        left_btn_layout.addStretch()

        # 🔷 오른쪽버튼
        self.pinky_content_area = QFrame()
        right_layout = QVBoxLayout(self.pinky_content_area)

        self.right_btn1 = QPushButton("부품 상차 완료")
        self.right_btn1.clicked.connect(lambda _, b=self.right_btn1: self.apply_click_feedback(b))
        self.right_btn1.clicked.connect(lambda: self.send_done_by_role("1", "load")) #지니
        
        self.right_btn2 = QPushButton("모듈 입고 완료")
        self.right_btn2.clicked.connect(lambda _, b=self.right_btn2: self.apply_click_feedback(b))
        self.right_btn2.clicked.connect(lambda: self.send_done_by_role("3", "unload")) #지니

        self.right_btn3 = QPushButton("모듈 상차 완료")
        self.right_btn3.clicked.connect(lambda _, b=self.right_btn3: self.apply_click_feedback(b))
        self.right_btn3.clicked.connect(lambda: self.send_done_by_role("4", "load")) #지니

        self.right_btn4 = QPushButton("모듈 출고 완료")
        self.right_btn4.clicked.connect(lambda _, b=self.right_btn4: self.apply_click_feedback(b))
        self.right_btn4.clicked.connect(lambda: self.send_done_by_role("4", "unload")) #지니
        
        for btn in [self.right_btn1, self.right_btn2, self.right_btn3, self.right_btn4]:
            self.setup_primary_button(btn, right_layout)

        bottom_row_layout = QHBoxLayout()
        bottom_row_layout.addWidget(self.right_btn3)
        bottom_row_layout.addWidget(self.right_btn4)

        right_layout.addLayout(bottom_row_layout)
        layout_pinky.addLayout(left_btn_layout, 2)   # 왼쪽 비율
        layout_pinky.addWidget(self.pinky_content_area, 8)  # 오른쪽 비율
        
        main_layout_pinky.addLayout(layout_pinky)
        layout.addWidget(self.pinky_all_panel)
        self.pinky_all_panel.hide()

        ######pinky 선택 패널##################################
        self.pinky_shared_panel = QFrame()
        self.pinky_shared_panel.setStyleSheet("background-color: #F0F8FF; border-radius: 8px; border: 1px solid #90CAF9;")
        grid = QGridLayout(self.pinky_shared_panel)
        grid.setSpacing(10)
        grid.setContentsMargins(20, 20, 20, 20)
        font_title = QFont("Arial", 12, QFont.Weight.Bold)
        font_val = QFont("Arial", 16, QFont.Weight.Bold)
        t1=QLabel("이름:"); t1.setFont(font_title); self.lbl_name = QLabel("-"); self.lbl_name.setFont(font_val); self.lbl_name.setStyleSheet("color: #3F51B5;")
        t2=QLabel("배터리:"); t2.setFont(font_title); self.lbl_bat = QLabel("-"); self.lbl_bat.setFont(font_val)
        t3=QLabel("모드:"); t3.setFont(font_title); self.lbl_state = QLabel("-"); self.lbl_state.setFont(font_val)
        t4=QLabel("위치:"); t4.setFont(font_title); self.lbl_loc = QLabel("-"); self.lbl_loc.setFont(font_val)
        grid.addWidget(t1,0,0); grid.addWidget(self.lbl_name,0,1); grid.addWidget(t2,1,0); grid.addWidget(self.lbl_bat,1,1)
        grid.addWidget(t3,2,0); grid.addWidget(self.lbl_state,2,1); grid.addWidget(t4,3,0); grid.addWidget(self.lbl_loc,3,1)
        
        # [상/하차 완료 버튼]
        btn_layout = QHBoxLayout()
        self.btn_load_complete = QPushButton("상차 완료")
        self.btn_load_complete.clicked.connect(lambda _, b=self.btn_load_complete: self.apply_click_feedback(b))
        self.btn_load_complete.clicked.connect(self.on_load_complete) 

        self.btn_unload_complete = QPushButton("하차 완료")
        self.btn_unload_complete.clicked.connect(lambda _, b=self.btn_unload_complete: self.apply_click_feedback(b))
        self.btn_unload_complete.clicked.connect(self.on_unload_complete)

        for btn in [self.btn_load_complete, self.btn_unload_complete]:
            self.setup_primary_button(btn)

        btn_layout.addWidget(self.btn_load_complete)
        btn_layout.addWidget(self.btn_unload_complete)
        grid.addLayout(btn_layout, 4, 0, 1, 2)

        # 5:5 분할 (수동조작 / 자동이동)
        control_nav_layout = QHBoxLayout()

        # 1. 왼쪽: 수동 조작 (십자 형태)
        self.grp_manual = QGroupBox("Manual") # self로 변경
        grid_manual = QGridLayout()
        
        def style_manual_btn(btn):
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            btn.setMinimumHeight(40)
            btn.setFont(QFont("Arial", 12, QFont.Weight.Bold))
            return btn

        btn_up = style_manual_btn(QPushButton("▲"))
        btn_up.clicked.connect(lambda _, b=btn_up: self.apply_click_feedback(b))
        btn_up.clicked.connect(lambda: self.send_manual_cmd("FORWARD"))

        btn_down = style_manual_btn(QPushButton("▼"))
        btn_down.clicked.connect(lambda _, b=btn_down: self.apply_click_feedback(b))
        btn_down.clicked.connect(lambda: self.send_manual_cmd("BACKWARD"))

        btn_left = style_manual_btn(QPushButton("◀"))
        btn_left.clicked.connect(lambda _, b=btn_left: self.apply_click_feedback(b))
        btn_left.clicked.connect(lambda: self.send_manual_cmd("LEFT"))

        btn_right = style_manual_btn(QPushButton("▶"))
        btn_right.clicked.connect(lambda _, b=btn_right: self.apply_click_feedback(b))
        btn_right.clicked.connect(lambda: self.send_manual_cmd("RIGHT"))

        btn_stop = style_manual_btn(QPushButton("Stop"))
        btn_stop.clicked.connect(lambda _, b=btn_stop: self.apply_click_feedback(b))
        btn_stop.clicked.connect(lambda: self.send_manual_cmd("STOP"))
        

        for btn in [btn_up, btn_down, btn_left, btn_right, btn_stop]:
            self.setup_primary_button(btn)

        grid_manual.addWidget(btn_up, 0, 1)
        grid_manual.addWidget(btn_left, 1, 0)
        grid_manual.addWidget(btn_stop, 1, 1)
        grid_manual.addWidget(btn_right, 1, 2)
        grid_manual.addWidget(btn_down, 2, 1)
        self.grp_manual.setLayout(grid_manual)

        # 2. 오른쪽: 자동 이동 (검수대, 창고, 조립대)
        self.grp_nav = QWidget()
        vbox_nav = QVBoxLayout(self.grp_nav)
        
        def style_nav_btn(btn):
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            btn.setMinimumHeight(45)
            return btn

        btn_standby = style_nav_btn(QPushButton("대기장소"))
        btn_standby.clicked.connect(lambda _, b=btn_standby: self.apply_click_feedback(b))
        btn_standby.clicked.connect(lambda _, b=btn_standby: self.send_nav_cmd("INSPECTION_ZONE", b))
        
        btn_inspect = style_nav_btn(QPushButton("검수대"))
        btn_inspect.clicked.connect(lambda _, b=btn_inspect: self.apply_click_feedback(b))
        btn_inspect.clicked.connect(lambda _, b=btn_inspect: self.send_nav_cmd("INSPECTION_ZONE", b))
        
        btn_assembly = style_nav_btn(QPushButton("조립대"))
        btn_assembly.clicked.connect(lambda _, b=btn_assembly: self.apply_click_feedback(b))
        btn_assembly.clicked.connect(lambda _, b=btn_assembly: self.send_nav_cmd("ASSEMBLY_ZONE", b))
        
        btn_parts = style_nav_btn(QPushButton("모듈 창고"))
        btn_parts.clicked.connect(lambda _, b=btn_parts: self.apply_click_feedback(b))
        btn_parts.clicked.connect(lambda _, b=btn_parts: self.send_nav_cmd("PARTS_WAREHOUSE", b))


        for btn in [btn_standby, btn_inspect, btn_assembly, btn_parts]:
            self.setup_primary_button(btn, vbox_nav)

        control_nav_layout.addWidget(self.grp_manual, 1)
        control_nav_layout.addWidget(self.grp_nav, 1)

        grid.addLayout(control_nav_layout, 5, 0, 1, 2)

        ################################################
        # 🔷 manipulator 전체 전용 패널
        self.jetcobot_shared_panel = QFrame()
        self.pinky_shared_panel.setStyleSheet("background-color: #F0F8FF; border-radius: 8px; border: 1px solid #90CAF9;")

        # 🔹 전체를 세로로 구성
        main_jet_layout = QVBoxLayout(self.jetcobot_shared_panel)

        # ===============================
        # 🔹 1️⃣ 상단 상태 표시 영역
        status_layout = QHBoxLayout()

        # ===== 왼쪽 그룹 (상태 / 모드)
        left_status_layout = QHBoxLayout()
        # 🔹 오른쪽 추가 영역 컨테이너
        self.jetco_extra_container = QWidget()
        self.jetco_extra_layout = QHBoxLayout(self.jetco_extra_container)
        self.jetco_extra_layout.setContentsMargins(0,0,0,0)

        status_layout.addWidget(self.jetco_extra_container)

        font_title = QFont("Arial", 12, QFont.Weight.Bold)
        font_val = QFont("Arial", 14, QFont.Weight.Bold)

        lbl_jetco_status = QLabel("상태 :")
        self.style_info_label(lbl_jetco_status)
        self.lbl_jetco_status_value = QLabel("-")
        self.style_info_label(self.lbl_jetco_status_value)

        lbl_jetco_mode = QLabel("모드 :")
        self.style_info_label(lbl_jetco_mode)
        self.style_info_label(t2)
        self.lbl_jetco_mode_vale = QLabel("-")
        self.style_info_label(self.lbl_jetco_mode_vale)

        left_status_layout.addWidget(lbl_jetco_status)
        left_status_layout.addSpacing(10)
        left_status_layout.addWidget(self.lbl_jetco_status_value)
        left_status_layout.addSpacing(10)
        left_status_layout.addWidget(lbl_jetco_mode)
        left_status_layout.addSpacing(10)
        left_status_layout.addWidget(self.lbl_jetco_mode_vale)

    
        # ===== 오른쪽 그룹 (storage 전용)
        right_status_layout = QHBoxLayout()

        self.lbl_slot_title = QLabel("slot_id :")
        self.style_info_label(self.lbl_slot_title)

        self.lbl_slot_value = QLineEdit(self)
        self.style_info_label(self.lbl_slot_value)
        self.lbl_slot_value.editingFinished.connect(self.on_slot_value_edited)

        self.lbl_part_title = QLabel("part_id :")
        self.style_info_label(self.lbl_part_title)

        self.lbl_part_value = QLineEdit(self)
        self.style_info_label(self.lbl_part_value)
        self.lbl_part_value.editingFinished.connect(self.on_part_value_edited)

        right_status_layout.addWidget(self.lbl_slot_title)
        right_status_layout.addSpacing(10)
        right_status_layout.addWidget(self.lbl_slot_value)
        right_status_layout.addSpacing(10)
        right_status_layout.addWidget(self.lbl_part_title)
        right_status_layout.addSpacing(10)
        right_status_layout.addWidget(self.lbl_part_value)


        # ===== 좌우 배치
        status_layout.addLayout(left_status_layout, 1)
        status_layout.addLayout(right_status_layout, 1)

        main_jet_layout.addLayout(status_layout)

        # ===============================
        # 🔹 2️⃣ 버튼 영역 (가로 분할)
        button_area = QHBoxLayout()

        # 왼쪽 버튼 영역
        left_layout = QVBoxLayout()
        self.btn_jetco_common1 = QPushButton("pnp start")
        self.btn_jetco_common1.clicked.connect(self.handle_jetco_common1)
        
        self.btn_jetco_common2 = QPushButton("test_btn")
        self.btn_jetco_common2.clicked.connect(self.handle_jetco_common2)

        for btn in [self.btn_jetco_common1, self.btn_jetco_common2]:
            self.setup_primary_button(btn, left_layout)
            
        left_layout.addStretch()

        # 오른쪽 버튼 영역
        right_layout = QVBoxLayout()
        self.btn_jetco_action1 = QPushButton("Action1")
        self.btn_jetco_action1.clicked.connect(self.handle_jetco_action1)
        self.btn_jetco_action2 = QPushButton("Action2")
        self.btn_jetco_action2.clicked.connect(self.handle_jetco_action2)
        
        for btn in [self.btn_jetco_action1, self.btn_jetco_action2]:
            self.setup_primary_button(btn, right_layout)
            

        # assembly 전용 토글
        self.assembly_box = CollapsibleBox("Assembly 선택")
        for i in range(1, 4):
            btn = QPushButton(f"모듈{i}")
            btn.clicked.connect(lambda _, m=f"모듈{i}": self.select_module(m))
            self.assembly_box.add_widget(btn)

        # storage 전용 토글
        self.manual_request_box = CollapsibleBox("Request 선택")
        self.btn_jetco_manual_pick = QPushButton(f"Pick")
        self.btn_jetco_manual_place = QPushButton(f"Place")
        self.manual_request_box.add_widget(self.btn_jetco_manual_pick)
        self.manual_request_box.add_widget(self.btn_jetco_manual_place)
        # self.btn_jetco_manual_pick.clicked.connect(self.on_jetco_manual_pick_clicked)
        self.btn_jetco_manual_place.clicked.connect(self.on_jetco_manual_place_clicked)

        right_layout.addWidget(self.assembly_box)
        right_layout.addWidget(self.manual_request_box)

        right_layout.addStretch()

        button_area.addLayout(left_layout, 1)
        button_area.addLayout(right_layout, 1)

        main_jet_layout.addLayout(button_area)

        self.jetcobot_shared_panel.hide()

        ##############################################3

        layout.addWidget(self.pinky_shared_panel)
        layout.addWidget(self.jetcobot_shared_panel)
        self.jetcobot_shared_panel.hide()
        self.pinky_shared_panel.hide()
        layout.addStretch(1)
        self.tab_status.setLayout(layout)

    # 🔷 jetcobot 왼쪽 button1 분기
    def handle_jetco_common1(self):
        self.apply_click_feedback(self.sender())
        
        if not self.current_viewing_robot:
            QMessageBox.warning(self, "오류", "로봇을 먼저 선택하세요")
            return

        if self.current_viewing_robot == "jetcobot1": # storage
            self.ros_thread.send_storage_manip_start()
            self.add_log("[jetcobot1] /jetcobot/storage/boot True publish")
            return
        elif self.current_viewing_robot == "jetcobot2": # assembly
            self.ros_thread.send_assembly_manip_start()
            self.add_log("[jetcobot] /jetcobot/storage/boot True publish")
            return

        elif self.current_viewing_robot == "jetcobot3": # openmanipulator
            self.add_log("OpenManipulator → PNP START")
            self.ros_thread.send_manip_start()

        else:
            QMessageBox.warning(self, "오류", "지원되지 않는 로봇")

    # 🔷 jetcobot 왼쪽 button2 분기
    def handle_jetco_common2(self):
        self.apply_click_feedback(self.sender())
        
        if not self.current_viewing_robot:
            QMessageBox.warning(self, "오류", "로봇을 먼저 선택하세요")
            return

        if self.current_viewing_robot == "jetcobot1": # storage
            self.ros_thread.send_storage_manip_stop()
            self.add_log("[jetcobot1] /jetcobot/storage/boot False publish")
            return

        elif self.current_viewing_robot == "jetcobot2": # assembly
            self.add_log("Assembly Jetcobot → PNP START")
            # self.ros_thread.send_assembly_pnp()

        elif self.current_viewing_robot == "jetcobot3": # openmanipulator
            self.add_log("OpenManipulator → PNP START")
            # self.ros_thread.

        else:
            QMessageBox.warning(self, "오류", "지원되지 않는 로봇")

    # 🔷 jetcobot 오른쪽 action1 분기
    def handle_jetco_action1(self):
        self.apply_click_feedback(self.sender())

        if not self.current_viewing_robot:
            QMessageBox.warning(self, "오류", "로봇을 먼저 선택하세요")
            return

        if self.current_viewing_robot == "jetcobot1": # storage
            self.ros_thread.send_storage_auto_pub()
            if "jetcobot1" not in self.robot_data_storage:
                self.robot_data_storage["jetcobot1"] = {"id": "jetcobot1"}
            self.robot_data_storage["jetcobot1"]["mode"] = "AUTO"
            self.lbl_jetco_mode_vale.setText("AUTO")
            self.add_log("[jetcobot1] /jetcobot/storage/set_mode publish: Auto Mode")
            return
        
        elif self.current_viewing_robot == "jetcobot2": # assembly
            self.ros_thread.send_assembly_assembly_stack_request_pub(int(self.selected_module[2]))
            self.add_log("[jetcobot2] /jetcobot/assembly/stack/request publish")
            return

        elif self.current_viewing_robot == "jetcobot3": # openmanipulator
            self.add_log("OpenManipulator → PNP START")
            # self.ros_thread.

        else:
            QMessageBox.warning(self, "오류", "지원되지 않는 로봇")

    # 🔷 jetcobot 오른쪽 action2 분기
    def handle_jetco_action2(self):
        self.apply_click_feedback(self.sender())

        if not self.current_viewing_robot:
            QMessageBox.warning(self, "오류", "로봇을 먼저 선택하세요")
            return

        if self.current_viewing_robot == "jetcobot1": # storage
            self.ros_thread.send_storage_manual_pub()
            if "jetcobot1" not in self.robot_data_storage:
                self.robot_data_storage["jetcobot1"] = {"id": "jetcobot1"}
            self.robot_data_storage["jetcobot1"]["mode"] = "MANUAL"
            self.lbl_jetco_mode_vale.setText("MANUAL")
            self.add_log("[jetcobot1] /jetcobot/storage/set_mode publish: Manual Mode")
            return

        elif self.current_viewing_robot == "jetcobot2": # assembly
            self.add_log("Assembly Jetcobot → PNP START")
            # self.ros_thread.

        elif self.current_viewing_robot == "jetcobot3": # openmanipulator
            self.add_log("OpenManipulator → PNP START")
            # self.ros_thread.

        else:
            QMessageBox.warning(self, "오류", "지원되지 않는 로봇")

    # 🔷 storage 수동 명령 토글
    def on_jetco_manual_place_clicked(self):
        if self.current_viewing_robot == "jetcobot1":
            if self.part_id is None or self.section is None or self.section_id is None:
                QMessageBox.warning(self, "입력 오류", "part_id와 slot_id(A-1)를 올바르게 입력하세요.")
                return
            # 실제 publish는 ros_thread에게 위임``
            self.ros_thread.send_storage_manual_request_place(self.part_id, self.section, self.section_id)
            #초기화
            self.part_id = None
            self.section = None
            self.section_id = None
            self.add_log("[jetcobot1] /jetcobot/storage/set_mode publish: Manual Mode")
            return
        
    def on_part_value_edited(self):
        text = self.lbl_part_value.text().strip()
        if not text:
            self.part_id = None
            return

        try:
            self.part_id = int(text)
        except ValueError:
            self.part_id = None
            self.add_log("[입력오류] part_id는 숫자여야 합니다.")

    def on_slot_value_edited(self): # 입력 데이터는 A-1 꼴로 고정
        text = self.lbl_slot_value.text().strip().upper()
        if not text:
            self.section = None
            self.section_id = None
            return

        parts = text.split("-", 1)
        if len(parts) != 2:
            self.section = None
            self.section_id = None
            self.add_log("[입력오류] slot_id 형식은 A-1 입니다.")
            return

        section, section_id_text = parts[0], parts[1]
        if len(section) != 1 or not section.isalpha() or not section_id_text.isdigit():
            self.section = None
            self.section_id = None
            self.add_log("[입력오류] slot_id 형식은 A-1 입니다.")
            return

        self.section = section
        self.section_id = int(section_id_text)

    def setup_log_tab(self):
        layout = QVBoxLayout()
        self.log_text_edit = QTextEdit(); self.log_text_edit.setReadOnly(True)
        layout.addWidget(self.log_text_edit); self.tab_log.setLayout(layout)

    def setup_db_tab_full(self):
        main = QVBoxLayout(); top = QHBoxLayout()
        
        grp_warehouse = QGroupBox("창고 현황")
        layout_warehouse = QHBoxLayout() # 전체 가로 배치

        sections = ['A', 'B', 'C']
        for section in sections:
            # 각 구역별 그룹박스 (예: 창고 A)
            grp_section = QGroupBox(f"창고 {section}")
            layout_section = QVBoxLayout() # 내부 세로 배치
            layout_section.setSpacing(5)
            layout_section.setContentsMargins(5, 10, 5, 5)

            for i in range(1, 4): # 1, 2, 3
                slot_id = f"{section}-{i}"
                # 카드 생성 (타이틀: 1번 칸)
                card = WarehouseCard(f"{i}번 칸", slot_id) 
                layout_section.addWidget(card)
                self.warehouse_cards[slot_id] = card # 갱신을 위해 저장
            
            grp_section.setLayout(layout_section)
            layout_warehouse.addWidget(grp_section)

        grp_warehouse.setLayout(layout_warehouse)

        # 나머지 UI
        grp_c = QGroupBox("제어 패널"); grp_c.setMaximumHeight(150); lc = QVBoxLayout()
        in_l = QHBoxLayout(); self.part_input = QLineEdit(); self.part_input.setPlaceholderText("부품 ID")
        btn_s = QPushButton("검색"); btn_s.clicked.connect(self.on_search_clicked)
        self.lbl_res = QLabel("결과: -"); in_l.addWidget(QLabel("ID:")); in_l.addWidget(self.part_input); in_l.addWidget(btn_s); in_l.addWidget(self.lbl_res)
        btns = QHBoxLayout(); b1=QPushButton("최신 주문"); b1.clicked.connect(self.load_latest_order_from_db)
        b2=QPushButton("새로고침"); b2.clicked.connect(self.load_verification_table)
        b3=QPushButton("초기화"); b3.setStyleSheet("background-color: #f44336; color: white;"); b3.clicked.connect(self.reset_db)
        btns.addWidget(b1); btns.addWidget(b2); btns.addWidget(b3)
        lc.addLayout(in_l); lc.addLayout(btns); grp_c.setLayout(lc)
        grp_h = QGroupBox("주문 이력"); grp_h.setMaximumHeight(150); lh = QVBoxLayout()
        self.history_table = QTableWidget(); self.history_table.setColumnCount(3)
        self.history_table.setHorizontalHeaderLabels(["No.", "프로젝트", "날짜"])
        self.history_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.history_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.history_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.history_table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.history_table.cellClicked.connect(self.on_history_cell_clicked)
        lh.addWidget(self.history_table); grp_h.setLayout(lh)
        top.addWidget(grp_c, 6); top.addWidget(grp_h, 4)
        bot = QHBoxLayout()
        
        # 검수 목록 테이블
        grp_t = QGroupBox("검수 목록"); lt = QVBoxLayout()
        self.db_table = QTableWidget(); self.db_table.setColumnCount(5)
        self.db_table.setHorizontalHeaderLabels(["ID", "제품명", "목표", "현재", "상태"])
        self.db_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        lt.addWidget(self.db_table); grp_t.setLayout(lt)
        
        bot.addWidget(grp_warehouse, 1); bot.addWidget(grp_t, 1)
        main.addLayout(top); main.addLayout(bot)
        self.tab_db.setLayout(main)

    def on_tab_changed(self, index):
        if index == 2: 
            self.map_widget.hide(); self.control_group.hide()
            self.load_verification_table(); self.load_quote_history() 
        else: 
            self.map_widget.show(); self.control_group.show()

    # 수동 조작 명령 전송 함수
    def send_manual_cmd(self, direction):
        if self.current_viewing_robot:
            self.add_log(f"[{self.current_viewing_robot}] 수동 조작: {direction}")
        else:
            QMessageBox.warning(self, "오류", "로봇을 먼저 선택해주세요.")

    # 자동 이동 명령 전송 함수
    def send_nav_cmd(self, location, button_obj=None): #지니 
        if self.current_viewing_robot:
            role_map = {
                "INSPECTION_ZONE": "1",
                "PARTS_WAREHOUSE": "4",
                "ASSEMBLY_ZONE": "3",
            }
            role_id = role_map.get(location)
            if role_id:
                self.ros_thread.send_move_role(self.current_viewing_robot, role_id)
                self.add_log(f"[{self.current_viewing_robot}] 자동 이동 명령: {location} -> /move_role {role_id}")
                self.refresh_assignment_ui()
            else:
                self.add_log(f"[{self.current_viewing_robot}] 자동 이동 명령: {location}")
            
            # 2. 상태 메시지 업데이트
            target_name = ""
            if location == "INSPECTION_ZONE": target_name = "검수대 이동중"
            elif location == "PARTS_WAREHOUSE": target_name = "모듈 창고 이동중"
            elif location == "ASSEMBLY_ZONE": target_name = "조립대 이동중"
            self.lbl_state.setText(target_name)

                
        else:
            QMessageBox.warning(self, "오류", "로봇을 먼저 선택해주세요.")
          
    def role_id_to_text(self, role_id): #지니
        role_map = {
            "0": "대기",
            "1": "검수대",
            "3": "조립대",
            "4": "모듈 창고",
        }
        if role_id is None:
            return "-"
        return role_map.get(str(role_id), str(role_id))

    def refresh_assignment_ui(self): #지니
        assignments = self.ros_thread.robot_role_assignments
        for rid, display_name in self.robot_name_map.items():
            label = self.pinky_status_labels.get(rid)
            if not label:
                continue
            role_text = self.role_id_to_text(assignments.get(rid))
            label.setText(f"{display_name} : {role_text}")

        if self.current_viewing_robot:
            current_role = assignments.get(self.current_viewing_robot)
            if current_role is not None:
                self.lbl_state.setText(self.role_id_to_text(current_role))

  # pinky, Jetcobot toggle 분기
    def select_robot(self, robot_id):
        self.current_viewing_robot = robot_id.lower().replace(" ", "")
        
        self.info_label.hide()
        self.tabs.setCurrentIndex(0)
        self.pinky_all_panel.hide()
        self.pinky_shared_panel.hide()
        self.jetcobot_shared_panel.hide()
        if robot_id == "main_control":
            self.pinky_all_panel.show()

        elif "pinky" in robot_id:
            self.pinky_shared_panel.show()

        elif "jetcobot" in robot_id:
            self.jetcobot_shared_panel.show()

            if robot_id == "jetcobot1":  # storage
                self.btn_jetco_common1.setText("Start")
                self.btn_jetco_common2.setText("Shutdown")
                self.btn_jetco_action1.setText("Auto")
                self.btn_jetco_action2.setText("Manual")
                self.btn_jetco_action2.show()

                self.lbl_slot_title.show()
                self.lbl_slot_value.show()
                self.lbl_part_title.show()
                self.lbl_part_value.show()

                self.assembly_box.hide()
                self.manual_request_box.show()

            elif robot_id == "jetcobot2":  # assembly
                self.btn_jetco_common1.setText("Start")
                self.btn_jetco_common2.setText("Shutdown")
                self.btn_jetco_action1.setText("Send")
                self.btn_jetco_action2.hide()
                self.lbl_jetco_mode_vale.setText("MANUAL")
                if "jetcobot2" not in self.robot_data_storage:
                    self.robot_data_storage["jetcobot2"] = {"id": "jetcobot2"}
                self.robot_data_storage["jetcobot2"]["mode"] = "MANUAL"

                self.lbl_slot_title.hide()
                self.lbl_slot_value.hide()
                self.lbl_part_title.hide()
                self.lbl_part_value.hide()

                self.assembly_box.show()
                self.manual_request_box.hide()

            elif robot_id == "jetcobot3":
                self.btn_jetco_common1.setText("pnp start")
                self.btn_jetco_common2.setText("test_btn")
                self.btn_jetco_action1.setText("test_btn1")
                self.btn_jetco_action2.setText("test_btn2")
                self.btn_jetco_action2.show()

                self.lbl_slot_title.hide()
                self.lbl_slot_value.hide()
                self.lbl_part_title.hide()
                self.lbl_part_value.hide()

                self.assembly_box.hide()
                self.manual_request_box.hide()

        is_pinky = "pinky" in self.current_viewing_robot
        self.btn_load_complete.setVisible(is_pinky); self.btn_unload_complete.setVisible(is_pinky)
        
        self.grp_manual.setVisible(is_pinky)
        self.grp_nav.setVisible(is_pinky)
        
        if self.current_viewing_robot in self.robot_data_storage: 
            self.refresh_detail_view(self.robot_data_storage[self.current_viewing_robot])
        else: 
            display_name = self.robot_name_map.get(self.current_viewing_robot, self.current_viewing_robot.upper())
            self.lbl_name.setText(display_name)
            self.lbl_bat.setText("-"); self.lbl_state.setText("-"); self.lbl_loc.setText("-")
            self.lbl_jetco_status_value.setText("-"); self.lbl_jetco_mode_vale.setText("-") ##
            self.lbl_slot_value.setText(""); self.lbl_part_value.setText("") ##

    # assembly 전용 토글
    def select_module(self, module_name):
        if self.current_viewing_robot == "jetcobot2":
            self.selected_module = module_name
            self.assembly_box.set_selected(module_name)

    # log
    def add_log(self, message):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        if self.log_text_edit: self.log_text_edit.append(f"[{ts}] {message}")

    ##########################################
    # button effect, styles Setting
    ##########################################
    # button blink effect
    def apply_click_feedback(self, button_obj):
        if not button_obj:
            return

        button_obj.setDown(True)

        button_obj.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                font-weight: bold;
                padding: 10px;
                border-radius: 6px;
            }

            QPushButton:pressed {
                background-color: #1976D2;
                padding-top: 12px;
                padding-left: 12px;
            }
        """)

        QTimer.singleShot(
            120,
            lambda: button_obj.setDown(False)
        )

    # Label style 
    def style_info_label(self, widget):
        widget.setStyleSheet("""
            QLabel {
                background-color: #F0F8FF;
                border-radius: 8px;
                border: 1px solid #90CAF9;
                padding: 4px;
            }
        """)

        font = QFont("Arial", 14, QFont.Weight.Bold)
        widget.setFont(font)

    # Label style1
    def style_info_label_1(self, widget):
        widget.setStyleSheet("""
            QLabel {
                background-color: #F0F8FF;
                border-radius: 8px;
                border: 1px solid #90CAF9;
                padding: 4px;
            }
        """)

        font = QFont("Arial", 14, QFont.Weight.Bold)
        widget.setFont(font)

    # button stlye
    def setup_primary_button(self, button, layout=None):
        button.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                font-weight: bold;
                padding: 10px;
                border-radius: 6px;
            }
            QPushButton:pressed {
                background-color: #1976D2;
            }
        """)

        button.setMinimumHeight(50)
        button.setSizePolicy(QSizePolicy.Policy.Expanding,
                            QSizePolicy.Policy.Fixed)

        if layout is not None:
            layout.addWidget(button)


    ##############################################################################
    # Button Event Handlers
    # 버튼에 연결되는 실제 동작 로직
    ##############################################################################

    # camera
    def update_camera_image(self, image): self.top_camera_label.setPixmap(QPixmap.fromImage(image))
    

    # MYSLQ DB #############################################################
    def handle_unload_event(self, aruco_id):
        self.add_log(f"출고 신호 수신: ID {aruco_id}")
        try:
            conn = mysql.connector.connect(host=DB_HOST, user=DB_USER, password=DB_PASS, database=DB_NAME)
            cursor = conn.cursor()
            sql = "UPDATE request_list SET current_qty = GREATEST(current_qty - 1, 0) WHERE aruco_id = %s"
            cursor.execute(sql, (aruco_id,))
            sql_free = "UPDATE warehouse_slots SET is_occupied=0, current_part_id=NULL WHERE current_part_id=%s LIMIT 1"
            cursor.execute(sql_free, (aruco_id,))
            conn.commit(); conn.close()
            self.load_verification_table()
            self.add_log(f"출고 및 슬롯 해제 완료: ID {aruco_id}")
        except Exception as e: print(f"DB Error: {e}")

    def on_history_cell_clicked(self, row, col):
        item = self.history_table.item(row, 0)
        if item: self.load_filtered_verification_table(item.text())

    def load_filtered_verification_table(self, quote_id):
        try:
            conn = mysql.connector.connect(host=DB_HOST, user=DB_USER, password=DB_PASS, database=DB_NAME)
            cursor = conn.cursor()
            cursor.execute("SELECT part_id FROM quote_details WHERE quote_id = %s", (quote_id,))
            part_rows = cursor.fetchall()
            if not part_rows: conn.close(); return
            t_ids = [str(r[0]) for r in part_rows]
            fmt = ','.join(['%s'] * len(t_ids))
            sql = f"SELECT aruco_id, name, target_qty, current_qty FROM request_list WHERE aruco_id IN ({fmt})"
            cursor.execute(sql, tuple(t_ids)); rows = cursor.fetchall(); conn.close()
            self.db_table.setRowCount(0)
            for i, (aid, name, tgt, cur) in enumerate(rows):
                self.db_table.insertRow(i)
                status = "✅ 완료" if cur >= tgt and tgt > 0 else "⚠️ 진행중" if cur > 0 else "대기"
                self.db_table.setItem(i,0,QTableWidgetItem(str(aid)))
                self.db_table.setItem(i,1,QTableWidgetItem(str(name)))
                self.db_table.setItem(i,2,QTableWidgetItem(str(tgt)))
                self.db_table.setItem(i,3,QTableWidgetItem(str(cur)))
                self.db_table.setItem(i,4,QTableWidgetItem(status))
        except Exception: pass

    def load_quote_history(self):
        try:
            conn = mysql.connector.connect(host=DB_HOST, user=DB_USER, password=DB_PASS, database=DB_NAME)
            cur = conn.cursor(); cur.execute("SELECT quote_id, project_name, created_at FROM quotes ORDER BY quote_id DESC")
            rows = cur.fetchall(); conn.close()
            self.history_table.setRowCount(0)
            for i, (qid, proj, date) in enumerate(rows):
                self.history_table.insertRow(i)
                ds = date.strftime("%Y-%m-%d %H:%M") if date else "-"
                self.history_table.setItem(i,0,QTableWidgetItem(str(qid)))
                self.history_table.setItem(i,1,QTableWidgetItem(str(proj)))
                self.history_table.setItem(i,2,QTableWidgetItem(ds))
        except Exception: pass

    def on_search_clicked(self):
        pid = self.part_input.text().strip()
        if not pid: return
        try:
            conn = mysql.connector.connect(host=DB_HOST, user=DB_USER, password=DB_PASS, database=DB_NAME)
            cur = conn.cursor(); cur.execute("SELECT COALESCE(SUM(quantity), 0) FROM inventory_history WHERE part_id = %s", (pid,))
            res = cur.fetchone()[0]; conn.close()
            self.lbl_res.setText(f"결과: {res}개")
        except: self.lbl_res.setText("결과: 에러")

    def refresh_search_result(self, detected_id):
        cur = self.part_input.text().strip()
        if cur and str(detected_id) == cur: self.on_search_clicked()

    def load_latest_order_from_db(self):
        try:
            conn = mysql.connector.connect(host=DB_HOST, user=DB_USER, password=DB_PASS, database=DB_NAME)
            cursor = conn.cursor(); cursor.execute("SELECT MAX(quote_id) FROM quotes")
            row = cursor.fetchone()
            if not row or row[0] is None: QMessageBox.warning(self, "없음", "주문 내역 없음"); conn.close(); return
            lid = row[0]
            sql = """SELECT qd.part_id, p.part_name, qd.req_quantity 
                     FROM quote_details qd JOIN parts p ON qd.part_id = p.part_id WHERE qd.quote_id = %s"""
            cursor.execute(sql, (lid,)); items = cursor.fetchall()
            if not items: conn.close(); return
            for pid, pname, qty in items:
                cname = pname.split('(')[0].strip() if '(' in pname else pname
                cursor.execute("SELECT aruco_id FROM request_list WHERE aruco_id = %s", (pid,))
                if cursor.fetchone(): cursor.execute("UPDATE request_list SET target_qty = target_qty + %s WHERE aruco_id = %s", (qty, pid))
                else: cursor.execute("INSERT INTO request_list (aruco_id, name, target_qty, current_qty) VALUES (%s, %s, %s, 0)", (pid, cname, qty))
            conn.commit(); conn.close()
            self.load_verification_table(); self.load_quote_history()
            QMessageBox.information(self, "완료", f"주문 #{lid} 불러오기 성공"); self.add_log(f"주문 #{lid} 로드됨")
        except Exception as e: print(f"DB Error: {e}")
    
    def load_verification_table(self):
        try:
            conn = mysql.connector.connect(host=DB_HOST, user=DB_USER, password=DB_PASS, database=DB_NAME)
            cur = conn.cursor()
            
            # 1. request_list (검수 목록) 업데이트
            cur.execute("SELECT aruco_id, name, target_qty, current_qty FROM request_list")
            rows = cur.fetchall()
            self.db_table.setRowCount(0)
            for i, (aid, name, tgt, cur_qty) in enumerate(rows):
                self.db_table.insertRow(i)
                status = "✅ 완료" if cur_qty >= tgt and tgt > 0 else "⚠️ 진행중" if cur_qty > 0 else "대기"
                self.db_table.setItem(i,0,QTableWidgetItem(str(aid)))
                self.db_table.setItem(i,1,QTableWidgetItem(str(name)))
                self.db_table.setItem(i,2,QTableWidgetItem(str(tgt)))
                self.db_table.setItem(i,3,QTableWidgetItem(str(cur_qty)))
                self.db_table.setItem(i,4,QTableWidgetItem(status))
                col = QColor(200,255,200) if cur_qty >= tgt and tgt > 0 else QColor(255,255,224) if cur_qty > 0 else QColor(255,255,255)
                for c in range(5): self.db_table.item(i,c).setBackground(col)

            # 2. warehouse_slots (창고 카드) 업데이트
            # DB에서 슬롯 정보를 가져와서 카드에 반영 - 수정
            # parts 테이블과 조인하여 부품 이름까지 가져옴
            query = """
                SELECT s.slot_id, s.is_occupied, s.current_part_id, p.part_name 
                FROM warehouse_slots s 
                LEFT JOIN parts p ON s.current_part_id = p.part_id
            """
            cur.execute(query)
            slot_rows = cur.fetchall()
            
            for slot_id, occupied, part_id, part_name in slot_rows:
                if slot_id in self.warehouse_cards:
                    display_name = part_name if part_name else (str(part_id) if part_id else "-")
                    self.warehouse_cards[slot_id].update_info(display_name, occupied)

            conn.close()
        except Exception as e: 
            print(f"Update Error: {e}")
    ########################################################################

    def reset_db(self):
        if QMessageBox.question(self, '확인', '초기화?', QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No) == QMessageBox.StandardButton.Yes:
            try:
                conn = mysql.connector.connect(host=DB_HOST, user=DB_USER, password=DB_PASS, database=DB_NAME)
                cur = conn.cursor(); cur.execute("TRUNCATE TABLE request_list"); cur.execute("TRUNCATE TABLE inventory_history")
                # 창고 슬롯도 초기화
                cur.execute("UPDATE warehouse_slots SET is_occupied=0, current_part_id=NULL")
                conn.commit(); conn.close()
                self.load_verification_table(); self.load_quote_history()
                QMessageBox.information(self, "완료", "초기화됨"); self.add_log("데이터 초기화됨")
            except Exception: pass

    def closeEvent(self, event):
        self.ros_thread.stop()
        self.camera_thread.stop()
        event.accept()

    ##############################################################################            
    # ros_thread connection method
    ##############################################################################

    # GUI Robot State Update System
    def update_ros_data(self, data):
        rid = data.get("id", "")
        if not rid: return
        if rid not in self.robot_data_storage: self.robot_data_storage[rid] = {}
        self.robot_data_storage[rid].update(data)
        if "location" in data:
            try:
                loc = data["location"]
                x, y = map(int, loc.split(","))
                self.map_widget.update_position(self.robot_name_map.get(rid, rid), x, y)
            except: pass
        if self.current_viewing_robot == rid: 
            self.refresh_detail_view(self.robot_data_storage[rid])

    def refresh_detail_view(self, data):
        rid = data.get("id", "")
        display_name = self.robot_name_map.get(rid, rid.upper() if rid else "Unknown")
        
        self.lbl_name.setText(display_name)
        self.lbl_bat.setText("-"); self.lbl_state.setText("-"); self.lbl_loc.setText("-") ##
        self.lbl_jetco_status_value.setText("-"); self.lbl_jetco_mode_vale.setText("-") ##
        if "battery" in data: self.lbl_bat.setText(f"{data['battery']:.2f}%")
        if "state" in data: self.lbl_state.setText(data['state'])
        if "location" in data: self.lbl_loc.setText(data['location'])
        if "status" in data: self.lbl_jetco_status_value.setText(data["status"])
        if "mode" in data: self.lbl_jetco_mode_vale.setText(data["mode"])
        if rid == "jetcobot2": self.lbl_jetco_mode_vale.setText("MANUAL")
        if "slot_id" in data: self.lbl_slot_value.setText(str(data["slot_id"]))
        elif "slot" in data: self.lbl_slot_value.setText(str(data["slot"]))
        if "part_id" in data: self.lbl_part_value.setText(str(data["part_id"]))
        elif "part" in data: self.lbl_part_value.setText(str(data["part"]))

    def on_load_complete(self): 
        if self.current_viewing_robot:
            self.ros_thread.send_load_done(self.current_viewing_robot)
            self.add_log(f"명령 전송: {self.current_viewing_robot} -> /load_done True")
            QMessageBox.information(self, "명령", "상차 완료 명령 전송")

    def on_unload_complete(self): 
        if self.current_viewing_robot:
            self.ros_thread.send_unload_done(self.current_viewing_robot)
            self.add_log(f"명령 전송: {self.current_viewing_robot} -> /unload_done True")
            QMessageBox.information(self, "명령", "하차 완료 명령 전송")
    
    # assembly 전용 토글 - send 버튼 연결
    def handle_send(self):
        if not self.selected_module:
            QMessageBox.warning(self, "오류", "모듈을 먼저 선택하세요")
            return

        module_code = self.module_map[self.selected_module]
        # self.ros_thread.                                           # ros_thread 연결
        self.add_log(f"{self.current_viewing_robot} -> {module_code} 명령 전송")
        QMessageBox.information(self, "명령", f"{self.selected_module} 명령 전송 완료")

    def on_slot_allocated(self, slot_id):
        self.add_log(f"📍 빈 슬롯 찾음: {slot_id} -> 로봇팔 전송")
        self.ros_thread.send_arm_target(slot_id)

    #작업 시작 버튼 처리 함수
    def on_start_random_assignment(self): #지니
        assignments = self.ros_thread.assign_random_work_and_move()
        if not assignments:
            QMessageBox.warning(self, "오류", "업무 할당에 실패했습니다. ROS 연결 상태를 확인하세요.")
            return
        self.refresh_assignment_ui()
        QMessageBox.information(self, "업무 시작", "업무 할당 완료")

    # 작업 종료 버튼 처리 함수
    def on_stop_all_assignments(self): #지니
        move_role_keys = sorted(self.ros_thread.move_role_pubs.keys())
        if not move_role_keys:
            QMessageBox.warning(self, "오류", "로봇 퍼블리셔가 준비되지 않았습니다.")
            return

        for robot_key in move_role_keys:
            robot_id = robot_key.lstrip("/")
            self.ros_thread.send_move_role(robot_id, "0")

        self.refresh_assignment_ui()
        QMessageBox.information(self, "작업종료", "전체 로봇을 대기로 변경했습니다.")

    #지니 : 메인컨트롤 상하차 버튼 관련
    def send_done_by_role(self, role_id, done_type):
        assignments = self.ros_thread.robot_role_assignments
        target_robot = None
        for robot_id, assigned_role in assignments.items():
            if str(assigned_role) == str(role_id):
                target_robot = robot_id
                break

        if not target_robot:
            QMessageBox.warning(self, "오류", f"업무 {role_id}에 배정된 로봇이 없습니다.")
            return

        if done_type == "load":
            self.ros_thread.send_load_done(target_robot)
            self.add_log(f"[메인컨트롤] 업무 {role_id} -> {target_robot} /load_done True")
            QMessageBox.information(self, "명령", f"{target_robot} 상차 완료 전송")
        elif done_type == "unload":
            self.ros_thread.send_unload_done(target_robot)
            self.add_log(f"[메인컨트롤] 업무 {role_id} -> {target_robot} /unload_done True")
            QMessageBox.information(self, "명령", f"{target_robot} 하차 완료 전송")
        else:
            QMessageBox.warning(self, "오류", f"지원하지 않는 완료 타입: {done_type}")
