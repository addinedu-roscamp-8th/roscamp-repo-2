import json  
import os  

import rclpy  
from rclpy.action import ActionClient  
from rclpy.node import Node 
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy  
from std_msgs.msg import String, Bool, Int32  

from action_msgs.msg import GoalStatus  
from geometry_msgs.msg import PoseStamped  
from nav2_msgs.action import NavigateToPose  

# 로그 포맷(타임스탬프 제거)
os.environ.setdefault(
    "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{name}]: {message}"
)


class ScenarioManager(Node):
    # 시나리오 상태머신 클래스
    def __init__(self):
        super().__init__("scenario_manager")  # 노드 이름
        self._declare_params()  # 파라미터 선언
        qos = self._default_qos()  # QoS 기본값 생성

        # go_to_goal 액션 클라이언트(로컬 네비게이션 브리지)
        self._nav_client = ActionClient(self, NavigateToPose, "go_to_goal")
        # FSM 상태 + 현재 goal 추적
        self._state = "READY"  # 현재 FSM 상태(대기/이동/대기입력)
        self._active_goal = None  # 진행 중인 네비게이션 goal 핸들
        self._target_point = None  # 현재 이동 목표 포인트 이름 (예: point1)
        self._post_nav_state = None  # 도착 후 전이할 다음 상태
        # NAV 중 새 명령이 들어올 때 재지정 큐
        self._pending_start = False  # 대기장소 이동 예약 여부
        self._pending_target = None  # 취소 후 이동할 새 목표
        self._pending_post_state = None  # 새 목표 도착 후 상태
        # GUI 입력 분리 (이동/상차/하차)
        self._move_sub = self.create_subscription(
            String, "/move_role", self._move_cb, qos  # 이동 역할
        )
        self._load_sub = self.create_subscription(
            Bool, "/load_done", self._load_cb, qos  # 상차 완료
        )
        self._unload_sub = self.create_subscription(
            Bool, "/unload_done", self._unload_cb, qos  # 하차 완료
        )
        self._manip_sub = self.create_subscription(
            Bool, "/pick_and_place/done", self._manip_cb, qos  # 매니퓰레이터 완료 신호
        )
        # 업무2 전용(jetcobot) 완료 신호
        self._manip_sub_jetcobot = self.create_subscription(
            Bool, "/jetcobot/pick_and_place/done", self._manip_jetcobot_cb, qos
        )
        # 도착 지점 알림(포인트 이름으로 토픽발행)
        self._arrived_pub = self.create_publisher(Int32, "/arrived_point", 10)
        # 매니퓰레이터 시작 신호
        self._manip_start_pub = self.create_publisher(Bool, "/pick_and_place/start", 10)

    def _declare_params(self):
        # 파라미터 선언
        self.declare_parameter("goal_file", "")  # goal_points.json 경로
        self.declare_parameter("start_point", "point0")  # 대기장소 포인트

    def _default_qos(self):
        # QoS 기본값 생성
        qos = QoSProfile(depth=10)  # 큐 깊이
        qos.reliability = ReliabilityPolicy.RELIABLE  # 신뢰성
        qos.durability = DurabilityPolicy.VOLATILE  # 휘발성
        return qos  # QoS 반환

    def _move_cb(self, msg):
        # 이동 역할 콜백 (1/3/4/0)
        cmd = msg.data.strip()
        if not cmd:
            return
        if cmd == "0":
            self._go_start_point()
            return
        if cmd in ("1", "3", "4"):
            if cmd == "1":
                self._interrupt_and_go("point1-1", "WAIT_SPACE_FROM_1")
            elif cmd == "3":
                self._interrupt_and_go("point3", "WAIT_SPACE_FROM_3")
            elif cmd == "4":
                self._interrupt_and_go("point4-2", "WAIT_SPACE_FROM_4")
            return
        self.get_logger().info(f"Unknown move role: {cmd}")

    def _load_cb(self, msg):
        # 상차 완료 콜백 (Bool)
        if not msg.data:
            return
        if self._state == "WAIT_SPACE_FROM_1":  # 업무1 상차 완료
            self._start_nav("point2", "WAIT_Q_FROM_2")
        elif self._state == "WAIT_SPACE_FROM_4":  # 업무3 상차 완료
            self._start_nav("point1-2", "WAIT_Q_FROM_1_TO_4")
        else:
            self.get_logger().info(f"Ignoring load_done in {self._state}")

    def _unload_cb(self, msg):
        # 하차 완료 콜백 (Bool)
        if not msg.data:
            return
        if self._state == "WAIT_Q_FROM_4_TO_3":  # 업무2 하차 완료
            self._start_nav("point3", "WAIT_SPACE_FROM_3")
        elif self._state == "WAIT_Q_FROM_1_TO_4":  # 업무3 하차 완료
            self._start_nav("point4-2", "WAIT_SPACE_FROM_4")
        else:
            self.get_logger().info(f"Ignoring unload_done in {self._state}")

    def _manip_cb(self, msg):
        # 매니퓰레이터 완료 신호 콜백 (Bool)
        if not msg.data:
            return  # False는 무시
        # 기본 done 토픽은 업무1 하차 완료만 처리
        if self._state == "WAIT_Q_FROM_2":  # 업무1 하차 완료
            self._start_nav("point1-1", "WAIT_SPACE_FROM_1")  # point1 복귀
        else:
            self.get_logger().info(f"Ignoring manip_done in {self._state}")  # 무시

    def _manip_jetcobot_cb(self, msg):
        # jetcobot done 토픽은 업무2 상차 완료만 처리
        if not msg.data:
            return
        if self._state == "WAIT_SPACE_FROM_3":
            self._start_nav("point4-1", "WAIT_Q_FROM_4_TO_3")
        else:
            self.get_logger().info(f"Ignoring jetcobot manip_done in {self._state}")

    def _start_nav(self, point_name, post_state):
        # 네비게이션 시작
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose not available")  # 서버 없음
            self._state = "READY"  # 상태 복귀
            return
        # 목적지/도착 후 상태 저장
        self._state = "NAV"  # 이동 상태로 전환
        self._post_nav_state = post_state  # 도착 후 상태 저장
        self._target_point = point_name  # 목표 포인트 저장
        self._send_nav_goal(point_name)  # goal 전송

    def _send_nav_goal(self, point_name):
        # 액션 goal 전송
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose not available")  # 서버 없음
            self._state = "READY"  # 상태 복귀
            return
        goal = NavigateToPose.Goal()  # Nav2 goal 메시지
        goal.pose = self._build_goal_pose(point_name)  # 포인트 → Pose 변환
        future = self._nav_client.send_goal_async(goal)  # 비동기 전송
        future.add_done_callback(self._nav_goal_response_cb)  # 응답 콜백 등록

    def _go_start_point(self):
        # 대기장소 이동
        start_point = self.get_parameter("start_point").value  # 대기장소 이름
        self._interrupt_and_go(start_point, "READY")  # 이동 후 READY

    def _interrupt_and_go(self, point_name, post_state):
        # 이동 중이면 현재 goal 취소하고 새 목표를 큐에 저장
        if self._state == "NAV" and self._active_goal is not None:
            self.get_logger().info(f"Canceling current goal to go {point_name}")  # 로그
            self._pending_start = False  # 대기장소 예약 해제
            self._pending_target = point_name  # 새 목표 저장
            self._pending_post_state = post_state  # 새 목표 도착 후 상태 저장
            cancel_future = self._active_goal.cancel_goal_async()  # 취소 요청
            cancel_future.add_done_callback(self._cancel_done_cb)  # 취소 콜백
            return
        self._start_nav(point_name, post_state)  # 바로 이동 시작

    def _cancel_done_cb(self, future):
        # 취소 완료 콜백
        _ = future.result()  # 취소 결과 수신
        if self._pending_target:
            target = self._pending_target  # 새 목표
            post_state = self._pending_post_state or "READY"  # 기본 상태
            self._pending_target = None  # 큐 초기화
            self._pending_post_state = None  # 큐 초기화
            self._start_nav(target, post_state)  # 취소 후 새 목표 이동
            return
        if self._pending_start:
            self._pending_start = False  # 예약 해제
            start_point = self.get_parameter("start_point").value  # 대기장소
            self._start_nav(start_point, "READY")  # 취소 후 대기장소 이동

    def _nav_goal_response_cb(self, future):
        # goal 수락/거부 콜백
        goal_handle = future.result()  # goal 수락 여부 확인
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected")  # 거부
            self._state = "READY"  # 상태 복귀
            return
        # goal 핸들 저장(취소 가능하도록)
        self._active_goal = goal_handle  # 핸들 저장
        result_future = goal_handle.get_result_async()  # 결과 대기
        result_future.add_done_callback(self._nav_result_cb)  # 결과 콜백

    def _nav_result_cb(self, future):
        # 이동 결과 콜백
        res = future.result()  # 이동 결과 수신
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"Navigation failed: {res.result.error_code}")  # 실패
            self._state = "READY"  # 상태 복귀
            return

        # 성공 시 도착 토픽 발행 + 다음 상태 전이
        arrived = self._target_point or "unknown"  # 도착 지점 이름
        arrived_id = self._point_to_id(arrived)  # 포인트 → 숫자 ID
        self._arrived_pub.publish(Int32(data=arrived_id))  # 도착 토픽 발행
        self.get_logger().info(f"Arrived at {arrived} ({arrived_id})")  # 로그 출력
        # 도착 시 매니퓰레이터 시작 신호
        self._manip_start_pub.publish(Bool(data=True))
        self._state = self._post_nav_state or "READY"  # 상태 전이
        self._target_point = None  # 목표 초기화
        self._post_nav_state = None  # 다음 상태 초기화

    def _build_goal_pose(self, point_name):
        # 포인트 이름을 Pose로 변환
        goal_file = self.get_parameter("goal_file").value  # 파라미터 경로
        if not goal_file:
            goal_file = os.path.join(os.path.dirname(__file__), "goal_points.json")
            if not os.path.exists(goal_file):
                try:
                    from ament_index_python.packages import get_package_share_directory

                    goal_file = os.path.join(
                        get_package_share_directory("nav2_waypoint"),
                        "goal_points.json",
                    )
                except Exception:
                    pass
        with open(goal_file, "r", encoding="utf-8") as handle:
            config = json.load(handle)  # JSON 로드
        points = config.get("points", {})  # 포인트 목록
        if point_name not in points:
            raise RuntimeError(f"Invalid point name: {point_name}")  # 예외
        goal = points[point_name]  # 포인트 좌표
        frame_id = config.get("frame_id", "map")  # 좌표계

        pose = PoseStamped()  # Pose 메시지 생성
        pose.header.frame_id = frame_id  # frame 지정
        pose.header.stamp = self.get_clock().now().to_msg()  # 시간 스탬프
        pose.pose.position.x = float(goal["x"])  # x 좌표
        pose.pose.position.y = float(goal["y"])  # y 좌표
        pose.pose.orientation.z = float(goal["yaw_z"])  # yaw z
        pose.pose.orientation.w = float(goal["yaw_w"])  # yaw w
        return pose  # Pose 반환

    def _point_to_id(self, point_name):
        # 포인트 이름 → 숫자 ID
        mapping = {
            "point0": 0,
            "point1": 1,
            "point2": 2,
            "point3": 3,
            "point4": 4,
            "point1-1": 11,
            "point1-2": 12,
            "point4-1": 41,
            "point4-2": 42,
        }
        return mapping.get(point_name, -1)


def main():
    rclpy.init()  # rclpy 초기화
    node = ScenarioManager()  # 노드 생성
    try:
        rclpy.spin(node)  # 콜백 처리 루프
    finally:
        node.destroy_node()  # 노드 정리
        if rclpy.ok():
            rclpy.shutdown()  # rclpy 종료


if __name__ == "__main__":
    main()  # 엔트리 포인트
