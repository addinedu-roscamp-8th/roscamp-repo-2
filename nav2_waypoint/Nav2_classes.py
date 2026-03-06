#!/usr/bin/env python3
import json
import os

os.environ.setdefault(
    "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{name}]: {message}"
)

from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


STALE_SEC = 2.0


class GoToGoal(Node):
    def __init__(self):
        super().__init__("go_to_goal")

        # ---- Action ----
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._goal_future = None
        self._result_future = None
        self._goal_msg = NavigateToPose.Goal()
        self._nav_goal_handle = None
        self._busy = False
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            "go_to_goal",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )

        # ---- State ----
        self.done = False
        self.succeeded = False
        self._last_pose = None
        self._last_plan_len = None
        self._last_plan_time = None
        self._last_cmd_vel = None
        self._last_cmd_vel_time = None
        self._last_stop_reason = None

        self._obstacles = {
            "global": {"count": None, "time": None},
            "local": {"count": None, "time": None},
        }

        self._declare_params()
        qos = self._default_qos()

        # ---- Subscribers ----
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._pose_cb, qos
        )
        self.create_subscription(Path, "/plan", self._plan_cb, qos)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, qos)
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/obstacle_layer",
            lambda m: self._obstacle_cb(m, "global"),
            qos,
        )
        self.create_subscription(
            OccupancyGrid,
            "/local_costmap/obstacle_layer",
            lambda m: self._obstacle_cb(m, "local"),
            qos,
        )

        self.create_timer(1.0, self._report_stop_reason)

    # ======================
    # Setup helpers
    # ======================
    def _default_qos(self):
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        return qos

    def _declare_params(self):
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw_z", 0.0)
        self.declare_parameter("yaw_w", 1.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("point_name", "")
        self.declare_parameter("goal_file", "")

    # ======================
    # Callbacks
    # ======================
    def _pose_cb(self, msg):
        self._last_pose = msg

    def _plan_cb(self, msg):
        self._last_plan_len = len(msg.poses)
        self._last_plan_time = self.now()

    def _cmd_vel_cb(self, msg):
        self._last_cmd_vel = msg
        self._last_cmd_vel_time = self.now()

    def _obstacle_cb(self, msg, scope):
        self._obstacles[scope]["count"] = sum(v > 0 for v in msg.data)
        self._obstacles[scope]["time"] = self.now()

    # ======================
    # Action logic
    # ======================
    def send(self):
        self.get_logger().info("Waiting for navigate_to_pose action...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("navigate_to_pose not available")
            self.done = True
            return

        self.get_logger().info("Sending goal...")
        self._goal_msg.pose = self._build_goal_pose()
        future = self._client.send_goal_async(self._goal_msg)
        future.add_done_callback(self._goal_response_cb)

    def _goal_cb(self, goal_request):
        if self._busy and self._nav_goal_handle is not None:
            self.get_logger().warn("Preempting current goal for new request")
            self._nav_goal_handle.cancel_goal_async()
            return GoalResponse.ACCEPT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        self._busy = True
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose not available")
            goal_handle.abort()
            result = NavigateToPose.Result()
            result.error_code = 107
            self._busy = False
            return result

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_handle.request.pose
        nav_future = self._client.send_goal_async(nav_goal)
        nav_goal_handle = await nav_future
        if not nav_goal_handle or not nav_goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected")
            goal_handle.abort()
            result = NavigateToPose.Result()
            result.error_code = 105
            self._busy = False
            return result

        self._nav_goal_handle = nav_goal_handle
        nav_result_future = nav_goal_handle.get_result_async()
        nav_result = await nav_result_future
        self._nav_goal_handle = None

        if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()

        self._busy = False
        return nav_result.result

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self.done = True
            return

        self.get_logger().info("Goal accepted")
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        res = future.result()
        self.succeeded = res.status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(
            f"Goal finished: {self._format_result_reason(res.result.error_code)}"
        )
        self.done = True

    # ======================
    # Utils
    # ======================
    def now(self):
        return self.get_clock().now()

    def _recent(self, t):
        return t and (self.now() - t).nanoseconds / 1e9 < STALE_SEC

    def _build_goal_pose(self):
        point_name = self.get_parameter("point_name").value
        goal_file = self.get_parameter("goal_file").value

        if point_name:
            if not goal_file:
                goal_file = os.path.join(os.path.dirname(__file__), "goal_points.json")
                if not os.path.exists(goal_file):
                    try:
                        from ament_index_python.packages import (
                            get_package_share_directory,
                        )

                        goal_file = os.path.join(
                            get_package_share_directory("nav2_waypoint"),
                            "goal_points.json",
                        )
                    except Exception:
                        pass
            with open(goal_file, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            goal = cfg["points"][point_name]
            frame_id = cfg.get("frame_id", "map")
            x, y, yaw_z, yaw_w = (
                goal["x"],
                goal["y"],
                goal["yaw_z"],
                goal["yaw_w"],
            )
        else:
            x = self.get_parameter("x").value
            y = self.get_parameter("y").value
            yaw_z = self.get_parameter("yaw_z").value
            yaw_w = self.get_parameter("yaw_w").value
            frame_id = self.get_parameter("frame_id").value

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = yaw_z
        pose.pose.orientation.w = yaw_w
        return pose

    def _report_stop_reason(self):
        if not self._last_pose:
            return
        reason = self._infer_nav_reason()
        if reason == "이동 중":
            self._last_stop_reason = None
            return
        if reason == self._last_stop_reason:
            return
        self._last_stop_reason = reason
        self.get_logger().info(f"정지 사유: {reason}")

    def _infer_nav_reason(self):
        if not self._recent(self._last_plan_time):
            return "경로 없음"
        if not self._recent(self._last_cmd_vel_time):
            return "속도 명령 없음"

        cmd = self._last_cmd_vel
        if abs(cmd.linear.x) < 1e-3 and abs(cmd.angular.z) < 1e-3:
            return "정지 상태 (" + self._obstacle_status() + ")"

        return "이동 중"

    def _obstacle_status(self):
        for scope in ("global", "local"):
            obs = self._obstacles[scope]
            if self._recent(obs["time"]) and (obs["count"] or 0) > 0:
                return "장애물 감지"
        return "장애물 미감지"

    def _format_result_reason(self, code):
        table = {
            0: "성공",
            104: "진행 불가(정지)",
            105: "경로 추종 실패",
            106: "제어 명령 없음",
            107: "컨트롤러 타임아웃",
        }
        if code in table:
            return table[code]
        if 200 <= code < 300:
            return "경로 계산 실패"
        return f"오류 코드 {code}"
