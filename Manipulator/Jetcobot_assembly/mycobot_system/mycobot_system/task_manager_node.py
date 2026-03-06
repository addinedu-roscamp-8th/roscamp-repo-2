#!/usr/bin/env python3
import asyncio
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from enum import Enum, auto
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Bool, Int32

# from jetcobot_interfaces.action import ObserveMarker, ExecutePnP, ExecuteTask
from smartfactory_interfaces.action import ObserveMarker, ExecutePnP, ExecuteTask

# ==================================================
# Utility
# ==================================================
def coords_to_T(coords_mm_deg):
    x, y, z, rx, ry, rz = coords_mm_deg
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
    T[:3, 3] = np.array([x, y, z], dtype=float) * 1e-3
    return T


def T_to_coords(T):
    t_mm = T[:3, 3] * 1e3
    rpy = R.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
    return [
        float(t_mm[0]), float(t_mm[1]), float(t_mm[2]),
        float(rpy[0]), float(rpy[1]), float(rpy[2]),
    ]


# ==================================================
# FSM State
# ==================================================
class TaskState(Enum):
    IDLE = auto()
    REQUEST_VISION = auto()
    WAIT_VISION = auto()
    REQUEST_PNP = auto()
    WAIT_PNP = auto()
    UPDATE_STACK = auto()
    DONE = auto()
    ERROR = auto()


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')

        # =====================================
        # Publishers
        # =====================================
        self.assembly_pub = self.create_publisher(Bool, '/assembly_start', 10)
        self.task_done_pub = self.create_publisher(Bool, '/jetcobot/pick_and_place/done', 10)
        self._running = False
        self.assembly_timer = self.create_timer(0.1, self.publish_assembly_state)

        # =====================================
        # Action clients
        # =====================================
        self.vision_client = ActionClient(self, ObserveMarker, '/vision/observe_marker')
        self.pnp_client = ActionClient(self, ExecutePnP, '/manipulator/execute_pnp_action')
        self.self_task_client = ActionClient(self, ExecuteTask, '/task/execute')

        # =====================================
        # FSM variables
        # =====================================
        self.state = TaskState.IDLE
        self.marker_ids = []
        self.current_index = 0
        self.stack_level = 0
        self.pick_base_target = None
        self._busy_goal = False

        # Place/stack strategy
        # self.assembly_pose = [60, 180, 180, -180, 0, -45]
        # self.assembly_pose = [60, 180, 115, -180, 0, -45]
        self.assembly_pose = [20, 180, 115, -180, 0, -45]

        self.stack_dz_mm = 27.0
        self.stack_dz_sign = -1.0

        # Observe parameters
        self.obs_samples = 12
        self.obs_timeout = 0.5
        self.obs_outlier = 0.03

        # =====================================
        # Action Server
        # =====================================
        self.action_server = ActionServer(
            self,
            ExecuteTask,
            '/task/execute',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        # =====================================
        # GUI Topic Subscription (Int32)
        # =====================================
        self.gui_req_sub = self.create_subscription(
            Int32,
            '/jetcobot/assembly/stack/request',
            self.gui_request_cb,
            10
        )

        self.get_logger().info("🧠 TaskManager ready")
        self.get_logger().info("🧩 GUI trigger topic: /jetcobot/assembly/stack/request (std_msgs/Int32, 1=run)")

    # ==================================================
    # GUI Topic -> ExecuteTask Trigger
    # ==================================================
    def gui_request_cb(self, msg: Int32):
        module_id = int(msg.data)

        if module_id != 1:
            self.get_logger().info(f"ℹ️ module_id={module_id} ignored")
            return

        if self._busy_goal:
            self.get_logger().warn("⚠️ Task busy → ignore GUI request")
            return

        if not self.self_task_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("⚠️ /task/execute not ready")
            return

        goal_msg = ExecuteTask.Goal()
        goal_msg.marker_ids = [1, 2, 3]
        goal_msg.reset_stack = True

        self.get_logger().info("📥 GUI module_id=1 → ExecuteTask [1,2,3], reset_stack=True")

        fut = self.self_task_client.send_goal_async(goal_msg)
        fut.add_done_callback(self._on_gui_goal_response)

    def _on_gui_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"❌ Goal send failed: {e}")
            return

        if not goal_handle.accepted:
            self.get_logger().warn("⚠️ Goal rejected")
            return

        self.get_logger().info("✅ Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_gui_result)

    def _on_gui_result(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"🏁 Task finished: success={result.success}, message='{result.message}'")
        except Exception as e:
            self.get_logger().error(f"❌ Result error: {e}")

    # ==================================================
    # Publisher
    # ==================================================
    def publish_assembly_state(self):
        msg = Bool()
        msg.data = bool(self._running)
        self.assembly_pub.publish(msg)

    # ==================================================
    # Action Callbacks
    # ==================================================
    def goal_cb(self, goal_request):
        if self._busy_goal:
            return GoalResponse.REJECT
        if len(goal_request.marker_ids) == 0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    # ==================================================
    # ExecuteTask Action
    # ==================================================
    async def execute_cb(self, goal_handle):
        goal = goal_handle.request
        result = ExecuteTask.Result()
        feedback = ExecuteTask.Feedback()

        self._busy_goal = True
        self._running = True

        self.marker_ids = list(goal.marker_ids)
        self.current_index = 0
        self.stack_level = 0 if bool(goal.reset_stack) else self.stack_level

        try:
            while self.current_index < len(self.marker_ids):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Canceled"
                    self.reset_fsm()
                    return result

                # feedback (optional)
                feedback.state = "VISION"
                feedback.current_index = int(self.current_index)
                feedback.total = int(len(self.marker_ids))
                goal_handle.publish_feedback(feedback)

                marker_id = int(self.marker_ids[self.current_index])
                base_target = await self.call_vision(marker_id)
                if base_target is None:
                    goal_handle.abort()
                    result.success = False
                    result.message = "Vision failed"
                    self.reset_fsm()
                    return result

                feedback.state = "PNP"
                goal_handle.publish_feedback(feedback)

                pick = self.make_pregrasp(base_target)
                place = self.make_place_pose()

                ok = await self.call_pnp(pick, place)
                if not ok:
                    goal_handle.abort()
                    result.success = False
                    result.message = "PnP failed"
                    self.reset_fsm()
                    return result

                self.stack_level += 1
                self.current_index += 1

            goal_handle.succeed()
            result.success = True
            result.message = "Task completed"
            self.task_done_pub.publish(Bool(data=True))
            self.reset_fsm()
            return result

        except Exception as e:
            self.get_logger().error(f"❌ Exception: {e}")
            goal_handle.abort()
            result.success = False
            result.message = f"Exception: {e}"
            self.reset_fsm()
            return result

    # ==================================================
    # Vision / PnP helpers
    # ==================================================
    async def call_vision(self, marker_id: int):
        if not await self.wait_action_server(self.vision_client, 3.0):
            self.get_logger().error("❌ Vision action server timeout")
            return None

        goal = ObserveMarker.Goal()
        goal.marker_id = int(marker_id)
        goal.n_samples = int(self.obs_samples)
        goal.timeout_sec = float(self.obs_timeout)
        goal.outlier_thresh = float(self.obs_outlier)

        goal_handle = await self.vision_client.send_goal_async(goal)
        if not goal_handle.accepted:
            self.get_logger().error("❌ Vision goal rejected")
            return None

        res = await goal_handle.get_result_async()
        if not res.result.ok:
            self.get_logger().error("❌ Vision returned ok=false")
            return None

        return list(res.result.base_target_coords)

    async def call_pnp(self, pick_coords, place_coords) -> bool:
        if not await self.wait_action_server(self.pnp_client, 3.0):
            self.get_logger().error("❌ PnP action server timeout")
            return False

        goal = ExecutePnP.Goal()
        goal.pick_coords = list(pick_coords)
        goal.place_coords = list(place_coords)

        goal_handle = await self.pnp_client.send_goal_async(goal)
        if not goal_handle.accepted:
            self.get_logger().error("❌ PnP goal rejected")
            return False

        res = await goal_handle.get_result_async()
        if not res.result.success:
            self.get_logger().error("❌ PnP returned success=false")
            return False

        return True

    async def wait_action_server(self, client: ActionClient, timeout: float) -> bool:
        t0 = self.get_clock().now().nanoseconds * 1e-9
        while not client.wait_for_server(timeout_sec=0.1):
            if (self.get_clock().now().nanoseconds * 1e-9 - t0) > timeout:
                return False
            await asyncio.sleep(0.0)
        return True

    # ==================================================
    # Strategy (✅ 여기 빠져서 에러 났던 부분)
    # ==================================================
    def make_pregrasp(self, base_target):
        # base_target: [x,y,z,rx,ry,rz] (mm, deg)
        x, y, z, rx, ry, rz = base_target

        T_base_target = np.eye(4)
        T_base_target[:3, :3] = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
        T_base_target[:3, 3] = np.array([x, y, z], dtype=float) * 1e-3

        # target -> pregrasp offset
        T_target_pregrasp = np.eye(4)
        T_target_pregrasp[:3, 3] = [0.0, 0.0, 0.110]  # 11cm 위
        T_target_pregrasp[:3, :3] = R.from_euler('x', 185, degrees=True).as_matrix()

        # ee yaw +45
        T_ee_z_45 = np.eye(4)
        T_ee_z_45[:3, :3] = R.from_euler('z', 45, degrees=True).as_matrix()

        T_base_pregrasp = T_base_target @ T_target_pregrasp @ T_ee_z_45
        return T_to_coords(T_base_pregrasp)

    def make_place_pose(self):
        T_base_assembly = coords_to_T(self.assembly_pose)

        dz_m = self.stack_level * self.stack_dz_mm * self.stack_dz_sign * 1e-3
        T_stack = np.eye(4)
        T_stack[:3, 3] = [0.0, 0.0, dz_m]

        T_base_place = T_base_assembly @ T_stack
        return T_to_coords(T_base_place)

    def reset_fsm(self):
        self._busy_goal = False
        self._running = False
        self.current_index = 0
        self.marker_ids = []
        self.pick_base_target = None
        self.state = TaskState.IDLE


# ==================================================
# Main
# ==================================================
def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
