## manipulator_node.py

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from pymycobot.mycobot280 import MyCobot280

# from jetcobot_interfaces.srv import ExecutePnP as ExecutePnPService
# from jetcobot_interfaces.srv import GetCurrentCoords
# from jetcobot_interfaces.action import ExecutePnP as ExecutePnPAction

from smartfactory_interfaces.srv import ExecutePnP as ExecutePnPService
from smartfactory_interfaces.srv import GetCurrentCoords
from smartfactory_interfaces.action import ExecutePnP as ExecutePnPAction


class ManipulatorNode(Node):
    def __init__(self):
        super().__init__('manipulator_node')

        # ===============================
        # Robot connection
        # ===============================
        self.mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
        self.mc.thread_lock = True
        self.get_logger().info("🤖 MyCobot connected")

        # ===============================
        # Parameters
        # ===============================
        self.declare_parameter('speed_fast', 100)
        self.declare_parameter('speed_slow', 60)

        self.declare_parameter('gripper_open', 100)
        self.declare_parameter('gripper_close', 22)

        self.declare_parameter('home_pose', [0, 90, -90, -50, 0, 45])
        self.declare_parameter('via_pose',  [90, 0, -40, -45, 0, 45])

        # ===============================
        # Service: GetCurrentCoords
        # ===============================
        self.get_coords_srv = self.create_service(
            GetCurrentCoords,
            '/manipulator/get_current_coords',
            self.get_current_coords_cb
        )
        self.get_logger().info("📡 GetCurrentCoords service ready")

        # ===============================
        # Legacy Service: ExecutePnP
        # ===============================
        self.srv = self.create_service(
            ExecutePnPService,
            '/manipulator/execute_pnp',
            self.execute_pnp_srv_cb
        )
        self.get_logger().info("✅ ExecutePnP service ready")

        # ===============================
        # Action Server: ExecutePnP
        # ===============================
        self.action_server = ActionServer(
            self,
            ExecutePnPAction,
            '/manipulator/execute_pnp_action',
            execute_callback=self.execute_pnp_action_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )
        self.get_logger().info("🚀 ExecutePnP ActionServer ready")

    # ==================================================
    # GetCurrentCoords Service
    # ==================================================
    def get_current_coords_cb(self, req, res):
        coords = self.mc.get_coords()
        res.coords = list(coords) if coords is not None else []
        return res

    # ==================================================
    # Legacy Service
    # ==================================================
    def execute_pnp_srv_cb(self, req, res):
        try:
            self.execute_pick_and_place(
                list(req.pick_coords),
                list(req.place_coords)
            )
            res.success = True
            res.message = "PnP executed"
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    # ==================================================
    # Action callbacks
    # ==================================================
    def goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().warn("🛑 ExecutePnP canceled")
        return CancelResponse.ACCEPT

    def execute_pnp_action_cb(self, goal_handle):
        feedback = ExecutePnPAction.Feedback()
        result = ExecutePnPAction.Result()

        pick = list(goal_handle.request.pick_coords)
        place = list(goal_handle.request.place_coords)

        try:
            self.execute_pick_and_place(pick, place)
        except Exception as e:
            goal_handle.abort()
            result.success = False
            result.message = str(e)
            return result

        goal_handle.succeed()
        result.success = True
        result.message = "PnP done"
        return result

    # ==================================================
    # Core motion primitive
    # ==================================================
    def execute_pick_and_place(self, pick, place):
        v_fast = int(self.get_parameter('speed_fast').value)
        v_slow = int(self.get_parameter('speed_slow').value)

        grip_open = int(self.get_parameter('gripper_open').value)
        grip_close = int(self.get_parameter('gripper_close').value)

        home_pose = list(self.get_parameter('home_pose').value)
        via_pose  = list(self.get_parameter('via_pose').value)

        # open
        self.mc.set_gripper_value(grip_open, 50)
        time.sleep(0.5)

        # pick
        self.mc.send_coords(pick, v_fast, 0)
        time.sleep(1.0)

        self.mc.set_gripper_value(grip_close, 50)
        time.sleep(0.5)

        # via
        self.mc.send_angles(via_pose, v_fast)
        time.sleep(1.0)

        # place
        self.mc.send_coords(place, v_fast, 0)
        time.sleep(1.0)

        self.mc.set_gripper_value(grip_open, 50)
        time.sleep(0.5)

        # home
        self.mc.send_angles(home_pose, v_fast)
        time.sleep(1.0)

    # ==================================================
    def destroy_node(self):
        # try:
        #     self.mc.release_all_servos()
        # except Exception:
        #     pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
