#!/usr/bin/env python3
import sys
import math
import time
from enum import IntEnum
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64

import tf2_ros
from tf2_ros import TransformException

'''
패키지 등록한다음에 ros2 run 패키지명
'''


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quat(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, min_output=-1.0, max_output=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def update_gains(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd

    def reset(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def compute(self, error, current_time=None):
        if current_time is None:
            current_time = time.time()

        if self.last_time is None:
            self.last_time = current_time
            self.previous_error = error
            return 0.0

        dt = current_time - self.last_time
        if dt <= 0.0:
            return 0.0

        p = self.kp * error

        self.integral += error * dt
        max_integral = abs(self.max_output) / (abs(self.ki) + 1e-6)
        self.integral = max(-max_integral, min(max_integral, self.integral))
        i = self.ki * self.integral

        d = self.kd * (error - self.previous_error) / dt

        out = p + i + d
        out = max(self.min_output, min(self.max_output, out))

        self.previous_error = error
        self.last_time = current_time
        return out


class Mode(IntEnum):
    TURN_TO_GOAL = 0
    GO_STRAIGHT = 1
    FINAL_ALIGN = 2


class GoalMover(Node):
    """
    - TF(map->base_link)로 현재 pose를 읽어서 /cmd_vel PID 제어
    - goal_callback(x, y, yaw_deg)로 목표를 직접 세팅
    """

    def __init__(self):
        super().__init__("goal_mover_amcl_pose")

        # ---- Parameters ----
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        # PID gains
        self.declare_parameter("linear_P", 0.5)
        self.declare_parameter("linear_I", 0.0)
        self.declare_parameter("linear_D", 0.0)
        self.declare_parameter("angular_P", 0.2)
        self.declare_parameter("angular_I", 0.0)
        self.declare_parameter("angular_D", 0.05)

        # Tolerances
        self.declare_parameter("angle_tolerance_deg", 12.0)
        self.declare_parameter("pos_tolerance", 0.03)
        self.declare_parameter("final_yaw_tolerance_deg", 5.0)

        self.declare_parameter("enable_pid", 1.0)

        # Speed limits
        self.declare_parameter("max_linear_speed", 0.30)
        self.declare_parameter("max_angular_speed", 1.5)

        self.declare_parameter("min_linear_speed", 0.06)
        self.declare_parameter("min_angular_speed", 0.10)
        self.declare_parameter("min_speed_distance_threshold", 0.30)

        # TF lookup
        self.declare_parameter("tf_timeout_sec", 0.2)

        # Control rate
        self.declare_parameter("control_period_sec", 0.01)

        self._load_params()

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- PID ----
        self.linear_pid = PIDController(
            kp=self.linear_P, ki=self.linear_I, kd=self.linear_D,
            min_output=-self.max_linear_speed, max_output=self.max_linear_speed,
        )
        self.angular_pid = PIDController(
            kp=self.angular_P, ki=self.angular_I, kd=self.angular_D,
            min_output=-self.max_angular_speed, max_output=self.max_angular_speed,
        )

        # ---- State ----
        self.goal_msg: Optional[PoseStamped] = None
        self.mode: Mode = Mode.TURN_TO_GOAL

        # ---- ROS I/O ----
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.distance_error_pub = self.create_publisher(Float64, "/distance_error", 10)
        self.angle_error_pub = self.create_publisher(Float64, "/angle_error", 10)
        self.final_yaw_error_pub = self.create_publisher(Float64, "/final_yaw_error", 10)

        self.timer = self.create_timer(self.control_period_sec, self.control_loop)

    def _load_params(self):
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.linear_P = float(self.get_parameter("linear_P").value)
        self.linear_I = float(self.get_parameter("linear_I").value)
        self.linear_D = float(self.get_parameter("linear_D").value)

        self.angular_P = float(self.get_parameter("angular_P").value)
        self.angular_I = float(self.get_parameter("angular_I").value)
        self.angular_D = float(self.get_parameter("angular_D").value)

        self.angle_tolerance = math.radians(float(self.get_parameter("angle_tolerance_deg").value))
        self.final_yaw_tolerance = math.radians(float(self.get_parameter("final_yaw_tolerance_deg").value))
        self.pos_tolerance = float(self.get_parameter("pos_tolerance").value)

        self.enable_pid = float(self.get_parameter("enable_pid").value) > 0.5

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.min_linear_speed = float(self.get_parameter("min_linear_speed").value)
        self.min_angular_speed = float(self.get_parameter("min_angular_speed").value)
        self.min_speed_distance_threshold = float(self.get_parameter("min_speed_distance_threshold").value)

        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.control_period_sec = float(self.get_parameter("control_period_sec").value)

    def goal_callback(self, x: float, y: float, yaw_deg: float, frame_id: str = "map"):
        yaw = math.radians(yaw_deg)
        qx, qy, qz, qw = quat_from_yaw(yaw)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.goal_msg = msg
        self.mode = Mode.TURN_TO_GOAL
        self.linear_pid.reset()
        self.angular_pid.reset()

        self.get_logger().info(
            f"🚀 Auto goal set: ({x:.2f}, {y:.2f}, yaw={yaw_deg:.1f}deg) frame={frame_id}"
        )

    def _get_robot_pose_in_map(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed: {self.map_frame}->{self.base_frame}: {e}")
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        return x, y, yaw

    def _goal_xy_yaw_in_map(self) -> Optional[Tuple[float, float, float]]:
        if self.goal_msg is None:
            return None
        gx = self.goal_msg.pose.position.x
        gy = self.goal_msg.pose.position.y
        gq = self.goal_msg.pose.orientation
        gyaw = yaw_from_quat(gq.x, gq.y, gq.z, gq.w)
        return gx, gy, gyaw

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if self.goal_msg is None:
            return

        robot = self._get_robot_pose_in_map()
        goal = self._goal_xy_yaw_in_map()
        if robot is None or goal is None:
            return

        rx, ry, ryaw = robot
        gx, gy, gyaw = goal

        dx = gx - rx
        dy = gy - ry
        dist = math.hypot(dx, dy)
        heading_to_goal = math.atan2(dy, dx)
        heading_err = normalize_angle(heading_to_goal - ryaw)
        final_yaw_err = normalize_angle(gyaw - ryaw)

        dm = Float64(); dm.data = dist
        am = Float64(); am.data = heading_err
        fm = Float64(); fm.data = final_yaw_err
        self.distance_error_pub.publish(dm)
        self.angle_error_pub.publish(am)
        self.final_yaw_error_pub.publish(fm)

        if dist < self.pos_tolerance:
            self.mode = Mode.FINAL_ALIGN

        cmd = Twist()

        if self.mode == Mode.TURN_TO_GOAL:
            if abs(heading_err) <= self.angle_tolerance:
                self.mode = Mode.GO_STRAIGHT
                self.angular_pid.reset()
                cmd.angular.z = 0.0
                cmd.linear.x = 0.0
            else:
                w = self.angular_pid.compute(heading_err) if self.enable_pid else (self.angular_P * heading_err)
                w = max(-self.max_angular_speed, min(self.max_angular_speed, w))
                if abs(w) < self.min_angular_speed:
                    w = math.copysign(self.min_angular_speed, w)
                cmd.angular.z = w
                cmd.linear.x = 0.0

        elif self.mode == Mode.GO_STRAIGHT:
            if abs(heading_err) > self.angle_tolerance:
                self.mode = Mode.TURN_TO_GOAL
                self.linear_pid.reset()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                v = self.linear_pid.compute(dist) if self.enable_pid else (self.linear_P * dist)
                v = max(-self.max_linear_speed, min(self.max_linear_speed, v))
                if dist > self.min_speed_distance_threshold:
                    if abs(v) < self.min_linear_speed:
                        v = math.copysign(self.min_linear_speed, v)
                cmd.linear.x = v
                cmd.angular.z = 0.0

        elif self.mode == Mode.FINAL_ALIGN:
            if abs(final_yaw_err) <= self.final_yaw_tolerance:
                self.get_logger().info("✅ reached goal pose. stop.")
                self.goal_msg = None
                self.mode = Mode.TURN_TO_GOAL
                self.linear_pid.reset()
                self.angular_pid.reset()
                self._publish_stop()
                return
            else:
                w = self.angular_pid.compute(final_yaw_err) if self.enable_pid else (self.angular_P * final_yaw_err)
                w = max(-self.max_angular_speed, min(self.max_angular_speed, w))
                if abs(w) < self.min_angular_speed:
                    w = math.copysign(self.min_angular_speed, w)
                cmd.angular.z = w
                cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)


def _parse_goal_from_argv(argv):
    """
    Usage:
      ros2 run actions goal_mover X Y YAW [--yaw-rad] [--frame map]
    """
    if len(argv) < 4:
        print("Usage: goal_mover X Y YAW [--yaw-rad] [--frame map]")
        raise SystemExit(2)

    x = float(argv[1])
    y = float(argv[2])
    yaw_in = float(argv[3])

    # 기본은 degree
    yaw_deg = yaw_in

    if "--yaw-rad" in argv:
        yaw_deg = math.degrees(yaw_in)

    frame = "map"
    if "--frame" in argv:
        i = argv.index("--frame")
        if i + 1 < len(argv):
            frame = argv[i + 1]

    return x, y, yaw_deg, frame


def main(args=None):
    # ✅ 1) CLI에서 goal을 먼저 파싱
    x, y, yaw_deg, frame = _parse_goal_from_argv(sys.argv)

    # ✅ 2) ROS init + node 생성
    rclpy.init(args=args)
    node = GoalMover()

    # ✅ 3) "Jupyter처럼" 실행 즉시 goal 설정
    node.goal_callback(x, y, yaw_deg, frame_id=frame)

    # ✅ 4) spin
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()



# 실행 예시
# ros2 run your_pkg goal_mover 0.88 0.12 0