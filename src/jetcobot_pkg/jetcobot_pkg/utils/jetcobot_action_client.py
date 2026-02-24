#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple, Type, Any

from rclpy.node import Node
from rclpy.action import ActionClient


# =========================
# ✅ 공통 결과/피드백 상태 저장용
# =========================
@dataclass
class _ActionState:
    busy: bool = False
    goal_handle: Any = None

    last_feedback_progress: float = 0.0
    last_feedback_state: str = ""

    done_ready: bool = False
    done_success: bool = False
    done_message: str = ""


# =========================
# ✅ 공통 ActionClient 베이스
# - feedback/result 콜백, busy/done 상태관리, 공통 API 제공
# =========================
class _CommonActionClient:
    """
    ✅ 공통 ActionClient
    - send_goal_msg(goal_msg)만 호출하면 동일한 콜백/상태 로직으로 처리
    - action_done()/is_busy()/last_feedback() API 공통
    """

    def __init__(
        self,
        node: Node,
        action_type: Type,
        action_name: str,
        wait_server_timeout_sec: float = 1.0,
        log_prefix: str = "",
    ):
        self._node = node
        self._action_name = str(action_name)
        self._wait_server_timeout_sec = float(wait_server_timeout_sec)
        self._log_prefix = str(log_prefix) if log_prefix else self._action_name

        self._client = ActionClient(node, action_type, action_name)
        self._st = _ActionState()

    # -------------------------
    # ✅ 공통 Public API
    # -------------------------
    def is_busy(self) -> bool:
        return bool(self._st.busy)

    def last_feedback(self) -> Tuple[float, str]:
        return float(self._st.last_feedback_progress), str(self._st.last_feedback_state)

    def action_done(self) -> Optional[Tuple[bool, str]]:
        """
        ✅ 완료 결과를 '한 번만' 꺼내감
        return:
          - None: 아직 완료 아님
          - (success, message): 완료됨
        """
        if not self._st.done_ready:
            return None

        out = (bool(self._st.done_success), str(self._st.done_message))

        # consume
        self._st.done_ready = False
        self._st.done_success = False
        self._st.done_message = ""

        return out

    # -------------------------
    # ✅ 핵심: 공통 send (Goal msg를 받아 전송)
    # -------------------------
    def send_goal_msg(self, goal_msg) -> bool:
        if self._st.busy:
            self._node.get_logger().warn(f"[{self._log_prefix}] send_goal ignored: already busy")
            return False

        if not self._client.wait_for_server(timeout_sec=self._wait_server_timeout_sec):
            self._node.get_logger().error(f"[{self._log_prefix}] action server not available: {self._action_name}")
            return False

        # 상태 초기화
        self._st.busy = True
        self._st.goal_handle = None
        self._st.done_ready = False
        self._st.done_success = False
        self._st.done_message = ""
        self._st.last_feedback_progress = 0.0
        self._st.last_feedback_state = ""

        self._node.get_logger().info(f"[{self._log_prefix}] send_goal -> Goal Sent!")

        send_future = self._client.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)
        return True

    # -------------------------
    # ✅ 공통 Internal callbacks
    # (네가 쓰던 feedback/result/done 처리 로직 그대로)
    # -------------------------
    def _feedback_cb(self, fb_msg):
        fb = fb_msg.feedback
        self._st.last_feedback_progress = float(getattr(fb, "progress", 0.0))
        self._st.last_feedback_state = str(getattr(fb, "state", ""))

        self._node.get_logger().info(
            f"[{self._log_prefix} FB] {self._st.last_feedback_progress:.1f}% {self._st.last_feedback_state}"
        )

    def _goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self._finish(False, f"goal_response exception: {e}")
            return

        if not goal_handle.accepted:
            self._finish(False, "Goal rejected")
            return

        self._node.get_logger().info(f"[{self._log_prefix}] Goal accepted ✅")
        self._st.goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as e:
            self._finish(False, f"result exception: {e}")
            return

        success = bool(getattr(result, "success", False))
        message = str(getattr(result, "message", ""))

        self._finish(success, message)

    def _finish(self, success: bool, message: str):
        self._st.done_ready = True
        self._st.done_success = bool(success)
        self._st.done_message = str(message)

        self._st.busy = False
        self._st.goal_handle = None

        self._node.get_logger().info(
            f"[{self._log_prefix} DONE] success={self._st.done_success} msg={self._st.done_message}"
        )


# =========================
# ✅ 개별 Action Client 설정(실제로 import하는 부분)
# =========================

from smartfactory_interfaces.action import Pick, MoveToPose, Place 

class PickClient(_CommonActionClient):
    """
    Pick action goal:
      - pick_coords: length 6
    """

    def __init__(self, node: Node, action_name: str = "/pick", wait_server_timeout_sec: float = 1.0):
        super().__init__(node=node, action_type=Pick, action_name=action_name,
                         wait_server_timeout_sec=wait_server_timeout_sec, log_prefix="PICK")

    def send_goal(self, pick_coords: List[float], safe_pick: bool) -> bool:
        if pick_coords is None or len(pick_coords) != 6:
            self._node.get_logger().error("[PICK] send_goal failed: pick_coords must be length 6")
            return False
        if safe_pick is None:
            safe_pick = False

        goal = Pick.Goal()
        goal.pick_coords = [float(x) for x in pick_coords]
        goal.safe = bool(safe_pick)
        return self.send_goal_msg(goal)


class MoveToPoseClient(_CommonActionClient):
    """
    MoveToPose action goal:
      - pose: length 6
      - use_angles: False=coords, True=angles   (예시)
    """

    MODE_COORDS = False
    MODE_ANGLES = True

    def __init__(self, node: Node, action_name: str = "/move_to_pose", wait_server_timeout_sec: float = 1.0):
        super().__init__(node=node, action_type=MoveToPose, action_name=action_name,
                         wait_server_timeout_sec=wait_server_timeout_sec, log_prefix="MOVE")

    def send_goal_coords(self, coords6: List[float]) -> bool:
        if coords6 is None or len(coords6) != 6:
            self._node.get_logger().error("[MOVE] send_goal_coords failed: coords must be length 6")
            return False

        goal = MoveToPose.Goal()
        goal.use_angles = bool(self.MODE_COORDS)          # ✅ action 필드명에 맞춰 수정
        goal.pose = [float(x) for x in coords6]  # ✅ action 필드명에 맞춰 수정
        return self.send_goal_msg(goal)

    def send_goal_angles(self, angles6: List[float]) -> bool:
        if angles6 is None or len(angles6) != 6:
            self._node.get_logger().error("[MOVE] send_goal_angles failed: angles must be length 6")
            return False

        goal = MoveToPose.Goal()
        goal.use_angles = bool(self.MODE_ANGLES)          # ✅ action 필드명에 맞춰 수정
        goal.pose = [float(x) for x in angles6]  # ✅ action 필드명에 맞춰 수정
        return self.send_goal_msg(goal)


class PlaceClient(_CommonActionClient):
    """
    Place action goal:
      - place_coords: length 6
    """

    def __init__(self, node: Node, action_name: str = "/place", wait_server_timeout_sec: float = 1.0):
        super().__init__(node=node, action_type=Place, action_name=action_name,
                         wait_server_timeout_sec=wait_server_timeout_sec, log_prefix="PLACE")

    def send_goal(self, place_coords: List[float], safe_pick: bool) -> bool:
        if place_coords is None or len(place_coords) != 6:
            self._node.get_logger().error("[PLACE] send_goal failed: place_coords must be length 6")
            return False
        if safe_pick is None:
            safe_pick = False

        goal = Place.Goal()
        goal.place_coords = [float(x) for x in place_coords]
        goal.safe = bool(safe_pick)
        return self.send_goal_msg(goal)
