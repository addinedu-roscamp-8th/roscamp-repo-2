#!/usr/bin/env python3
import os

os.environ.setdefault(
    "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{name}]: {message}"
)

import rclpy
from rclpy.executors import MultiThreadedExecutor

from keyboard_trigger import KeyboardTrigger
from gui_trigger import GuiTrigger
from Nav2_classes import GoToGoal
from scenario_manager import ScenarioManager

'''
GUI 노드 : 업무 1할당(1), point1로 이동
    도착하면 도착토픽
    GUI 노드 : 부품 상차 완료(SPACE) → point2 이동
    도착하면 도착토픽
    매니퓰레이터1 노드 : 하차 완료(Q) → point1 이동

GUI 노드 : 업무 2할당(3), point3로 이동
    도착하면 도착토픽
    매니퓰레이터2 노드 : 모듈 상차 완료(W) → point4 이동
    도착하면 도착토픽
    GUI 노드 : 하차 완료(U) → point3 이동

GUI 노드 : 업무 3할당(4), point4로 이동
    도착하면 도착토픽
    GUI 노드 : 모듈 상차 완료(SPACE) → point1 이동
    도착하면 도착토픽
    GUI 노드 : 하차 완료(U) → point4 이동

GUI 노드 : 대기 장소로 이동(0)
GUI 노드 : 할당 업무 변경(1/3/4)

0 : 대기 장소로 이동
1/3/4 : 1/3/4로 이동, 할당업무 부품입고/모듈입고/모듈출고
SPACE : GUI 상차 완료 
U : GUI 하차 완료 
매니퓰레이터 입력
W : 매니퓰레이터 상차 완료 (업무2에서 사용)
Q : 매니퓰레이터 하차 완료 (업무1에서 사용)

코드 주석달기
정리하기
패키지화하기
'''

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    gui_node = GuiTrigger()
    nav_node = GoToGoal()
    scenario_node = ScenarioManager()

    executor.add_node(gui_node)
    executor.add_node(nav_node)
    executor.add_node(scenario_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        gui_node.destroy_node()
        nav_node.destroy_node()
        scenario_node.destroy_node()
        rclpy.shutdown()

 
if __name__ == "__main__":
    main()
 
 
