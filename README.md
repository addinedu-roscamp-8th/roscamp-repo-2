# Smart Factory ROS2 Project

## 🎥 Demo Video
[![Pick & Place Demo](https://img.youtube.com/vi/RbTi8bsz5DQ/maxresdefault.jpg)](https://youtu.be/RbTi8bsz5DQ?si=6ymwndZ8VQvB9C4A)

> Click the image to watch the full demo on YouTube.
[https://youtu.be/JyPYKrjtD1M](https://youtu.be/RbTi8bsz5DQ?si=6ymwndZ8VQvB9C4A)
> 

## Project Overview

![심화8기_Resonance_판넬제작](https://github.com/user-attachments/assets/5266e106-a396-4f37-827a-bdc054ff8e86)

본 프로젝트는 ROS2 기반 스마트팩토리 자동화 시스템으로 Control PC, Manipulator, Mobile Robot이 domain bridge를 통해 협업하도록 구성되어 있습니다.

> # 📊 **프로젝트 전체 요약 및 기술 설명은 아래 PPT를 참고해주세요.**
> 🔗 https://docs.google.com/presentation/d/168jklXeM5bWbdeiVo7vnc3Sdlypl-KNElhoRHj36eDc/edit?usp=drive_link
> <img width="972" height="546" alt="스크린샷 2026-03-10 084920" src="https://github.com/user-attachments/assets/9903538f-5962-4583-98df-b67dcea10280" />

## Project Domain ID

| Name            | ID  |
|-----------------|-----|
| Control PC      | 30  |
| OpenManipulator | 31  |
| storage_cobot   | 32  |
| assembly_cobot  | 33  |
| pinkt1 (pinky_b44f) | 34  |
| pinkt2 (pinky_c0bd) | 35  |
| pinkt3 (pinky_1542) | 36  |

## build 
* 빌드는 src directory 위치에서 colcon build 하시면 됩니다.

## cmd list
```
1. ros2 launch gui_pkg smartfactory_system.launch.py # gui & domain_bridge launch file, 전체 실행 런치파일 
2. ros2 launch gui_pkg gui.launch.py # gui launch
3. ros2 launch system_bridge smartfactory_bridge.launch.py # domain_bridge launch file
```

# Package description

## Manipulator
* JetCobot과 OpenManipulator를 제어하는 매니퓰레이터 패키지 (RSBP에서 실행)

## MobileRobot
* Pinky 모바일 로봇을 위한 패키지 (RSBP에서 실행)

## bom_web
* BOM을 변환하는 웹 애플리케이션

## gui_pkg
1. main_window.py - 버튼, 레이블, UI 이벤트 처리 담당, 버튼 동작 코드 여기
2. gui_node.py - ROS2 통신 전담 노드 (rclpy Node) - ROS 통신은 전부 여기

## smartfactory_interfaces
* PJT 전체 interface 모음

## system_bridge
* domain bridge pkg

## system_manager
* 로봇과 Control PC 간 토픽 기반 통신을 관리하고 작업 순서를 제어하는 시스템 관리 pkg






