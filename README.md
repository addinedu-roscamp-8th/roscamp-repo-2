# Project Domain ID

| Name            | ID  |
|-----------------|-----|
| Control PC      | 30  |
| OpenManipulator | 31  |
| storage_cobot   | 32  |
| assembly_cobot  | 33  |
| pinkt1 (pinky_b44f) | 34  |
| pinkt2 (pinky_c0bd) | 35  |
| pinkt3 (pinky_1542) | 36  |


# gui - openmanipulator 단순연동 완료

## build 
* 빌드는 src directory 위치에서 colcon build 하시면 됩니다.

## cmd list
```
1. ros2 launch system_bridge smartfactory_bridge.launch.py # domain_bridge launch file
2. ros2 run gui_pkg main # gui run
3. ros2 launch gui_pkg gui.launch.py # gui launch
4. ros2 launch gui_pkg smartfactory_system.launch.py # gui & domain_bridge launch file
```

## gui_pkg 설명
1. main_window.py - 버튼, 레이블, UI 이벤트 처리 담당, 버튼 동작 코드 여기
2. gui_node.py - ROS2 통신 전담 노드 (rclpy Node) - ROS 통신은 전부 여기

# system_bridge
* smartfactory_bridge.yaml - domain id 연동 , 토픽 정의는 여기
* 이제 yaml 파일 추가하고 바로 빌드하시면 launch파일에 자동으로 등록됩니다.  
(CMakeLists.txt 에 파일명 명시하지 않아도 됩니다.)

## Domain Bridge YAML Naming Rules
```
상단에 정해진 domain ID로 yaml파일 작성해 주시기 바랍니다.

1. 파일명 : bridge_<from_robot>_to_<to_robot>.yaml
2. 파일내 최상단 주석 : # bridge_for_<from_robot>
3. name 필드 : name: <from_robot>_to_<to_robot>
4. Domain ID : from_domain: <from_robot_domain_id>  
               to_domain: <to_robot_domain_id>
```


# smartfactory_interfaces
1. msg directory - msg 정의, ** 파일이름, 반드시 대문자로 시작, CamelCase 사용

2. CMakeLists.txt - set에 파일 위치 기입 후 colcon build --packages-select smartfactory_interfaces 로 interface pkg 먼저 빌드 후 사용
