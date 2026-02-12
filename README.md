# gui - openmanipulator 단순연동 완료

### build 
* 빌드는 src directory 위치에서 colcon build 하시면 됩니다.

### cmd list
1. ros2 launch system_bridge smartfactory_bridge.launch.py # domain_bridge launch file
2. ros2 run state_manager task_planner # 미완성
3. ros2 run gui_pkg main # gui run 
4. ros2 launch gui_pkg gui.launch.py # gui launch 
5. ros2 launch gui_pkg smartfactory_system.launch.py # gui & domain_bridge launch file


### gui_pkg 설명
1. main_window.py - 버튼, 레이블, UI 이벤트 처리 담당, 버튼 동작 코드 여기
2. gui_node.py - ROS2 통신 전담 노드 (rclpy Node) - ROS 통신은 전부 여기

### system_bridge 설명
* 현재 from domain 30, to domain 31 입니다
1. smartfactory_bridge.yaml - domain id 연동 , 토픽 정의는 여기

### smartfactory_interfaces 설명
1. msg directory - msg 정의, ** 파일이름, 반드시 대문자로 시작, CamelCase 사용
2. CMakeLists.txt - set에 파일 위치 기입 후 colcon build --packages-select smartfactory_interfaces 로 interface pkg 먼저 빌드 후 사용