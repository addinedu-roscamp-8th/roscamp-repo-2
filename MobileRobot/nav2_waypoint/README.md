# nav2_waypoint

키보드 입력을 GUI/매니퓰레이터 신호로 분리하고, 시나리오 상태머신으로 Nav2 목표를 보내는 ROS2 Python

## 개발 환경
- OS: Ubuntu 22.04 LTS
- ROS 2: Jazzy
- 언어: Python (rclpy)
- 네비게이션: Nav2

## 구성 파일
nav2_waypoint/
- package.xml
- setup.py
- setup.cfg
- README.md
- goal_points.json : 목표 포인트 좌표
- run_scenario.py : 전체 노드를 한 번에 실행하는 런처
- scenario_manager.py : 시나리오 FSM (GUI/매니퓰레이터 입력 → 목표 전환)
- Nav2_classes.py : `go_to_goal` 액션 서버/브리지
- keyboard_trigger.py : 키보드 입력 → `/keyboard_raw`
- gui_trigger.py : `/keyboard_raw` → `/gui_cmd` 필터
- manip_trigger.py : (테스트용) 키보드로 완료 신호 흉내

## 패키지 파일별 설명
- `package.xml` : ROS2 패키지 메타데이터(이름/버전/의존성)
- `setup.py` : 파이썬 패키지 설치 및 `ros2 run` 엔트리 등록
- `setup.cfg` : 콘솔 스크립트 설치 경로 설정
- `resource/nav2_waypoint` : ament 인덱스 등록 파일
- `README.md` : 패키지 설명서
- `goal_points.json` : 목표 포인트 좌표 정의
- `run_scenario.py` : 전체 노드를 동시에 실행하는 런처
- `scenario_manager.py` : 시나리오 FSM, 입력 처리 및 목표 전환
- `Nav2_classes.py` : `go_to_goal` 액션 서버/브리지 및 상태 로그
- `keyboard_trigger.py` : 키보드 입력 → `/keyboard_raw`
- `gui_trigger.py` : GUI 입력 필터 → `/move_role`, `/상차완료`, `/하차완료`
- `manip_trigger.py` : (테스트용) 완료 신호 발행

## 주요 토픽/액션
### 토픽(Topic)
- `/keyboard_raw` (String): 키보드 원시 입력
- `/move_role` (String): 이동 역할(1/3/4/0)
- `/load_done` (Bool): 상차 완료 신호
- `/unload_done` (Bool): 하차 완료 신호
- `/pick_and_place/done` (Bool): 매니퓰레이터 완료 신호
- `/arrived_point` (Int32): 도착 포인트 ID 알림

### 액션(Action)
- `/go_to_goal` (NavigateToPose action): 시나리오 → 네비게이션 브리지

## 키 매핑
### 이동 역할(`/move_role`)
- `1` : 업무1 시작 → point1
- `3` : 업무2 시작 → point3
- `4` : 업무3 시작 → point4
- `0` : 대기 장소(point0)

### 상차 완료(`/load_done`)
- `True` : 상차 완료

### 하차 완료(`/unload_done`)
- `True` : 하차 완료

### 매니퓰레이터 완료(`/pick_and_place/done`)
- `True` : 완료 신호

> 입력의 유효성은 `scenario_manager`의 상태에 따라 결정됩니다.  
> 예: `WAIT_SPACE_FROM_3` 상태에서는 `W`만 유효합니다.

## 실행 방법
```bash
cd /home/pinky/pinky_pro
colcon build --packages-select nav2_waypoint --symlink-install
source /home/pinky/pinky_pro/install/setup.bash
ros2 run nav2_waypoint run_scenario
```

## 동작 흐름(요약)
- 키보드 입력 → `/keyboard_raw`
- GUI 필터 → `/move_role`, `/load_done`, `/unload_done`
- `scenario_manager`가 상태머신으로 목표 설정
- `go_to_goal`이 Nav2로 목표 전달
- 도착 시 `/arrived_point` 발행
