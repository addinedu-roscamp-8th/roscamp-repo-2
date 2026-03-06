# ROS2 Manipulator Project

MyCobot 280 기반의 비전-매니퓰레이션-태스크 통합 ROS 2 워크스페이스입니다.  
`vision`, `manipulator`, `task_manager` 3개 노드가 연동되어 마커 인식 후 Pick-and-Place stack 작업을 수행합니다.

## 1. 구성

- `src/mycobot_system`: 실행 노드(vision/manipulator/task_manager/trigger), launch 파일
- `src/smartfactory_interfaces`: action/srv/msg 인터페이스 정의

주요 실행 노드:
- `manipulator_node`
- `vision_node`
- `task_manager_node`
- `pick_and_place_trigger_node` (옵션)

## 2. 요구 사항

- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy
- Python 패키지:
  - `pymycobot`
  - `numpy`
  - `scipy`
  - `opencv` (aruco 포함 빌드 필요)
- 하드웨어:
  - MyCobot 280 (기본 포트: `/dev/ttyJETCOBOT`)
  - 카메라 (기본 index: `0`)

## 3. 빌드

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 4. 실행
## 🎬 Demo Video

https://www.youtube.com/watch?v=Ma1Qjp9Qrbk&si=AKAygBHIJUH43ac_

[![Watch the demo](https://img.youtube.com/vi/영상ID/0.jpg)](https://www.youtube.com/watch?v=영상ID)

### A. 런치 파일로 3노드 동시 실행

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mycobot_system assembly_task.launch.py
```

### B. 수동 실행(런치 대체)

터미널 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run mycobot_system manipulator_node
```

터미널 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run mycobot_system vision_node
```

터미널 3:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run mycobot_system task_manager_node
```

## 5. Stack 작업 트리거

### 방법 1. 토픽 트리거 (GUI 연동 방식)

```bash
ros2 topic pub --once /jetcobot/assembly/stack/request std_msgs/msg/Int32 "{data: 1}"
```

### 방법 2. 액션 직접 호출

```bash
ros2 action send_goal /task/execute smartfactory_interfaces/action/ExecuteTask "{marker_ids: [1, 2, 3], reset_stack: true}"
```

## 6. 주요 인터페이스

- Action
  - `/task/execute` (`smartfactory_interfaces/action/ExecuteTask`)
  - `/vision/observe_marker` (`smartfactory_interfaces/action/ObserveMarker`)
  - `/manipulator/execute_pnp_action` (`smartfactory_interfaces/action/ExecutePnP`)
- Service
  - `/manipulator/get_current_coords` (`smartfactory_interfaces/srv/GetCurrentCoords`)
  - `/manipulator/execute_pnp` (`smartfactory_interfaces/srv/ExecutePnP`, legacy)
- Topic
  - `/jetcobot/assembly/stack/request` (`std_msgs/msg/Int32`)
  - `/assembly_start` (`std_msgs/msg/Bool`)
  - `/jetcobot/pick_and_place/done` (`std_msgs/msg/Bool`)
  - `/jetcobot/assembly/db_update` (`smartfactory_interfaces/msg/SectionResult`)

## 7. 기술 상세

### 7.1 vision_node

- 역할: ArUco 마커를 관측해 베이스 좌표계 목표 위치(`base_target_coords`)를 계산하고 `/vision/observe_marker` 액션으로 반환
- 인식 파이프라인:
  - `detectMarkers` -> `cornerSubPix`(옵션) -> `solvePnP`
  - `solvePnP`는 `IPPE_SQUARE` 우선, 실패 시 `ITERATIVE` fallback
- 샘플 처리/필터링:
  - 지정 시간 내 다중 샘플(`n_samples`) 수집
  - `tvec` median 기준 거리 outlier 제거(`outlier_thresh`)
  - 유효 샘플 평균으로 위치 계산, 자세는 quaternion averaging으로 계산
  - 참고: 이동평균(MA) 필터는 사용하지 않고, outlier rejection + 평균 방식 사용
- 좌표 변환:
  - `T_base_target = T_base_ee @ T_ee_cam @ T_cam_target`
  - `T_base_ee`: manipulator 서비스(`/manipulator/get_current_coords`)에서 수신
  - `T_ee_cam`: hand-eye 고정 행렬
- 위치 보정:
  - 기본 보정: `offset_x`, `offset_y`, `offset_z`
  - 처짐 보정: `z += sag_coeff * x`
  - 거리 기반 추가 보정: `x > dist_threshold`일 때 `extra_offset_x`, `extra_offset_z` 적용
- 부가 기능:
  - 계산된 `(x, y)`를 섹션(A/B/C)으로 분류
  - `/jetcobot/assembly/db_update`로 `SectionResult` 발행
- 실행 모델:
  - `ReentrantCallbackGroup` + `MultiThreadedExecutor`
  - 액션 콜백 내부에서 서비스 호출을 `await`로 처리해 블로킹 최소화

### 7.2 manipulator_node

- 역할: 실제 MyCobot 동작 수행(Pick-and-Place)
- 하드웨어 연결: `MyCobot280('/dev/ttyJETCOBOT', 1000000)`
- 제공 인터페이스:
  - Service: `/manipulator/get_current_coords`
  - Legacy Service: `/manipulator/execute_pnp`
  - Action: `/manipulator/execute_pnp_action`
- 모션 시퀀스:
  - gripper open -> pick coords -> gripper close -> via pose -> place coords -> gripper open -> home pose
  - `send_coords`, `send_angles` 후 `sleep` 기반 동기 시퀀스 제어
- 주요 파라미터:
  - 속도: `speed_fast`, `speed_slow`
  - 그리퍼: `gripper_open`, `gripper_close`
  - 자세: `home_pose`, `via_pose`

### 7.3 task_manager_node

- 역할: vision/manipulator를 액션 체인으로 묶는 상위 오케스트레이터
- 액션 서버: `/task/execute`
- GUI/토픽 트리거:
  - `/jetcobot/assembly/stack/request`에서 `Int32(data=1)` 수신 시 `ExecuteTask(marker_ids=[1,2,3], reset_stack=True)` 전송
- 내부 FSM 흐름:
  - marker loop -> vision 호출 -> pregrasp 생성 -> pnp 호출 -> stack level 증가
  - 취소/실패 시 즉시 abort 및 FSM reset
- 전략/좌표 생성:
  - `make_pregrasp`: target 기준 z +110mm 오프셋, x축 185도 회전, ee yaw +45도
  - `make_place_pose`: `assembly_pose` 기준으로 `stack_dz_mm`와 `stack_level`에 따라 적층 z 오프셋 계산
- 상태 토픽:
  - `/assembly_start`에 동작 상태 주기 발행
  - 완료 시 `/jetcobot/pick_and_place/done` 발행

## 8. 트러블슈팅

- `The passed action type is invalid`
  - 원인: `jetcobot_interfaces` 사용
  - 해결: `smartfactory_interfaces/action/ExecuteTask` 사용

- `ImportError: cannot import name 'PathSubstitution'`
  - 원인: `launch` 패키지 버전 불일치
  - 해결:

```bash
sudo apt update
sudo apt install --only-upgrade -y \
  ros-jazzy-launch ros-jazzy-launch-ros ros-jazzy-ros2launch \
  ros-jazzy-launch-xml ros-jazzy-launch-yaml \
  ros-jazzy-launch-testing ros-jazzy-launch-testing-ros
```

- 로봇 연결 실패
  - `/dev/ttyJETCOBOT` 권한/장치명 확인
  - 케이블/전원 연결 상태 확인

