# Jetcobot Package Nodes   
jetcobot 패키지 내의 node 구성

</d>   


## ROS 통신

### Camera Node  
- publish      
    - /parts - 카메라 인식 부품 정보 토픽   

### Task Manager Node

- publish   
    - /storage_start - Storage 작업 시작 토픽(외부)

- subscribe   
    - /assembly_start - Assembly 작업 시작 토픽(외부)
    - /parts 

- services
    - /set_task_mode - manual 또는 auto 모드 지정 서비스(외부)
    - /manual_pick - manual pick 명령 서비스(외부)

- action clients
    - /pick
    - /move_to_pose
    - /place   

### Jetcobot Node
- publish   
    - /action_done - action 완료 topic(외부)

- actions
    - /pick - 좌표로 이동 + 그리퍼 close
    - /move_to_pose - 좌표/자세로 이동
    - /place - 좌표로 이동 + 그리퍼 open
   


#### 내부 통신 구조
<img src="/home/jetcobot/robot_ws/src/jetcobot_pkg/docs/img/Screenshot from 2026-02-06 11-25-29.png" width="500">