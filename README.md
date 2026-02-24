## 프로젝트 개요
스마트 팩토리 프로젝트에서 부품을 인식하여 부품 창고에 보관하는 역할을 하는 cobot의 ROS 패키지 파일

## Demo 영상

### 1차 - storage 기본 기능

<img width="320" height="180" alt="image" src="https://github.com/user-attachments/assets/1bce6122-7876-44d7-a9a1-612a3794ac66" />

<https://youtube.com/shorts/XGbRtqUMsYY?feature=share>

### 2차 storage assembly 통신 기능

<https://www.youtube.com/shorts/-jcJspVIAEA?feature=share>


## 개발 환경
OS: Ubuntu 22.04 LTS (Jammy)

ROS 2: Jazzy Jalisco

Language: Python

Vision: OpenCV, ArUco

Motion Planning: pymycobot API

Hardware: Elephant Robotics pymycobot280, mount camera

## 주요 기능
1. 관측 영역에서의 부품들을 내부 DB에 기록 및 DB topic 발행

2. 관측되는 여러 부품 중 선별하여 부품 하나 선정

3. 선정된 부품을 pick하여 부품이 속한 창고 위치에 place


## 노드 구성 및 기능
- 📷 **Camera Node**
  - Aruco 마커 Cobot base 기준 좌표 추정
  - 위치 변화량 기반 부품 정지 상태 판단
  - 여러 부품 위치, id, 정지 상태, 정지 시간 정보를 묶어서 topic 발행
 
- 🦾 **Jetcobot Node**
  - 집을 부품의 좌표와 놓아야 하는 좌표를 받아 Pick and Place하는 action server (구분 동작 수행시 Feedback 전송)
 
- 🚦 **Task Manager Node**
  - 구독한 부품 정보에 기반하여 pick할 물체 선정
  - cobot에게 action 명령 보내는 주체

## 현버전 평가 및 한계
### 평가
- Pick and Place 성공률 - **80%**, 관측 영역의 특정 지점에서 pick 실패
- Pick 정밀성 - **Robust**, gripper가 집는 위치가 부품 1cm 내외로 변함
- Robot Path Planning 성공률 - **90%**, 가끔 그리퍼가 workspace 한계치까지 이동하면 IK 에러 발생
### 한계
1. 하드웨어 한계
- Manipulator Backlash, send_coords 부정확 >> send_angles로 대체 방안
- 저품질 Camera, marker 위치 추정 정확도 감소
2. 소프트웨어 한계
- pymycobot API로는 path planning 제약 있음 (로봇이 지나가는 경로의 장애물 / 위치 회피 X)
- FeedBack 실패 판단 기능 부재 (모터 드라이버 접근 권한이 제한되어 있어 실시간 로봇의 위치 판단 X)

## 주요 변경 사항 

#### 2026.01.30 주요 변경 사항 

- Required
  1. 조립용 Cobot이 Aruco Marker 인식하기 편한 방향으로 Place 하기
  3. 창고 Section 나누고 해당 section에 물체가 이미 있으면 위치 살짝 다르게 두는 알고리즘
  4. 같은 id 부품 여러개 들어왔을 때 구분하는 알고리즘
  5. jetcobot 끼리 공유 가능한 action server, client api 제작  
  5. 배포용 rosdep key, package.xml 파일에 작성
  7. Service로 Maual/Auto 모드 전환 기능

- Optional
  1. jetcobot camera calibration 재측정
  2. ~~Robot pose 토픽으로 구독 받아서 base 기준 마커 위치 추정 (현재 고정 pose 사용)~~

추후 추가 기능
  1. Marker 없이 물체 pose estimation
  2. cobot gripper가 집는 곳에 물체가 있으면 집는 pose 바꾸기


#### 2026.02.06 주요 변경 사항 

interfaces 
- action 3개로 재분류
- msg DB 통신용 topic 생성
- srv jetcobot한테서 현재 coords/angles 요청하는 service 생성
- cobot 통신용 topic 생성

jetcobotpkg
- docs 파일 추가 - 마크다운 파일 폴더
- utils - cobot: apply_xyz_offset_to_sendcoords 함수 추가
	- jetcobot_action_client 유틸 생성

- Camera Node    
  - jetcobot한테 coords/angles 받는 service client 추가
- Jetcobot Node    
  - Action Server 3개로 재분류   
  - Offset 적용을 target / base 로 설정하는 기능 추가   
  - Z 축, 물체 높이 고정 사용 기능 추가   
  - 등 대대적인 대공사   
- Task Manager Node     
  - DB 통신용 topic 추가   
  - jetcobot 끼리 동작 시작 알려주는 토픽 발행 구독 기능 추가   
  - Action 재분류 한거에 맞추어 전체적인 수정   
    - state 분류가 많아짐   
    - action 실패 시 동작 과정 복구 및 원위치 기능 단순한 수준으로 추가 
		    	
추후 과제

- YOLO로 좌표 만드는 node 개발
- DB, GUI 정보 연결
- jetcobot traffic 을 이동 경로 비교를 통해 고도화
- 외부 오차 요인 분석 list화
- Calibration 파일들 정리해서 추가하기

#### 2026.02.10 주요 변경 사항 

interfaces 
- 부팅용 통신 topic 생성
- 수동 자동 모드 topic 생성

jetcobotpkg

topic 이름들 분류하여서 directory 구조로 수정
- utils
- Camera Node
- Jetcobot Node
- Task Manager Node   
  - 서브 부팅 기능 추가   
  - 자동 / 수동 모드 추가
		    	
추후 과제

- YOLO로 객체 인식 하고 좌표 받는 pipe line 구성 및 테스트
- DB, GUI domain bridge 연결
- jetcobot traffic 규칙을 설정하여 부딪히지 않는 선에서 동시 작동
- Calibration 파일들 정리해서 추가하기
- 상세 설계 문서 작업