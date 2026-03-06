# mycobot280 ROS Action API
pymycobot 라이브러리를 기반으로 ROS Action을 통해 jetcobot을 동작 시키는 API

## 사용 규칙
### - "jetcobot_node.py"와 같은 형식의 Action Server에서만 작동
- "jetcobot_node.py"를 복붙하여 cobot에서 node로 실행. (Action Server)
- "jetcobot_action_client.py"를 pkg 폴더 안에 구성

## 로봇 설정
⭐⭐⭐ jetcobot_node.py의 $전역 설정$을 본인 환경에 맞추어 변경한다.

## 코드 적용

### 1) 정의하기
1. 모듈에서 사용할 '액션 클라이언트' class import 

```python
from <*your path*>.jetcobot_action_client import (
    PickClient,
    MoveToPoseClient,
    PlaceClient
)
```

2. 코드에서 객체 지정 

    *node class 안에서 정의합니다* 

```python
class YourNodeClass(Node):
    def __init__(self):
        super().__init__("<your_node_name>")

        #---밑에 추가--#
        self.pick_cli = PickClient(self, action_name="/pick")
        self.move_cli = MoveToPoseClient(self, action_name="/move_to_pose")
        self.place_cli = PlaceClient(self, action_name="/place")
```
<br/>  

### 2) Client 객체 적용

각 Action Client의 객체는 기본적으로 Action을 보내는 기능을 가진다.   
본 api는 함수를 통해 action을 보내고 Feedback, Result를 받는 기능을 기본적으로 탑재하였다.   
피드백 받는 메세지를 변경하고 싶으면 jetcobot_action_client.py 파일을 수정한다


파라미터로 넣어주는 좌표는 pymycobot의 send_coords, send_angles의 형태와 ***동일한 형식***의 좌표를 넣어주어야 한다.   
x y z : translation 좌표[mm]    
rx ry rz : intrinsic ZYX euler 각도 [deg]   
추가로 TCP 적용 기능이 jetcobot_node.py에 있기에 그리퍼에 맞게 설정하고 입력 좌표를 설정 TCP로 넣어줄 수 있다.


-  ⭐⭐ **send_goal()** 

    모든 객체가 동일하게 "send_goal"을 붙인 꼴이 action을 보내는 기능을 한다.   
    ㄴ *Move to Pose 만 뒤에 _coords 혹은 _angles라고 명시해줘야 한다.*

    ***Pick Client***   
    - Input Parameters 
        - List[float] = pick 하려는 목표의 좌표[x, y, z, rx, ry, rz] (frame: base, 단위: mm, deg)      
            *- Z 축 반전하는 기능이 내부에 있기에, Z축 방향 반전을 해서 좌표를 입력하지 말것!*
        - Bool = safe pick 기능 실행 여부
    - Output Parameters 
        - Bool = action이 성공적으로 보내졌는지의 여부   

        safe pick을 true로 두면 물체를 집기 이전에 물체의 z축에서 일정 거리만큼 멀어진 이후 물체를 집는 위치로 이동한다.    

        출력으로는 action을 보내는데, 문제가 생겨 안 보내지면 false를 반환한다. 이를 활용하여 다음과 같이 action이 보내지는 것을 확인하는 알고리즘을 구현할 수 있다.

        ```python
        # self.pick_coords = 목표의 좌표
        # self.safe_pick = safe pick 사용 여부 (bool)
        if not self.pick_cli.send_goal(self.pick_coords, self.safe_pick):
                self.get_logger().error("send_goal failed.. Trying Again")
                return
        ```
    <br/>        

    ***Place Client***   
    - Input Parameters: 
        - List[float] = place 하려는 목표의 좌표[x, y, z, rx, ry, rz] (frame: base, 단위: mm, deg)      
     
        - Bool = safe place 기능 실행 여부
    - Output Parameters: 
        - Bool = action이 성공적으로 보내졌는지의 여부   

        safe place를 true로 두면 물체를 두기 이전에 물체의 z축에서 일정 거리만큼 멀어진 이후 물체를 두는 위치로 이동한다.   

        출력으로는 action을 보내는데, 문제가 생겨 안 보내지면 false를 반환한다. 이를 활용하여 다음과 같이 action이 보내지는 것을 확인하는 알고리즘을 구현할 수 있다.


        ```python
        # self.place_coords = 목표의 좌표
        # self.safe_place = safe pick 사용 여부 (bool)
        if not self.place_cli.send_goal(self.place_coords, self.safe_place):
                self.get_logger().error("send_goal failed.. Trying Again")
                return
        ```
    <br/> 

    ***Move to Pose Client***   
    - Input Parameters: 
        - List[float] = [x, y, z, rx, ry, rz] (frame: base, 단위: mm, deg)    
        혹은 [$\theta_1$ , $\theta_2$, $\theta_3$, $\theta_4$, $\theta_5$, $\theta_6$] (단위: deg)         

    - Output Parameters: 
        - Bool = action이 성공적으로 보내졌는지의 여부   

        end effector를 두고 싶은 좌표 혹은 각도를 입력하면 이동하는 action이다. 좌표로 넣을때는 **send_goal_coords** 각도로 넣을때는 **send_goal_angles** 라고 **꼭!** 명시해줘야한다.   

        출력으로는 action을 보내는데, 문제가 생겨 안 보내지면 false를 반환한다. 이를 활용하여 다음과 같이 action이 보내지는 것을 확인하는 알고리즘을 구현할 수 있다.

        ```python
        # self.pick_coords = 목표의 좌표
        # self.safe_pick = safe pick 사용 여부 (bool)
        if not self.move_cli.send_goal_angles(HOME_ANGLES):
                self.get_logger().error("send_goal failed.. Trying Again")
                return
        ```
    <br/>   
<br/>

### 3) 공용 API
- ⭐ **action_done()** 

    - Input Parameters: 
        *없음*
    - Output Parameters: 
        - Bool = action result "success"
        - String = action result "msg"

    action을 수행하고 나서 완료하였는지의 판단과 action.result를 반환하는 기능을 제공한다.   

    만약, action 수행이 아직 안끝났으면 "None" 을 반환한다   
    action 수행이 완료가 되었으면 action.result 값을 반환한다.

    아래와 같은 형식을 통해 action 수행 여부를 판단 + result를 받아볼 수 있다.

    ```python
    # pick action done 확인 알고리즘

    done = self.pick_cli.action_done()

    if done is None: # Action 완료 X
        return False

    success, message = done # Action 완료 O

    if success: # Action 수행 성공
        self.get_logger().info(f"[TASK DONE] success={success} msg={message}")
        return True

    else: # Action 수행 실패 - error 발생
        self.get_logger().error(f"[TASK FAIL] success={success} msg={message}")
        return False
    ```
<br/>

-  **is_busy()**
    - Input Parameters: 
     *없음*
    - Output Parameters: 
        - Bool = action result "success"

    action이 현재 바쁜 상태인지 확인할 있다.   
    action을 수행중이면 True, 완료 혹은 수행 이전 상태에서는 False를 반환한다.

<br/>

-  **last_feedback()**
    - Input Parameters: 
        *없음*
    - Output Parameters: 
        - float = action feedback "progress"
        - string = action feedback "state"

    action의 가장 최근 feedback을 반환한다.






    
