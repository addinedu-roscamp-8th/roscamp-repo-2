import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

class ControlPC(Node):
    def __init__(self):
        super().__init__('control_pc')

        #=== Open / mobile 연동 ===
        self.sub_done = self.create_subscription(Bool, '/pick_and_place/done', self.on_open_done, 10) # 오픈 → PC
        self.pub_done = self.create_publisher(Bool, '/pick_and_place/done',10) # PC → 모바일
        self.pub_start = self.create_publisher(Bool, '/pick_and_place/start', 10) # PC → 오픈

        #=== jetcobot / mobile 연동 ===
        self.pub_start_jetcobot = self.create_publisher(Bool, 'jetcobot/pick_and_place/start', 10) # PC → 제코봇
        self.pub_done_jetcobot = self.create_publisher(Bool, 'jetcobot/pick_and_place/done', 10) # PC → 모바일
        
        self.sub_done_jetcobot = self.create_subscription(Bool, 'jetcobot/pick_and_place/done', self.on_jetcobot_done, 10) # 제코봇 → PC 
        self.sub_arrived_point = self.create_subscription(Int32,'/arrived_point',self.on_arrived_point,10) # 모바일 → PC (도착 포인트)

        # === Internal state flags ===        
        self.waiting_open_done = False
        self.waiting_jetcobot_done = False



    # === Open manipulator done callback ===
    def on_open_done(self, msg: Bool):
        if not msg.data:
            return

        if not self.waiting_open_done:
            self.get_logger().info('Open 기다리는 작업 없음 → done 무시')
            return

        self.get_logger().info('Open 매니퓰레이터 완료 → 모바일 done 전달')
        self.pub_done.publish(Bool(data=True))
        self.waiting_open_done = False

    # === Jetcobot manipulator done callback ===
    def on_jetcobot_done(self, msg: Bool):
        if not msg.data:
            return

        if not self.waiting_jetcobot_done:
            self.get_logger().info('Jetcobot 기다리는 작업 없음 → done 무시')
            return

        self.get_logger().info('Jetcobot 매니퓰레이터 완료 → 모바일 done 전달')
        self.pub_done_jetcobot.publish(Bool(data=True))
        self.waiting_jetcobot_done = False

    # === Mobile arrived point callback ===
    def on_arrived_point(self, msg: Int32):
        value = msg.data

        if value == 2:
            self.get_logger().info('arrived_point=2')
            if self.waiting_open_done:
                self.get_logger().info('Open 이미 작업 중 → start 무시')
            else:
                self.get_logger().info('모바일 도착 → 오픈 매니퓰레이터 start 전달')
                self.pub_start.publish(Bool(data=True))
                self.waiting_open_done = True
        elif value == 3:
            self.get_logger().info('arrived_point=3')
            if self.waiting_jetcobot_done:
                self.get_logger().info('Jetcobot 이미 작업 중 → start 무시')
            else:
                self.get_logger().info('모바일 도착 → 제코봇 start 전달')
                self.pub_start_jetcobot.publish(Bool(data=True))
                self.waiting_jetcobot_done = True
        else:
            self.get_logger().info(f'arrived_point={value}')


def main():
    rclpy.init()
    node = ControlPC()
    rclpy.spin(node)
    rclpy.shutdown()
