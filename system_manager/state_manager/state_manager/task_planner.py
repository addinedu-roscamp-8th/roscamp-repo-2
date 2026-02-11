import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

class ControlPC(Node):
    def __init__(self):
        super().__init__('control_pc')



def main():
    rclpy.init()
    node = ControlPC()
    rclpy.spin(node)
    rclpy.shutdown()
