import launch
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='jetcobot_pkg',
            executable='yolo_move_node',
            name='camera_udp_node'),

        Node(
            package='jetcobot_pkg',
            executable='udp_stream_node',
            name='udp_stream_node'),

        Node(
            package='jetcobot_pkg',
            executable='jetcobot_node',
            name='jetcobot_node')
    ])
