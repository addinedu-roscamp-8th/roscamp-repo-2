import launch
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='jetcobot_pkg',
            executable='camera_node',
            name='camera_node'),
        
        Node(
            package='jetcobot_pkg',
            executable='jetcobot_node',
            name='jetcobot_node'),

        Node(
            package='jetcobot_pkg',
            executable='task_manager_node',
            name='task_manager_node')
        
    ])