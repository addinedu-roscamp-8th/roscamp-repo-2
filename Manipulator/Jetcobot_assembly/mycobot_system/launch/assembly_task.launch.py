from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mycobot_system',
            executable='manipulator_node',
            name='manipulator',
            output='screen'
        ),
        Node(
            package='mycobot_system',
            executable='vision_node',
            name='vision',
            output='screen'
        ),
        Node(
            package='mycobot_system',
            executable='task_manager_node',
            name='task_manager',
            output='screen'
        )
    ])