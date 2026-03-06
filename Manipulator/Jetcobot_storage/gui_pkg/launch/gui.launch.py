from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    gui_node = Node(
        package='gui_pkg',
        executable='gui_node',
        output='screen'
    )

    return LaunchDescription([gui_node])
