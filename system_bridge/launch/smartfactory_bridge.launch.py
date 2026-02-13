from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    bridge_yaml = os.path.join(
        get_package_share_directory('system_bridge'),
        'domain_bridge',
        'bridge_for_openmanipulator.yaml'
    )

    domain_bridge_node = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='bridge_for_openmanipulator',
        output='screen',
        arguments=[bridge_yaml],
    )

    return LaunchDescription([
        domain_bridge_node
    ])
