from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    bridge_dir = os.path.join(
        get_package_share_directory('system_bridge'),
        'domain_bridge'
    )

    launch_nodes = []

    # domain_bridge 폴더 안의 모든 yaml 파일 검색
    for file in os.listdir(bridge_dir):
        if file.endswith('.yaml'):
            yaml_path = os.path.join(bridge_dir, file)

            node = Node(
                package='domain_bridge',
                executable='domain_bridge',
                name=f"bridge_{file.replace('.yaml','')}",
                output='screen',
                arguments=[yaml_path],
            )

            launch_nodes.append(node)

    return LaunchDescription(launch_nodes)
