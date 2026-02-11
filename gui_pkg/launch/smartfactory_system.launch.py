from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('system_bridge'),
                'launch',
                'smartfactory_bridge.launch.py'
            )
        )
    )

    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gui_pkg'),
                'launch',
                'gui.launch.py'
            )
        )
    )

    return LaunchDescription([
        bridge_launch,
        gui_launch
    ])
