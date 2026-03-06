from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jetcobot_pkg'
launch_files = [f for f in glob('launch/*') if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), launch_files)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetcobot',
    maintainer_email='jinoobluee@gmail.com',
    description='Jetcobot Camera Manipulator Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = jetcobot_pkg.camera_node:main',
            'jetcobot_node = jetcobot_pkg.jetcobot_node:main',
            'task_manager_node = jetcobot_pkg.task_manager_node:main',
            'yolo_move_node = jetcobot_pkg.yolo_move_node:main',
            'udp_stream_node = jetcobot_pkg.udp_stream_node:main'
        ],
    },
)
