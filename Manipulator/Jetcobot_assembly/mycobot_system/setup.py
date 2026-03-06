from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mycobot_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # ✅ launch 파일 설치 (폴더가 있으면 복사, 없어도 에러 안남)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # ✅ config 폴더 내의 npz 등 설정 파일 설치 (핵심 수정)
        # mycobot_system/config/ 안에 파일들이 있다면 아래 경로가 맞습니다.
        (os.path.join('share', package_name, 'config'), glob('mycobot_system/config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetcobot',
    maintainer_email='jetcobot@todo.todo',
    description='MyCobot ROS2 vision/manipulation system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = mycobot_system.vision_node:main',
            'manipulator_node = mycobot_system.manipulator_node:main',
            'task_manager_node = mycobot_system.task_manager_node:main',
            'pick_and_place_trigger_node = mycobot_system.pick_and_place_trigger_node:main',
        ],
    },
)
