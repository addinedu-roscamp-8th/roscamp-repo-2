from setuptools import setup

package_name = "nav2_waypoint"

setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    py_modules=[
        "run_scenario",
        "keyboard_trigger",
        "gui_trigger",
        "manip_trigger",
        "scenario_manager",
        "Nav2_classes",
        "battery_tcp_bridge",
        "battery_cmd_publisher",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml", "goal_points.json"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pinky",
    maintainer_email="pinky@example.com",
    description="Nav2 waypoint scenario nodes",
    license="MIT",
    entry_points={
        "console_scripts": [
            "run_scenario = run_scenario:main",
            "keyboard_trigger = keyboard_trigger:main",
            "gui_trigger = gui_trigger:main",
            "manip_trigger = manip_trigger:main",
            "battery_tcp_bridge = battery_tcp_bridge:main",
            "battery_cmd_publisher = battery_cmd_publisher:main",
        ],
    },
)
