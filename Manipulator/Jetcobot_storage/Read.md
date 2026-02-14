Jetcobot_storage (Empty by Design)

This directory is intentionally left empty.

All Jetcobot_storage low-level control and bringup
(e.g. hardware interface, ros2_control, motor drivers)
are executed directly on the Jetcobot onboard controller.

High-level task and scenario logic
(e.g. pick-and-place execution, storage handling flows)
is implemented outside this directory
under the workspace-level task coordinator or scenario manager.
