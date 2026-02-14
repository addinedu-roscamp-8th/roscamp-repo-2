# OpenManipulator_X (Empty by Design)

This directory is intentionally left empty.

All OpenManipulator_X low-level control and bringup
(e.g. hardware, ros2_control, MoveIt bringup)
are executed directly on the RSBP (on-board controller).

High-level task and scenario logic
(e.g. pick-and-place, assembly flows)
is implemented outside this directory
under the workspace-level scenario manager.
