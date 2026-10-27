# Robotiq 2F-85 model for AX

Source: [PickNikRobotics/ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper), commit `a74d007d8f2f06dc6a503ad21038ba869d4999a6`.

This is a model-only vendor subset, not the complete upstream driver package.
The original 2F-85 Xacro, ros2_control macro, nine visual DAE assets, nine collision STL assets, and BSD-3-Clause LICENSE are included unchanged.
Every imported file was verified against its upstream Git blob SHA; see `UPSTREAM_FILES.json`.

Local changes are limited to package metadata and CMake installation rules so the selected model can build without hardware drivers or ros2_control controllers. Original package metadata and CMake rules are retained under `upstream/` for reference.

The previous AX implementation (commit `4f85007`, now removed from the working tree) invoked the macro with `include_ros2_control="false"` for RViz preview. No AX source in the current working tree invokes it. This does not activate a serial port or hardware driver. The upstream `mimic` relationships are interpreted by joint_state_publisher. No claim of Gazebo grasping or controller compatibility is made by this model-only integration.

Do not install another package named `robotiq_description` in the same workspace without first replacing this subset and checking the pinned interfaces.
