# OpenArm ROS2 Control GUI

This project contains my OpenArm ROS2 Jazzy setup and custom Python GUI scripts for controlling the OpenArm v1.0 robotic arm through ROS2 action controllers.

## Features

- ROS2 Jazzy + OpenArm v1.0
- Classic CAN support for CANable / gs_usb
- Custom OpenArmHW modifications for MIT mode activation
- Left arm GUI control with Tkinter
- Joint delta control for joint1 to joint7
- Home and Emergency Home
- Gripper Open / Close
- Safe small-angle control with waypoint interpolation

## Important Safety Notes

Do not run:

- `openarm-can-demo`
- `openarm-can-zero-position-calibration`
- `set_zero`
- RViz `Plan & Execute` during first hardware tests

Use small joint deltas first, such as `0.01` or `0.02` rad.