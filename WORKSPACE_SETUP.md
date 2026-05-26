# Workspace Setup and Transfer Guide

This file explains what someone else needs to install in order to build and run this ROS 2 workspace on another machine.

## What to share

Share the workspace source tree, especially:

- `src/my_robot_description`
- `src/my_robot_bringup`
- `src/my_robot_controller`
- `src/my_robot_webui`

Do not rely on sharing `build/`, `install/`, or `log/`. The receiver should rebuild the workspace on their own machine.

If they want the optional YOLO-based perception workflow, also share:

- `yolov8n.pt`

If they want the optional dashboard natural-language command routing, they will need their own API key in:

- `.my_robot_webui.env`

## Recommended platform

- Ubuntu `22.04`
- ROS 2 `Humble`

This workspace is currently aligned with Python `3.10`, which matches Ubuntu 22.04 well.

## Required system packages

Install these first:

```bash
sudo apt update
sudo apt install -y \
  git \
  python3-pip \
  python3-colcon-common-extensions \
  build-essential \
  ros-humble-desktop \
  ros-humble-xacro \
  ros-humble-gazebo-ros \
  ros-humble-rviz2 \
  ros-humble-robot-state-publisher \
  ros-humble-cv-bridge \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs
```

`ros-humble-desktop` covers the standard ROS 2 desktop stack. The extra packages above are listed explicitly because the workspace depends on them directly in launch files and Python nodes.

## Required Python packages

Install these for the controller and dashboard code:

```bash
pip3 install --user numpy opencv-python
```

If the receiver wants YOLO/sign detection features, also install:

```bash
pip3 install --user ultralytics
```

## Optional packages for the browser dashboard desktop stream

These are only needed if they want Gazebo and RViz embedded inside the browser through the dashboard:

```bash
sudo apt install -y x11vnc novnc websockify
```

They also need a Linux desktop session with a valid `DISPLAY`.

Without these packages, the dashboard still works for:

- mission launch
- robot commands
- mission status
- camera streaming

But the embedded desktop panel will not work.

## Build steps

After installing ROS 2 and dependencies:

```bash
cd /path/to/ros2_workspace
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Main run commands

### Parking mission

```bash
source /opt/ros/humble/setup.bash
source /path/to/ros2_workspace/install/setup.bash
ros2 launch my_robot_bringup parking_mission.launch.xml
```

### Road mission

```bash
source /opt/ros/humble/setup.bash
source /path/to/ros2_workspace/install/setup.bash
ros2 launch my_robot_bringup my_robot_mission.launch.xml
```

### Web dashboard

```bash
source /opt/ros/humble/setup.bash
source /path/to/ros2_workspace/install/setup.bash
ros2 run my_robot_webui dashboard_server
```

To start the dashboard in road mode:

```bash
ros2 run my_robot_webui dashboard_server --stack-mode road
```

## Quick checklist

- Ubuntu 22.04 installed
- ROS 2 Humble installed
- apt packages from this file installed
- Python packages from this file installed
- workspace copied
- `colcon build` completed
- `source install/setup.bash` run
- optional `.my_robot_webui.env` created
- optional `yolov8n.pt` copied
