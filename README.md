# Vehicle Gateway

The goal of this project is to create a pluginlib-based C++ library that can interface with several vehicle SDK's.

# Installation

This package is developed and on Ubuntu 22.04 LTS with ROS 2 Humble.

If you have installed ROS 2 Humble from APT on your system (the recommended method unless you prefer to work from source), the following steps will create a workspace for this repo and build it:

```
mkdir -p ~/vg_ws/src
cd ~/vg_ws/src
git clone https://github.com/osrf/vehicle_gateway
vcs import src < src/vehicle_gateway/dependencies.repos
cd ~/vg_ws
source /opt/ros/humble/setup.bash
colcon build
```
