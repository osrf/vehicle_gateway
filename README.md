# Vehicle Gateway

The goal of this project is to create a pluginlib-based C++ library that can interface with several vehicle SDK's.

# Installation

This package is developed and on Ubuntu 22.04 LTS with ROS 2 Humble.

If you have installed ROS 2 Humble from APT on your system (the recommended method unless you prefer to work from source), the following steps will create a workspace for this repo and build it:

First, install ROS 2 and Gazebo Garden.
Note that Gazebo Garden must be installed separately, following its instructions below; it is not the version that ships with Ubuntu 22.04 LTS.
This is why `ros_gz` must be built as part of the repository dependencies in subsequent steps.
 * [ROS 2 Humble binary installation instructure](http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
 * [Gazebo Garden binary installation instructions](https://gazebosim.org/docs/garden/install_ubuntu)

```bash
sudo apt install python3-kconfiglib python3-jinja2 python3-jsonschema ros-humble-gps-msgs
pip3 install pyros-genmsg
mkdir -p ~/vg_ws/src
cd ~/vg_ws/src
git clone https://github.com/osrf/vehicle_gateway
cd ~/vg_ws
vcs import src < src/vehicle_gateway/dependencies.repos
rosdep update && rosdep install --from-paths src --ignore-src -y
source /opt/ros/humble/setup.bash
colcon build --event-handlers console_direct+
source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```
