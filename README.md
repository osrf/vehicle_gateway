# Vehicle Gateway

The goal of this project is to create a pluginlib-based C++ library that can interface with several vehicle SDK's.

# Installation

This package is developed and on Ubuntu 22.04 LTS with ROS 2 Humble. We suggest installing ROS 2 Humble from binary packages, and building Gazebo Garden from source. The following steps will describe this process, resulting in two workspaces: one for Gazebo Garden, and an overlaid workspace for the Vehicle Gateway (this project), in order to make rebuilds faster.

To keep paths short, `vg` stands for "Vehicle Gateway". After these steps, you'll have two workspaces in the `~/vg` directory, like this:

```bash
vg
├── gz_ws
│   ├── build
│   ├── install
│   ├── log
│   └── src
└── vg_ws
    ├── build
    ├── install
    ├── log
    └── src
```

First, install ROS 2 Humble using the `.deb` packages using APT [according to these instructions](http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Next, install a few dependencies and set up our workspace source directories:
```bash
sudo apt install python3-kconfiglib python3-jinja2 python3-jsonschema ros-humble-gps-msgs
pip3 install pyros-genmsg
mkdir -p ~/vg/vg_ws/src
cd ~/vg/vg_ws/src
git clone https://github.com/osrf/vehicle_gateway
cd ~/vg/vg_ws
vcs import src < src/vehicle_gateway/dependencies.repos
vcs import src < src/vehicle_gateway/gazebo.repos
```

Next, build Gazebo Garden from source. The full instructions are [here](https://gazebosim.org/docs/garden/install_ubuntu_src), and summarized in the following command sequence:

```bash
mkdir -p ~/vg/gz_ws/src
cd ~/vg/gz_ws
vcs import src < ../vg_ws/src/vehicle_gateway/gazebo.repos
sudo apt install -y $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')
source /opt/ros/humble/setup.bash
colcon build --merge-install
```
Now you should have Gazebo available to any terminals that source `~/vg/gz_ws/install/setup.bash`

We can now build the Vehicle Gateway itself, by overlaying its workspace on top of the Gazebo Garden workspace (which in turn is overlaying the ROS 2 Humble system installation). The Vehicle Gateway build will also download and build the PX4 firmware, to allow software-in-the-loop (SITL) simulation:

```bash
cd ~/vg/vg_ws
rosdep update && rosdep install --from-paths src --ignore-src -y
source ~/vg/gz_ws/install/setup.bash
colcon build --merge-install --event-handlers console_direct+
```

# Run a PX4 Quadcopter demo
```bash
cd ~/vg/vg_ws
source install/setup.bash
ros2 launch px4_sim x500_launch.py
```

# Dockerfile

```bash
cd Docker
docker build -t vehicle_gateway .
```

Install rocker

```bash
sudo apt-get install python3-rocker
```

Run the container with rocker to visualize the GUI

```bash
rocker --x11 vehicle_gateway ros2 launch px4_sim plane.launch.py
```
