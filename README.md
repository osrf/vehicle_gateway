# Vehicle Gateway

The goal of this project is to create a pluginlib-based C++ library that can interface with several vehicle SDK's.

# Installation

This package is developed and on Ubuntu 22.04 LTS with ROS 2 Humble, and uses Gazebo Garden for simulation. To save lots of compile time, we recommend installing ROS 2 Humble and Gazebo Garden from binary packages.

### Binary ROS 2 Humble Installation
First, install ROS 2 Humble using the `.deb` packages using APT [according to these instructions](http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Binary Gazebo Installation
Next, install Gazebo Garden. The full instructions are [here](https://gazebosim.org/docs/garden/install_ubuntu), and summarized as follows:

```bash
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
```

Note that Gazebo uses multicast sockets to communicate between its components, and depending on your firewall configuration, these may be blocked and prevent Gazebo from starting. If Gazebo does not start, or if it only shows an empty black window, try this:
```bash
sudo ufw disable
```

Because the `ros_gz` bridge that ships with ROS 2 Humble targets the previous version of Gazebo (Fortress), we will remove it and build our own version of the ROS-Gazebo link packages in the next section. To avoid some warning about having multiple versions of those packages, you can remove them from your ROS 2 Humble installation:
```bash
sudo apt remove ros-humble-ros-gz-bridge
```

### Build the Vehicle Gateway
We can now build the Vehicle Gateway itself. To keep paths short, we will make a colcon workspace named `vg` for "Vehicle Gateway", in your home directory. The Vehicle Gateway build will also download and build the PX4 firmware and Betaflight firmware, to allow software-in-the-loop (SITL) simulation of multiple autopilot software stacks.

At time of writing, the `rosdep` command has to include a lot of `--skip-key` because currently Gazebo Garden is not yet in `rosdep`.

```bash
sudo apt install python3-kconfiglib python3-jinja2 python3-jsonschema ros-humble-gps-msgs gcc-arm-none-eabi libfuse2 python3-pip git python3-vcstool python3-future rsync
pip3 install pyros-genmsg
mkdir -p ~/vg/src
cd ~/vg/src
git clone https://github.com/osrf/vehicle_gateway
cd ~/vg
vcs import src < src/vehicle_gateway/dependencies.repos
source /opt/ros/humble/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -y --skip-keys="gz-transport12 gz-common5 gz-math7 gz-msgs9 gz-gui7 gz-cmake3 gz-sim7"
colcon build --event-handlers console_direct+
```

### If necessary: build Gazebo from source
In the event that you need or want to test out pre-release changes that are only available in the very latest Gazebo source code, you can always build Gazebo Garden from source. Note that this is considerably more complicated and requires significant compile time. Instructions to do this are [provided here](build_gazebo_from_source.md).

# Run a PX4 Quadcopter demo

Now that the software stack is built, we can do some fun and interesting things! Here is a minimal example that runs the PX4 software-in-the-loop (SITL) in an empty Gazebo world. Gazebo is used to generate all the sensors used to feed the PX4, and the PX4 SITL actuator commands are simulated by the Gazebo physics engine:

```bash
cd ~/vg
source install/setup.bash
ros2 launch px4_sim px4_sim.launch.py drone_type:='x500'
```

For a more visually-interesting world that can be used for simulating vision-guided experiments, we created Null Island, a tiny island with a few launch pads for small drones. The launch pad can be selected as a command-line parameter to `ros2 launch` as shown here:

```bash
cd ~/vg
source install/setup.bash
ros2 launch px4_sim px4_sim.launch.py drone_type:='x500' position_name:=pad_1 world_name:=null_island
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
rocker --x11 vehicle_gateway ros2 launch px4_sim px4_sim.launch.py drone_type:='x500' world_name:=null_island model_pose:="-9.7948, -8.31, 2"
```
