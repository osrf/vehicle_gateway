# Betaflight

The Gazebo support for Betaflight was added in this [PR](https://github.com/betaflight/betaflight/pull/12346).
It's already available in `master` branch.

## Packages available

 - **betaflight_configurator**:  This package download and install in the workspace the
 ground control station. To connect with the SITL please use "Manual" and "Port: tcp://127.0.0.1:5761"

  - Modes: Configure arm in AUX1 (value 1500 - 2000)
  - Motors: Select PWM and 3D

 - **betaflight_controller**: When the drone is connected to SITL this requires an RC, otherwise
 the flight controller will generate a failsafe (RX_FAILSAFE). This program will open a
 UDP socket in the port 9004. Values are between [1000 - 2000]
   - Connect a joystick
   - You need to run `ros2 run joy joy_node`
   - And the `ros2 run betaflight_controller main`.

 - **betaflight_gazebo**: This plugin will send the state data to the SITL and it will received
 the data from the motors [-1, 1].
  - Sending state output to SITL at UDP link: 127.0.0.1:9003
  - Receiving PWM from SITL at UDP server: 127.0.0.1:9003

 - **betaflight_sim**: Download and install the betaflight flight controller.

## Run the simulation

```bash
ros2 launch betaflight_sim quadcopter.launch.py
```

## Test
