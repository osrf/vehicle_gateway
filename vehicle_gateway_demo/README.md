# Wingman demo

Launch the simulation with two VTOLs typing the following commands:

```bash
export MULTIROBOT_CONFIG=`ros2 pkg prefix vehicle_gateway_demo`/share/vehicle_gateway_demo/config/two_vtols.yaml
ros2 launch px4_sim px4_sim_multi.launch.py
```

When the simulation is running we need to launch the leader vehicle. Open one terminal and run:

```bash
ros2 run vehicle_gateway_demo vtol_position_control 1 `ros2 pkg prefix vehicle_gateway_demo`/share/vehicle_gateway_demo/config/leader_follower_multicast_discovery.json
```

Then you can run the wingman vehicle or follower with this other command

```bash
ros2 run vehicle_gateway_demo follower 2 `ros2 pkg prefix vehicle_gateway_demo`/share/vehicle_gateway_demo/config/leader_follower_multicast_discovery.json
```


If you want to visualize both drones you can also run QGroundControl
