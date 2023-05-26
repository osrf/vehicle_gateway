# vehicle_gateway_multi

This package will create a bridge between the ROS 2 topic `/px4_<vehicle_id>/fmu/out/vehicle_gps_position"`
and the zenoh key `/vehicle_gateway/<vehicle_id>/telemetry`.

# Test it

Launch the `px4_sim_multi.launch.py` example from the `px4_sim` package:

```bash
export MULTIROBOT_CONFIG=<path to your workspace>/src/vehicle_gateway/px4_sim/config/multi.yaml
ros2 launch px4_sim px4_sim_multi.launch.py
```

This launch file will launch a Gazebo simulation with two vehicles: one quadcopter and one vtol both in the same DDS domain id.
If you want to use a different DDS domain id you can easily modify this in the `multi.yaml` file.

Then you should run the ROS 2 <-> Zenoh bridge. Type this for the vehicle one:

```bash
# export ROS_DOMAIN_ID=<dds domain id vechile 1>
ros2 run vehicle_gateway_multi vehicle_gateway_multi_bridge <path to your workspace>/src/vehicle_gateway/vehicle_gateway_multi/config/zenoh_all_localhost.json5 1
```

Type this for the vehicle two:
```bash
# export ROS_DOMAIN_ID=<dds domain id vechile 2>
ros2 run vehicle_gateway_multi vehicle_gateway_multi_bridge <path to your workspace>/src/vehicle_gateway/vehicle_gateway_multi/config/zenoh_all_localhost.json5 2
```

Now we can check that everything is working if we use the following program to subscribe the zenoh key `vehicle_gateway/*/telemetry`

```bash
ros2 run vehicle_gateway_multi vehicle_gateway_multi_bridge_client
```
