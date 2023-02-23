add_launch_test(test_arm_disarm.py
  TARGET test_arm_disarm_vtol
  ARGS "vehicle_type:=standard_vtol"
  TIMEOUT 120
)

add_launch_test(test_arm_disarm.py
  TARGET test_arm_disarm_x500
  ARGS "vehicle_type:=x500"
  TIMEOUT 120
)

add_launch_test(test_takeoff_land.py
  TARGET test_takeoff_land_vtol
  ARGS "vehicle_type:=standard_vtol"
  TIMEOUT 120
)

add_launch_test(test_takeoff_land.py
  TARGET test_takeoff_land_x500
  ARGS "vehicle_type:=x500"
  TIMEOUT 120
)
