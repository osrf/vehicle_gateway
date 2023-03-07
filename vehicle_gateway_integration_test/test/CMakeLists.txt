add_launch_test(test_px4.py
  TARGET test_arm_disarm_vtol
  ARGS "vehicle_type:=standard_vtol" "script_test:=arm_disarm.py"
  TIMEOUT 240
)

add_launch_test(test_px4.py
  TARGET test_arm_disarm_x500
  ARGS "vehicle_type:=x500" "script_test:=arm_disarm.py"
  TIMEOUT 240
)

add_launch_test(test_px4.py
  TARGET test_takeoff_land_vtol
  ARGS "vehicle_type:=standard_vtol" "script_test:=takeoff_land.py"
  TIMEOUT 240
)

add_launch_test(test_px4.py
  TARGET test_takeoff_land_x500
  ARGS "vehicle_type:=x500" "script_test:=takeoff_land.py"
  TIMEOUT 120
)

install(
  PROGRAMS
    script/arm_disarm.py
    script/takeoff_land.py
  DESTINATION lib/${PROJECT_NAME}
)
