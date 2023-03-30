add_launch_test(test_px4.py
  TARGET test_arm_disarm_vtol
  ARGS "vehicle_type:=standard_vtol" "script_test:=arm_disarm.py"
  TIMEOUT 120
)

add_launch_test(test_px4.py
  TARGET test_arm_disarm_x500
  ARGS "vehicle_type:=x500" "script_test:=arm_disarm.py"
  TIMEOUT 120
)

add_launch_test(test_px4.py
  TARGET test_takeoff_land_vtol
  ARGS "vehicle_type:=standard_vtol" "script_test:=takeoff_land.py"
  TIMEOUT 120
)

add_launch_test(test_px4.py
  TARGET test_takeoff_land_x500
  ARGS "vehicle_type:=x500" "script_test:=takeoff_land.py"
  TIMEOUT 120
)

add_launch_test(test_px4.py
  TARGET test_mc_fw_mc
  ARGS "vehicle_type:=standard_vtol" "script_test:=mc_fw_mc.py"
  TIMEOUT 120
)

install(
  PROGRAMS
    script/arm_disarm.py
    script/takeoff_land.py
    script/mc_fw_mc.py
  DESTINATION lib/${PROJECT_NAME}
)
