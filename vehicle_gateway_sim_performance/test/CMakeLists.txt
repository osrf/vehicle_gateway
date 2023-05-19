foreach(val RANGE 1 12 2)
  add_launch_test(test_px4_multirobot.py
    TARGET test_performance_position_control_vtol_${val}
    ENV "NUMBER_OF_VEHICLES=${val}"
    ARGS "vehicle_type:=standard_vtol" "script_test:=vtol_position_control"
    TIMEOUT 700
  )
endforeach()

foreach(val RANGE 1 10 2)
  add_launch_test(test_px4_multirobot_multiple_domain_id.py
    TARGET test_performance_position_control_vtol_different_domain_id_${val}
    ENV "ROS_DOMAIN_ID=${val};NUMBER_OF_VEHICLES=${val}"
    ARGS "vehicle_type:=standard_vtol" "script_test:=vtol_position_control"
    TIMEOUT 700
  )
endforeach()
