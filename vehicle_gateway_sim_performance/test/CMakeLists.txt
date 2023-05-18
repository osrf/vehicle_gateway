foreach(val RANGE 1 12 2)
  add_launch_test(test_px4_multirobot.py
    TARGET test_performance_position_control_vtol_${val}
    ENV "NUMBER_OF_VEHICLES=${val}"
    ARGS "vehicle_type:=standard_vtol" "script_test:=vtol_position_control"
    TIMEOUT 700
  )
endforeach()

install(
  PROGRAMS
    script/takeoff_land_multirobot.py
  DESTINATION lib/${PROJECT_NAME}
)
