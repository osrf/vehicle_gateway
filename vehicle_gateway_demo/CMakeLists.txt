cmake_minimum_required(VERSION 3.5)

project(vehicle_gateway_demo)

find_package(ament_cmake REQUIRED)

find_package(ament_cmake)
find_package(control_toolbox REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vehicle_gateway_px4 REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)

add_executable(aruco_demo
  src/aruco_demo.cpp
)
target_include_directories(aruco_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(aruco_demo
  control_toolbox
  rclcpp
  std_msgs
  vehicle_gateway_px4
  aruco_opencv_msgs
)

add_executable(circles
  src/circles.cpp
)
ament_target_dependencies(circles
  rclcpp
  vehicle_gateway_px4
)

add_executable(vtol_position_control
  src/vtol_position_control.cpp
)
ament_target_dependencies(vtol_position_control
  rclcpp
  vehicle_gateway_px4
)

add_executable(follower
  src/follower.cpp
)
ament_target_dependencies(follower
  rclcpp
  vehicle_gateway_px4
)


install(DIRECTORY
  launch
  config
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  TARGETS
    aruco_demo
    circles
    follower
    vtol_position_control
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
