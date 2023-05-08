cmake_minimum_required(VERSION 3.5)

project(vehicle_gateway_multi)

find_package(ament_cmake REQUIRED)

find_package(ament_cmake)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(vehicle_gateway_multi_bridge
  src/vehicle_gateway_multi_bridge.cpp
)
ament_target_dependencies(vehicle_gateway_multi_bridge
  rclcpp
  std_msgs
)
include_directories(zenoh)

# install(DIRECTORY
#   launch
#   config
#   DESTINATION
#     share/${PROJECT_NAME}
# )

install(
  TARGETS
    vehicle_gateway_multi_bridge
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
