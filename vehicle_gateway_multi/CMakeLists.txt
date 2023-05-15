cmake_minimum_required(VERSION 3.5)

project(vehicle_gateway_multi)

find_package(ament_cmake REQUIRED)

find_package(ament_cmake)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(zenohc REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(vehicle_gateway_multi_bridge
  src/vehicle_gateway_multi_bridge.cpp
)
target_link_libraries(vehicle_gateway_multi_bridge PRIVATE
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  zenohc::lib
  nlohmann_json::nlohmann_json)

install(DIRECTORY
  config
  DESTINATION
    share/${PROJECT_NAME}
)

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
