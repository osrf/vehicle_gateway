cmake_minimum_required(VERSION 3.8)
project(vehicle_gateway_px4)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(vehicle_gateway REQUIRED)
find_package(pluginlib REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(vehicle_gateway_px4 src/vehicle_gateway_px4.cpp)
target_include_directories(vehicle_gateway_px4 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  vehicle_gateway_px4
  pluginlib
  px4_msgs
  rclcpp
  vehicle_gateway
)

add_executable(vehicle_gateway_px4_node src/vehicle_gateway_main.cpp)
target_include_directories(vehicle_gateway_px4_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  vehicle_gateway_px4_node
  vehicle_gateway
  "pluginlib"
  rclcpp
  px4_msgs
)

pluginlib_export_plugin_description_file(vehicle_gateway plugins.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(vehicle_gateway_px4 PRIVATE "VEHICLE_GATEWAY_PX4_BUILDING_LIBRARY")

install(
  TARGETS vehicle_gateway_px4
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS vehicle_gateway_px4_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(
  vehicle_gateway_px4
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(
  px4_msgs
  pluginlib
  rclcpp
  vehicle_gateway
)

ament_export_include_directories(
  include
)
ament_package()
