cmake_minimum_required(VERSION 3.5)

project(betaflight_demo)

find_package(ament_cmake REQUIRED)

find_package(ament_cmake)
find_package(control_toolbox REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vehicle_gateway_betaflight REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(PKG_MSP REQUIRED IMPORTED_TARGET msp)
include_directories(${PKG_MSP_INCLUDE_DIRS})
link_directories(${PKG_MSP_LIBRARY_DIRS})

add_executable(altitude_control
  src/altitude_control.cpp
)
target_include_directories(altitude_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(altitude_control
  control_toolbox
  rclcpp
  std_msgs
  vehicle_gateway_betaflight
)
target_link_libraries(altitude_control
  ${PKG_MSP_LIBRARIES}
)

install(DIRECTORY
  launch
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  TARGETS altitude_control
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
