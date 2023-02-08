cmake_minimum_required(VERSION 3.8)
project(vehicle_gateway_betaflight)

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
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vehicle_gateway REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(PKG_MSP REQUIRED IMPORTED_TARGET msp)

message("msp_LIBRARIES " ${PKG_MSP_LIBRARIES})
message("msp__INCLUDE_DIRS " ${PKG_MSP_INCLUDE_DIRS})
message("PKG_MSP "  ${PKG_MSP})

include_directories(${PKG_MSP_INCLUDE_DIRS})
link_directories(${PKG_MSP_LIBRARY_DIRS})

add_library(vehicle_gateway_betaflight src/vehicle_gateway_betaflight.cpp)
target_include_directories(vehicle_gateway_betaflight PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  vehicle_gateway_betaflight
  pluginlib
  vehicle_gateway
)

target_link_libraries(vehicle_gateway_betaflight
	${PKG_MSP_LIBRARIES}
)

add_executable(vehicle_gateway_betaflight_node src/vehicle_gateway_betaflight_main.cpp)
target_include_directories(vehicle_gateway_betaflight_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  vehicle_gateway_betaflight_node
  vehicle_gateway
  pluginlib
)

target_link_libraries(vehicle_gateway_betaflight_node
 ${PKG_MSP_LIBRARIES}
)

pluginlib_export_plugin_description_file(vehicle_gateway plugins.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(vehicle_gateway_betaflight PRIVATE "VEHICLE_GATEWAY_BETAFLIGHT_BUILDING_LIBRARY")

install(
  TARGETS vehicle_gateway_betaflight
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS vehicle_gateway_betaflight_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(
  vehicle_gateway_betaflight
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(
  msp
  pluginlib
  rclcpp
  vehicle_gateway
)
ament_environment_hooks(${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH})

ament_export_include_directories(
  include
)
ament_package()
