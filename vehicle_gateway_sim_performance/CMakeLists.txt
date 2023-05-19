cmake_minimum_required(VERSION 3.8)
project(vehicle_gateway_sim_performance)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vehicle_gateway_px4 REQUIRED)

#============================================================================
# Find gz-cmake
#============================================================================
# If you get an error at this line, you need to install gz-cmake
find_package(gz-cmake3 REQUIRED)

# Find the Gazebo transport library
#--------------------------------------
# Find gz-transport
gz_find_package(gz-transport12 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

add_executable(system_metric_collector
  src/main.cpp
  src/linux_cpu_system_measurement.cpp
  src/linux_memory_system_measurement.cpp
)
target_include_directories(system_metric_collector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(system_metric_collector
  gz-transport${GZ_TRANSPORT_VER}::core
)

ament_export_include_directories(
  include
)

add_executable(vtol_position_control
  src/vtol_position_control.cpp
)
ament_target_dependencies(vtol_position_control
  rclcpp
  vehicle_gateway_px4
)

install(
  TARGETS
    system_metric_collector
    vtol_position_control
  EXPORT
    export_${PROJECT_NAME}
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)

  add_subdirectory(test)
endif()

ament_package()
