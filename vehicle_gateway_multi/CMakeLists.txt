cmake_minimum_required(VERSION 3.5)

project(vehicle_gateway_multi)

#------------------------------------------------------------------------
# Compile as C++17

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(ament_cmake)
find_package(px4_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(zenohc REQUIRED)

add_executable(vehicle_gateway_multi_bridge
  src/vehicle_gateway_multi_bridge.cpp
)

ament_target_dependencies(vehicle_gateway_multi_bridge
  px4_msgs
  nlohmann_json
  rclcpp
  std_msgs
  yaml_cpp_vendor
)

target_link_libraries(vehicle_gateway_multi_bridge
  zenohc::lib
)

add_executable(vehicle_gateway_multi_bridge_client
  src/vehicle_gateway_multi_bridge_client.cpp
)
target_link_libraries(vehicle_gateway_multi_bridge_client
  zenohc::lib
)

install(DIRECTORY
  config
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  TARGETS
    vehicle_gateway_multi_bridge
    vehicle_gateway_multi_bridge_client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
