cmake_minimum_required(VERSION 3.8)
project(vehicle_gateway)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)

add_executable(vehicle_gateway_node src/vehicle_gateway_node.cpp)
target_include_directories(vehicle_gateway_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(vehicle_gateway_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  vehicle_gateway_node
  "pluginlib"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS vehicle_gateway_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
