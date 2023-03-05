cmake_minimum_required(VERSION 3.5)
project(vehicle_gateway_integration_test)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)

  add_subdirectory(test)
endif()

ament_package()
