cmake_minimum_required(VERSION 3.5)
project(vehicle_gateway_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
