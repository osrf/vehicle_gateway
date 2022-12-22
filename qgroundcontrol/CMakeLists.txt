cmake_minimum_required(VERSION 3.5)
project(qgroundcontrol)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

file(DOWNLOAD
  https://github.com/mavlink/qgroundcontrol/releases/download/v4.2.4/QGroundControl.AppImage
  QGroundControl.AppImage
)

message("CMAKE_BUILD_DIR " ${CMAKE_BUILD_DIR})
message("CMAKE_BINARY_DIR " ${CMAKE_BINARY_DIR})

install(PROGRAMS
  ${CMAKE_BINARY_DIR}/QGroundControl.AppImage
 TYPE BIN
)

ament_export_libraries(qgroundcontrol)

ament_package()
