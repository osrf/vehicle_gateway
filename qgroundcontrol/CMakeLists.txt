cmake_minimum_required(VERSION 3.5)
project(qgroundcontrol)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

if(NOT EXISTS "${CMAKE_BINARY_DIR}/QGroundControl.AppImage")
  file(DOWNLOAD
    https://github.com/mavlink/qgroundcontrol/releases/download/v4.2.4/QGroundControl.AppImage
    QGroundControl.AppImage
  )
endif()

install(PROGRAMS
  ${CMAKE_BINARY_DIR}/QGroundControl.AppImage
  TYPE BIN
)

ament_package()
