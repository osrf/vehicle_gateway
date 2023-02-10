cmake_minimum_required(VERSION 3.5)
project(betaflight_configurator)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

if(NOT EXISTS "${CMAKE_BINARY_DIR}/betaflight-configurator_10.9.0_linux64-portable.zip")
  file(DOWNLOAD
https://github.com/betaflight/betaflight-configurator/releases/download/10.9.0/betaflight-configurator_10.9.0_linux64-portable.zip
    betaflight-configurator_10.9.0_linux64-portable.zip
    SHOW_PROGRESS
  )
  file(ARCHIVE_EXTRACT
    INPUT ${CMAKE_BINARY_DIR}/betaflight-configurator_10.9.0_linux64-portable.zip
    DESTINATION ${PROJECT_BINARY_DIR}/betaflight-configurator
  )
endif()

install(DIRECTORY
  "${PROJECT_BINARY_DIR}/betaflight-configurator/Betaflight Configurator/"
  DESTINATION
  ${CMAKE_INSTALL_PREFIX}/share/betaflight_configurator/bin
  USE_SOURCE_PERMISSIONS
)

ament_environment_hooks(cmake/99_betaflight_configurator_setup.sh.in)

ament_package()
