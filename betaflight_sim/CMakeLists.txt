cmake_minimum_required(VERSION 3.5)
project(betaflight_sim)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

find_program(MAKE_EXECUTABLE NAMES make REQUIRED)

message(cp ./obj/main/betaflight_SITL.elf ${CMAKE_INSTALL_PREFIX}/share/betaflight_sim/bin)

externalproject_add(betaflight-firmware
  PREFIX
      ${PROJECT_BINARY_DIR}/betaflight_sim
  GIT_REPOSITORY
    https://github.com/nikhil-sethi/betaflight
  GIT_TAG
    airsim-sitl
  INSTALL_COMMAND
    mkdir -p ${CMAKE_INSTALL_PREFIX}/share/betaflight_sim/bin/ &&
    cp ./obj/main/betaflight_SITL.elf ${CMAKE_INSTALL_PREFIX}/share/betaflight_sim/bin/
  BUILD_COMMAND pwd && ${MAKE_EXECUTABLE} TARGET=SITL -j
  BUILD_IN_SOURCE ON
  CONFIGURE_COMMAND ""
  PATCH_COMMAND git checkout . &&
  patch -p1 < ${CMAKE_CURRENT_SOURCE_DIR}/patches/target.hh.patch &&
  patch -p1 < ${CMAKE_CURRENT_SOURCE_DIR}/patches/udp.cc.patch &&
    patch -p1 < ${CMAKE_CURRENT_SOURCE_DIR}/patches/tools.mk.patch
)

ament_environment_hooks(cmake/99_betaflight_setup.sh.in)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  worlds
  DESTINATION
    share/${PROJECT_NAME}
)

install(DIRECTORY
  models
  config
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/share/betaflight_sim
)

ament_package()
