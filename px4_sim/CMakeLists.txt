cmake_minimum_required(VERSION 3.5)
project(px4_sim)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

find_program(MAKE_EXECUTABLE NAMES make REQUIRED)

ExternalProject_Add(px4-firmware
  PREFIX
      ${PROJECT_BINARY_DIR}/px4-alex
  GIT_REPOSITORY
    https://github.com/ahcorde/PX4-Autopilot
  GIT_TAG
    ahcorde/support/gazebo_garden
  INSTALL_COMMAND
    mkdir -p ${CMAKE_INSTALL_PREFIX}/share/px4_sim && rsync -avL --exclude .git --exclude build build/px4_sitl_default/bin Tools/simulation/gz/models build/px4_sitl_default/etc ${CMAKE_INSTALL_PREFIX}/share/px4_sim
  BUILD_COMMAND pwd && ls && ${MAKE_EXECUTABLE} px4_ros2 -j && make px4_sitl_default -j
  BUILD_IN_SOURCE ON
  CONFIGURE_COMMAND ""
  PATCH_COMMAND patch -p1 < ${CMAKE_CURRENT_SOURCE_DIR}/patches/GzBridge.cpp.patch && patch -p1 < ${CMAKE_CURRENT_SOURCE_DIR}/patches/GzBridge.hpp.patch
)

install(DIRECTORY
  launch
  worlds
 DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(cmake/99_px4_setup.sh.in)

ament_export_libraries(px4)

ament_package()
