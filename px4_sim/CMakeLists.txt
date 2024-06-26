cmake_minimum_required(VERSION 3.22)
project(px4_sim)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

find_program(MAKE_EXECUTABLE NAMES make REQUIRED)

externalproject_add(px4-firmware
  PREFIX
      ${PROJECT_BINARY_DIR}/px4-src
  GIT_REPOSITORY
    https://github.com/PX4/PX4-Autopilot
  GIT_TAG
    3cc940cb06f233a22a8d1b948bc05f067491e212
  INSTALL_COMMAND
    mkdir -p ${CMAKE_INSTALL_PREFIX}/share/px4_sim &&
    rsync -avL --exclude .git --exclude build
      build/px4_sitl_default/bin
      Tools/simulation/gz/models
      build/px4_sitl_default/etc
      ${CMAKE_INSTALL_PREFIX}/share/px4_sim &&
    cp ${CMAKE_CURRENT_LIST_DIR}/dds_topics.yaml
      ${PROJECT_BINARY_DIR}/px4-src/src/px4-firmware/src/modules/uxrce_dds_client/dds_topics.yaml
  BUILD_COMMAND ${MAKE_EXECUTABLE} px4_ros2 -j && make px4_sitl_default -j
  BUILD_IN_SOURCE ON
  CONFIGURE_COMMAND ""
)

install(DIRECTORY
  launch
  DESTINATION
    share/${PROJECT_NAME}
)

ament_environment_hooks(cmake/99_px4_setup.sh.in)

ament_export_libraries(px4)

ament_package()
