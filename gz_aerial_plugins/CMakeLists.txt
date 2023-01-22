cmake_minimum_required(VERSION 3.8)
project(gz_aerial_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(CMAKE_AUTOMOC ON)
set(OpenGL_GL_PREFERENCE LEGACY)

# Find Qt5
find_package(Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
)

#============================================================================
# Find gz-cmake
#============================================================================
# If you get an error at this line, you need to install gz-cmake
find_package(gz-cmake3 REQUIRED)

# Find the Ignition gui library
#--------------------------------------
# Find gz-gui
gz_find_package(gz-gui7 REQUIRED)
set(GZ_GUI_VER ${gz-gui7_VERSION_MAJOR})

#--------------------------------------
# Find gz-common
# Always use the profiler component to get the headers, regardless of status.
gz_find_package(gz-common5
  REQUIRED
)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

qt5_add_resources(resources_rcc src/DroneHmi.qrc)

include_directories(SYSTEM
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
  ${Qt5QuickControls2_INCLUDE_DIRS}
)

# Generate examples
add_library(DroneHmi SHARED ${headers_MOC}
  src/DroneHmi.cpp
  ${resources_rcc}
)
target_link_libraries(DroneHmi
  gz-gui7::gz-gui7
  gz-common5::gz-common5
  ${Qt5Core_LIBRARIES}
  ${Qt5Qml_LIBRARIES}
  ${Qt5Quick_LIBRARIES}
  ${Qt5QuickControls2_LIBRARIES}
)

install(
  TARGETS DroneHmi
  DESTINATION share/${PROJECT_NAME}/lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
