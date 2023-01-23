cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(betaflight_gazebo)

#------------------------------------------------------------------------
# Compile as C++17

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#============================================================================
# Find gz-cmake
#============================================================================
find_package(gz-cmake3 3.0.0 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

#============================================================================
# Search for project-specific dependencies
#============================================================================

#--------------------------------------
# Find gz-sim
gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

gz_find_package(gz-plugin2 REQUIRED)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-math7 REQUIRED)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

gz_find_package(gz-transport12 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

#======================================
# Build plugin

add_library(BetaflightPlugin
  SHARED
  src/BetaflightPlugin.cpp
  src/BetaflightSocket.cpp
  src/Util.cpp
)
target_include_directories(BetaflightPlugin PUBLIC
  include
)
target_link_libraries(BetaflightPlugin PUBLIC
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::register
  gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)

# Install directories
install(TARGETS BetaflightPlugin
  DESTINATION lib
)
