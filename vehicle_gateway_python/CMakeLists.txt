cmake_minimum_required(VERSION 3.5)

project(vehicle_gateway_python)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Figure out Python3 debug/release before anything else can find_package it
if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  find_package(python_cmake_module REQUIRED)
  find_package(PythonExtra REQUIRED)

  # Force FindPython3 to use the debug interpreter where ROS 2 expects it
  set(Python3_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pluginlib REQUIRED)
find_package(vehicle_gateway REQUIRED)
find_package(vehicle_gateway_px4 REQUIRED)

# Find python before pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

ament_python_install_package(vehicle_gateway)

# Set the build location and install location for a CPython extension
function(configure_build_install_location _library_name)
  # Install library for actual use
  install(TARGETS ${_library_name}
    DESTINATION "${PYTHON_INSTALL_DIR}/vehicle_gateway"
  )
endfunction()

# Split from main extension and converted to pybind11
pybind11_add_module(_vehicle_gateway_pybind11 SHARED
  src/vehicle_gateway_python/_vehicle_gateway_pybind11.cpp
  src/vehicle_gateway_python/destroyable.cpp
  src/vehicle_gateway_python/vehicle_gateway.cpp
)

target_link_libraries(_vehicle_gateway_pybind11 PRIVATE
  pluginlib::pluginlib
  vehicle_gateway::vehicle_gateway
  vehicle_gateway_px4::vehicle_gateway_px4
)

configure_build_install_location(_vehicle_gateway_pybind11)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
