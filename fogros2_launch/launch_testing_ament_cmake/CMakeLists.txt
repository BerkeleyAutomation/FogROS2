cmake_minimum_required(VERSION 3.5)

project(launch_testing_ament_cmake)

find_package(ament_cmake REQUIRED)

ament_package(
  CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()

  include(cmake/add_launch_test.cmake)

  ament_index_has_resource(LAUNCH_TESTING_INSTALL_PREFIX packages launch_testing)
  if(NOT LAUNCH_TESTING_INSTALL_PREFIX)
      message(FATAL_ERROR "launch_testing package not found")
  endif()

  # Provides PYTHON_EXECUTABLE_DEBUG
  find_package(python_cmake_module REQUIRED)
  find_package(PythonExtra REQUIRED)

  # Test argument passing.  This test won't pass unless you give it an argument
  add_launch_test(
    "${LAUNCH_TESTING_INSTALL_PREFIX}/share/launch_testing/examples/args_launch_test.py"
    ARGS "dut_arg:=--anything"
  )
endif()
