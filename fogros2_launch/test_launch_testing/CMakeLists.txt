cmake_minimum_required(VERSION 3.5)

project(test_launch_testing)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

ament_export_dependencies(ament_cmake)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_executable(locking test/dummy_tests/locking.cpp)
  ament_add_gtest_executable(dummy test/dummy_tests/dummy.cpp)

  # pytest scripts are to be found in the source tree.
  ament_add_pytest_test(test_pytest_py "test/test_launch_testing/actions/test_pytest.py"
    PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test/dummy_tests"
  )

  # gtest binaries are to be found in the build tree.
  ament_add_pytest_test(test_gtest_py "test/test_launch_testing/actions/test_gtest.py"
    PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  )
endif()

ament_package()
