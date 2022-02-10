# Copyright 2019 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#  This file contains modified code from the following open source projects
#  published under the licenses listed below:

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


macro(parse_launch_test_arguments namespace filename)
  cmake_parse_arguments(${namespace}
    ""
    "TARGET;TIMEOUT;PYTHON_EXECUTABLE"
    "ARGS;LABELS"
    ${ARGN})

  if(NOT ${namespace}_TIMEOUT)
    set(${namespace}_TIMEOUT 60)
  endif()

  if(NOT ${namespace}_PYTHON_EXECUTABLE)
    set(${namespace}_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
    if(WIN32)
      if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(${namespace}_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
      endif()
    endif()
  endif()

  set(${namespace}_FILE_NAME NOTFOUND)
  if(IS_ABSOLUTE ${filename})
    set(${namespace}_FILE_NAME ${filename})
  else()
    find_file(${namespace}_FILE_NAME ${filename}
              PATHS ${CMAKE_CURRENT_SOURCE_DIR}
              NO_DEFAULT_PATH
              NO_CMAKE_FIND_ROOT_PATH)
    if(NOT ${namespace}_FILE_NAME)
      message(FATAL_ERROR "Can't find launch test file \"${filename}\"")
    endif()
  endif()

  if (NOT ${namespace}_TARGET)
    # strip PROJECT_SOURCE_DIR and PROJECT_BINARY_DIR from absolute filename to get unique test name (as rostest does it internally)
    set(${namespace}_TARGET ${${namespace}_FILE_NAME})
    rostest__strip_prefix(${namespace}_TARGET "${PROJECT_SOURCE_DIR}/")
    rostest__strip_prefix(${namespace}_TARGET "${PROJECT_BINARY_DIR}/")
    string(REPLACE "/" "_" ${namespace}_TARGET ${${namespace}_TARGET})
  endif()

  if(NOT ${namespace}_LABELS)
    set(${namespace}_LABELS "launch_test")
  endif()

  set(${namespace}_RESULT_FILE "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${${namespace}_TARGET}.xunit.xml")
endmacro()


#
# Add a launch test
#
# :param filename: The launch test file containing the test to run
# :type filename: string
# :param TARGET: The test target name
# :type TARGET: string
# :param PYTHON_EXECUTABLE: The python executable to use for the test
# :type PYTHON_EXECUTABLE: string
# :param TIMEOUT: The test timeout in seconds
# :type TIMEOUT: integer
# :param LABELS: The test labels
# :type LABELS: string
# :param ARGS: Launch arguments to pass to the launch test
# :type ARGS: string
function(add_launch_test filename)
  # Convert filename to CMake path before calling macro to
  # avoid problems with backslashes in the filename string.
  file(TO_CMAKE_PATH "${filename}" filename)
  parse_launch_test_arguments(_launch_test ${filename} ${ARGN})

  set(cmd
    "${_launch_test_PYTHON_EXECUTABLE}"
    "-m"
    "launch_testing.launch_test"
    "${_launch_test_FILE_NAME}"
    "${_launch_test_ARGS}"
    "--junit-xml=${_launch_test_RESULT_FILE}"
    "--package-name=${PROJECT_NAME}"
  )

  ament_add_test(
    "${_launch_test_TARGET}"
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/launch_test/${_launch_test_TARGET}.txt"
    RESULT_FILE "${_launch_test_RESULT_FILE}"
    TIMEOUT "${_launch_test_TIMEOUT}"
    ${_launch_test_UNPARSED_ARGUMENTS}
  )
  set_tests_properties(
    "${_launch_test_TARGET}"
    PROPERTIES
    LABELS "${_launch_test_LABELS}"
  )
endfunction()

macro(rostest__strip_prefix var prefix)
  string(LENGTH ${prefix} prefix_length)
  string(LENGTH ${${var}} var_length)
  if(${var_length} GREATER ${prefix_length})
    string(SUBSTRING "${${var}}" 0 ${prefix_length} var_prefix)
    if("${var_prefix}" STREQUAL "${prefix}")
      # passing length -1 does not work for CMake < 2.8.5
      # http://public.kitware.com/Bug/view.php?id=10740
      string(LENGTH "${${var}}" _rest)
      math(EXPR _rest "${_rest} - ${prefix_length}")
      string(SUBSTRING "${${var}}" ${prefix_length} ${_rest} ${var})
    endif()
  endif()
endmacro()
