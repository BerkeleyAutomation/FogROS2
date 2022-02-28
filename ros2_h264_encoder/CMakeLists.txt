cmake_minimum_required(VERSION 3.5)
project(ros2_h264_encoder)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(h264_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)

find_package(PkgConfig)
pkg_check_modules(PC_SWSCALE REQUIRED libswscale>=5)
pkg_check_modules(X264 REQUIRED x264)

include_directories(include)

add_library(
  ${PROJECT_NAME} SHARED
  src/h264_publisher.cpp
  src/manifest.cpp
  src/ros2_encoder.cpp
)

add_executable(
  h264_encoder_node
  src/ros2_encoder_handler.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${X264_LIBRARIES}
  ${PC_SWSCALE_LIBRARIES}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  h264_msgs
  image_transport
  rclcpp
  pluginlib
  sensor_msgs
)

ament_target_dependencies(
  h264_encoder_node
  h264_msgs
  image_transport
  rclcpp
  pluginlib
  sensor_msgs
)

target_link_libraries(
  h264_encoder_node
  ${X264_LIBRARIES}
  ${PC_SWSCALE_LIBRARIES}
)

# This will load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ${PROJECT_NAME} h264_encoder_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib                 # Shared libraries must be in lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # Node executables must be in lib/<pkg>
)

install(
  DIRECTORY include
  DESTINATION include
)

pluginlib_export_plugin_description_file(image_transport h264_plugins.xml)

ament_package()
