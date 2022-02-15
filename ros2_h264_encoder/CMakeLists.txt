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
find_package(pluginlib REQUIRED)
find_package(PkgConfig)

pkg_check_modules(PC_SWSCALE REQUIRED libswscale>=5)
pkg_check_modules(X264 REQUIRED x264)

add_executable(ros2_h264_encoder src/ros2_encoder.cpp src/ros2_encoder_handler.cpp src/h264_publisher.cpp)
ament_target_dependencies(ros2_h264_encoder sensor_msgs h264_msgs rclcpp)

target_include_directories(ros2_h264_encoder
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(
  ros2_h264_encoder
  ${PC_SWSCALE_LIBRARIES}
  ${X264_LIBRARIES}
)    

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ros2_h264_encoder
  DESTINATION lib/${PROJECT_NAME})  

ament_package()
