#!/bin/bash
set -e

# setup ros2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
source $ROS_WS/install/setup.bash

# work with CycloneDDS DDS implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(pwd)/install/share/fogros2/configs/cyclonedds.xml

exec "$@"