#!/bin/bash
set -e

# setup ros2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
source $ROS_WS/install/setup.bash

# work with CycloneDDS DDS implementation
ver=$(lsb_release -a 2>&1 | awk 'FNR==4 {gsub(/\./, ""); print $2}')
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(pwd)/install/share/fogros2/configs/cyclonedds.ubuntu.$ver.xml

exec "$@"