#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros2_eloquent/install/setup.bash"
exec "$@"
