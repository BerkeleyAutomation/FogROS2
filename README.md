# FogROS2
Run the docker by 
```
FOGROS_EXAMPLES=~/Desktop/FogROS2/examples
docker run -it --rm \
    -v "${FOGROS_EXAMPLES}":/opt/ros2_ws/src/fogros2_examples \
    keplerc/ros2:latest /bin/bash
```
then run the test examples
```
cd /opt/ros2_ws/src/
colcon build --allow-overriding fogros2_examples
. /opt/ros2_ws/install/setup.bash
ros2 launch fogros2_examples talker.launch.py
```