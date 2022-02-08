# FogROS2
Run the docker by 
```
FOGROS_REPO=~/Desktop/FogROS2
docker run -it --rm \
    --net=host --cap-add=NET_ADMIN \
    -v "${FOGROS_REPO}/examples":/opt/ros2_ws/src/fogros2_examples \
    -v "${FOGROS_REPO}/fogros2":/opt/ros2_ws/src/fogros2 \
    -v "${FOGROS_REPO}/launch":/opt/ros2_eloquent/src/ros2/launch \
    -v "${FOGROS_REPO}/launch_ros":/opt/ros2_eloquent/src/ros2/launch_ros \
    keplerc/ros2:latest /bin/bash
```


To build mounted version of `launch`, run
```
cd /opt/ros2_eloquent
colcon build --symlink-install
```

to run aws related functionalities, make sure to use `aws configure`
to configure the credentials. 

To run the test examples, 
```
cd /opt/ros2_ws/
colcon build 
. /opt/ros2_ws/install/setup.bash
ros2 run fogros2 fogros2
```
Note that I haven't connected the `launch` with the existing 
FogROS2 launching infrastructure. To update the 
node that is sent to the cloud, run
```
ros2 launch fogros2_examples talker.launch.py
```