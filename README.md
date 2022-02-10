# FogROS2

# Installation(in Docker)
```
FOGROS_REPO=~/Desktop/FogROS2
git clone --recurse-submodules https://github.com/BerkeleyAutomation/FogROS2.git {FOGROS_REPO}
```
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

### In Docker 
First, build our mounted version of `launch`, run
```
cd /opt/ros2_eloquent
colcon build --symlink-install
```

Second, run `aws configure` 
to configure the AWS credentials. 

(on the first terminal), run the FogROS server, 
```
cd /opt/ros2_ws/
colcon build 
. /opt/ros2_ws/install/setup.bash
ros2 run fogros2 fogros2
```

(on the second terminal), run the robotics applications that need to be "FogROS-ed", 
```
ros2 launch fogros2_examples talker.launch.py
```


# Run your own robotics applications 
Step 1: Mount your robotics application to docker's folder. 
For example, 
```
docker run -it --rm \
    --net=host --cap-add=NET_ADMIN \
       ....
    -v FOLDER_IN_YOUR_LOCAL_DIR:/opt/ros2_ws/src/YOUR_PKG_NAME \
       ...
    keplerc/ros2:latest /bin/bash
```
you may also `git clone` your development repo to the docker instead. 


Step 2: Write the FogROSlaunch file
Example of launch file can be found in https://github.com/BerkeleyAutomation/FogROS2/blob/main/examples/fogros2_examples/launch/talker.launch.py. 

Note a few points that are different from normal launch file: 
1. use `FogROSLaunchDescription` instead of `LaunchDescription` class 
2. tag your `Node` with `to_cloud`. FogROS will only push nodes that `to_cloud=True`



fogros2
    - fogros2_examples -> examples using FogROS2
    - fogros2 -> orchestrator of cloud Nodes
    - fogros2_launch -> ROS 2 launch fork for FogROS2 purposes
    - fogros2_launch_ros  -> (ROS specific extensions for FogROS2)