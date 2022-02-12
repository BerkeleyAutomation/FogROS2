# FogROS2

# Installation(in Docker)
Build it:
```bash
docker build -t fogros2:latest .
```

Run the docker by:
```
FOGROS_REPO=~/Desktop/FogROS2 
docker run -it --rm \
    --net=host --cap-add=NET_ADMIN \
    -v "${FOGROS_REPO}":/home/root/fog_ws/src/fogros2 \
    fogros2 /bin/bash
```

### In Docker 

First, run `aws configure` 
to configure the AWS credentials. 

(on the first terminal), run the FogROS server, 
```
cd /home/root/fog_ws
colcon build --merge-install
. /home/root/fog_ws/install/setup.bash
ros2 run fogros2 fogros2
```

(on the second terminal), run the robotics applications that need to be "FogROS-ed", 
```
. /home/root/fog_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
export CYCLONEDDS_URI=file:///home/root/fog_ws/cyclonedds.xml
ros2 launch fogros2_examples talker.launch.py
```


# Run your own robotics applications 
Step 1: Mount your robotics application to docker's folder. 
For example, 
```
docker run -it --rm \
    --net=host --cap-add=NET_ADMIN \
       ....
    -v FOLDER_IN_YOUR_LOCAL_DIR:/home/root/fog_ws/src/YOUR_PKG_NAME \
       ...
    keplerc/ros2:latest /bin/bash
```
you may also `git clone` your development repo to the docker instead. 


Step 2: Write the FogROSlaunch file
Example of launch file can be found in https://github.com/BerkeleyAutomation/FogROS2/blob/main/examples/fogros2_examples/launch/talker.launch.py. 

Note a few points that are different from normal launch file: 
1. use `FogROSLaunchDescription` instead of `LaunchDescription` class 
2. tag your `Node` with `to_cloud`. FogROS will only push nodes that `to_cloud=True`
