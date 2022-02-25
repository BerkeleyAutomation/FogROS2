# FogROS2

[`FogROS2`](https://github.com/BerkeleyAutomation/FogROS2) extends ROS 2 for the cloud deployment of computational graphs in a security-conscius manner. It allows researchers to easily deploy ROS abstractions across cloud providers with minimal effort, security and correspondingly, securly gain access to additional computing substrates including CPU cores, GPUs, FPGAs or TPUs, as well as predeployed software made available by other researchers. To do so, `FogROS2` extends the ROS 2 launch system introducing additional syntax to allow roboticists to specify at launch time which components of their architecture will be deployed to the cloud and which ones on the edge.

If you find this useful, please cite our work:

```
TODO:introduce citation in here before going public
```


- [FogROS2](#fogros2)
  - [Architecture](#architecture)
  - [Install](#install)
    - [Natively](#natively)
    - [Docker](#docker)
  - [Launch ROS 2 computational graphs in the cloud](#launch-ros-2-computational-graphs-in-the-cloud)
    - [Native](#native)
    - [Docker](#docker-1)
  - [Run your own robotics applications](#run-your-own-robotics-applications)

## Architecture
(TODO: describe in here the architecture of FogROS)

fogros2
    - fogros2_examples -> examples using FogROS2
    - fogros2 -> orchestrator of cloud Nodes
    - fogros2_launch -> ROS 2 launch fork for FogROS2 purposes
    - fogros2_launch_ros  -> (ROS specific extensions for FogROS2)


## Install
### Natively
`FogROS2` is actually a ROS meta-package, so you can just fetch it in your favourite workspace, build it, source the workspace as an overlay and start using its capabilities.

#### Install Dependencies

Install ROS by 
```
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-rolling-desktop


sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y python3-colcon-common-extensions
```

Install FogROS dependencies by
```
sudo apt install python3-pip wireguard unzip awscli
sudo pip3 install wgconfig boto3 paramiko scp awscli
```

```bash
cd <your-ros2-workspace>/src
git clone https://github.com/BerkeleyAutomation/FogROS2
cd ../
colcon build --merge-install  # re-build the workspace
source install/setup.bash
```

### Docker
Alternatively, if your dev. environment is not ready to natively build ROS 2 packages, you can use the Dockerized dev. environment we've put together as follows:
```bash
git clone https://github.com/BerkeleyAutomation/FogROS2
cd FogROS2
docker build -t fogros2:latest .
```
then, you can run the docker container:
```bash
docker run -it fogros2
# FOGROS_REPO=~/Desktop/FogROS2 
# docker run -it --rm \
#    --net=host --cap-add=NET_ADMIN \
#    -v "${FOGROS_REPO}":/home/root/fog_ws/src/fogros2 \
#    fogros2 /bin/bash
```

## Launch ROS 2 computational graphs in the cloud
TODO: replace this with fogros2 tooling that's cloud-agnostic. E.g. `ros2 fog configure --aws`, instead of `fogros2`.

### Native 
```bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
export CYCLONEDDS_URI=file://$(pwd)/install/share/fogros2/configs/cyclonedds.xml
ros2 launch fogros2_examples talker.launch.py
```

### Docker (Recommended)

First, run `aws configure` and configure the AWS credentials. 

Second, run the robotics applications that need to be "FogROS-ed",
```bash
# connect to running container
docker run -it --rm --net=host --cap-add=NET_ADMIN fogros2
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
export CYCLONEDDS_URI=file://$(pwd)/install/share/fogros2/configs/cyclonedds.xml
ros2 launch fogros2_examples talker.launch.py
```


## Run your own robotics applications 
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


## Command Line Interface
We currently support the following CLIs for easier debugging and development. 

```bash
# list the existing FogROS instances 
ros2 fog list

# SSH to the corresponding instance 
# the -n name can be found by the above list command 
ros2 fog connect -n 368

# delete the existing FogROS instance 
ros2 fog delete -n 368 
# or all of the existing instances 
ros2 fog delete -a
```

## Developer

Here are several commands that one may find it useful when developing: 
```bash

# starting the second terminal for fogros docker
docker exec -it $(docker ps | grep fogros2 | awk '{print $1}') /bin/bash

colcon build --merge-install  && ros2 launch fogros2_examples gqcnn_docker.launch.py
```