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
      - [Install Dependencies](#install-dependencies)
    - [Docker](#docker)
  - [Launch ROS 2 computational graphs in the cloud](#launch-ros-2-computational-graphs-in-the-cloud)
    - [Native](#native)
    - [Docker (Recommended)](#docker-recommended)
  - [Run your own robotics applications](#run-your-own-robotics-applications)
  - [Setting Up Automatic Image Transport](#setting-up-automatic-image-transport)
  - [Command Line Interface](#command-line-interface)
  - [Developer](#developer)

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
sudo apt install python3-pip wireguard unzip
sudo pip3 install wgconfig boto3 paramiko scp 

# install AWS CLI
curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip"
unzip awscliv2.zip
sudo ./aws/install
```

```bash
cd <your-ros2-workspace>/src
git clone --recurse-submodules https://github.com/BerkeleyAutomation/FogROS2
cd ../
colcon build --merge-install  # re-build the workspace
source install/setup.bash
```

### Docker
Alternatively, if your dev. environment is not ready to natively build ROS 2 packages, you can use the Dockerized dev. environment we've put together as follows:
```bash
git clone --recurse-submodules https://github.com/BerkeleyAutomation/FogROS2
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

## Setting Up Automatic Image Transport 
Step 1: Identify all topics that need to use a compressed transport.

Step 2: In a `fogros2.CloudNode`, add the parameter `stream_topics=[]`, where `stream_topics` is a list of tuples 
where each tuple is just a pair of `(TOPIC_NAME, TRANSPORT_TYPE)` values. 

`TOPIC_NAME` is the string that represents the name of the topic that publishes `sensor_msgs/Image`

Valid `TRANSPORT_TYPE` values are `compressed`, `theora`, and `raw` if only `iumage-transport` and `image-transport-plugins` are installed on the system. `h264` is another valid `TRANSPORT_TYPE` if step 3 is followed.

Optional Step 3: If using H.264, please also clone the H.264 decoder found [here](https://github.com/clydemcqueen/h264_image_transport) into the workspace's src directory. The current repo only contains the encoder and the full image transport pipeline will not work without the decoder also. 

Example of `stream_topics` argument:

`stream_topics=[('/camera/image_raw', 'h264'), ('/camera2/image_raw', 'compressed')]`

Adding the above argument to a `fogros2.CloudNode` makes the topic `/camera/image_raw` publish using the `h264 image_transport`, and makes the topic `/camera2/image_raw` publish using the `compressed image_transport`.

Please note that all cloud nodes that are expecting raw images will be remapped to `TOPIC_NAME/cloud` to remove any topic naming conflicts. (TODO: Automate remapping)

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
```


## Running Examples: 

#### To run gqcnn
```
ros2 launch fogros2_examples gqcnn_docker.launch.py
```
and run gqcnn's client: 
```
docker run --net=host --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --env CYCLONEDDS_URI=file:///tmp/cyclonedds.xml -v $(pwd)/install/share/fogros2/configs/cyclonedds.xml:/tmp/cyclonedds.xml --rm -it keplerc/gqcnn_ros:pj ros2 launch gqcnn_ros client.launch.py
```
in ros workspace. 

#### To run vslam
```
ros2 launch fogros2_examples vslam.launch.py
```
and run vslam's client: 
```
docker run --net=host --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --env CYCLONEDDS_URI=file:///tmp/cyclonedds.xml -v $(pwd)/install/share/fogros2/configs/cyclonedds.xml:/tmp/cyclonedds.xml --rm -it -v /home/gdpmobile7/rgbd_dataset_freiburg1_xyz:/dataset -v $(pwd)/output:/output mjd3/orbslam-ros ros2 launch orb_slam2_ros orb_slam2_d435_rgbd_client_launch.py dataset:=/dataset compress:=0
```
in ros workspace. 

#### TODO
we mark as a TODO item to streamline the launching process of the client docker. 




