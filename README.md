# FogROS2

[`FogROS2`](https://github.com/BerkeleyAutomation/FogROS2) extends ROS 2 for cloud deployment of computational graphs in a security-conscious manner. It allows researchers to easily and securely deploy ROS abstractions across cloud providers with minimal effort, thus gaining access to additional computing substrates including CPU cores, GPUs, FPGAs, or TPUs, as well as pre-deployed software made available by other researchers. To do so, `FogROS2` extends the ROS 2 launch system, introducing additional syntax to allow roboticists to specify at launch time which components of their architecture will be deployed to the cloud and which components will be deployed on the edge.


- [FogROS2](#fogros2)
  - [Install](#install)
    - [Quickstart](#quickstart)
    - [Docker (Recommended)](#docker-recommended)
    - [Natively](#natively)
      - [Install Dependencies](#install-dependencies)
  - [Launch ROS 2 computational graphs in the cloud](#launch-ros-2-computational-graphs-in-the-cloud)
    - [Docker (Recommended)](#docker-recommended-1)
    - [Native](#native)
  - [Run your own robotics applications](#run-your-own-robotics-applications)
  - [Setting Up Automatic Image Transport](#setting-up-automatic-image-transport)
  - [Command Line Interface](#command-line-interface)
  - [Some Common Issues](#some-common-issues)
  - [Running Examples:](#running-examples)

## Install
### Quickstart
If you are new to ROS and Ubuntu, and want to install FogROS 2 (and ROS 2) and its requisites from scratch, follow instructions [here](https://github.com/BerkeleyAutomation/FogROS2/blob/humble/QUICKSTART.md).
### Docker (Recommended)
Alternatively, you can simplify reproduction using an OS virtualization environment with Docker. You can also watch our video tutorial [here](https://www.youtube.com/embed/oEnmZXojkcI?start=1&end=800). 
```bash
git clone -b humble https://github.com/BerkeleyAutomation/FogROS2
cd FogROS2

# Install AWS CLI
sudo apt install awscli

# Configure AWS Basic Settings. To run the next command, you need to have your security credentials, an output format and AWS Region. See https://docs.aws.amazon.com/cli/latest/userguide/cli-configure-quickstart.html
aws configure

#Build Docker Image
docker build -t fogros2 .
```
By default, this command will build a docker image for ROS Rolling and Ubuntu 22.04 (jammy). These defaults can be changed using the `--build-arg` flag (e.g., `docker build -t fogros2:focal-humble . --build-arg UBUNTU_DISTRO=focal --build-arg ROS_DISTRO=humble` will build a ROS Humble image with Ubuntu 20.04 (focal)).
*Note: the Dockerfile is cooked for x86_64. If you're using a workstation with an Arm-based architecture (e.g. an M1), build the container with the `docker build --platform linux/amd64 -t fogros2 .`*.

### Natively
`FogROS2` is actually a ROS meta-package, so you can just fetch it in your workspace, build it, source the workspace as an overlay and start using its capabilities. You can also watch our video tutorial [here](https://www.youtube.com/embed/JlV4DhArb8Q?start=1&end=402). 

#### Install Dependencies

ROS 2 dependencies:
```
# If using Ubuntu 22.04
sudo apt install ros-rolling-rmw-cyclonedds-cpp
```

FogROS 2 dependencies:
```
sudo apt install python3-pip wireguard unzip
sudo pip3 install wgconfig boto3 paramiko scp

# Install AWS CLI
sudo apt install awscli

# Configure AWS Basic Settings. To run the next command, you need to have your security credentials, an output format and AWS Region. See https://docs.aws.amazon.com/cli/latest/userguide/cli-configure-quickstart.html
aws configure
```

```bash
source /opt/ros/<your-ros2-distro>/setup.bash
mkdir -p ~/fog_ws/src
cd ~/fog_ws/src
git clone -b humble https://github.com/BerkeleyAutomation/FogROS2
cd ../
colcon build  # re-build the workspace
source install/setup.bash
```


## Launch ROS 2 computational graphs in the cloud

### Docker (Recommended)
You can see this in our video tutorial [here](https://www.youtube.com/embed/oEnmZXojkcI?start=801)

```bash
# launch fogros2 container
docker run -it --rm --net=host -v $HOME/.aws:/root/.aws --cap-add=NET_ADMIN fogros2

# launch talker node on the cloud
ros2 launch fogros2_examples talker.aws.launch.py
```

(*Note: the Dockerfile is cooked for x86_64. If you're using a workstation with an Arm-based architecture (e.g. an M1), run the container with the `docker run -it --platform linux/amd64 --rm --net=host --cap-add=NET_ADMIN fogros2`*.)

### Native
Note: These commands must be run from the root of your ROS workspace. You can see this in our video tutorial [here.](https://www.youtube.com/embed/JlV4DhArb8Q?start=403)
```bash
source /opt/ros/<your-ros2-distro>/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
export CYCLONEDDS_URI=file://$(pwd)/install/fogros2/share/fogros2/configs/cyclonedds.ubuntu.$(lsb_release -rs | sed 's/\.//').xml

ros2 launch fogros2_examples talker.aws.launch.py
```

## Run your own robotics applications
If using, for example, Docker take the following steps: 

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
You may also `git clone` your development repo to the docker container instead.


Step 2: Write the FogROS2 launch file. Examples of launch files can be found in the talker*.launch.py [here](https://github.com/BerkeleyAutomation/FogROS2/tree/humble/fogros2_examples/launch).


## Setting Up Automatic Image Transport
Step 1: Identify all topics that need to use a compressed transport.

Step 2: In a `fogros2.CloudNode`, add the parameter `stream_topics=[]`, where `stream_topics` is a list of tuples
where each tuple is just a pair of `(TOPIC_NAME, TRANSPORT_TYPE)` values.

`TOPIC_NAME` is the string that represents the name of the topic that publishes `sensor_msgs/Image`

Valid `TRANSPORT_TYPE` values are `compressed`, `theora`, and `raw` if only `image-transport` and `image-transport-plugins` are installed on the system. `h264` is another valid `TRANSPORT_TYPE` if step 3 is followed.

Step 3 (Optional): If using H.264, please also clone the H.264 decoder found [here](https://github.com/clydemcqueen/h264_image_transport) into the workspace's src directory. The current repo only contains the encoder and the full image transport pipeline will not work without the decoder also.

Example of `stream_topics` argument:

`stream_topics=[('/camera/image_raw', 'h264'), ('/camera2/image_raw', 'compressed')]`

Adding the above argument to a `fogros2.CloudNode` makes the topic `/camera/image_raw` publish using the `h264 image_transport`, and makes the topic `/camera2/image_raw` publish using the `compressed image_transport`.

Please note that all cloud nodes that are expecting raw images will be remapped to `TOPIC_NAME/cloud` to remove any topic naming conflicts. (TODO: Automate remapping)

## Command Line Interface
We currently support the following CLI commands for easier debugging and development.

```bash
# List existing FogROS instances
ros2 fog list

# Connect via SSH to the corresponding instance (e.g., named "ascent-corona")
# the instance name can be found by the list command above
ros2 fog connect ascent-corona

# delete the existing FogROS instance (e.g. named "ascent-corona")
ros2 fog delete ascent-corona
# or all of the existing instances
ros2 fog delete all
```

## Some Common Issues
1. Warning: _2 packages has stderr outputs: fogros2 fogros2_examples_ after running colcon build. This warning occurs in Ubuntu 22.04 (jammy) builds, but does not affect functionality. See https://github.com/BerkeleyAutomation/FogROS2/issues/45. Your installation should still work.  
2. _[WARN] [1652044293.921367226] [fogros2.scp]: [Errno None] Unable to connect to port 22 on xx.xx.xx.xxx, retrying..._ . This warning occurs when AWS has not yet started the instance. This message should eventually be replaced by _SCP Connected!_ once the instance is started.
3. _WARNING: Running pip as the 'root' user can result in broken permissions and conflicting behavior with the system package manager. It is recommended to use a virtual environment instead: https://pip.pypa.io/warnings/venv_. This warning is often seen when installing packages on the cloud instance, but can be ignored.

## Running Examples:
We have used FogROS for 3 example use-cases (motion planning, grasp planning, and SLAM map building). Please see our [examples repo](https://github.com/BerkeleyAutomation/fogros2-examples) for these and how to run them.
