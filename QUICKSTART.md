Install FogROS 2 and ROS 2 from Scratch
---

This is a quick start guide for installing FogROS 2 (and ROS 2) and its requisites from scratch (e.g., in a VM).  New contributors to the project can start here.

1. Install Ubuntu 20.04 or Ubuntu 22.04. See [here](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) for a tutorial. 

2. Upgrade
```bash
sudo apt update
sudo apt upgrade
```

3. Reboot

4. Get UTF-8 locale installed

```
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

5. Setup sources for ROS 2 (https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html)

```
sudo apt update
sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

6. Install ROS 2 Packages (https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html)

```
sudo apt update
sudo apt install ros-rolling-desktop
```

7. Add env to startup

```
echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc
source /opt/ros/rolling/setup.bash
```

8. Choose and set a `ROS_DOMAIN_ID` (in range 0 to 101) (https://docs.ros.org/en/rolling/Concepts/About-Domain-ID.html)

```
export ROS_DOMAIN_ID=99
echo 'export ROS_DOMAIN_ID=99' >> ~/.bashrc
```
9. Install colcon, git, and h.264 dependencies

```
sudo apt install python3-colcon-common-extensions
sudo apt install git
```

10. Create a workspace

```
mkdir -p ~/fog_ws/src
```

11. Clone

```
cd ~/fog_ws/src
git clone -b humble --recurse-submodules https://github.com/BerkeleyAutomation/FogROS2.git
# if using Ubuntu 20.04
cp FogROS2/fogros2/configs/cyclonedds.ubuntu.2004.xml ../cyclonedds.xml
# if using Ubuntu 22.04
cp FogROS2/fogros2/configs/cyclonedds.ubuntu.2204.xml ../cyclonedds.xml
```

12. Build

```
cd ~/fog_ws
colcon build --merge-install
```

13. Install AWS CLI

```
sudo apt install awscli
```

14. Configure AWS Basic Settings. To run the next command, you need to have your [security credentials, an output format and AWS Region.](https://docs.aws.amazon.com/cli/latest/userguide/cli-configure-quickstart.html)

```
aws configure
```

15. Install additional dependencies

```
sudo apt install build-essential cmake git python3-colcon-common-extensions python3-pip python3-vcstool wget emacs-nox unzip wireguard iproute2 curl net-tools ssh
sudo apt install emacs-gtk
pip3 install boto3 paramiko scp wgconfig unique-names-generator
```

16. If using Ubuntu 22.04

```
sudo apt install ros-rolling-rmw-cyclonedds-cpp
```
   
17. Run basic example

```
cd ~/fog_ws
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
# if using Ubuntu 20.04
export CYCLONEDDS_URI=file://$(pwd)/install/share/fogros2/configs/cyclonedds.ubuntu.2004.xml
# if using Ubuntu 22.04
export CYCLONEDDS_URI=file://$(pwd)/install/share/fogros2/configs/cyclonedds.ubuntu.2204.xml

ros2 launch fogros2_examples talker.aws.launch.py
```

18. You are done. Refer to our [README](https://github.com/BerkeleyAutomation/FogROS2/blob/main/README.md) for additional information including [Command Line Interface commmands](https://github.com/BerkeleyAutomation/FogROS2#command-line-interface) and [Docker installation](https://github.com/BerkeleyAutomation/FogROS2#docker).
