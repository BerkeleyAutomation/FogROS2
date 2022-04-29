ARG DISTRO=rolling
FROM ubuntu:jammy

ARG DISTRO

RUN apt update && apt install -y curl gnupg2 lsb-release sudo && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y ros-rolling-desktop

RUN apt update && sudo apt install -y \
  build-essential \
  cmake \
  python3-colcon-common-extensions \
  python3-pip \
  python3-vcstool \
  ros-${DISTRO}-rmw-cyclonedds-cpp \
  unzip \
  wireguard \
  iproute2 \
  curl \
  net-tools \
  ssh

RUN rm -rf /var/lib/apt/lists/*
 
# install some pip packages needed for testing
RUN python3 -m pip install --no-cache-dir -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-runner 

RUN curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip"

RUN unzip awscliv2.zip && rm awscliv2.zip
RUN ./aws/install
RUN python3 -m pip install --no-cache-dir -U boto3 paramiko scp wgconfig

# Create FogROS2 worspace and build it
ENV ROS_WS=/home/root/fog_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}/src
COPY .  ${ROS_WS}/src/

WORKDIR ${ROS_WS}
RUN . /opt/ros/${DISTRO}/setup.sh && \
      colcon build --merge-install --cmake-clean-cache

# setup entrypoint
ENV ROS_DISTRO=${DISTRO}
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["bash"]
