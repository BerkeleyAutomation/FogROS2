ARG DISTRO=rolling
FROM ros:$DISTRO

ARG DISTRO

RUN apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python3-vcstool \
  wget \
  emacs-nox \
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
  pytest-runner \
  setuptools \
  opencv-python

RUN curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip"
# RUN curl "https://awscli.amazonaws.com/awscli-exe-linux-aarch64.zip" -o "awscliv2.zip"

RUN unzip awscliv2.zip && rm awscliv2.zip
RUN ./aws/install
RUN pip3 install boto3 paramiko scp wgconfig

# Create FogROS2 worspace and build it
RUN mkdir -p /home/root/fog_ws/src
WORKDIR /home/root/fog_ws/src
COPY .  /home/root/fog_ws/src/fogros2
COPY ./fogros2/configs/cyclonedds.xml /home/root/fog_ws

WORKDIR /home/root/fog_ws
RUN . /opt/ros/$DISTRO/setup.sh && \
      colcon build --merge-install --cmake-clean-cache

CMD ["bash"]
