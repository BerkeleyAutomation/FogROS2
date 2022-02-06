
ARG FROM_IMAGE=osrf/ros2:devel
FROM $FROM_IMAGE

RUN apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python3-vcstool \
  wget
# install some pip packages needed for testing
RUN python3 -m pip install -U \
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
  setuptools
# install Fast-RTPS dependencies
RUN apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
RUN apt install --no-install-recommends -y \
  libcunit1-dev

RUN apt update & apt install -y wget emacs-nox
RUN mkdir -p /opt/ros2_eloquent/src
WORKDIR /opt/ros2_eloquent
RUN  wget https://raw.githubusercontent.com/ros2/ros2/eloquent/ros2.repos
WORKDIR /opt/ros2_eloquent/src
RUN vcs import . < ../ros2.repos

WORKDIR /opt/ros2_eloquent
# RUN rosdep init
RUN rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro eloquent -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"
RUN colcon build --symlink-install

COPY ./ros_entrypoint.sh /

RUN apt install -y unzip
RUN curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip"
RUN unzip awscliv2.zip
RUN ./aws/install
RUN pip3 install boto3 paramiko scp

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]