FROM ros:humble

# TODO: every ssh server using this will have the same key :rofl:
RUN apt update && apt install -y vim software-properties-common gnupg lsb-release locales ros-humble-rmw-cyclonedds-cpp openssh-server sudo curl python3-colcon-common-extensions wireguard unzip python3-pip iproute2

# Shouldn't be needed but oh well...
RUN python3 -m pip install boto3 paramiko scp wgconfig kubernetes

RUN useradd 'ubuntu' -m -s /bin/bash && mkdir '/home/ubuntu/.ssh' && echo 'ubuntu ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers

CMD sleep infinity
