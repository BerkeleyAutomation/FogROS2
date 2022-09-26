FROM ros:humble

RUN apt-get update && apt-get install -y wireguard openssh-client ros-humble-rmw-cyclonedds-cpp python3-pip curl apt-transport-https ca-certificates gnupg iproute2

RUN curl -LO https://dl.k8s.io/release/v1.25.0/bin/linux/amd64/kubectl
RUN sudo install -o root -g root -m 0755 kubectl /usr/local/bin/kubectl

RUN curl -O https://dl.google.com/dl/cloudsdk/channels/rapid/downloads/google-cloud-cli-400.0.0-linux-x86_64.tar.gz
RUN tar -xf google-cloud-cli-400.0.0-linux-x86_64.tar.gz

RUN mkdir -p /root/fog_ws/src
WORKDIR /root/fog_ws/src

COPY . .

CMD sleep infinity