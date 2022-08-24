# Copyright 2022 The Regents of the University of California (Regents)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Copyright ©2022. The Regents of the University of California (Regents).
# All Rights Reserved. Permission to use, copy, modify, and distribute this
# software and its documentation for educational, research, and not-for-profit
# purposes, without fee and without a signed licensing agreement, is hereby
# granted, provided that the above copyright notice, this paragraph and the
# following two paragraphs appear in all copies, modifications, and
# distributions. Contact The Office of Technology Licensing, UC Berkeley, 2150
# Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-7201,
# otl@berkeley.edu, http://ipira.berkeley.edu/industry-info for commercial
# licensing opportunities. IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE. REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY,
# PROVIDED HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
# MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

import abc
import json
import os
import subprocess

from rclpy import logging

from .command_builder import BashBuilder
from .dds_config_builder import CycloneConfigBuilder
from .name_generator import get_unique_name
from .scp import SCPClient
from .util import (
    MissingEnvironmentVariableException,
    instance_dir,
    make_zip_file,
)


class CloudInstance(abc.ABC):
    """Abstract Base Class for Cloud Instances (e.g., AWS, GCP)."""

    def __init__(
        self,
        ros_workspace=os.path.dirname(os.getenv("COLCON_PREFIX_PATH")),
        working_dir_base=instance_dir(),
        launch_foxglove=False,
    ):
        if "RMW_IMPLEMENTATION" not in os.environ:
            raise MissingEnvironmentVariableException(
                "RMW_IMPLEMENTATION environment variable not set!"
            )
        if "CYCLONEDDS_URI" not in os.environ:
            raise MissingEnvironmentVariableException(
                "CYCLONEDDS_URI environment variable not set!"
            )

        # others
        self.logger = logging.get_logger(__name__)
        self.cyclone_builder = None
        self.scp = None
        self._ip = None
        self.ros_workspace = ros_workspace
        self.ros_distro = os.getenv("ROS_DISTRO")
        self.logger.debug(f"Using ROS workspace: {self.ros_workspace}")
        self._name = get_unique_name()
        self._working_dir_base = working_dir_base
        self._working_dir = os.path.join(self._working_dir_base, self._name)
        os.makedirs(self._working_dir, exist_ok=True)
        self._ssh_key_path = None
        self._is_created = False
        self.cloud_service_provider = None
        self.dockers = []
        self.launch_foxglove = launch_foxglove

    @abc.abstractmethod
    def create(self):
        pass

    def info(self, flush_to_disk=True):
        info_dict = {
            "name": self._name,
            "cloud_service_provider": self.cloud_service_provider,
            "ros_workspace": self.ros_workspace,
            "working_dir": self._working_dir,
            "ssh_key_path": self._ssh_key_path,
            "public_ip": self._ip,
        }
        if flush_to_disk:
            with open(os.path.join(self._working_dir, "info"), "w+") as f:
                json.dump(info_dict, f)
        return info_dict

    def connect(self):
        self.scp = SCPClient(self._ip, self._ssh_key_path)
        self.scp.connect()

    @property
    def ip(self):
        return self._ip

    @property
    def is_created(self):
        return self._is_created

    @property
    def name(self):
        return self._name

    def apt_install(self, args):
        self.scp.execute_cmd(
            f"sudo DEBIAN_FRONTEND=noninteractive apt-get install -y {args}"
        )

    def pip_install(self, args):
        self.scp.execute_cmd(f"sudo pip3 install {args}")

    def install_cloud_dependencies(self):
        self.apt_install("wireguard unzip docker.io python3-pip")

    def install_ros(self):
        # setup sources
        self.apt_install("software-properties-common gnupg lsb-release")
        self.scp.execute_cmd("sudo add-apt-repository universe")
        self.scp.execute_cmd(
            "sudo curl -sSL "
            "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key "
            "-o /usr/share/keyrings/ros-archive-keyring.gpg"
        )
        self.scp.execute_cmd(
            'echo "deb [arch=$(dpkg --print-architecture) '
            "signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] "
            "http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && "
            'echo $UBUNTU_CODENAME) main" | '
            "sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null"
        )

        # Run apt-get update after adding universe and ROS2 repos.
        self.scp.execute_cmd("sudo apt-get update")

        # set locale
        self.apt_install("locales")
        self.scp.execute_cmd("sudo locale-gen en_US en_US.UTF-8")
        self.scp.execute_cmd(
            "sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8"
        )
        self.scp.execute_cmd("export LANG=en_US.UTF-8")

        # install ros2 packages
        self.apt_install(f"ros-{self.ros_distro}-desktop")

        # source environment
        self.scp.execute_cmd(f"source /opt/ros/{self.ros_distro}/setup.bash")

    def configure_rosbridge(self):
        # install rosbridge
        self.apt_install(f"ros-{self.ros_distro}-rosbridge-suite")

        # source ros and launch rosbridge through ssh
        subprocess.call(f"chmod 400 {self._ssh_key_path}", shell=True)
        rosbridge_launch_script = (
            "ssh -o StrictHostKeyChecking=no -i "
            f"{self._ssh_key_path}"
            " ubuntu@"
            f"{self._ip}"
            f' "source /opt/ros/{self.ros_distro}/setup.bash && '
            'ros2 launch rosbridge_server rosbridge_websocket_launch.xml &"'
        )
        self.logger.info(rosbridge_launch_script)
        subprocess.Popen(rosbridge_launch_script, shell=True)

    def install_colcon(self):
        # ros2 repository
        self.scp.execute_cmd(
            "sudo sh -c 'echo \"deb [arch=amd64,arm64] "
            'http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > '
            "/etc/apt/sources.list.d/ros2-latest.list'"
        )
        self.scp.execute_cmd(
            "curl -s"
            " https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc"
            " | sudo apt-key add -"
        )

        self.pip_install("colcon-common-extensions")

    def push_ros_workspace(self):
        # configure ROS env
        workspace_path = self.ros_workspace  # os.getenv("COLCON_PREFIX_PATH")
        zip_dst = "/tmp/ros_workspace"
        make_zip_file(workspace_path, zip_dst)
        self.scp.execute_cmd("echo removing old workspace")
        self.scp.execute_cmd("rm -rf ros_workspace.zip ros2_ws fog_ws")
        self.scp.send_file(f"{zip_dst}.zip", "/home/ubuntu/")
        self.scp.execute_cmd("unzip -q /home/ubuntu/ros_workspace.zip")
        self.scp.execute_cmd("echo successfully extracted new workspace")

    def push_to_cloud_nodes(self):
        self.scp.send_file(
            f"/tmp/to_cloud_{self._name}", "/tmp/to_cloud_nodes"
        )

    def push_and_setup_vpn(self):
        self.scp.send_file(
            f"/tmp/fogros-cloud.conf{self._name}", "/tmp/fogros-aws.conf"
        )
        self.scp.execute_cmd(
            "sudo cp /tmp/fogros-aws.conf /etc/wireguard/wg0.conf && "
            "sudo chmod 600 /etc/wireguard/wg0.conf && sudo wg-quick up wg0"
        )

    def configure_DDS(self):
        # configure DDS
        self.cyclone_builder = CycloneConfigBuilder(["10.0.0.1"])
        self.cyclone_builder.generate_config_file()
        self.scp.send_file("/tmp/cyclonedds.xml", "~/cyclonedds.xml")

    def launch_cloud_node(self):
        cmd_builder = BashBuilder()
        cmd_builder.append(f"source /opt/ros/{self.ros_distro}/setup.bash")
        cmd_builder.append(
            "cd /home/ubuntu/fog_ws && colcon build --cmake-clean-cache"
        )
        cmd_builder.append(". /home/ubuntu/fog_ws/install/setup.bash")
        cmd_builder.append(self.cyclone_builder.env_cmd)
        ros_domain_id = os.environ.get("ROS_DOMAIN_ID")
        if not ros_domain_id:
            ros_domain_id = 0
        cmd_builder.append(
            f"ROS_DOMAIN_ID={ros_domain_id} "
            "ros2 launch fogros2 cloud.launch.py"
        )
        self.logger.info(cmd_builder.get())
        self.scp.execute_cmd(cmd_builder.get())

    def add_docker_container(self, cmd):
        self.dockers.append(cmd)

    def launch_cloud_dockers(self):
        # launch foxglove docker (if launch_foxglove specified)
        if self.launch_foxglove:
            self.configure_rosbridge()
            self.scp.execute_cmd(
                "sudo docker run -d --rm -p '8080:8080' "
                "ghcr.io/foxglove/studio:latest"
            )

        # launch user specified dockers
        for docker_cmd in self.dockers:
            self.scp.execute_cmd(docker_cmd)
