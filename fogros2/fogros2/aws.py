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

import abc
import json
import os
import random
import subprocess
import threading

import boto3
from botocore.exceptions import ClientError
from rclpy import logging

from .command_builder import BashBuilder
from .dds_config_builder import CycloneConfigBuilder
from .name_generator import get_unique_name
from .scp import SCP_Client
from .util import instance_dir, make_zip_file


class CloudInstance(abc.ABC):
    def __init__(
        self,
        ros_workspace=os.path.dirname(os.getenv("COLCON_PREFIX_PATH")),
        working_dir_base=instance_dir(),
        launch_foxglove=False,
    ):
        assert "RMW_IMPLEMENTATION" in os.environ, "RMW_IMPLEMENTATION environment variable not set"
        assert "CYCLONEDDS_URI" in os.environ, "CYCLONEDDS_URI environment variable not set"
            
        # others
        self.logger = logging.get_logger(__name__)
        self.cyclone_builder = None
        self.scp = None
        self.public_ip = None
        self.ros_workspace = ros_workspace
        self.ros_distro = os.getenv("ROS_DISTRO")
        self.logger.info(f"using ROS workspace: {self.ros_workspace}")
        self.ssh_key_path = None
        self.unique_name = get_unique_name()
        self.logger.info(f"New instance name: {self.unique_name}")
        self.working_dir = os.path.join(working_dir_base, self.unique_name)
        os.makedirs(self.working_dir, exist_ok=True)
        self.ready_lock = threading.Lock()
        self.ready_state = False
        self.cloud_service_provider = None
        self.dockers = []
        self.launch_foxglove = launch_foxglove

    def create(self):
        raise NotImplementedError("Cloud SuperClass not implemented")

    def info(self, flush_to_disk=True):
        info_dict = {
            "name": self.unique_name,
            "cloud_service_provider": self.cloud_service_provider,
            "ros_workspace": self.ros_workspace,
            "working_dir": self.working_dir,
            "ssh_key_path": self.ssh_key_path,
            "public_ip": self.public_ip,
        }
        if flush_to_disk:
            with open(os.path.join(self.working_dir, "info"), "w+") as f:
                json.dump(info_dict, f)
        return info_dict

    def connect(self):
        self.scp = SCP_Client(self.public_ip, self.ssh_key_path)
        self.scp.connect()

    @staticmethod
    @abc.abstractmethod
    def delete(instance_name):
        """Terminate this CloudInstance"""
        pass

    def get_ssh_key_path(self):
        return self.ssh_key_path

    def get_ip(self):
        return self.public_ip

    def get_name(self):
        return self.unique_name

    def get_ready_state(self):
        self.ready_lock.acquire()
        ready = self.ready_state
        self.ready_lock.release()
        return ready

    def set_ready_state(self, ready=True):
        self.ready_lock.acquire()
        self.ready_state = ready
        self.ready_lock.release()
        return ready

    def apt_install(self, args):
        self.scp.execute_cmd(f"sudo DEBIAN_FRONTEND=noninteractive apt-get install -y {args}")

    def pip_install(self, args):
        self.scp.execute_cmd(f"sudo pip3 install {args}")

    def install_cloud_dependencies(self):
        self.apt_install("wireguard unzip docker.io python3-pip")
        self.pip_install("wgconfig boto3 paramiko scp unique-names-generator")

    def install_ros(self):
        # setup sources
        self.apt_install("software-properties-common gnupg lsb-release")
        self.scp.execute_cmd("sudo add-apt-repository universe")
        # self.apt_install("curl gnupg lsb-release")
        self.scp.execute_cmd(
            "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
        )
        self.scp.execute_cmd(
            """echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null"""
        )

        # Run apt-get update after adding universe and ROS2 repos.
        self.scp.execute_cmd("sudo apt-get update")

        # set locale
        self.apt_install("locales")
        self.scp.execute_cmd("sudo locale-gen en_US en_US.UTF-8")
        self.scp.execute_cmd("sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8")
        self.scp.execute_cmd("export LANG=en_US.UTF-8")

        # install ros2 packages
        self.apt_install(f"ros-{self.ros_distro}-desktop")

        # source environment
        self.scp.execute_cmd(f"source /opt/ros/{self.ros_distro}/setup.bash")

    def configure_rosbridge(self):
        # install rosbridge
        self.apt_install(f"ros-{self.ros_distro}-rosbridge-suite")

        # source ros and launch rosbridge through ssh
        subprocess.call("chmod 400 " + self.ssh_key_path, shell=True)
        rosbridge_launch_script = (
            "ssh -o StrictHostKeyChecking=no -i "
            + self.ssh_key_path
            + " ubuntu@"
            + self.public_ip
            + f' "source /opt/ros/{self.ros_distro}/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml &"'
        )
        self.logger.info(rosbridge_launch_script)
        subprocess.Popen(rosbridge_launch_script, shell=True)

    def install_colcon(self):
        # ros2 repository
        self.scp.execute_cmd(
            """sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'"""
        )
        self.scp.execute_cmd(
            "curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -"
        )

        self.pip_install("colcon-common-extensions")

    def push_ros_workspace(self):
        # configure ROS env
        workspace_path = self.ros_workspace  # os.getenv("COLCON_PREFIX_PATH")
        zip_dst = "/tmp/ros_workspace"
        make_zip_file(workspace_path, zip_dst)
        self.scp.execute_cmd("echo removing old workspace")
        self.scp.execute_cmd("rm -rf ros_workspace.zip ros2_ws fog_ws")
        self.scp.send_file(zip_dst + ".zip", "/home/ubuntu/")
        self.scp.execute_cmd("unzip -q /home/ubuntu/ros_workspace.zip")
        self.scp.execute_cmd("echo successfully extracted new workspace")

    def push_to_cloud_nodes(self):
        self.scp.send_file("/tmp/to_cloud_" + self.unique_name, "/tmp/to_cloud_nodes")

    def push_and_setup_vpn(self):
        self.scp.send_file("/tmp/fogros-cloud.conf" + self.unique_name, "/tmp/fogros-aws.conf")
        self.scp.execute_cmd(
            "sudo cp /tmp/fogros-aws.conf /etc/wireguard/wg0.conf && sudo chmod 600 /etc/wireguard/wg0.conf && sudo wg-quick up wg0"
        )

    def configure_DDS(self):
        # configure DDS
        self.cyclone_builder = CycloneConfigBuilder(["10.0.0.1"])
        self.cyclone_builder.generate_config_file()
        self.scp.send_file("/tmp/cyclonedds.xml", "~/cyclonedds.xml")

    def launch_cloud_node(self):
        cmd_builder = BashBuilder()
        cmd_builder.append(f"source /opt/ros/{self.ros_distro}/setup.bash")
        cmd_builder.append("cd /home/ubuntu/fog_ws && colcon build --merge-install --cmake-clean-cache")
        cmd_builder.append(". /home/ubuntu/fog_ws/install/setup.bash")
        cmd_builder.append(self.cyclone_builder.env_cmd)
        ros_domain_id = os.environ.get("ROS_DOMAIN_ID")
        if not ros_domain_id:
            ros_domain_id = 0
        cmd_builder.append(f"ROS_DOMAIN_ID={ros_domain_id} ros2 launch fogros2 cloud.launch.py")
        self.logger.info(cmd_builder.get())
        self.scp.execute_cmd(cmd_builder.get())

    def add_docker_container(self, cmd):
        self.dockers.append(cmd)

    def launch_cloud_dockers(self):
        # launch foxglove docker (if launch_foxglove specified)
        if self.launch_foxglove:
            self.configure_rosbridge()
            self.scp.execute_cmd("sudo docker run -d --rm -p '8080:8080' ghcr.io/foxglove/studio:latest")

        # launch user specified dockers
        for docker_cmd in self.dockers:
            self.scp.execute_cmd(docker_cmd)


class RemoteMachine(CloudInstance):
    def __init__(self, ip, ssh_key_path):
        super().__init__()
        self.ip = ip
        self.ssh_key_path = ssh_key_path
        self.unique_name = "REMOTE" + str(random.randint(10, 1000))
        self.set_ready_state()  # assume it to be true

    def create(self):
        # since the machine is assumed to be established
        # no need to create
        pass

    @staticmethod
    def delete(instance_name):
        pass


class AWS(CloudInstance):
    def __init__(self, ami_image, region="us-west-1", ec2_instance_type="t2.micro", disk_size=30, **kwargs):
        super().__init__(**kwargs)
        self.cloud_service_provider = "AWS"

        self.region = region
        self.ec2_instance_type = ec2_instance_type
        self.ec2_instance_disk_size = disk_size  # GB
        self.aws_ami_image = ami_image

        # key & security group names
        self.ec2_security_group = "FOGROS2_SECURITY_GROUP"
        self.ec2_key_name = "FogROS2KEY-" + self.unique_name
        self.ssh_key_path = os.path.join(self.working_dir, self.ec2_key_name + ".pem")

        # aws objects
        self.ec2_instance = None
        self.ec2_resource_manager = boto3.resource("ec2", self.region)
        self.ec2_boto3_client = boto3.client("ec2", self.region)

        # after config
        self.ssh_key = None
        self.ec2_security_group_ids = None

        self.create()

    def create(self):
        self.logger.info("creating EC2 instance")
        self.create_security_group()
        self.generate_key_pair()
        self.create_ec2_instance()
        self.info(flush_to_disk=True)
        self.connect()
        self.install_ros()
        self.install_colcon()
        self.install_cloud_dependencies()
        self.push_ros_workspace()
        # self.push_to_cloud_nodes()
        self.info(flush_to_disk=True)
        self.set_ready_state()

    def info(self, flush_to_disk=True):
        info_dict = super().info(flush_to_disk)
        info_dict["ec2_region"] = self.region
        info_dict["ec2_instance_type"] = self.ec2_instance_type
        info_dict["disk_size"] = self.ec2_instance_disk_size
        info_dict["aws_ami_image"] = self.aws_ami_image
        info_dict["ec2_instance_id"] = self.ec2_instance.instance_id
        if flush_to_disk:
            with open(os.path.join(self.working_dir, "info"), "w+") as f:
                json.dump(info_dict, f)
        return info_dict

    def get_ssh_key(self):
        return self.ssh_key

    def get_default_vpc(self):
        response = self.ec2_boto3_client.describe_vpcs(Filters=[{"Name": "is-default", "Values": ["true"]}])
        vpcs = response.get("Vpcs", [])

        if len(vpcs) == 0:
            self.logger.warn("No default VPC found.  Creating one.")
            response = self.ec2_boto3_client.create_default_vpc()
            vpc_id = response["Vpc"]["VpcId"]
            self.logger.warn(f"Created new default VPC {vpc_id}")
            return vpc_id

        if len(vpcs) > 1:
            # This shouldn't happen, but just in case, warn.
            self.logger.warn("Multiple default VPCs.  This may lead to undefined behavior.")

        vpc_id = vpcs[0].get("VpcId", "")
        self.logger.info(f"Using VPC {vpc_id}")
        return vpc_id

    def create_security_group(self):
        vpc_id = self.get_default_vpc()
        try:
            response = self.ec2_boto3_client.describe_security_groups(GroupNames=[self.ec2_security_group])
            security_group_id = response["SecurityGroups"][0]["GroupId"]
        except ClientError as e:
            # check if the group does not exist. we'll create one in
            # that case.  Any other error is unexpected and re-thrown.
            if e.response["Error"]["Code"] != "InvalidGroup.NotFound":
                raise e

            self.logger.warn("security group does not exist, creating.")
            response = self.ec2_boto3_client.create_security_group(
                GroupName=self.ec2_security_group,
                Description="Security group used by FogROS 2 (safe to delete when FogROS 2 is not in use)",
                VpcId=vpc_id,
            )
            security_group_id = response["GroupId"]
            self.logger.info("Security Group Created %s in vpc %s." % (security_group_id, vpc_id))

            data = self.ec2_boto3_client.authorize_security_group_ingress(
                GroupId=security_group_id,
                IpPermissions=[
                    {
                        "IpProtocol": "-1",
                        "FromPort": 0,
                        "ToPort": 65535,
                        "IpRanges": [{"CidrIp": "0.0.0.0/0"}],
                    }
                ],
            )
            self.logger.info("Ingress Successfully Set %s" % data)

        ec2_security_group_ids = [security_group_id]
        self.logger.info(f"security group id is {ec2_security_group_ids}")
        self.ec2_security_group_ids = ec2_security_group_ids

    def generate_key_pair(self):
        ec2_keypair = self.ec2_boto3_client.create_key_pair(KeyName=self.ec2_key_name)
        ec2_priv_key = ec2_keypair["KeyMaterial"]

        # Since we're writing an SSH key, make sure to write with user-only permissions.
        with open(os.open(self.ssh_key_path, os.O_CREAT | os.O_WRONLY, 0o600), "w") as f:
            f.write(ec2_priv_key)

        self.ssh_key = ec2_priv_key
        return ec2_priv_key

    def create_ec2_instance(self):
        #
        # start EC2 instance
        # note that we can start muliple instances at the same time
        #
        instances = self.ec2_resource_manager.create_instances(
            ImageId=self.aws_ami_image,
            MinCount=1,
            MaxCount=1,
            InstanceType=self.ec2_instance_type,
            KeyName=self.ec2_key_name,
            SecurityGroupIds=self.ec2_security_group_ids,
            ClientToken="FogROS2-" + self.unique_name,
            TagSpecifications=[
                {"ResourceType": "instance", "Tags": [{"Key": "FogROS2-Name", "Value": self.unique_name}]}
            ],
            BlockDeviceMappings=[
                {
                    "DeviceName": "/dev/sda1",
                    "Ebs": {
                        "VolumeSize": self.ec2_instance_disk_size,
                        "VolumeType": "standard",
                    },
                }
            ],
        )

        self.logger.info(f"Created instance: {instances}, type: {self.ec2_instance_type}")
        instance = instances[0]
        # use the boto3 waiter
        self.logger.info("waiting for launching to finish")
        instance.wait_until_running()
        self.logger.info("launch finished")
        # reload instance object
        instance.reload()
        self.ec2_instance = instance
        self.public_ip = instance.public_ip_address
        while not self.public_ip:
            instance.reload()
            self.logger.info("waiting for launching to finish")
            self.public_ip = instance.public_ip_address
        self.logger.info("EC2 instance is created with ip address: " + self.public_ip)
        return instance

    @staticmethod
    def delete(instance_name, region):
        ec2 = boto3.resource("ec2", region)
        instance = ec2.Instance(instance_name)
        print(instance.terminate())
