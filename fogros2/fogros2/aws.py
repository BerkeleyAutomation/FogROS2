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

import boto3
import random
import logging

class CloudInstance:
    def __init__(self):
        self.scp = None
        self.public_ip = None
        self.ssh_key_path = None
        self.name = None
        self.unique_name = None

    def create(self):
        raise NotImplementedError("Cloud SuperClass not implemented")

    def connect(self):
        self.scp = SCP_Client(self.ip, self.ssh_key_path)
        self.scp.connect()

    def get_ssh_key_path(self):
        return self.ssh_key_path

    def get_ip(self):
        return self.public_ip

    def get_name(self):
        return self.unique_name


class AWS(CloudInstance):
    def __init__(
        self,
        region="us-west-1",
        store_key_path="/home/root/fog_ws/",
        ec2_instance_type="t2.micro",
    ):
        super().__init__()
        self.region = region
        self.ec2_instance_type = ec2_instance_type
        self.ec2_instance_disk_size = 30  # GB
        self.aws_ami_image = "ami-08b3b42af12192fe6"

        # key & security group names
        self.unique_name = "AWS" + str(random.randint(10, 1000))
        self.ec2_security_group = "FOGROS_SECURITY_GROUP" + self.unique_name
        self.ec2_key_name = "FogROSKEY" + self.unique_name
        self.ssh_key_path = store_key_path + self.ec2_key_name + ".pem"

        # aws objects
        self.ec2_instance = None
        self.ec2_resource_manager = boto3.resource("ec2", self.region)
        self.ec2_boto3_client = boto3.client("ec2", self.region)

        # after config

        self.ssh_key = None
        self.ec2_security_group_ids = None

        # others
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.WARNING)

    def create(self):
        print("creating EC2 instance")
        self.create_security_group()
        self.generate_key_pair()
        self.create_ec2_instance()

    def get_ssh_key(self):
        return self.ssh_key

    def create_security_group(self):
        response = self.ec2_boto3_client.describe_vpcs()
        vpc_id = response.get("Vpcs", [{}])[0].get("VpcId", "")
        try:
            response = self.ec2_boto3_client.create_security_group(
                GroupName=self.ec2_security_group,
                Description="DESCRIPTION",
                VpcId=vpc_id,
            )
            security_group_id = response["GroupId"]
            self.logger.info(
                "Security Group Created %s in vpc %s." % (security_group_id, vpc_id)
            )

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
        except ClientError as e:
            self.logger.error(e)
        self.logger.warn("security group id is " + str(ec2_security_group_ids))
        self.ec2_security_group_ids = ec2_security_group_ids

    def generate_key_pair(self):
        ec2_keypair = self.ec2_boto3_client.create_key_pair(KeyName=self.ec2_key_name)
        ec2_priv_key = ec2_keypair["KeyMaterial"]
        self.logger.info(ec2_priv_key)

        with open(self.ssh_key_path, "w+") as f:
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

        self.logger.info("Have created the instance: ", instances)
        self.logger.info("type: " + self.ec2_instance_type)
        instance = instances[0]
        # use the boto3 waiter
        self.logger.info("wait for launching to finish")
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
        self.logger.warn("EC2 instance is created with ip address: " + self.public_ip)
        return instance
