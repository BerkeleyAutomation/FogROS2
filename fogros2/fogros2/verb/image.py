import time
from fogros2.fogros2.verb.delete import *

import boto3
from ros2cli.verb import VerbExtension
from fogros2.fogros2.verb.aws_cloud_instance import AWSCloudInstance
from botocore.exceptions import ClientError
import fogros2.fogros2.verb.ssh
import fogros2.fogros2.verb.list




class ImageVerb(VerbExtension):

    def __init__(self, region="us-west-1", ec2_instance_type="t2.micro", ami_image):
        self.region = region
        self.ec2_instance_type = ec2_instance_type
        self.ami_image = ami_image


        # 1. launch from generic Ubuntu AMI
        machine1 = fogros2.AWSCloudInstance(
            region=region, ec2_instance_type=ec2_instance_type, ami_image=ami_image
        )
        # making sure that the instance is created

        # look @ aws_cloud_instance
        current_state = machine1.create_ec2_instance()
        while current_state == 'off':
            self.ec2_instance.reload()
            self.logger.info("Waiting for launching to finish")
            self._ip = self.ec2_instance.public_ip_address

        # connect and shutdown

        # using delete from delete.py file


        client = boto3.client("ec2")
        fogros2.fogros2.verb.delete_instances(client, instance, dry_run)

        # image

        # delete

    def main(self, *, args):
        regions = args.region




        # trying to connect with a new method
        # import socket
        # retries = 10
        # retry_delay = 10
        # retry_count = 0
        # ec2 = boto3.resource('ec2', region_name=self.region)
        # ec2_id = self.ec2_instance_type
        # instance = ec2.Instance(id=ec2_id)
        # print("starting instance " + ec2_id)
        # instance.start()
        # instance.wait_until_running()
        # while retry_count <= retries:
        #     sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #     result = sock.connect_ex((instance.public_ip_address, 22))
        #     if result == 0:
        #         print("Instance is UP & accessible on port 22, the IP address is:  ", instance.public_ip_address)
        #         break
        #     else:
        #         print("instance is still down retrying . . . ")
        #         time.sleep(retry_delay)
        # except ClientError as e:
        #     print('Error', e)





