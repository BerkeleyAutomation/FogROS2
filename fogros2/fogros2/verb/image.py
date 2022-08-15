import time

import boto3
import socket
from botocore.exceptions import ClientError
import fogros2.fogros2.verb.ssh
import fogros2.fogros2.verb.list


class ImageVerb(VerbExtension):

    def __init__(self, region="us-west-1", ec2_instance_type="t2.micro", ami_image):
        self.region = region
        self.ec2_instance_type = ec2_instance_type
        self.ami_image = ami_image
        self.instance = None

        # image
    def main(self, *, args):
        regions = args.region
        # 1. launch from generic Ubuntu AMI
        machine1 = fogros2.fogros2.AWSCloudInstance(
            region=self.region, ec2_instance_type=self.ec2_instance_type, ami_image=self.ami_image
        )
        # look @ aws_cloud_instance
        self.instance = machine1.create_ec2_instance()

        retries = 10
        retry_delay = 10
        retry_count = 0
        # ec2 = boto3.resource('ec2', region_name=self.region)
        # ec2_id = self.ec2_instance_type
        # instance = ec2.Instance(id=ec2_id)
        print("starting instance")
        self.instance.start()
        self.instance.wait_until_running()
        while retry_count <= retries:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            result = sock.connect_ex((self.instance.public_ip_address, 22))
            if result == 0:
                print("Instance is UP & accessible on port 22, the IP address is:  ", instance.public_ip_address)
                break
            else:
                print("instance is still down retrying . . . ")
                time.sleep(retry_delay)
        # except ClientError as e:
        #     print('Error', e)

        # using delete from delete.py file
        self.del_instance(args)

    def del_instance(self, args):
        client = boto3.client("ec2")
        fogros2.fogros2.verb.delete.delete_instances(client, self.instance, args)






