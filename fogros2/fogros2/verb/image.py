import json
import os
import boto3
from ros2cli.verb import VerbExtension
from fogros2.util import work_dir


class ImageVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "--name", "-n", type=str, nargs=1, help="Select FogROS instance name to create as new image"
        )

    def create_image(self, instance):
        pwd = os.path.join(work_dir(), instance)
        info_path = os.path.join(pwd, "info")
        if not os.path.isfile(info_path):
            print("the info file does not exist, likely that the instance is not fully initialized")
            return 
        with open(info_path) as f:
            instance_info = json.loads(f.read())

        if instance_info["cloud_service_provider"] == "AWS":
            print("AWS EC2 instance ID is " + instance_info["ec2_instance_id"])

            image_name = "AWS_FogROS_image_" + instance
            client = boto3.client("ec2", region_name=instance_info["ec2_region"])
            client.create_image(InstanceId=instance_info["ec2_instance_id"],  Name=image_name)

            images = client.describe_images(Owners=["self"])["Images"]
            for image in images:
                if image["Name"] == image_name:
                    print("The detailed information of the image")
                    print(image)
                    print("Image ID: " + image["ImageId"])

    def main(self, *, args):
        self.create_image(args.name[0])
