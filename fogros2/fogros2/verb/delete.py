import json
import os
import shutil
import boto3
import botocore

from ros2cli.verb import VerbExtension

from fogros2 import AWS
from fogros2.util import instance_dir
from botocore.exceptions import ClientError

class DeleteVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "--all", "-A", action="store_true", default=False, help="Delete All existing FogROS instances"
        )
        parser.add_argument("--region", nargs='*', help="Set AWS region.  Overrides config/env settings.")
        parser.add_argument("--name", "-n", type=str, nargs=1, help="Select FogROS instance name to delete")

    def delete_instance(self, instance_name, region, instance_id):
        pwd = os.path.join(instance_dir(), instance)
        if os.path.exists(pwd):
            with open(os.path.join(pwd, "info")) as f:
                instance_info = json.loads(f.read())

        if instance_info["cloud_service_provider"] == "AWS":
            print("Terminating EC2 instance")
            # AWS.delete(instance_info["ec2_instance_id"], instance_info["ec2_region"])
            AWS.delete(instance_id, region)

        if os.path.exists(pwd):
            print("Removing Instance Dir")
            shutil.rmtree(pwd)

        print(f"Delete {instance} successfully!")

    def main(self, *, args):
        region = args.region # TODO: "*" or "all"
        
        client = boto3.client("ec2", region)

        if args.all == True:
            # Any instance with a FogROS2-Name tag.
            tag_filter = {"Name":"tag-key", "Values":["FogROS2-Name"]}
        else:
            # only instances with specific name tag.
            tag_filter = {"Name":"tag:FogROS2-Name", "Values":args.name}
        
        ec2_instances = client.describe_instances(
            Filters=[{"Name":"instance.group-name", "Values":["FOGROS2_SECURITY_GROUP"]},
                     tag_filter])

        delete_count = 0
        for res in ec2_instances['Reservations']:
            for inst in res['Instances']:
                tag_map = { t['Key']:t['Value'] for t in inst['Tags'] } if 'Tags' in inst else {}
                print(f"Deleting {tag_map.get('FogROS2-Name', '(unknown)')} {inst['InstanceId']}")
                # self.delete_instance(tag_map.get('FogROS2-Name'), region, instance['InstanceId'])
                print(f"    terminating instance {inst['InstanceId']}")
                response = client.terminate_instances(InstanceIds=[inst['InstanceId']])
                print(f"    deleting key pair {inst['KeyName']}")
                response = client.delete_key_pair(KeyName=inst['KeyName'])
                if 'FogROS2-Name' in tag_map:
                    inst_dir = os.path.join(instance_dir(), tag_map['FogROS2-Name'])
                    print(f"    removing instance data {inst_dir}")
                    if os.path.exists(inst_dir):
                        shutil.rmtree(inst_dir)
                print(f"    done.")
                delete_count += 1

        if delete_count == 0:
            print("No instances deleted.")


        
