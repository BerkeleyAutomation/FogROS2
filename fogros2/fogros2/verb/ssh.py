import json
import os
import boto3
import botocore

from ros2cli.verb import VerbExtension
from fogros2.util import instance_dir
from botocore.exceptions import ClientError

class SSHVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        parser.add_argument("--name", "-n", type=str, nargs=1, help="Select FogROS instance name to connect")
        parser.add_argument("--region", nargs='*', help="Set AWS region.  Overrides config/env settings.")
        parser.add_argument(
            "--user", "-u", type=str, nargs="?", default="ubuntu", help="User name of the remote SSH instance"
        )

    def query_region(self, region, name):
        client = boto3.client("ec2", region)
        ec2_instances = client.describe_instances(
            Filters=[{"Name":"instance.group-name", "Values":["FOGROS2_SECURITY_GROUP"]},
                     {"Name":"tag:FogROS2-Name", "Values":name}])
        return ec2_instances
        
    def main(self, *, args):
        regions = args.region

        if regions is None or len(regions) == 0:
            regions = [ None ]
        elif "*" in regions or "all" in regions:
            client = boto3.client("ec2")
            response = client.describe_regions()
            regions = [ r["RegionName"] for r in response["Regions"] ]
            
        if len(regions) == 1:
            instances = [ self.query_region(regions[0], args.name) ]
        else:
            from concurrent.futures import ThreadPoolExecutor, as_completed
            with ThreadPoolExecutor(max_workers=len(regions)) as executor:
                futures = [ executor.submit(self.query_region, r, args.name) for r in regions ]
                instances = [ f.result() for f in as_completed(futures) ]

        for ec2_instances in instances:
            for res in ec2_instances['Reservations']:
                for inst in res['Instances']:
                    tag_map = { t['Key']:t['Value'] for t in inst['Tags'] } if 'Tags' in inst else {}
                    name = tag_map['FogROS2-Name']
                    key_name = inst['KeyName']
                    key_path = os.path.join(instance_dir(), name, key_name + ".pem")
                    if 'PublicIpAddress' not in inst:
                        print(f"Warning: matching instance does not have a public IP address")
                        continue
                    public_ip = inst['PublicIpAddress']
                    os.execvp("ssh", ("ssh", "-i", key_path, f"{args.user}@{public_ip}"))

        # Since execvp replaces the current process, if here, we
        # haven't found a matching instance.
        print("No matching instance found.")
