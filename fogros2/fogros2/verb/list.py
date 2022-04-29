import json
import os
import boto3
import botocore

from ros2cli.verb import VerbExtension
from fogros2.util import instance_dir
from botocore.exceptions import ClientError

class ListVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "--region", nargs='*', help="Set AWS region.  Overrides config/env settings.")

    def query_region(self, region):
        # print(f"Scanning {region}")
        client = boto3.client("ec2", region)
        #response = client.describe_vpcs()
        #vpc_id = response.get("Vpcs", [{}])[0].get("VpcId", "")

        ec2_instances = client.describe_instances(
            Filters=[{"Name":"instance.group-name", "Values":["FOGROS2_SECURITY_GROUP"]},
                     {"Name":"tag-key", "Values":["FogROS2-Name"]}])

        # For each instance, we also look up the block device mapping
        # to get the disk size.
        for res in ec2_instances["Reservations"]:
            for inst in res["Instances"]:
                volumes = client.describe_volumes(
                    VolumeIds = [ m['Ebs']['VolumeId'] for m in inst['BlockDeviceMappings'] ])
                # Update the ec2_instances structure to include the
                # block device info.
                for i in range(len(volumes['Volumes'])):
                    # Attachments duplicates info we already have, so
                    # delete it.  Not really necessary, but it cleans
                    # things up when debugging.
                    del volumes['Volumes'][i]['Attachments']
                    inst['BlockDeviceMappings'][i]['Ebs']['VolumeInfo'] = volumes['Volumes'][i]
                    
        return [region, ec2_instances]

    def print_region_info(self, region, ec2_instances):
        # import pprint
        # pp = pprint.PrettyPrinter(indent=4)
        for res in ec2_instances["Reservations"]:
            for inst in res["Instances"]:
                # pp.pprint(inst)
                tag_map = { t['Key']:t['Value'] for t in inst['Tags'] } if 'Tags' in inst else {}
                print(f"====== {tag_map.get('FogROS2-Name', '(unknown)')} ======")
                print("cloud_service_provider: AWS")
                print(f"ec2_region: {region}")
                print(f"ec2_instance_type: {inst.get('InstanceType', '(unknown)')}")
                print(f"ec2_instance_id: {inst.get('InstanceId', '(unknown)')}")
                print(f"public_ip: {inst.get('PublicIpAddress', '(none)')}")
                # for ni in inst['NetworkInterfaces']:
                #     print(f"    public_ip: {ni['Association']['PublicIp']}")
                print(f"ssh_key: {inst.get('KeyName', '(unknown)')}")
                for bdm in inst['BlockDeviceMappings']:
                    if 'Ebs' in bdm and 'VolumeInfo' in bdm['Ebs']:
                        print(f"disk_size: {bdm['Ebs']['VolumeInfo']['Size']}")
                print(f"aws_ami_image: {inst.get('ImageId', '(unknown)')}")
                print(f"state: {inst.get('State', {'Name':'(unknown)'})['Name']}")
        
    def main(self, *, args):
        regions = args.region
        if regions is None or len(regions) == 0:
            regions = [ None ] # use default (should we default to "all")
        elif "*" in regions or "all" in regions:
            client = boto3.client("ec2")
            response = client.describe_regions()
            regions = [ r["RegionName"] for r in response["Regions"] ]
            
        if len(regions) == 1:
            self.print_region_info(*self.query_region(regions[0]))
        else:
            from concurrent.futures import ThreadPoolExecutor, as_completed
            with ThreadPoolExecutor(max_workers=len(regions)) as executor:
                futures = [ executor.submit(self.query_region, r) for r in regions ]
                for f in as_completed(futures):
                    self.print_region_info(*f.result())
