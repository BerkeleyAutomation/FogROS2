import os
import shutil

import boto3
from botocore.exceptions import NoRegionError
from ros2cli.verb import VerbExtension

from ..util import instance_dir


class DeleteVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "name", type=str, nargs=1, help="FogROS instance name to delete, or 'all' to delete all instances"
        )
        parser.add_argument("--region", nargs="*", help="Set AWS region (overrides config/env settings)")
        parser.add_argument("--dry-run", action="store_true", help="Show what would happen, but do not execute")

    def query_region(self, region, args):
        try:
            client = boto3.client("ec2", region)
        except NoRegionError:
            raise RuntimeError("AWS is not configured! Please run `aws configure` first.")

        if "all" in args.name:
            # Any instance with a FogROS2-Name tag.
            tag_filter = {"Name": "tag-key", "Values": ["FogROS2-Name"]}
        else:
            # only instances with specific name tag.
            tag_filter = {"Name": "tag:FogROS2-Name", "Values": args.name}

        ec2_instances = client.describe_instances(
            Filters=[{"Name": "instance.group-name", "Values": ["FOGROS2_SECURITY_GROUP"]}, tag_filter]
        )

        if len(ec2_instances["Reservations"]) == 0:
            print("No EC2 instances found with the specified name; check list to be sure name is correct!")

        return [client, ec2_instances]

    def delete_instances(self, client, ec2_instances, dry_run):
        delete_count = 0
        for res in ec2_instances["Reservations"]:
            for inst in res["Instances"]:
                tag_map = {t["Key"]: t["Value"] for t in inst["Tags"]} if "Tags" in inst else {}
                print(f"Deleting {tag_map.get('FogROS2-Name', '(unknown)')} {inst['InstanceId']}")
                print(f"    terminating instance {inst['InstanceId']}")
                if not dry_run:
                    response = client.terminate_instances(InstanceIds=[inst["InstanceId"]])
                    if "TerminatingInstances" not in response or inst["InstanceId"] not in map(
                        lambda x: x["InstanceId"], response["TerminatingInstances"]
                    ):
                        raise RuntimeError(f"Could not terminate instance {inst['InstanceId']}!")
                print(f"    deleting key pair {inst['KeyName']}")
                if not dry_run:
                    response = client.delete_key_pair(KeyName=inst["KeyName"])
                    if response["ResponseMetadata"]["HTTPStatusCode"] != 200:
                        raise RuntimeError(f"Could not delete key pair {inst['KeyName']}!")
                if "FogROS2-Name" in tag_map:
                    inst_dir = os.path.join(instance_dir(), tag_map["FogROS2-Name"])
                    if os.path.exists(inst_dir):
                        print(f"    removing instance data {inst_dir}")
                        if not dry_run:
                            shutil.rmtree(inst_dir)
                print(f"    done.")
                delete_count += 1

        return delete_count

    def main(self, *, args):
        regions = args.region
        if regions is None or len(regions) == 0:
            regions = [None]
        elif "*" in regions or "all" in regions:
            client = boto3.client("ec2")
            response = client.describe_regions()
            regions = [r["RegionName"] for r in response["Regions"]]

        if len(regions) == 1:
            delete_count = self.delete_instances(*self.query_region(regions[0], args), args.dry_run)
        else:
            from concurrent.futures import ThreadPoolExecutor, as_completed

            with ThreadPoolExecutor(max_workers=len(regions)) as executor:
                futures = [executor.submit(self.query_region, r, args) for r in regions]
                delete_count = sum([self.delete_instances(*f.result(), args.dry_run) for f in futures])

        if delete_count == 0:
            print("No instances deleted")
