import boto3
import os
import shutil

from botocore.exceptions import NoRegionError
from ros2cli.verb import VerbExtension

from ..util import instance_dir


class ImageVerb(VerbExtension):

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "instance name",
            type=str,
            nargs="*",
            help="Set instance name. Can be found when running 'ros2 fog list' command next to ssh_key after "
                 "'FogROS2KEY-'",
        )
        parser.add_argument(
            "--dry-run",
            action="store_true",
            help="Show what would happen, but do not execute",
        )

    def query_region(self, region, name):
        try:
            client = boto3.client("ec2", region)
        except NoRegionError:
            raise RuntimeError(
                "AWS is not configured! Please run `aws configure` first."
            )

        ec2_instances = client.describe_instances(
            Filters=[
                {
                    "Name": "instance.group-name",
                    "Values": ["FOGROS2_SECURITY_GROUP"],
                },
                {"Name": "tag:FogROS2-Name", "Values": name},
            ]
        )

        if len(ec2_instances["Reservations"]) == 0:
            print(
                "No EC2 instances found with the specified name; "
                "check list to be sure name is correct!"
            )

        return ec2_instances

    def shutdown(self, client, ec2_instances, dry_run):
        shutdown_count = 0
        for res in ec2_instances["Reservations"]:
            for inst in res["Instances"]:
                tag_map = (
                    {t["Key"]: t["Value"] for t in inst["Tags"]}
                    if "Tags" in inst
                    else {}
                )
                print(
                    f"Shutting down  {tag_map.get('FogROS2-Name', '(unknown)')} "
                    f"{inst['InstanceId']}"
                )
                print(f" instance {inst['InstanceId']}")
                if not dry_run:
                    response = client.terminate_instances(
                        InstanceIds=[inst["InstanceId"]]
                    )
                    if "TerminatingInstances" not in response or inst[
                        "InstanceId"
                    ] not in map(
                        lambda x: x["InstanceId"],
                        response["TerminatingInstances"],
                    ):
                        raise RuntimeError(
                            "Could not terminate instance"
                            f" {inst['InstanceId']}!"
                        )
                print(f"    deleting key pair {inst['KeyName']}")

        return shutdown_count

    def create_ami(self, client, instances, dry_run):
        image_count = 0
        for ec2_instances in instances:
            for res in ec2_instances["Reservations"]:
                for inst in res["Instances"]:
                    tag_map = (
                        {t["Key"]: t["Value"] for t in inst["Tags"]}
                        if "Tags" in inst
                        else {}
                    )
                    print(
                        f"Converting {tag_map.get('FogROS2-Name', '(unknown)')} "
                        f"{inst['InstanceId']} to AMI."
                    )
                    name = tag_map["FogROS2-Name"]
                    key_name = inst["KeyName"]
                    inst_id = inst['InstanceId']

                    if not dry_run:
                        response = client.create_image(inst_id, key_name)
                        if response["ResponseMetadata"]["HTTPStatusCode"] != 200:
                            raise RuntimeError(
                                f"Could not create image for {inst['KeyName']}!"
                            )

                    print("    done.")
                    image_count += 1

        return image_count

    def del_instance(self, client, ec2_instances, dry_run):
        delete_count = 0
        for res in ec2_instances["Reservations"]:
            for inst in res["Instances"]:
                tag_map = (
                    {t["Key"]: t["Value"] for t in inst["Tags"]}
                    if "Tags" in inst
                    else {}
                )
                print(
                    f"Deleting {tag_map.get('FogROS2-Name', '(unknown)')} "
                    f"{inst['InstanceId']}"
                )
                print(f"    deleting key pair {inst['KeyName']}")
                if not dry_run:
                    response = client.delete_key_pair(KeyName=inst["KeyName"])
                    if response["ResponseMetadata"]["HTTPStatusCode"] != 200:
                        raise RuntimeError(
                            f"Could not delete key pair {inst['KeyName']}!"
                        )
                if "FogROS2-Name" in tag_map:
                    inst_dir = os.path.join(
                        instance_dir(), tag_map["FogROS2-Name"]
                    )
                    if os.path.exists(inst_dir):
                        print(f"    removing instance data {inst_dir}")
                        if not dry_run:
                            shutil.rmtree(inst_dir)
                print("    done.")
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
            image_count = self.create_ami(
                *self.query_region(regions[0], args), args.dry_run
            )
            delete_count = self.delete_instances(
                *self.query_region(regions[0], args), args.dry_run
            )
        else:
            from concurrent.futures import ThreadPoolExecutor

            with ThreadPoolExecutor(max_workers=len(regions)) as executor:
                futures = [
                    executor.submit(self.query_region, r, args)
                    for r in regions
                ]
                image_count = sum(
                    [
                        self.create_ami(*f.result(), args.dry_run)
                        for f in futures
                    ]
                )
                delete_count = sum(
                    [
                        self.delete_instances(*f.result(), args.dry_run)
                        for f in futures
                    ]
                )
        if image_count == 0:
            print("No image was created")
        if delete_count == 0:
            print("No instance was deleted")







