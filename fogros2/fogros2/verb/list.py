# Copyright 2022 The Regents of the University of California (Regents)
#
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
#
# Copyright ©2022. The Regents of the University of California (Regents).
# All Rights Reserved. Permission to use, copy, modify, and distribute this
# software and its documentation for educational, research, and not-for-profit
# purposes, without fee and without a signed licensing agreement, is hereby
# granted, provided that the above copyright notice, this paragraph and the
# following two paragraphs appear in all copies, modifications, and
# distributions. Contact The Office of Technology Licensing, UC Berkeley, 2150
# Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-7201,
# otl@berkeley.edu, http://ipira.berkeley.edu/industry-info for commercial
# licensing opportunities. IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE. REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY,
# PROVIDED HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
# MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

import boto3
from botocore.exceptions import NoRegionError
from ros2cli.verb import VerbExtension


class ListVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "--region",
            nargs="*",
            help="Set AWS region (overrides config/env settings)",
        )

    def query_region(self, region):
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
                {"Name": "tag-key", "Values": ["FogROS2-Name"]},
            ]
        )

        # For each instance, we also look up the block device mapping
        # to get the disk size.
        for res in ec2_instances["Reservations"]:
            for inst in res["Instances"]:
                volumes = client.describe_volumes(
                    VolumeIds=[
                        m["Ebs"]["VolumeId"]
                        for m in inst["BlockDeviceMappings"]
                    ]
                )
                # Update the ec2_instances structure to include the
                # block device info.
                for i in range(len(volumes["Volumes"])):
                    # Attachments duplicates info we already have, so
                    # delete it.  Not really necessary, but it cleans
                    # things up when debugging.
                    del volumes["Volumes"][i]["Attachments"]
                    inst["BlockDeviceMappings"][i]["Ebs"][
                        "VolumeInfo"
                    ] = volumes["Volumes"][i]

        return [region, ec2_instances]

    def print_region_info(self, region, ec2_instances):
        if len(ec2_instances["Reservations"]) == 0:
            print("No FogROS instances found")
        for res in ec2_instances["Reservations"]:
            for inst in res["Instances"]:
                tag_map = (
                    {t["Key"]: t["Value"] for t in inst["Tags"]}
                    if "Tags" in inst
                    else {}
                )
                print(
                    f"====== {tag_map.get('FogROS2-Name', '(unknown)')} ======"
                )
                print("cloud_service_provider: AWS")
                print(f"ec2_region: {region}")
                print(
                    "ec2_instance_type: "
                    f"{inst.get('InstanceType', '(unknown)')}"
                )
                print(
                    f"ec2_instance_id: {inst.get('InstanceId', '(unknown)')}"
                )
                print(f"public_ip: {inst.get('PublicIpAddress', '(none)')}")
                print(f"ssh_key: {inst.get('KeyName', '(unknown)')}")
                for bdm in inst["BlockDeviceMappings"]:
                    if "Ebs" in bdm and "VolumeInfo" in bdm["Ebs"]:
                        print(f"disk_size: {bdm['Ebs']['VolumeInfo']['Size']}")
                print(f"aws_ami_image: {inst.get('ImageId', '(unknown)')}")
                print(
                    f"state: {inst.get('State', {'Name':'(unknown)'})['Name']}"
                )

    def main(self, *, args):
        regions = args.region
        if regions is None or len(regions) == 0:
            regions = [None]  # use default (should we default to "all")
        elif "*" in regions or "all" in regions:
            client = boto3.client("ec2")
            response = client.describe_regions()
            regions = [r["RegionName"] for r in response["Regions"]]

        if len(regions) == 1:
            self.print_region_info(*self.query_region(regions[0]))
        else:
            from concurrent.futures import ThreadPoolExecutor, as_completed

            with ThreadPoolExecutor(max_workers=len(regions)) as executor:
                futures = [
                    executor.submit(self.query_region, r) for r in regions
                ]
                for f in as_completed(futures):
                    self.print_region_info(*f.result())
