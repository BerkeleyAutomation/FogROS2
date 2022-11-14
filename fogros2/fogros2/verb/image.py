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


class ImageVerb(VerbExtension):

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "name",
            type=str,
            nargs="*",
            help="Set instance name. \
                Can be found when running 'ros2 fog list' command next to ssh_key after "
                 "'FogROS2KEY-'",
        )
        parser.add_argument(
            "--region",
            nargs="*",
            help="Set AWS region (overrides config/env settings)",
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
        print("Instance name: ", name)
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

        return [client, ec2_instances]

    def create_ami(self, client, ec2_instances, dry_run):
        image_count = 0
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
                name = tag_map["FogROS2-Name"] + "-image"
                inst_id = inst['InstanceId']

                if not dry_run:
                    response = client.create_image(InstanceId=inst_id, Name=name)
                    if response["ResponseMetadata"]["HTTPStatusCode"] != 200:
                        raise RuntimeError(
                            f"Could not create image for {inst['KeyName']}!"
                        )

                print("done.")
                image_count += 1

        return image_count

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
                *self.query_region(regions[0], args.name), args.dry_run
            )
        else:
            from concurrent.futures import ThreadPoolExecutor

            with ThreadPoolExecutor(max_workers=len(regions)) as executor:
                futures = [
                    executor.submit(self.query_region, r, args.name)
                    for r in regions
                ]
                image_count = sum(
                    [
                        self.create_ami(*f.result(), args.dry_run)
                        for f in futures
                    ]
                )

        if image_count == 0:
            print("No image was created")
