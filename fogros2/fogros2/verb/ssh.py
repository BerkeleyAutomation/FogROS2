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

import os

import boto3
from botocore.exceptions import NoRegionError
from ros2cli.verb import VerbExtension

from ..util import instance_dir


class SSHVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "name",
            type=str,
            nargs=1,
            help="FogROS instance name to connect to",
        )
        parser.add_argument(
            "--region",
            nargs="*",
            help="Set AWS region (overrides config/env settings)",
        )
        parser.add_argument(
            "--user",
            "-u",
            type=str,
            nargs="?",
            default="ubuntu",
            help="User name of the remote SSH instance",
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
        return ec2_instances

    def main(self, *, args):
        regions = args.region

        if regions is None or len(regions) == 0:
            regions = [None]
        elif "*" in regions or "all" in regions:
            client = boto3.client("ec2")
            response = client.describe_regions()
            regions = [r["RegionName"] for r in response["Regions"]]

        if len(regions) == 1:
            instances = [self.query_region(regions[0], args.name)]
        else:
            from concurrent.futures import ThreadPoolExecutor, as_completed

            with ThreadPoolExecutor(max_workers=len(regions)) as executor:
                futures = [
                    executor.submit(self.query_region, r, args.name)
                    for r in regions
                ]
                instances = [f.result() for f in as_completed(futures)]

        for ec2_instances in instances:
            for res in ec2_instances["Reservations"]:
                for inst in res["Instances"]:
                    tag_map = (
                        {t["Key"]: t["Value"] for t in inst["Tags"]}
                        if "Tags" in inst
                        else {}
                    )
                    name = tag_map["FogROS2-Name"]
                    key_name = inst["KeyName"]
                    key_path = os.path.join(
                        instance_dir(), name, f"{key_name}.pem"
                    )
                    if "PublicIpAddress" not in inst:
                        print(
                            "Warning: matching instance does not have a "
                            "public IP address"
                        )
                        continue
                    public_ip = inst["PublicIpAddress"]
                    os.execvp(
                        "ssh",
                        ("ssh", "-i", key_path, f"{args.user}@{public_ip}"),
                    )

        # Since execvp replaces the current process, if here, we
        # haven't found a matching instance.
        print("No matching instance found")
