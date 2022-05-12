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

import json
import os

import boto3
from ros2cli.verb import VerbExtension

from ..util import work_dir


class ImageVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "--name",
            "-n",
            type=str,
            nargs=1,
            help="Select FogROS instance name to create as new image",
        )

    def create_image(self, instance):
        pwd = os.path.join(work_dir(), instance)
        info_path = os.path.join(pwd, "info")
        if not os.path.isfile(info_path):
            print(
                "The info file does not exist, it is likely that the instance"
                " is not fully initialized!"
            )
            return
        with open(info_path) as f:
            instance_info = json.loads(f.read())

        if instance_info["cloud_service_provider"] == "AWS":
            print(f"AWS EC2 instance ID is {instance_info['ec2_instance_id']}")

            image_name = f"AWS_FogROS_image_{instance}"
            client = boto3.client(
                "ec2", region_name=instance_info["ec2_region"]
            )
            client.create_image(
                InstanceId=instance_info["ec2_instance_id"], Name=image_name
            )

            images = client.describe_images(Owners=["self"])["Images"]
            for image in images:
                if image["Name"] == image_name:
                    print("The detailed information of the image")
                    print(image)
                    print(f"Image ID: {image['ImageId']}")

    def main(self, *, args):
        self.create_image(args.name[0])
