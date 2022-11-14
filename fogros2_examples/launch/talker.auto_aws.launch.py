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

from launch_ros.actions import Node
import fogros2
from utils import region_ami_selection, ec2_instance_type_selection


def generic_ubuntu_ami():
    return {
        "us-west-1": {"ami_image": "ami-02ea247e531eb3ce6"},
        "us-west-2": {"ami_image": "ami-017fecd1353bcc96e"},
        "us-east-1": {"ami_image": "ami-08c40ec9ead489470"},
        "us-east-2": {"ami_image": "ami-097a2df4ac947655f"},
        "ap-northeast-1": {"ami_image": "ami-03f4fa076d2981b45"},
        "ap-northeast-2": {"ami_image": "ami-0e9bfdb247cc8de84"},
        "ap-northeast-3": {"ami_image": "ami-08c2ee02329b72f26"},
    }


def generate_launch_description():
    """Talker example that launches the listener on AWS."""
    ld = fogros2.FogROSLaunchDescription()

    region, ami = region_ami_selection.find_nearest_region_and_ami(generic_ubuntu_ami())

    ec2_instance_type = ec2_instance_type_selection.find_cheapest_ec2_instance_type(region)

    print(region, ami, ec2_instance_type)
    machine1 = fogros2.AWSCloudInstance(
        region=region, ec2_instance_type=ec2_instance_type, ami_image=ami
    )

    listener_node = Node(
        package="fogros2_examples", executable="listener", output="screen"
    )

    talker_node = fogros2.CloudNode(
        package="fogros2_examples",
        executable="talker",
        output="screen",
        machine=machine1,
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld
