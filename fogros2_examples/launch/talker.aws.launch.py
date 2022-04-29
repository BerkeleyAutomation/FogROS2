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

from fogros2 import FogROSLaunchDescription, CloudNode, AWS
from launch_ros.actions import Node

def ami_image():
    # An AMI is an Amazon Web Services virtual machine image with a
    # pre-installed OS and dependencies.  We match the AMI in the
    # cloud to have the same OS release as the robot.  Currently we
    # support Ubuntu 20.04 and 22.04.

    import lsb_release
    ubuntu_release = lsb_release.get_os_release()['RELEASE']
    
    if ubuntu_release == "20.04":
        return "ami-00f25057ddc9b310b"
    if ubuntu_release == "22.04":
        return "ami-0b6030c78f8b2f076"

    raise ValueError(f"No AMI for {ubuntu_release}")


def generate_launch_description():
    """
    Talker example that launches the listener on AWS.
    """
    ld = FogROSLaunchDescription()
    machine1 = AWS(region="us-west-1", ec2_instance_type="t2.micro", ami_image=ami_image())

    talker_node = Node(package="fogros2_examples", executable="listener", output="screen")
    listener_node = CloudNode(
        package="fogros2_examples", executable="talker", output="screen", machine=machine1
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld
