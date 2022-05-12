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

from launch import FogROSLaunchDescription
from launch_ros.actions import Node
import fogros2

def generate_launch_description():
    ld = FogROSLaunchDescription()
    machine1 = fogros2.AWS(region="us-west-1", ec2_instance_type="t2.medium", ami_image="ami-09175f2ca3c3dc67c", launch_foxglove=True)
    talker_node = Node(
        package="fogros2_examples", executable="listener", output="screen")
    listener_node = fogros2.CloudNode(
        package="fogros2_examples", executable="talker", output="screen",
        machine = machine1)
    turtlesim_node = Node(
        package="turtlesim", executable="turtlesim_node", output="screen")

    ld.add_action(talker_node)
    ld.add_action(listener_node)
    ld.add_action(turtlesim_node)
    return ld
