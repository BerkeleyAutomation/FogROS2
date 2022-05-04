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

from launch_ros.actions import Node

import fogros2


def generate_launch_description():
    ld = fogros2.FogROSLaunchDescription()
    machine1 = fogros2.AWS(region="us-west-1", ec2_instance_type="t2.medium", ami_image="ami-00f25057ddc9b310b")

    # run this command locally:
    # sudo docker run --net=host --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --env CYCLONEDDS_URI=file:///tmp/cyclonedds.xml -v $(pwd)/install/share/fogros2/configs/cyclonedds.xml:/tmp/cyclonedds.xml --rm -it -v /home/gdpmobile7/rgbd_dataset_freiburg1_xyz:/dataset -v $(pwd)/output:/output mjd3/orbslam-ros ros2 launch orb_slam2_ros orb_slam2_d435_rgbd_client_launch.py dataset:=/dataset compress:=0

    machine1.add_docker_container(
        "sudo docker run --net=host --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --env CYCLONEDDS_URI=file:///tmp/cyclonedds.xml -v /home/ubuntu/cyclonedds.xml:/tmp/cyclonedds.xml --rm -it mjd3/orbslam-ros ros2 launch orb_slam2_ros orb_slam2_d435_rgbd_launch.py compress:=0"
    )
    # keplerc/gqcnn_ros:pj ros2 launch gqcnn_ros planner.launch.py")

    talker_node = Node(package="fogros2_examples", executable="listener", output="screen")
    listener_node = fogros2.CloudNode(
        package="fogros2_examples", executable="talker", output="screen", machine=machine1
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld
