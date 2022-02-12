# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from pathlib import Path
import sys
from threading import Event
from threading import Thread

import launch
import launch_pytest
import launch_ros

import pytest

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


@launch_pytest.fixture
def generate_test_description():
    path_to_test = Path(__file__).parent

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            executable=sys.executable,
            arguments=[str(path_to_test / 'executables' / 'talker.py')],
            additional_env={'PYTHONUNBUFFERED': '1'},
            name='demo_node_1',
            output='screen',
        ),
    ])


@pytest.mark.launch(fixture=generate_test_description)
def test_check_if_msgs_published():
    rclpy.init()
    try:
        node = MakeTestNode('test_node')
        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=5.0)
        assert msgs_received_flag, 'Did not receive msgs !'
    finally:
        rclpy.shutdown()


class MakeTestNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)
        self.msg_event_object = Event()

    def start_subscriber(self):
        # Create a subscriber
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.subscriber_callback,
            10
        )

        # Add a spin thread
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def subscriber_callback(self, data):
        self.msg_event_object.set()
