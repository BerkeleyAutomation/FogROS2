# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Tests for the SetRemap Action."""

from launch import LaunchContext
from launch.actions import PopLaunchConfigurations
from launch.actions import PushLaunchConfigurations

from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch_ros.actions.load_composable_nodes import get_composable_node_load_request
from launch_ros.descriptions import ComposableNode

import pytest


class MockContext:

    def __init__(self):
        self.launch_configurations = {}

    def perform_substitution(self, sub):
        return sub.perform(None)


def get_set_remap_test_remaps():
    return [
        pytest.param(
            [('from', 'to')],
            id='One remapping rule'
        ),
        pytest.param(
            [('from1', 'to1'), ('from2', 'to2')],
            id='Two remapping rules'
        ),
    ]


@pytest.mark.parametrize(
    'remapping_rules',
    get_set_remap_test_remaps()
)
def test_set_remap(remapping_rules):
    lc = MockContext()
    for src, dst in remapping_rules:
        SetRemap(src, dst).execute(lc)
    assert lc.launch_configurations == {'ros_remaps': remapping_rules}


def test_set_remap_is_scoped():
    lc = LaunchContext()
    push_conf = PushLaunchConfigurations()
    pop_conf = PopLaunchConfigurations()
    set_remap = SetRemap('from', 'to')

    push_conf.execute(lc)
    set_remap.execute(lc)
    assert lc.launch_configurations == {'ros_remaps': [('from', 'to')]}
    pop_conf.execute(lc)
    assert lc.launch_configurations == {}


def test_set_remap_with_node():
    lc = MockContext()
    node = Node(
        package='asd',
        executable='bsd',
        name='my_node',
        namespace='my_ns',
        remappings=[('from2', 'to2')]
    )
    set_remap = SetRemap('from1', 'to1')
    set_remap.execute(lc)
    node._perform_substitutions(lc)
    assert len(node.expanded_remapping_rules) == 2
    assert node.expanded_remapping_rules == [('from1', 'to1'), ('from2', 'to2')]


def test_set_remap_with_composable_node():
    lc = MockContext()
    node_description = ComposableNode(
        package='asd',
        plugin='my_plugin',
        name='my_node',
        namespace='my_ns',
        remappings=[('from2', 'to2')]
    )
    set_remap = SetRemap('from1', 'to1')
    set_remap.execute(lc)
    request = get_composable_node_load_request(node_description, lc)
    remappings = request.remap_rules
    assert len(remappings) == 2
    assert remappings[0] == 'from1:=to1'
    assert remappings[1] == 'from2:=to2'
