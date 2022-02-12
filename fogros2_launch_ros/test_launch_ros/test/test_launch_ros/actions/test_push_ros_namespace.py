# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Tests for the PushRosNamespace Action."""

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.actions.load_composable_nodes import get_composable_node_load_request
from launch_ros.descriptions import ComposableNode

import pytest


class MockContext:

    def __init__(self):
        self.launch_configurations = {}

    def perform_substitution(self, sub):
        return sub.perform(None)


class Config:

    def __init__(
        self,
        *,
        node_name=None,
        node_ns=None,
        push_ns=None,
        expected_ns=None,
        second_push_ns=None
    ):
        self.node_name = node_name
        self.push_ns = push_ns
        self.node_ns = node_ns
        self.expected_ns = expected_ns
        self.second_push_ns = second_push_ns

    def __repr__(self):
        return (
            f'TestConfig(node_name={self.node_name}, node_ns={self.node_ns}, '
            f'push_ns={self.push_ns}, expected_ns={self.expected_ns}, '
            f'second_push_ns={self.second_push_ns})'
        )


def get_test_cases():
    return (
        Config(
            push_ns='relative_ns',
            node_ns='node_ns',
            expected_ns='/relative_ns/node_ns'),
        Config(
            push_ns='relative_ns',
            node_ns='/node_ns',
            expected_ns='/node_ns'),
        Config(
            push_ns='relative_ns',
            node_ns='/',
            expected_ns='/'),
        Config(
            push_ns='relative_ns',
            node_ns='',
            expected_ns='/relative_ns'),
        Config(
            push_ns='relative_ns',
            second_push_ns='another_relative_ns',
            node_ns='node_ns',
            expected_ns='/relative_ns/another_relative_ns/node_ns'),
        Config(
            push_ns='relative_ns',
            second_push_ns='/absolute_ns',
            node_ns='node_ns',
            expected_ns='/absolute_ns/node_ns'),
        Config(
            node_name='my_node',
            push_ns='relative_ns',
            second_push_ns='/absolute_ns',
            node_ns='node_ns',
            expected_ns='/absolute_ns/node_ns'),
        Config(
            node_name='my_node',
            node_ns='node_ns',
            expected_ns='/node_ns'),
        Config(),
        Config(
            push_ns='',
            expected_ns='/'),
    )


@pytest.mark.parametrize('config', get_test_cases())
def test_push_ros_namespace(config):
    lc = MockContext()
    if config.push_ns is not None:
        pns1 = PushRosNamespace(config.push_ns)
        pns1.execute(lc)
    if config.second_push_ns is not None:
        pns2 = PushRosNamespace(config.second_push_ns)
        pns2.execute(lc)
    node = Node(
        package='dont_care',
        executable='whatever',
        namespace=config.node_ns,
        name=config.node_name
    )
    node._perform_substitutions(lc)
    expected_ns = (
        config.expected_ns if config.expected_ns is not None else Node.UNSPECIFIED_NODE_NAMESPACE
    )
    expected_name = (
        config.node_name if config.node_name is not None else Node.UNSPECIFIED_NODE_NAME
    )
    expected_fqn = expected_ns.rstrip('/') + '/' + expected_name
    assert expected_ns == node.expanded_node_namespace
    assert expected_fqn == node.node_name


@pytest.mark.parametrize('config', get_test_cases())
def test_push_ros_namespace_with_composable_node(config):
    lc = MockContext()
    if config.push_ns is not None:
        pns1 = PushRosNamespace(config.push_ns)
        pns1.execute(lc)
    if config.second_push_ns is not None:
        pns2 = PushRosNamespace(config.second_push_ns)
        pns2.execute(lc)
    node_description = ComposableNode(
        package='asd',
        plugin='my_plugin',
        namespace=config.node_ns,
        name=config.node_name,
    )
    request = get_composable_node_load_request(node_description, lc)
    expected_ns = config.expected_ns if config.expected_ns is not None else ''
    assert expected_ns == request.node_namespace
