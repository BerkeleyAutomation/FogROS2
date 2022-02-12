# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

"""Tests for the node name count tracking utility."""

import asyncio

from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchService

from launch_ros.actions.composable_node_container import ComposableNodeContainer
from launch_ros.actions.lifecycle_node import LifecycleNode
from launch_ros.actions.node import Node
from launch_ros.descriptions.composable_node import ComposableNode
from launch_ros.utilities import add_node_name
from launch_ros.utilities import get_node_name_count

import osrf_pycommon.process_utils

TEST_NODE_NAMESPACE = '/my_namespace'
TEST_NODE_NAME = 'my_node'


def test_node_name_count():
    lc = LaunchContext()
    add_node_name(lc, 'node_name_1')
    add_node_name(lc, 'node_name_1')
    add_node_name(lc, 'node_name_2')
    assert get_node_name_count(lc, 'node_name_1') == 2
    assert get_node_name_count(lc, 'node_name_2') == 1
    assert get_node_name_count(lc, 'node_name_3') == 0


def _launch(launch_description):
    loop = osrf_pycommon.process_utils.get_loop()
    ls = LaunchService()
    ls.include_launch_description(launch_description)
    launch_task = loop.create_task(ls.run_async())
    loop.run_until_complete(asyncio.sleep(5))
    if not launch_task.done():
        loop.create_task(ls.shutdown())
        loop.run_until_complete(launch_task)
    return ls.context


def test_launch_node_with_name():
    node = Node(
        package='demo_nodes_py',
        executable='listener_qos',
        name=TEST_NODE_NAME,
        namespace=TEST_NODE_NAMESPACE,
        output='screen',
    )
    ld = LaunchDescription([node])
    context = _launch(ld)
    assert get_node_name_count(context, f'{TEST_NODE_NAMESPACE}/{TEST_NODE_NAME}') == 1
    assert get_node_name_count(context, f'/{TEST_NODE_NAME}') == 0


def test_launch_node_without_name():
    node = Node(
        package='demo_nodes_py',
        executable='listener_qos',
        namespace=TEST_NODE_NAMESPACE,
        output='screen',
    )
    ld = LaunchDescription([node])
    context = _launch(ld)
    # For Nodes, when node_name is omitted, launch_ros is unable to track it
    assert get_node_name_count(context, f'{TEST_NODE_NAMESPACE}/{TEST_NODE_NAME}') == 0
    assert get_node_name_count(context, f'/{TEST_NODE_NAME}') == 0


def test_launch_node_with_name_without_namespace():
    node = Node(
        package='demo_nodes_py',
        executable='listener_qos',
        name=TEST_NODE_NAME,
        output='screen',
    )
    ld = LaunchDescription([node])
    context = _launch(ld)
    assert get_node_name_count(context, f'{TEST_NODE_NAMESPACE}/{TEST_NODE_NAME}') == 0
    assert get_node_name_count(context, f'/{TEST_NODE_NAME}') == 0


def test_launch_composable_node_with_names(pytestconfig):
    node = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name=TEST_NODE_NAME,
        namespace=TEST_NODE_NAMESPACE,
        composable_node_descriptions=[
            ComposableNode(
                package='composition',
                plugin='composition::Listener',
                name=TEST_NODE_NAME,
                namespace=TEST_NODE_NAMESPACE
            )
        ],
        output='screen'
    )
    ld = LaunchDescription([node])
    context = _launch(ld)
    captured = pytestconfig.pluginmanager.getplugin('capturemanager').read_global_capture()
    assert get_node_name_count(context, f'{TEST_NODE_NAMESPACE}/{TEST_NODE_NAME}') == 2
    expected_line = f'there are now at least 2 nodes with the name {TEST_NODE_NAMESPACE}' \
        f'/{TEST_NODE_NAME} created within this launch context'
    assert expected_line in captured.out


def test_launch_composable_node_without_component_name():
    node = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name=TEST_NODE_NAME,
        namespace=TEST_NODE_NAMESPACE,
        composable_node_descriptions=[
            ComposableNode(
                package='composition',
                plugin='composition::Listener',
                namespace=TEST_NODE_NAMESPACE
            )
        ],
        output='screen'
    )
    ld = LaunchDescription([node])
    context = _launch(ld)
    assert get_node_name_count(context, f'{TEST_NODE_NAMESPACE}/{TEST_NODE_NAME}') == 1
    # Unlike for Nodes, launch_ros is able to track component node names even when
    # node_name is not provided for the component
    assert get_node_name_count(context, f'{TEST_NODE_NAMESPACE}/listener') == 1


def test_launch_nodes_with_same_names(pytestconfig):
    node1 = Node(
        package='demo_nodes_py',
        executable='listener_qos',
        name=TEST_NODE_NAME,
        namespace=TEST_NODE_NAMESPACE,
        output='screen',
    )

    node2 = LifecycleNode(
        package='lifecycle',
        executable='lifecycle_listener',
        name=TEST_NODE_NAME,
        namespace=TEST_NODE_NAMESPACE,
        output='screen',
    )

    node3 = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name=TEST_NODE_NAME,
        namespace=TEST_NODE_NAMESPACE,
        composable_node_descriptions=[
            ComposableNode(
                package='composition',
                plugin='composition::Listener',
                name=TEST_NODE_NAME
            )
        ],
        output='screen'
    )

    ld = LaunchDescription([node1, node2, node3])
    context = _launch(ld)
    captured = pytestconfig.pluginmanager.getplugin('capturemanager').read_global_capture()
    assert get_node_name_count(context, f'{TEST_NODE_NAMESPACE}/{TEST_NODE_NAME}') == 3
    expected_line = f'there are now at least 3 nodes with the name {TEST_NODE_NAMESPACE}' \
        f'/{TEST_NODE_NAME} created within this launch context'
    assert expected_line in captured.out
    assert get_node_name_count(context, f'/{TEST_NODE_NAME}') == 1


def test_launch_nodes_with_different_names():
    node1 = Node(
        package='demo_nodes_py',
        executable='listener_qos',
        name=f'{TEST_NODE_NAME}_1',
        namespace=TEST_NODE_NAMESPACE,
        output='screen',
    )

    node2 = LifecycleNode(
        package='lifecycle',
        executable='lifecycle_listener',
        name=f'{TEST_NODE_NAME}_2',
        namespace=TEST_NODE_NAMESPACE,
        output='screen',
    )

    node3 = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name=f'{TEST_NODE_NAME}_3',
        namespace=TEST_NODE_NAMESPACE,
        composable_node_descriptions=[
            ComposableNode(
                package='composition',
                plugin='composition::Listener',
                name=f'{TEST_NODE_NAME}_4',
            )
        ],
        output='screen'
    )

    ld = LaunchDescription([node1, node2, node3])
    context = _launch(ld)
    assert get_node_name_count(context, f'{TEST_NODE_NAMESPACE}/{TEST_NODE_NAME}_1') == 1
    assert get_node_name_count(context, f'{TEST_NODE_NAMESPACE}/{TEST_NODE_NAME}_2') == 1
    assert get_node_name_count(context, f'{TEST_NODE_NAMESPACE}/{TEST_NODE_NAME}_3') == 1
    assert get_node_name_count(context, f'/{TEST_NODE_NAME}_4') == 1
