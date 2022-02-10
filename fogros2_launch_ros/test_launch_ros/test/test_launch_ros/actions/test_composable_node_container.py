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

"""Tests for the ComposableNodeContainer Action."""

import asyncio

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.utilities import get_node_name_count

import osrf_pycommon.process_utils

TEST_CONTAINER_NAME = 'test_component_container_node_name'
TEST_CONTAINER_NAMESPACE = 'test_component_container_namespace'
TEST_NODE_NAME = 'test_composable_node_name'
TEST_NODE_NAMESPACE = 'test_composable_node_namespace'


def _assert_launch_no_errors(actions, *, timeout_sec=5):
    ld = LaunchDescription(actions)
    ls = LaunchService(debug=True)
    ls.include_launch_description(ld)

    loop = osrf_pycommon.process_utils.get_loop()
    launch_task = loop.create_task(ls.run_async())
    loop.run_until_complete(asyncio.sleep(timeout_sec))
    if not launch_task.done():
        loop.create_task(ls.shutdown())
        loop.run_until_complete(launch_task)
    assert 0 == launch_task.result()
    return ls.context


def test_composable_node_container():
    """Nominal test for launching a ComposableNodeContainer."""
    actions = [
        ComposableNodeContainer(
            package='rclcpp_components',
            executable='component_container',
            name=TEST_CONTAINER_NAME,
            namespace=TEST_CONTAINER_NAMESPACE,
            composable_node_descriptions=[
                ComposableNode(
                    package='composition',
                    plugin='composition::Listener',
                    name=TEST_NODE_NAME,
                    namespace=TEST_NODE_NAMESPACE,
                )
            ],
        ),
    ]

    context = _assert_launch_no_errors(actions)
    assert get_node_name_count(context, f'/{TEST_CONTAINER_NAMESPACE}/{TEST_CONTAINER_NAME}') == 1
    assert get_node_name_count(context, f'/{TEST_NODE_NAMESPACE}/{TEST_NODE_NAME}') == 1


def test_composable_node_container_empty_list_of_nodes():
    """Test launching a ComposableNodeContainer with an empty list of nodes."""
    actions = [
        ComposableNodeContainer(
            package='rclcpp_components',
            executable='component_container',
            name=TEST_CONTAINER_NAME,
            namespace=TEST_CONTAINER_NAMESPACE,
            composable_node_descriptions=[]
        ),
    ]

    context = _assert_launch_no_errors(actions)
    assert get_node_name_count(context, f'/{TEST_CONTAINER_NAMESPACE}/{TEST_CONTAINER_NAME}') == 1
    assert get_node_name_count(context, f'/{TEST_NODE_NAMESPACE}/{TEST_NODE_NAME}') == 0


def test_composable_node_container_in_group_with_launch_configuration_in_description():
    """
    Test launch configuration is passed to ComposableNode description inside GroupAction.

    This is a regression test for #114.
    """
    actions = [
        GroupAction([
            DeclareLaunchArgument(name='test_arg', default_value='True'),
            ComposableNodeContainer(
                package='rclcpp_components',
                executable='component_container',
                name=TEST_CONTAINER_NAME,
                namespace=TEST_CONTAINER_NAMESPACE,
                composable_node_descriptions=[
                    ComposableNode(
                        package='composition',
                        plugin='composition::Listener',
                        name=TEST_NODE_NAME,
                        namespace=TEST_NODE_NAMESPACE,
                        parameters=[{'use_sim_time': LaunchConfiguration('test_arg')}],
                    )
                ],
            ),
        ], scoped=True),
    ]

    context = _assert_launch_no_errors(actions)
    assert get_node_name_count(context, f'/{TEST_CONTAINER_NAMESPACE}/{TEST_CONTAINER_NAME}') == 1
    assert get_node_name_count(context, f'/{TEST_NODE_NAMESPACE}/{TEST_NODE_NAME}') == 1
