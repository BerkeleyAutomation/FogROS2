# Copyright 2019 Open Source Robotics Foundation, Inc.
# Copyright 2020 Open Avatar Inc.
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

import asyncio
import io
import textwrap

from launch import LaunchService
from launch.frontend import Parser
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters
import osrf_pycommon.process_utils


def test_launch_component_container_yaml():
    yaml_file = textwrap.dedent(
        r"""
        launch:
            - node_container:
                pkg: rclcpp_components
                exec: component_container
                name: my_container
                namespace: ''
                args: 'test_args'
                composable_node:
                    -   pkg: composition
                        plugin: composition::Talker
                        name: talker
                        namespace: test_namespace
                        remap:
                            -   from: chatter
                                to: /remap/chatter
                        param:
                            -   name: use_sim_time
                                value: true
                        extra_arg:
                            -   name: use_intra_process_comms
                                value: 'true'

            - load_composable_node:
                target: my_container
                composable_node:
                    -   pkg: composition
                        plugin: composition::Listener
                        name: listener
                        namespace: test_namespace
                        remap:
                            -   from: chatter
                                to: /remap/chatter
                        param:
                            -   name: use_sim_time
                                value: true
                        extra_arg:
                            -   name: use_intra_process_comms
                                value: 'true'
        """
    )
    with io.StringIO(yaml_file) as f:
        check_launch_component_container(f)


def test_launch_component_container_xml():
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <node_container pkg="rclcpp_components" exec="component_container" name="my_container" namespace="" args="test_args">
                <composable_node pkg="composition" plugin="composition::Talker" name="talker" namespace="test_namespace">
                    <remap from="chatter" to="/remap/chatter" />
                    <param name="use_sim_time" value="true"/>
                    <extra_arg name="use_intra_process_comms" value="true"/>
                </composable_node>
            </node_container>

            <load_composable_node target="my_container">
                <composable_node pkg="composition" plugin="composition::Listener" name="listener" namespace="test_namespace">
                    <remap from="chatter" to="/remap/chatter" />
                    <param name="use_sim_time" value="true"/>
                    <extra_arg name="use_intra_process_comms" value="true"/>
                </composable_node>
            </load_composable_node>
        </launch>
        """  # noqa: E501
    )
    with io.StringIO(xml_file) as f:
        check_launch_component_container(f)


def check_launch_component_container(file):
    root_entity, parser = Parser.load(file)
    ld = parser.parse_description(root_entity)
    ls = LaunchService()
    ls.include_launch_description(ld)

    loop = osrf_pycommon.process_utils.get_loop()
    launch_task = loop.create_task(ls.run_async())

    node_container, load_composable_node = ld.describe_sub_entities()
    talker = node_container._ComposableNodeContainer__composable_node_descriptions[0]
    listener = load_composable_node._LoadComposableNodes__composable_node_descriptions[0]

    def perform(substitution):
        return perform_substitutions(ls.context, substitution)

    # Check container params
    assert perform(node_container._Node__package) == 'rclcpp_components'
    assert perform(node_container._Node__node_executable) == 'component_container'
    assert perform(node_container._Node__node_name) == 'my_container'
    assert perform(node_container._Node__node_namespace) == ''
    assert perform(node_container._Node__arguments[0]) == 'test_args'

    assert perform(load_composable_node._LoadComposableNodes__target_container) == 'my_container'

    # Check node parameters
    talker_remappings = list(talker._ComposableNode__remappings)
    listener_remappings = list(listener._ComposableNode__remappings)

    talker_params = evaluate_parameters(ls.context, talker._ComposableNode__parameters)
    listener_params = evaluate_parameters(ls.context, listener._ComposableNode__parameters)

    talker_extra_args = evaluate_parameters(ls.context, talker._ComposableNode__extra_arguments)
    listener_extra_args = evaluate_parameters(
        ls.context, listener._ComposableNode__extra_arguments)

    assert perform(talker._ComposableNode__package) == 'composition'
    assert perform(talker._ComposableNode__node_plugin) == 'composition::Talker'
    assert perform(talker._ComposableNode__node_name) == 'talker'
    assert perform(talker._ComposableNode__node_namespace) == 'test_namespace'
    assert (perform(talker_remappings[0][0]),
            perform(talker_remappings[0][1])) == ('chatter', '/remap/chatter')
    assert talker_params[0]['use_sim_time'] is True

    assert perform(listener._ComposableNode__package) == 'composition'
    assert perform(listener._ComposableNode__node_plugin) == 'composition::Listener'
    assert perform(listener._ComposableNode__node_name) == 'listener'
    assert perform(listener._ComposableNode__node_namespace) == 'test_namespace'
    assert (perform(listener_remappings[0][0]),
            perform(listener_remappings[0][1])) == ('chatter', '/remap/chatter')
    assert listener_params[0]['use_sim_time'] is True

    # Check extra arguments
    assert talker_extra_args[0]['use_intra_process_comms'] is True
    assert listener_extra_args[0]['use_intra_process_comms'] is True

    timeout_sec = 5
    loop.run_until_complete(asyncio.sleep(timeout_sec))
    if not launch_task.done():
        loop.create_task(ls.shutdown())
        loop.run_until_complete(launch_task)
    assert 0 == launch_task.result()
