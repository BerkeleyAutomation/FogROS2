# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Tests for the Node Action."""

import os
import pathlib
from typing import List
import unittest

from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import Shutdown
from launch.substitutions import EnvironmentVariable

import launch_ros.actions.node
from launch_ros.descriptions import Parameter
from launch_ros.descriptions import ParameterValue

import pytest
import yaml


class TestNode(unittest.TestCase):

    def _assert_launch_errors(self, actions):
        ld = LaunchDescription(actions)
        ls = LaunchService(debug=True)
        ls.include_launch_description(ld)
        assert 0 != ls.run()

    def _assert_launch_no_errors(self, actions):
        ld = LaunchDescription(actions)
        ls = LaunchService(debug=True)
        ls.include_launch_description(ld)
        assert 0 == ls.run()

    def _create_node(self, *, parameters=None, remappings=None, ros_arguments=None):
        return launch_ros.actions.Node(
            package='demo_nodes_py', executable='talker_qos', output='screen',
            name='my_node', namespace='my_ns',
            exec_name='my_node_process',
            ros_arguments=ros_arguments,
            arguments=['--number_of_cycles', '1'],
            parameters=parameters,
            remappings=remappings,
        )

    def _assert_type_error_creating_node(self, *, parameters=None, remappings=None):
        with self.assertRaises(TypeError):
            self._create_node(parameters=parameters, remappings=remappings)

    def test_launch_invalid_node(self):
        """Test launching an invalid node."""
        node_action = launch_ros.actions.Node(
            package='nonexistent_package', executable='node', output='screen')
        self._assert_launch_errors([node_action])

    def test_launch_node(self):
        """Test launching a node."""
        self._assert_launch_no_errors([self._create_node()])

    def test_launch_node_with_remappings(self):
        """Test launching a node with remappings."""
        # Pass remapping rules to node in a variety of forms.
        # It is redundant to pass the same rule, but the goal is to test different parameter types.
        os.environ['TOPIC_NAME'] = 'chatter'
        topic_prefix = 'new_'
        node_action = self._create_node(
            remappings=[
                ('chatter', 'new_chatter'),
                (EnvironmentVariable(name='TOPIC_NAME'), [
                    topic_prefix, EnvironmentVariable(name='TOPIC_NAME')])
            ],
        )
        self._assert_launch_no_errors([node_action])

        # Check the expanded parameters.
        expanded_remappings = node_action._Node__expanded_remappings
        assert len(expanded_remappings) == 2
        for i in range(2):
            assert expanded_remappings[i] == ('chatter', 'new_chatter')

    def test_launch_node_with_ros_arguments(self):
        node_action = self._create_node(ros_arguments=['--log-level', 'debug'])
        self._assert_launch_no_errors([node_action])

        cmd_string = ' '.join(node_action.process_details['cmd'])
        assert '--ros-args --log-level debug' in cmd_string

    def test_launch_required_node(self):
        # This node will never exit on its own, it'll keep publishing forever.
        long_running_node = launch_ros.actions.Node(
            package='demo_nodes_py', executable='talker_qos', output='screen',
            namespace='my_ns',
        )

        # This node will exit after publishing a single message. It is required, so we
        # tie on_exit to the Shutdown action which means that, once it exits, it should
        # bring down the whole launched system, including the above node that will never
        # exit on its own.
        required_node = launch_ros.actions.Node(
            package='demo_nodes_py', executable='talker_qos', output='screen',
            namespace='my_ns2', arguments=['--number_of_cycles', '1'],
            on_exit=Shutdown()
        )

        # If the on_exit functionality or Shutdown action breaks, this will never return.
        self._assert_launch_no_errors([required_node, long_running_node])

    def test_create_node_with_invalid_remappings(self):
        """Test creating a node with invalid remappings."""
        self._assert_type_error_creating_node(
            remappings={'chatter': 'new_chatter'},  # Not a list.
        )

        self._assert_type_error_creating_node(
            remappings=[{'chatter': 'new_chatter'}],  # List with elements not tuple.
        )

    def test_launch_node_with_parameter_files(self):
        """Test launching a node with parameters specified in yaml files."""
        parameters_file_dir = pathlib.Path(__file__).resolve().parent
        parameters_file_path = parameters_file_dir / 'example_parameters_0.yaml'
        # Pass parameter files to node in a variety of forms.
        # It is redundant to pass the same file, but the goal is to test different parameter types.
        os.environ['FILE_PATH'] = str(parameters_file_dir)
        node_action = self._create_node(
            parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'example_parameters_0.yaml'],
            ],
        )
        self._assert_launch_no_errors([node_action])

        # Check the expanded parameters.
        expanded_parameter_arguments = node_action._Node__expanded_parameter_arguments
        assert len(expanded_parameter_arguments) == 3
        for i in range(3):
            assert expanded_parameter_arguments[i] == (str(parameters_file_path), True)

    def test_launch_node_with_parameter_descriptions(self):
        """Test launching a node with parameters specified in a dictionary."""
        os.environ['PARAM1_VALUE'] = 'param1_value'
        os.environ['PARAM2'] = 'param2'
        node_action = self._create_node(
            parameters=[
                Parameter(
                    name='param1',
                    value=EnvironmentVariable(name='PARAM1_VALUE'),
                    value_type=str,
                ),
                Parameter(
                    name=EnvironmentVariable(name='PARAM2'),
                    value=[[EnvironmentVariable(name='PARAM2')], '_value'],
                    value_type=List[str],
                ),
                Parameter(
                    name='param_group1.list_params',
                    value=[1.2, 3.4],
                ),
                Parameter(
                    name=['param_group1.param_group2.', EnvironmentVariable('PARAM2'), '_values'],
                    value=['param2_value'],
                ),
                Parameter(
                    name='param3',
                    value='',
                ),
            ],
        )
        self._assert_launch_no_errors([node_action])

        expanded_parameter_arguments = node_action._Node__expanded_parameter_arguments
        assert len(expanded_parameter_arguments) == 5
        parameters = []
        for item, is_file in expanded_parameter_arguments:
            assert not is_file
            name, value = item.split(':=')
            parameters.append((name, yaml.safe_load(value)))
        assert parameters == [
            ('param1', 'param1_value'),
            ('param2', ['param2', '_value']),
            ('param_group1.list_params', [1.2, 3.4]),
            ('param_group1.param_group2.param2_values', ['param2_value']),
            ('param3', ''),
        ]

    def test_launch_node_with_parameter_dict(self):
        """Test launching a node with parameters specified in a dictionary."""
        os.environ['PARAM1_VALUE'] = 'param1_value'
        os.environ['PARAM2'] = 'param2'
        os.environ['PARAM4_INT'] = '100'
        node_action = self._create_node(
            parameters=[{
                'param1': EnvironmentVariable(name='PARAM1_VALUE'),
                EnvironmentVariable(name='PARAM2'): (EnvironmentVariable(name='PARAM2'), '_value'),
                'param_group1': {
                    'list_params': [1.2, 3.4],
                    'param_group2': {
                        (EnvironmentVariable('PARAM2'), '_values'): ['param2_value'],
                    }
                },
                'param3': '',
                'param4': ParameterValue(EnvironmentVariable(name='PARAM4_INT'), value_type=int),
            }],
        )
        self._assert_launch_no_errors([node_action])

        # Check the expanded parameters (will be written to a file).
        expanded_parameter_arguments = node_action._Node__expanded_parameter_arguments
        assert len(expanded_parameter_arguments) == 1
        file_path, is_file = expanded_parameter_arguments[0]
        assert is_file
        with open(file_path, 'r') as h:
            expanded_parameters_dict = yaml.load(h, Loader=yaml.FullLoader)
            assert expanded_parameters_dict == {
                '/my_ns/my_node': {
                    'ros__parameters': {
                        'param1': 'param1_value',
                        'param2': 'param2_value',
                        'param3': '',
                        'param4': 100,
                        'param_group1.list_params': (1.2, 3.4),
                        'param_group1.param_group2.param2_values': ('param2_value',),
                    }
                }
            }

    def test_create_node_with_invalid_parameters(self):
        """Test launching a node with invalid parameters."""
        self._assert_type_error_creating_node(parameters=[5.0])  # Invalid list values.
        self._assert_type_error_creating_node(parameters={'a': 5})  # Valid dict, not in a list.

        parameter_file_path = pathlib.Path(__file__).resolve().parent / 'example_parameters_0.yaml'
        self._assert_type_error_creating_node(
            parameters=str(parameter_file_path))  # Valid path, but not in a list.

        # If a parameter dictionary is specified, the node name is no longer required.
        node_action = launch_ros.actions.Node(
            package='demo_nodes_py', executable='talker_qos', output='screen',
            arguments=['--number_of_cycles', '1'],
            parameters=[{'my_param': 'value'}],
        )
        self._assert_launch_no_errors([node_action])

    def test_launch_node_with_invalid_parameter_dicts(self):
        """Test launching a node with invalid parameter dicts."""
        # Substitutions aren't expanded until the node action is executed, at which time a type
        # error should be raised and cause the launch to fail.
        # However, the types are checked in Node.__init__()
        # For each type of invalid parameter, check that they are detected at both the top-level
        # and at a nested level in the dictionary.

        # Key must be a string/Substitution evaluating to a string.
        with self.assertRaises(TypeError):
            self._create_node(parameters=[{5: 'asdf'}])

        with self.assertRaises(TypeError):
            self._create_node(parameters=[{
                'param_group': {
                    'param_subgroup': {
                        5: 'asdf',
                    },
                },
            }])

        # Nested lists are not supported.
        with self.assertRaises(TypeError):
            self._create_node(parameters=[{'param': [1, 2, [3, 4]]}])

        with self.assertRaises(TypeError):
            self._create_node(parameters=[{
                'param_group': {
                    'param_subgroup': {
                        'param': [1, 2, [3, 4]],
                    },
                },
            }])

        # Other types are not supported.
        with self.assertRaises(TypeError):
            self._create_node(parameters=[{'param': {1, 2}}])

        with self.assertRaises(TypeError):
            self._create_node(parameters=[{
                'param_group': {
                    'param_subgroup': {
                        'param': {1, 2},
                    },
                },
            }])

        with self.assertRaises(TypeError):
            self._create_node(parameters=[{'param': self}])

        with self.assertRaises(TypeError):
            self._create_node(parameters=[{
                'param_group': {
                    'param_subgroup': {
                        'param': self,
                    },
                },
            }])


def get_test_node_name_parameters():
    return [
        pytest.param(
            launch_ros.actions.Node(
                package='asd',
                executable='bsd',
                name='my_node',
            ),
            False,
            id='Node without namespace'
        ),
        pytest.param(
            launch_ros.actions.Node(
                package='asd',
                executable='bsd',
                namespace='my_ns',
            ),
            False,
            id='Node without name'
        ),
        pytest.param(
            launch_ros.actions.Node(
                package='asd',
                executable='bsd',
                name='my_node',
                namespace='my_ns',
            ),
            True,
            id='Node with fully qualified name'
        ),
    ]


@pytest.mark.parametrize(
    'node_object, expected_result',
    get_test_node_name_parameters()
)
def test_node_name(node_object, expected_result):
    lc = LaunchContext()
    node_object._perform_substitutions(lc)
    assert node_object.is_node_name_fully_specified() is expected_result
