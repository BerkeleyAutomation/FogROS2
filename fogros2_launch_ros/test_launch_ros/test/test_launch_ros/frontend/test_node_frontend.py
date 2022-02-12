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

import io
import pathlib
import sys
import textwrap

from launch import LaunchService
from launch.frontend import Parser

from launch_ros.utilities import evaluate_parameters

yaml_params = str(pathlib.Path(__file__).parent / 'params.yaml')

# Escape backslashes if any to keep them after parsing takes place
yaml_params = yaml_params.replace('\\', '\\\\')
python_executable = sys.executable.replace('\\', '\\\\')


def test_launch_frontend_xml():
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <let name="a_string" value="\'[2, 5, 8]\'"/>
            <let name="a_list" value="[2, 5, 8]"/>
            <let name="my_value" value="100"/>
            <node pkg="demo_nodes_py" exec="talker_qos" output="screen" name="my_talker" namespace="my_ns" exec_name="my_talker_process" args="--number_of_cycles 1" ros_args="--log-level info">
                <param name="param1" value="ads"/>
                <param name="param_group1">
                    <param name="param_group2">
                        <param name="param2" value="2"/>
                    </param>
                    <param name="param3" value="2, 5, 8" value-sep=", "/>
                    <param name="param4" value="$(var a_list)"/>
                    <param name="param5" value="$(var a_string)"/>
                    <param name="param6" value="2., 5., 8." value-sep=", "/>
                    <param name="param7" value="'2', '5', '8'" value-sep=", "/>
                    <param name="param8" value="&quot;'2'&quot;, &quot;'5'&quot;, &quot;'8'&quot;" value-sep=", "/>
                    <param name="param9" value="\'2\', \'5\', \'8\'" value-sep=", "/>
                    <param name="param10" value="&quot;'asd'&quot;, &quot;'bsd'&quot;, &quot;'csd'&quot;" value-sep=", "/>
                    <param name="param11" value="'\asd', '\bsd', '\csd'" value-sep=", "/>
                    <param name="param12" value="''"/>
                    <param name="param13" value="$(var my_value)" type="str"/>
                    <param name="param14" value="'2', '5', '8'" value-sep=", " type="list_of_str"/>
                    <param name="param15" value="2, 5, 8" value-sep=", " type="list_of_str"/>
                </param>
                <param from="{}"/>
                <env name="var" value="1"/>
                <remap from="foo" to="bar"/>
                <remap from="baz" to="foobar"/>
            </node>
            <node exec="{}" args="-c 'import sys; print(sys.argv[1:])'" name="my_listener" namespace="my_ns" output="screen"/>
        </launch>
        """.format(yaml_params, python_executable))  # noqa: E501

    with io.StringIO(xml_file) as f:
        check_launch_node(f)


def test_launch_frontend_yaml():
    yaml_file = textwrap.dedent(
        r"""
        launch:
            - let:
                name: 'a_string'
                value: "'[2, 5, 8]'"
            - let:
                name: 'a_list'
                value: '[2, 5, 8]'
            - let:
                name: 'my_value'
                value: '100'
            - node:
                pkg: demo_nodes_py
                exec: talker_qos
                output: screen
                name: my_talker
                namespace: my_ns
                exec_name: my_talker_process
                args: '--number_of_cycles 1'
                ros_args: '--log-level info'
                param:
                    -   name: param1
                        value: ads
                    -   name: param_group1
                        param:
                        -   name: param_group2
                            param:
                            -   name: param2
                                value: 2
                        -   name: param3
                            value: [2, 5, 8]
                        -   name: param4
                            value: $(var a_list)
                        -   name: param5
                            value: $(var a_string)
                        -   name: param6
                            value: [2., 5., 8.]
                        -   name: param7
                            value: ['2', '5', '8']
                        -   name: param8
                            value: ["'2'", "'5'", "'8'"]
                        -   name: param9
                            value: ["\\'2\\'", "\\'5\\'", "\\'8\\'"]
                        -   name: param10
                            value: ["'asd'", "'bsd'", "'csd'"]
                        -   name: param11
                            value: ['\asd', '\bsd', '\csd']
                        -   name: param12
                            value: ''
                        -   name: param13
                            value: '$(var my_value)'
                            type: str
                        -   name: param14
                            value: ["'2'", "'5'", "'8'"]
                            type: list_of_str
                        -   name: param15
                            value: ['2', '5', '8']
                            type: list_of_str
                    -   from: {}
                env:
                    -   name: var
                        value: '1'
                remap:
                    -   from: "foo"
                        to: "bar"
                    -   from: "baz"
                        to: "foobar"
            - node:
                exec: {}
                output: screen
                namespace: my_ns
                name: my_listener
                args: -c 'import sys; print(sys.argv[1:])'
        """.format(yaml_params, python_executable))  # noqa: E501

    with io.StringIO(yaml_file) as f:
        check_launch_node(f)


def check_launch_node(file):
    root_entity, parser = Parser.load(file)
    ld = parser.parse_description(root_entity)
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert(0 == ls.run())
    evaluated_parameters = evaluate_parameters(
        ls.context,
        ld.describe_sub_entities()[3]._Node__parameters
    )
    assert len(evaluated_parameters) == 3
    assert isinstance(evaluated_parameters[0], dict)
    assert isinstance(evaluated_parameters[1], dict)
    assert isinstance(evaluated_parameters[2], pathlib.Path)

    assert 'param1' in evaluated_parameters[0]
    assert evaluated_parameters[0]['param1'] == 'ads'

    param_dict = evaluated_parameters[1]
    assert 'param_group1.param_group2.param2' in param_dict
    assert 'param_group1.param3' in param_dict
    assert 'param_group1.param4' in param_dict
    assert 'param_group1.param5' in param_dict
    assert 'param_group1.param6' in param_dict
    assert 'param_group1.param7' in param_dict
    assert 'param_group1.param8' in param_dict
    assert 'param_group1.param9' in param_dict
    assert 'param_group1.param10' in param_dict
    assert 'param_group1.param11' in param_dict
    assert 'param_group1.param12' in param_dict
    assert 'param_group1.param13' in param_dict
    assert 'param_group1.param14' in param_dict
    assert 'param_group1.param15' in param_dict
    assert param_dict['param_group1.param_group2.param2'] == 2
    assert param_dict['param_group1.param3'] == [2, 5, 8]
    assert param_dict['param_group1.param4'] == [2, 5, 8]
    assert param_dict['param_group1.param5'] == '[2, 5, 8]'
    assert param_dict['param_group1.param6'] == [2., 5., 8.]
    assert param_dict['param_group1.param7'] == ['2', '5', '8']
    assert param_dict['param_group1.param8'] == ["'2'", "'5'", "'8'"]
    assert param_dict['param_group1.param9'] == ["'2'", "'5'", "'8'"]
    assert param_dict['param_group1.param10'] == ["'asd'", "'bsd'", "'csd'"]
    assert param_dict['param_group1.param11'] == ['asd', 'bsd', 'csd']
    assert param_dict['param_group1.param12'] == ''
    assert param_dict['param_group1.param13'] == '100'
    assert param_dict['param_group1.param14'] == ["'2'", "'5'", "'8'"]
    assert param_dict['param_group1.param15'] == ['2', '5', '8']

    # Check remappings exist
    remappings = ld.describe_sub_entities()[3]._Node__remappings
    assert remappings is not None
    assert len(remappings) == 2

    talker_node_action = ld.describe_sub_entities()[3]
    talker_node_cmd_string = ' '.join(talker_node_action.process_details['cmd'])
    assert '--ros-args --log-level info' in talker_node_cmd_string

    listener_node_action = ld.describe_sub_entities()[4]
    listener_node_cmd = listener_node_action.process_details['cmd']
    assert [
        sys.executable, '-c', 'import sys; print(sys.argv[1:])'
    ] == listener_node_cmd[:3]
