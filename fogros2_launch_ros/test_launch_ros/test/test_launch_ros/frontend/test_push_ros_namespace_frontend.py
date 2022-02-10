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
import textwrap

from launch import LaunchService
from launch.frontend import Parser


def test_launch_namespace_yaml():
    yaml_file = textwrap.dedent(
        r"""
        launch:
           - push-ros-namespace:
               namespace: 'asd'
        """
    )
    with io.StringIO(yaml_file) as f:
        check_launch_namespace(f)

    yaml_file_alias = textwrap.dedent(
        r"""
        launch:
           - push_ros_namespace:
               namespace: 'asd'
        """
    )
    with io.StringIO(yaml_file_alias) as f:
        check_launch_namespace(f)


def test_launch_namespace_xml():
    xml_file = textwrap.dedent(
        r"""
        <launch>
            <push-ros-namespace namespace="asd"/>
        </launch>
        """
    )
    with io.StringIO(xml_file) as f:
        check_launch_namespace(f)

    xml_file_alias = textwrap.dedent(
        r"""
        <launch>
            <push_ros_namespace namespace="asd"/>
        </launch>
        """
    )
    with io.StringIO(xml_file_alias) as f:
        check_launch_namespace(f)


def check_launch_namespace(file):
    root_entity, parser = Parser.load(file)
    ld = parser.parse_description(root_entity)
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert 'ros_namespace' in ls.context.launch_configurations
    assert '/asd' == ls.context.launch_configurations['ros_namespace']
