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

"""Test parsing a `DeclareLaunchArgument` action."""

import io
import textwrap

from launch.frontend import Parser

import pytest


def test_arg():
    xml_file = \
        """\
        <launch>
            <arg name="my_arg" default="asd" description="something"/>
        </launch>
        """
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = Parser.load(io.StringIO(xml_file))
    ld = parser.parse_description(root_entity)
    arg = ld.entities[0]
    assert 'my_arg' == arg.name
    assert 'asd' == ''.join([x.perform(None) for x in arg.default_value])
    assert 'something' == arg.description


def test_arg_with_choices():
    xml_file = \
        """\
        <launch>
            <arg name="my_arg" default="asd" description="something">
                <choice value="asd"/>
                <choice value="bsd"/>
            </arg>
        </launch>
        """
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = Parser.load(io.StringIO(xml_file))
    ld = parser.parse_description(root_entity)
    arg = ld.entities[0]
    assert 'my_arg' == arg.name
    assert 'asd' == ''.join([x.perform(None) for x in arg.default_value])
    assert "something. Valid choices are: ['asd', 'bsd']" == arg.description
    assert ['asd', 'bsd'] == arg.choices


def test_arg_wrong_attribute():
    xml_file = \
        """\
        <launch>
            <arg name="my_arg" whats_this="hello" default="asd" description="something"/>
        </launch>
        """
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = Parser.load(io.StringIO(xml_file))
    with pytest.raises(ValueError) as excinfo:
        parser.parse_description(root_entity)
    assert '`arg`' in str(excinfo.value)
    assert 'whats_this' in str(excinfo.value)


def test_arg_with_subtag():
    xml_file = \
        """\
        <launch>
            <arg name="my_arg" default="asd" description="something">
                <whats_this/>
            </arg>
        </launch>
        """
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = Parser.load(io.StringIO(xml_file))
    with pytest.raises(ValueError) as excinfo:
        parser.parse_description(root_entity)
    assert '`arg`' in str(excinfo.value)
    assert 'whats_this' in str(excinfo.value)
