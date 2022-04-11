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

"""Tests for the LaunchDescription class."""

import collections.abc
import logging
import os

from launch import Action
from launch import LaunchDescription
from launch import LaunchDescriptionEntity
from launch import LaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

logging.getLogger('launch').setLevel(logging.DEBUG)


def test_launch_description_constructors():
    """Test the constructors for LaunchDescription class."""
    LaunchDescription()
    LaunchDescription(None)
    LaunchDescription([])
    ld = LaunchDescription([LaunchDescriptionEntity()])
    assert len(ld.entities) == 1


def test_launch_description_get_launch_arguments():
    """Test the get_launch_arguments() method of the LaunchDescription class."""
    ld = LaunchDescription([])
    assert len(ld.get_launch_arguments()) == 0

    ld = LaunchDescription([DeclareLaunchArgument('foo')])
    la = ld.get_launch_arguments()
    assert len(la) == 1
    assert la[0]._conditionally_included is False

    ld = LaunchDescription([DeclareLaunchArgument('foo', condition=IfCondition('True'))])
    la = ld.get_launch_arguments()
    assert len(la) == 1
    assert la[0]._conditionally_included is True

    ld = LaunchDescription([
        IncludeLaunchDescription(LaunchDescriptionSource(LaunchDescription([
            DeclareLaunchArgument('foo'),
        ]))),
    ])
    la = ld.get_launch_arguments()
    assert len(la) == 1

    this_dir = os.path.dirname(os.path.abspath(__file__))
    ld = LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(this_dir, 'launch_file_with_argument.launch.py'))),
    ])
    la = ld.get_launch_arguments()
    assert len(la) == 1

    # From issue #144: get_launch_arguments was broken when an entitity had conditional
    # sub entities
    class EntityWithConditional(LaunchDescriptionEntity):

        def describe_conditional_sub_entities(self):
            return [('String describing condition', [DeclareLaunchArgument('foo')])]

    ld = LaunchDescription([
        EntityWithConditional()
    ])
    la = ld.get_launch_arguments()
    assert len(la) == 1


def test_launch_description_add_things():
    """Test adding things to the LaunchDescription class."""
    ld = LaunchDescription()
    assert len(ld.entities) == 0
    ld.add_entity(LaunchDescription())
    assert len(ld.entities) == 1
    ld.add_action(Action())
    assert len(ld.entities) == 2


class MockLaunchContext:

    def get_locals_as_dict(self):
        return {}


def test_launch_description_visit():
    """Test visiting entities in the LaunchDescription class."""
    ld = LaunchDescription([LaunchDescriptionEntity()])
    ld.add_action(Action())

    result = ld.visit(MockLaunchContext())
    assert isinstance(result, collections.abc.Iterable)
    for entity in result:
        assert isinstance(entity, LaunchDescriptionEntity)


def test_launch_description_deprecated():
    ld = LaunchDescription(deprecated_reason='DEPRECATED MESSAGE')
    ld.visit(MockLaunchContext())
    assert ld.deprecated is True
    assert ld.deprecated_reason == 'DEPRECATED MESSAGE'
