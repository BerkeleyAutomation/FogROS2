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

"""Tests for the IncludeLaunchDescription action class."""

import os

from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionSource
from launch import LaunchService
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ResetLaunchConfigurations
from launch.actions import SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions

import pytest


def test_include_launch_description_constructors():
    """Test the constructors for IncludeLaunchDescription class."""
    IncludeLaunchDescription(LaunchDescriptionSource(LaunchDescription()))
    IncludeLaunchDescription(
        LaunchDescriptionSource(LaunchDescription()),
        launch_arguments={'foo': 'FOO'}.items())


def test_include_launch_description_methods():
    """Test the methods of the IncludeLaunchDescription class."""
    ld = LaunchDescription()
    action = IncludeLaunchDescription(LaunchDescriptionSource(ld))
    assert 'IncludeLaunchDescription' in action.describe()
    assert isinstance(action.describe_sub_entities(), list)
    assert isinstance(action.describe_conditional_sub_entities(), list)
    # Result should only contain the launch description as there are no launch arguments.
    assert action.visit(LaunchContext()) == [ld]
    assert action.get_asyncio_future() is None
    assert len(action.launch_arguments) == 0

    ld2 = LaunchDescription([action])
    action2 = IncludeLaunchDescription(LaunchDescriptionSource(ld2))
    assert 'IncludeLaunchDescription' in action2.describe()
    assert isinstance(action2.describe_sub_entities(), list)
    assert isinstance(action2.describe_conditional_sub_entities(), list)
    # Result should only contain the launch description as there are no launch arguments.
    assert action2.visit(LaunchContext()) == [ld2]
    assert action2.get_asyncio_future() is None
    assert len(action2.launch_arguments) == 0


def test_include_launch_description_launch_file_location():
    """Test the ability of the IncludeLaunchDescription class to set the launch file location."""
    ld = LaunchDescription()
    action = IncludeLaunchDescription(LaunchDescriptionSource(ld, '<script>'))
    assert 'IncludeLaunchDescription' in action.describe()
    assert isinstance(action.describe_sub_entities(), list)
    assert isinstance(action.describe_conditional_sub_entities(), list)
    lc1 = LaunchContext()
    # Result should only contain the launch description as there are no launch arguments.
    assert action.visit(lc1) == [ld]
    assert lc1.locals.current_launch_file_directory == '<script>'
    assert action.get_asyncio_future() is None

    this_file = os.path.abspath(__file__)
    ld2 = LaunchDescription()
    action2 = IncludeLaunchDescription(LaunchDescriptionSource(ld2, this_file))
    assert 'IncludeLaunchDescription' in action2.describe()
    assert isinstance(action2.describe_sub_entities(), list)
    assert isinstance(action2.describe_conditional_sub_entities(), list)
    lc2 = LaunchContext()
    # Result should only contain the launch description as there are no launch arguments.
    assert action2.visit(lc2) == [ld2]
    assert lc2.locals.current_launch_file_directory == os.path.dirname(this_file)
    assert action2.get_asyncio_future() is None


def test_include_launch_description_launch_arguments():
    """Test the interactions between declared launch arguments and IncludeLaunchDescription."""
    # test that arguments are set when given, even if they are not declared
    ld1 = LaunchDescription([])
    action1 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld1),
        launch_arguments={'foo': 'FOO'}.items(),
    )
    assert len(action1.launch_arguments) == 1
    lc1 = LaunchContext()
    result1 = action1.visit(lc1)
    assert len(result1) == 2
    assert isinstance(result1[0], SetLaunchConfiguration)
    assert perform_substitutions(lc1, result1[0].name) == 'foo'
    assert perform_substitutions(lc1, result1[0].value) == 'FOO'
    assert result1[1] == ld1

    # test that a declared argument that is not provided raises an error
    ld2 = LaunchDescription([DeclareLaunchArgument('foo')])
    action2 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld2)
    )
    lc2 = LaunchContext()
    with pytest.raises(RuntimeError) as excinfo2:
        action2.visit(lc2)
    assert 'Included launch description missing required argument' in str(excinfo2.value)

    # test that a declared argument that is not provided raises an error, but with other args set
    ld2 = LaunchDescription([DeclareLaunchArgument('foo')])
    action2 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld2),
        launch_arguments={'not_foo': 'NOT_FOO'}.items(),
    )
    lc2 = LaunchContext()
    with pytest.raises(RuntimeError) as excinfo2:
        action2.visit(lc2)
    assert 'Included launch description missing required argument' in str(excinfo2.value)
    assert 'not_foo' in str(excinfo2.value)

    # test that a declared argument with a default value that is not provided does not raise
    ld2 = LaunchDescription([DeclareLaunchArgument('foo', default_value='FOO')])
    action2 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld2)
    )
    lc2 = LaunchContext()
    action2.visit(lc2)

    # Test that default arguments in nested IncludeLaunchDescription actions do not raise
    ld1 = LaunchDescription([DeclareLaunchArgument('foo', default_value='FOO')])
    action1 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld1),
    )
    ld2 = LaunchDescription([action1, DeclareLaunchArgument('foo2')])
    action2 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld2),
        launch_arguments={'foo2': 'FOO2'}.items(),
    )
    lc2 = LaunchContext()
    action2.visit(lc2)

    # Test that provided launch arguments of nested IncludeLaunchDescription actions do not raise
    ld1 = LaunchDescription([DeclareLaunchArgument('foo')])
    action1 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld1), launch_arguments={'foo': 'FOO'}.items(),
    )
    ld2 = LaunchDescription([action1, DeclareLaunchArgument('foo2')])
    action2 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld2),
        launch_arguments={'foo2': 'FOO2'}.items(),
    )
    lc2 = LaunchContext()
    action2.visit(lc2)

    # Test that arguments can not be passed from the parent launch description
    ld1 = LaunchDescription([DeclareLaunchArgument('foo')])
    action1 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld1)
    )
    ld2 = LaunchDescription([action1, DeclareLaunchArgument('foo2')])
    action2 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld2),
        launch_arguments={'foo': 'FOO', 'foo2': 'FOO2'}.items(),
    )
    ld3 = LaunchDescription([action2])
    ls = LaunchService()
    ls.include_launch_description(ld3)
    assert 1 == ls.run()

    # Test that arguments can be redeclared in the parent launch description
    ld1 = LaunchDescription([DeclareLaunchArgument('foo')])
    action1 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld1)
    )
    ld2 = LaunchDescription([action1, DeclareLaunchArgument('foo'), DeclareLaunchArgument('foo2')])
    action2 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld2),
        launch_arguments={'foo': 'FOO', 'foo2': 'FOO2'}.items(),
    )
    lc2 = LaunchContext()
    action2.visit(lc2)

    # Test that arguments after a ResetLaunchConfigurations action are not checked
    ld1 = LaunchDescription([DeclareLaunchArgument('foo')])
    action1 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld1)
    )
    ld2 = LaunchDescription(
        [
            DeclareLaunchArgument('foo2'),
            ResetLaunchConfigurations(),
            SetLaunchConfiguration('foo', 'asd'),
            action1])
    action2 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld2),
        launch_arguments={'foo2': 'FOO2'}.items(),
    )
    lc2 = LaunchContext()
    action2.visit(lc2)


def test_include_python():
    """Test including Python, with and without explicit PythonLaunchDescriptionSource."""
    this_dir = os.path.dirname(os.path.abspath(__file__))
    simple_launch_file_path = os.path.join(this_dir,
                                           '..',
                                           'launch_description_source',
                                           'simple.launch.py')

    # Explicitly construct with PythonLaunchDescriptionSource
    plds = PythonLaunchDescriptionSource(simple_launch_file_path)
    action0 = IncludeLaunchDescription(plds)

    # Construct action with path instead of PythonLaunchDescriptionSource object
    action1 = IncludeLaunchDescription(simple_launch_file_path)

    # The two actions should be equivalent
    for action in [action0, action1]:
        assert 'IncludeLaunchDescription' in action.describe()
        assert isinstance(action.describe_sub_entities(), list)
        assert isinstance(action.describe_conditional_sub_entities(), list)
        # Result should only contain a single launch description as there are no launch arguments.
        assert len(action.visit(LaunchContext())) == 1
        assert action.get_asyncio_future() is None
        assert len(action.launch_arguments) == 0

        assert action.launch_description_source.location == simple_launch_file_path
