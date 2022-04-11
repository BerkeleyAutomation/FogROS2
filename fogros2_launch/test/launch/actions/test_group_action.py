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

"""Tests for the GroupAction action class."""

from launch import Action
from launch import LaunchContext
from launch.actions import GroupAction
from launch.actions import PopLaunchConfigurations
from launch.actions import PushLaunchConfigurations
from launch.actions import ResetLaunchConfigurations
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration


def test_group_action_constructors():
    """Test the constructors for the GroupAction class."""
    GroupAction([])
    GroupAction([Action()])
    GroupAction([Action()], scoped=False)
    GroupAction([Action()], scoped=False, forwarding=False)
    GroupAction([Action()], scoped=False, forwarding=False, launch_configurations={'foo': 'FOO'})


def test_group_action_execute():
    """Test the execute() of the the GroupAction class."""
    lc1 = LaunchContext()

    assert len(lc1.launch_configurations) == 0
    assert len(GroupAction([], scoped=False).visit(lc1)) == 0
    assert len(lc1.launch_configurations) == 0

    assert len(lc1.launch_configurations) == 0
    result = GroupAction([], forwarding=True).visit(lc1)
    assert len(result) == 2  # push and pop actions, due to scope=True, forwarded=True
    assert isinstance(result[0], PushLaunchConfigurations)
    assert isinstance(result[1], PopLaunchConfigurations)
    for a in result:
        a.visit(lc1)
    assert len(lc1.launch_configurations) == 0

    assert len(lc1.launch_configurations) == 0
    result = GroupAction([], forwarding=False).visit(lc1)
    assert len(result) == 3  # push, reset, pop actions, due to scope=True, forwarded=False
    assert isinstance(result[0], PushLaunchConfigurations)
    assert isinstance(result[1], ResetLaunchConfigurations)
    assert isinstance(result[2], PopLaunchConfigurations)
    for a in result:
        a.visit(lc1)
    assert len(lc1.launch_configurations) == 0

    assert len(lc1.launch_configurations) == 0
    result = GroupAction([], forwarding=True, launch_configurations={'foo': 'FOO'}).visit(lc1)
    assert len(result) == 3  # push, set 1 launch_configurations, and pop actions
    assert isinstance(result[0], PushLaunchConfigurations)
    assert isinstance(result[1], SetLaunchConfiguration)
    assert isinstance(result[2], PopLaunchConfigurations)
    for a in result:
        a.visit(lc1)
    assert len(lc1.launch_configurations) == 0

    assert len(lc1.launch_configurations) == 0
    result = GroupAction([], forwarding=True,
                         launch_configurations={'foo': 'FOO', 'bar': 'BAR'}).visit(lc1)
    assert len(result) == 4  # push, set 2 launch_configurations, and pop actions
    assert isinstance(result[0], PushLaunchConfigurations)
    assert isinstance(result[1], SetLaunchConfiguration)
    assert isinstance(result[2], SetLaunchConfiguration)
    assert isinstance(result[3], PopLaunchConfigurations)
    for a in result:
        a.visit(lc1)
    assert len(lc1.launch_configurations) == 0

    assert len(lc1.launch_configurations) == 0
    result = GroupAction([], forwarding=False,
                         launch_configurations={'foo': 'FOO', 'bar': 'BAR'}).visit(lc1)
    assert len(result) == 3  # push, reset (which will set launch_configurations), and pop actions
    assert isinstance(result[0], PushLaunchConfigurations)
    assert isinstance(result[1], ResetLaunchConfigurations)
    assert isinstance(result[2], PopLaunchConfigurations)
    for a in result:
        a.visit(lc1)
    assert len(lc1.launch_configurations) == 0

    assert len(lc1.launch_configurations) == 0
    PushLaunchConfigurations().visit(lc1)
    result = GroupAction([], scoped=False, launch_configurations={'foo': 'FOO'}).visit(lc1)
    assert len(result) == 1  # set 1 launch_configurations
    assert isinstance(result[0], SetLaunchConfiguration)
    for a in result:
        a.visit(lc1)
    assert len(lc1.launch_configurations) == 1  # still set after group was included
    PopLaunchConfigurations().visit(lc1)
    assert len(lc1.launch_configurations) == 0

    assert len(lc1.launch_configurations) == 0
    PushLaunchConfigurations().visit(lc1)
    result = GroupAction([Action()], scoped=False, launch_configurations={'foo': 'FOO'}).visit(lc1)
    assert len(result) == 2  # set 1 launch_configurations, then the 1 included actions
    assert isinstance(result[0], SetLaunchConfiguration)
    assert isinstance(result[1], Action)
    for a in result:
        a.visit(lc1)
    assert len(lc1.launch_configurations) == 1  # still set after group was included
    PopLaunchConfigurations().visit(lc1)
    assert len(lc1.launch_configurations) == 0

    assert len(lc1.launch_configurations) == 0
    result = GroupAction([Action()], forwarding=True,
                         launch_configurations={'foo': 'FOO'}).visit(lc1)
    assert len(result) == 4  # push, set 1 launch_configurations, the 1 action, and pop actions
    assert isinstance(result[0], PushLaunchConfigurations)
    assert isinstance(result[1], SetLaunchConfiguration)
    assert isinstance(result[2], Action)
    assert isinstance(result[3], PopLaunchConfigurations)
    for a in result:
        a.visit(lc1)
    assert len(lc1.launch_configurations) == 0

    assert len(lc1.launch_configurations) == 0
    lc1.launch_configurations['foo'] = 'FOO'
    lc1.launch_configurations['bar'] = 'BAR'
    result = GroupAction([Action()], forwarding=False,
                         launch_configurations={'bar': LaunchConfiguration('bar'),
                                                'baz': 'BAZ'}).visit(lc1)
    # push, reset (which will set launch_configurations), 1 action, and pop actions
    assert len(result) == 4
    assert isinstance(result[0], PushLaunchConfigurations)
    assert isinstance(result[1], ResetLaunchConfigurations)
    assert isinstance(result[2], Action)
    assert isinstance(result[3], PopLaunchConfigurations)
    result[0].visit(lc1)  # Push
    assert 'foo' in lc1.launch_configurations.keys()  # Copied to new scope, before Reset
    assert lc1.launch_configurations['foo'] == 'FOO'
    assert 'bar' in lc1.launch_configurations.keys()  # Copied to new scope, before Reset
    assert lc1.launch_configurations['bar'] == 'BAR'
    result[1].visit(lc1)  # Reset
    assert 'foo' not in lc1.launch_configurations.keys()  # Cleared from scope in Reset
    assert 'bar' in lc1.launch_configurations.keys()  # Evaluated and forwarded in Reset
    assert lc1.launch_configurations['bar'] == 'BAR'
    assert 'baz' in lc1.launch_configurations.keys()  # Evaluated and added in Reset
    assert lc1.launch_configurations['baz'] == 'BAZ'
    result[2].visit(lc1)  # Action
    result[3].visit(lc1)  # Pop
    assert 'foo' in lc1.launch_configurations.keys()  # Still in original scope
    assert lc1.launch_configurations['foo'] == 'FOO'
    assert 'bar' in lc1.launch_configurations.keys()  # Still in original scope
    assert lc1.launch_configurations['bar'] == 'BAR'
    assert 'baz' not in lc1.launch_configurations.keys()  # Out of scope from pop, no longer exists
    assert len(lc1.launch_configurations) == 2
    lc1.launch_configurations.clear()

    assert len(lc1.launch_configurations) == 0
    lc1.launch_configurations['foo'] = 'FOO'
    lc1.launch_configurations['bar'] = 'BAR'
    result = GroupAction([Action()], forwarding=True,
                         launch_configurations={'foo': 'OOF'}).visit(lc1)
    # push, 1 set (overwrite), 1 action, and pop actions
    assert len(result) == 4
    assert isinstance(result[0], PushLaunchConfigurations)
    assert isinstance(result[1], SetLaunchConfiguration)
    assert isinstance(result[2], Action)
    assert isinstance(result[3], PopLaunchConfigurations)
    result[0].visit(lc1)  # Push
    assert 'foo' in lc1.launch_configurations.keys()  # Copied to new scope, before Set
    assert lc1.launch_configurations['foo'] == 'FOO'
    assert 'bar' in lc1.launch_configurations.keys()  # Copied to new scope
    assert lc1.launch_configurations['bar'] == 'BAR'
    result[1].visit(lc1)  # Set
    assert 'foo' in lc1.launch_configurations.keys()  # Overwritten in Set
    assert lc1.launch_configurations['foo'] == 'OOF'
    assert 'bar' in lc1.launch_configurations.keys()  # Untouched
    assert lc1.launch_configurations['bar'] == 'BAR'
    result[2].visit(lc1)  # Action
    result[3].visit(lc1)  # Pop
    assert 'foo' in lc1.launch_configurations.keys()  # Still in original scope with original value
    assert lc1.launch_configurations['foo'] == 'FOO'
    assert 'bar' in lc1.launch_configurations.keys()  # Still in original scope with original value
    assert lc1.launch_configurations['bar'] == 'BAR'
    lc1.launch_configurations.clear()
