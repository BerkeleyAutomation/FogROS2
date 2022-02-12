# Copyright 2021 Open Source Robotics Foundation, Inc.
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

"""Tests for the ResetLaunchConfigurations action class."""

from launch import LaunchContext
from launch.actions import ResetLaunchConfigurations
from launch.substitutions import LaunchConfiguration


def test_reset_launch_configurations_constructors():
    """Test the constructors for ResetLaunchConfigurations class."""
    ResetLaunchConfigurations()
    ResetLaunchConfigurations({})
    ResetLaunchConfigurations({'foo': 'FOO', 'bar': 'BAR'})


def test_reset_launch_configurations_execute():
    """Test the execute() of the ResetLaunchConfigurations class."""
    # Clear all existing launch configurations without initializer for
    # launch_configurations
    lc1 = LaunchContext()
    assert len(lc1.launch_configurations) == 0
    lc1.launch_configurations['foo'] = 'FOO'
    lc1.launch_configurations['bar'] = 'BAR'
    assert len(lc1.launch_configurations) == 2
    ResetLaunchConfigurations().visit(lc1)
    assert len(lc1.launch_configurations) == 0

    # Clear all existing launch configurations with initializer for
    # launch_configurations = None
    lc2 = LaunchContext()
    assert len(lc2.launch_configurations) == 0
    lc2.launch_configurations['foo'] = 'FOO'
    lc2.launch_configurations['bar'] = 'BAR'
    assert len(lc2.launch_configurations) == 2
    ResetLaunchConfigurations(launch_configurations=None).visit(lc2)
    assert len(lc2.launch_configurations) == 0

    # Clear all existing launch configurations with initializer for
    # launch_configurations = {}
    lc3 = LaunchContext()
    assert len(lc3.launch_configurations) == 0
    lc3.launch_configurations['foo'] = 'FOO'
    lc3.launch_configurations['bar'] = 'BAR'
    assert len(lc3.launch_configurations) == 2
    ResetLaunchConfigurations(launch_configurations={}).visit(lc3)
    assert len(lc3.launch_configurations) == 0

    # Pass through an existing launch configuration
    lc4 = LaunchContext()
    assert len(lc4.launch_configurations) == 0
    lc4.launch_configurations['foo'] = 'FOO'
    lc4.launch_configurations['bar'] = 'BAR'
    assert len(lc4.launch_configurations) == 2
    ResetLaunchConfigurations(launch_configurations={'foo': LaunchConfiguration('foo')}).visit(lc4)
    assert len(lc4.launch_configurations) == 1
    assert lc4.launch_configurations['foo'] == 'FOO'
    assert 'bar' not in lc4.launch_configurations.keys()

    # Add a launch configuration that did not exist
    lc5 = LaunchContext()
    assert len(lc5.launch_configurations) == 0
    lc5.launch_configurations['foo'] = 'FOO'
    lc5.launch_configurations['bar'] = 'BAR'
    assert len(lc5.launch_configurations) == 2
    ResetLaunchConfigurations(launch_configurations={'baz': 'BAZ'}).visit(lc5)
    assert len(lc5.launch_configurations) == 1
    assert lc5.launch_configurations['baz'] == 'BAZ'
    assert 'foo' not in lc5.launch_configurations.keys()
    assert 'bar' not in lc5.launch_configurations.keys()

    # Overwrite an existing launch configuration
    lc6 = LaunchContext()
    assert len(lc6.launch_configurations) == 0
    lc6.launch_configurations['foo'] = 'FOO'
    lc6.launch_configurations['bar'] = 'BAR'
    assert len(lc6.launch_configurations) == 2
    ResetLaunchConfigurations(launch_configurations={'foo': 'OOF'}).visit(lc6)
    assert len(lc6.launch_configurations) == 1
    assert lc6.launch_configurations['foo'] == 'OOF'
    assert 'bar' not in lc6.launch_configurations.keys()
