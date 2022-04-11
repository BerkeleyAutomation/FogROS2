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

"""Tests the Command substitution."""

import os
import pathlib

from launch.launch_context import LaunchContext
from launch.substitutions import Command
from launch.substitutions.substitution_failure import SubstitutionFailure

import pytest


@pytest.fixture
def commands():
    this_dir = pathlib.Path(__file__).parent

    commands = {
        'normal': str(this_dir / 'test_command' / 'normal_command.bash'),
        'failing': str(this_dir / 'test_command' / 'failing_command.bash'),
        'with_stderr': str(this_dir / 'test_command' / 'command_with_stderr.bash')
    }

    if os.name == 'nt':
        for key, value in commands.items():
            commands[key] = value.replace('bash', 'bat')
    return commands


def test_command(commands):
    """Test a simple command."""
    context = LaunchContext()
    command = Command(commands['normal'])
    output = command.perform(context)
    assert output == 'asd bsd csd\n'


def test_missing_command_raises(commands):
    """Test that a command that doesn't exist raises."""
    context = LaunchContext()
    command = Command('ros2_launch_test_command_i_m_not_a_command')
    with pytest.raises(SubstitutionFailure) as ex:
        command.perform(context)
    ex.match('file not found:')


def test_failing_command_rises(commands):
    """Test that a failing command raises."""
    context = LaunchContext()
    command = Command(commands['failing'])
    with pytest.raises(SubstitutionFailure) as ex:
        command.perform(context)
    ex.match('executed command failed. Command: .*failing_command')


def test_command_with_stderr_raises(commands):
    """Test that a command that produces stderr raises."""
    context = LaunchContext()
    command = Command(commands['with_stderr'])
    with pytest.raises(SubstitutionFailure) as ex:
        command.perform(context)
    ex.match(
        'executed command showed stderr output. Command: .*command_with_stderr'
        r'[\w\W]*asd bsd')


def test_command_with_stderr_ignored(commands):
    """Test `Command` substitution ignoring stderr."""
    context = LaunchContext()
    command = Command(commands['with_stderr'], on_stderr='ignore')
    output = command.perform(context)
    assert output == ''


def test_command_with_stderr_warn(commands):
    """Test `Command` substitution with `on_stderr='warn'`."""
    context = LaunchContext()
    command = Command(commands['with_stderr'], on_stderr='warn')
    output = command.perform(context)
    assert output == ''


def test_command_with_stderr_capture(commands):
    """Test `Command` substitution with `on_stderr='capture'`."""
    context = LaunchContext()
    command = Command(commands['with_stderr'], on_stderr='capture')
    output = command.perform(context)
    assert output == 'asd bsd\n'
