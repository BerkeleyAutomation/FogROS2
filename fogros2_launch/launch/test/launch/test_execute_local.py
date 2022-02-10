# Copyright 2021 Southwest Research Institute, All Rights Reserved.
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
#
# DISTRIBUTION A. Approved for public release; distribution unlimited.
# OPSEC #4584.

"""Tests for the ExecuteLocal Action."""

import os
import sys

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteLocal
from launch.actions import OpaqueFunction
from launch.actions import Shutdown
from launch.actions import TimerAction
from launch.descriptions import Executable

import pytest


@pytest.mark.parametrize('test_input,expected', [
    (None, [True, False]),
    ({'TEST_NEW_ENV': '2'}, [False, True])
])
def test_execute_process_with_env(test_input, expected):
    """Test launching a process with an environment variable."""
    os.environ['TEST_CHANGE_CURRENT_ENV'] = '1'
    additional_env = {'TEST_PROCESS_WITH_ENV': 'Hello World'}
    executable = ExecuteLocal(
        process_description=Executable(
            cmd=[sys.executable, 'TEST_PROCESS_WITH_ENV'],
            env=test_input,
            additional_env=additional_env
        ),
        output='screen'
    )
    ld = LaunchDescription([executable])
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    env = executable.process_details['env']
    assert env['TEST_PROCESS_WITH_ENV'] == 'Hello World'
    assert ('TEST_CHANGE_CURRENT_ENV' in env) is expected[0]
    if expected[0]:
        assert env['TEST_CHANGE_CURRENT_ENV'] == '1'
    assert ('TEST_NEW_ENV' in env) is expected[1]
    if expected[1]:
        assert env['TEST_NEW_ENV'] == '2'


def test_execute_process_with_on_exit_behavior():
    """Test a process' on_exit callback and actions are processed."""
    def on_exit_callback(event, context):
        on_exit_callback.called = True
    on_exit_callback.called = False

    executable_with_on_exit_callback = ExecuteLocal(
        process_description=Executable(cmd=[sys.executable, '-c', "print('callback')"]),
        output='screen', on_exit=on_exit_callback
    )
    assert len(executable_with_on_exit_callback.get_sub_entities()) == 0

    def on_exit_function(context):
        on_exit_function.called = True
    on_exit_function.called = False
    on_exit_action = OpaqueFunction(function=on_exit_function)
    executable_with_on_exit_action = ExecuteLocal(
        process_description=Executable(cmd=[sys.executable, '-c', "print('callback')"]),
        output='screen', on_exit=[on_exit_action]
    )
    assert executable_with_on_exit_action.get_sub_entities() == [on_exit_action]

    ld = LaunchDescription([
        executable_with_on_exit_callback,
        executable_with_on_exit_action
    ])
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert on_exit_callback.called
    assert on_exit_function.called


def test_execute_process_with_respawn():
    """Test launching a process with a respawn and respawn_delay attribute."""
    def on_exit_callback(event, context):
        on_exit_callback.called_count = on_exit_callback.called_count + 1
    on_exit_callback.called_count = 0

    respawn_delay = 2.0
    shutdown_time = 3.0   # to shutdown the launch service, so that the process only respawn once
    expected_called_count = 2   # normal exit and respawn exit

    def generate_launch_description():
        return LaunchDescription([

            ExecuteLocal(
                process_description=Executable(cmd=[sys.executable, '-c', "print('action')"]),
                respawn=True, respawn_delay=respawn_delay, on_exit=on_exit_callback
            ),

            TimerAction(
                period=shutdown_time,
                actions=[
                    Shutdown(reason='Timer expired')
                ]
            )
        ])

    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    assert 0 == ls.run()
    assert expected_called_count == on_exit_callback.called_count
