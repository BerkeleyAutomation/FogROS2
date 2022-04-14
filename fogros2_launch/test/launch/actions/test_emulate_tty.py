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

"""Tests for emulate_tty configuration of ExecuteProcess actions."""

import platform
import sys

import launch
import pytest


class OnExit(object):

    def __init__(self):
        self.returncode = None

    def handle(self, event, context):
        self.returncode = event.returncode


def tty_expected_unless_windows():
    return 1 if platform.system() != 'Windows' else 0


@pytest.mark.parametrize('test_input,expected', [
    # use the default defined by ExecuteProcess (default is off or "false" right now)
    (None, 0),
    # redundantly override the default via LaunchConfiguration
    ('true', tty_expected_unless_windows()),
    # override the default via LaunchConfiguration
    ('false', 0),
    # redundantly override the default via constructor
    (True, tty_expected_unless_windows()),
    # override the default via constructor
    (False, 0),
])
def test_emulate_tty(test_input, expected):
    on_exit = OnExit()
    ld = launch.LaunchDescription()
    kwargs = {}
    if isinstance(test_input, bool):
        kwargs['emulate_tty'] = test_input
    elif isinstance(test_input, str):
        ld.add_action(
            launch.actions.SetLaunchConfiguration(
                'emulate_tty',
                test_input
            )
        )
    ld.add_action(
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessExit(on_exit=on_exit.handle)
        )
    )
    ld.add_action(launch.actions.ExecuteProcess(
        cmd=[
            sys.executable,
            '-c',
            'import sys; sys.exit(sys.stdout.isatty())',
        ],
        **kwargs
    ))
    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    ls.run()
    assert on_exit.returncode == expected
