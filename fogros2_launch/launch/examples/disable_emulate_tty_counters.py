#!/usr/bin/env python3

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

"""
Script that demonstrates disabling tty emulation.

This is most significant for python processes which, without tty
emulation, will be buffered by default and have various other
capabilities disabled."
"""

import os
import sys
from typing import cast
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa

import launch  # noqa: E402


def generate_launch_description():
    ld = launch.LaunchDescription()

    # Disable tty emulation (on by default).
    ld.add_action(launch.actions.SetLaunchConfiguration('emulate_tty', 'false'))

    # Wire up stdout from processes
    def on_output(event: launch.Event) -> None:
        for line in event.text.decode().splitlines():
            print('[{}] {}'.format(
                cast(launch.events.process.ProcessIO, event).process_name, line))

    ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
        on_stdout=on_output,
    )))

    # Execute
    ld.add_action(launch.actions.ExecuteProcess(
        cmd=[sys.executable, './counter.py']
    ))
    return ld


if __name__ == '__main__':
    # ls = LaunchService(argv=argv, debug=True)  # Use this instead to get more debug messages.
    ls = launch.LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
