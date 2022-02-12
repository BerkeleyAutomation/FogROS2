# Copyright 2019 Apex.AI, Inc.
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

from sys import executable as __executable

from launch.actions import ExecuteProcess as __ExecuteProcess

from .proc_lookup import NO_CMD_ARGS
from .proc_lookup import NoMatchingProcessException
from .proc_lookup import resolveProcesses


def KeepAliveProc():
    """
    Generate a dummy launch.actions.ExecuteProcess to keep the launch alive.

    launch_test expects to shut down the launch itself when it's done running tests.  If all
    of the processes under test are expected to terminate on their own, it's necessary to add
    another process to keep the launch service alive while the tests are running.
    """
    script = """
import signal
import time

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
"""
    return __ExecuteProcess(
        cmd=[
            __executable,
            '-c',
            script
        ],
    )


__all__ = [
    'resolveProcesses',

    'KeepAliveProc',
    'NoMatchingProcessException',

    'NO_CMD_ARGS',
]
