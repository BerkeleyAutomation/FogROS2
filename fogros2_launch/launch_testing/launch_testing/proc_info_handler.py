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

"""
A module providing process info capturing classes.

To prevent pytest from rewriting this module assertions, please PYTEST_DONT_REWRITE.
See https://docs.pytest.org/en/latest/assert.html#disabling-assert-rewriting for
further reference.
"""


import threading
from launch.actions import ExecuteProcess  # noqa
from launch.events.process import ProcessExited

from .util import NoMatchingProcessException
from .util import resolveProcesses


class ProcInfoHandler:
    """Captures exit codes from processes when they terminate."""

    def __init__(self):
        self._proc_info = {}

    def append(self, process_info):
        self._proc_info[process_info.action] = process_info

    def __iter__(self):
        return self._proc_info.values().__iter__()

    def processes(self):
        """Get the ExecuteProcess launch actions of all recorded processes."""
        return self._proc_info.keys()

    def process_names(self):
        """Get the name of all recorded processes."""
        return map(
            lambda x: x.process_details['name'],
            self._proc_info.keys()
        )

    def __getitem__(self, key):
        """
        Get the ProcessExited event for the specified process.

        :param key: Either a string, or a launch.actions.ExecuteProcess object
        :returns launch.events.process.ProcessExited:
        """
        if isinstance(key, str):
            # Look up by process name
            for (launch_action, value) in self._proc_info.items():
                if key in launch_action.process_details['name']:
                    return value
            else:
                raise KeyError(key)
        else:
            return self._proc_info[key]


class ActiveProcInfoHandler:
    """Allows tests to wait on process termination before proceeding."""

    def __init__(self):
        self._sync_lock = threading.Condition()
        # Deliberately not calling the 'super' constructor here.  We're building this class
        # by composition so we can still give out the unsynchronized version
        self._proc_info_handler = ProcInfoHandler()

    @property
    def proc_event(self):
        return self._sync_lock

    def append(self, process_info):
        with self._sync_lock:
            self._proc_info_handler.append(process_info)
            self._sync_lock.notify()

    def __iter__(self):
        with self._sync_lock:
            return self._proc_info_handler.__iter__()

    def processes(self):
        """
        Get the ExecuteProcess launch actions of all recorded processes.

        :returns [launch.actions.ExecuteProcess]:
        """
        with self._sync_lock:
            return list(self._proc_info_handler.processes())

    def process_names(self):
        """
        Get the name of all recorded processes.

        :returns [string]:
        """
        with self._sync_lock:
            return list(self._proc_info_handler.process_names())

    def __getitem__(self, key):
        with self._sync_lock:
            return self._proc_info_handler[key]

    def assertWaitForShutdown(self,
                              process,
                              cmd_args=None,
                              *,
                              timeout=10):

        success = False

        def proc_is_shutdown():
            try:
                resolved_process = resolveProcesses(
                    info_obj=self._proc_info_handler,
                    process=process,
                    cmd_args=cmd_args,
                    strict_proc_matching=True
                )[0]
                process_event = self._proc_info_handler[resolved_process]
                return isinstance(process_event, ProcessExited)
            except NoMatchingProcessException:
                return False

        with self._sync_lock:
            success = self._sync_lock.wait_for(
                proc_is_shutdown,
                timeout=timeout
            )

        assert success, "Timed out waiting for process '{}' to finish".format(process)

    def assertWaitForStartup(self,
                             process,
                             cmd_args=None,
                             *,
                             timeout=10):
        success = False

        def proc_is_started():
            try:
                resolveProcesses(
                    info_obj=self._proc_info_handler,
                    process=process,
                    cmd_args=cmd_args,
                    strict_proc_matching=True
                )
                return True
            except NoMatchingProcessException:
                return False

        with self._sync_lock:
            success = self._sync_lock.wait_for(
                proc_is_started,
                timeout=timeout
            )

        assert success, "Timed out waiting for process '{}' to start".format(process)
