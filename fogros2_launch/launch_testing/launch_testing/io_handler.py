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
A module providing process IO capturing classes.

To prevent pytest from rewriting this module assertions, please PYTEST_DONT_REWRITE.
See https://docs.pytest.org/en/latest/assert.html#disabling-assert-rewriting for
further reference.
"""


import threading

from .asserts.assert_output import assertInStream
from .util import NoMatchingProcessException
from .util import resolveProcesses


class IoHandler:
    """
    Holds stdout captured from running processes.

    This class provides helper methods to enumerate the captured IO by individual processes
    """

    def __init__(self):
        self._sequence_list = []  # A time-ordered list of IO from all processes
        self._process_name_dict = {}  # A dict of time ordered lists of IO key'd by the process

    def track(self, process_name):
        if process_name not in self._process_name_dict:
            self._process_name_dict[process_name] = []

    def append(self, process_io):
        self._sequence_list.append(process_io)
        if process_io.process_name not in self._process_name_dict:
            self._process_name_dict[process_io.process_name] = []
        self._process_name_dict[process_io.process_name].append(process_io)

    def __iter__(self):
        return self._sequence_list.__iter__()

    def processes(self):
        """
        Get an iterable of unique launch.events.process.RunningProcessEvent objects.

        :returns [launch.actions.ExecuteProcess]:
        """
        return [val[0].action for val in self._process_name_dict.values() if len(val) > 0]

    def process_names(self):
        """
        Get the name of all unique processes that generated IO.

        :returns [string]:
        """
        return self._process_name_dict.keys()

    def __getitem__(self, key):
        """
        Get the output for a given process or process name.

        :param key: The process to get the output for
        :type key: String, or launch.actions.ExecuteProcess
        """
        if isinstance(key, str):
            return list(self._process_name_dict[key])
        else:
            return list(self._process_name_dict[key.process_details['name']])


class ActiveIoHandler:
    """
    Holds stdout captured from running processes.

    The ActiveIoHandler is meant to be used when capturing is still in progress and provides
    additional synchronization, as well as methods to wait on incoming IO
    """

    def __init__(self):
        self._sync_lock = threading.Condition()
        # Deliberately not calling the 'super' constructor here.  We're building this class
        # by composition so we can still give out the unsynchronized version
        self._io_handler = IoHandler()

    @property
    def io_event(self):
        return self._sync_lock

    def track(self, process_name):
        with self._sync_lock:
            self._io_handler.track(process_name)
            self._sync_lock.notify()

    def append(self, process_io):
        with self._sync_lock:
            self._io_handler.append(process_io)
            self._sync_lock.notify()

    def __iter__(self):
        with self._sync_lock:
            return list(self._io_handler).__iter__()

    def processes(self):
        """
        Get an iterable of unique launch.events.process.RunningProcessEvent objects.

        :returns [launch.actions.ExecuteProcess]:
        """
        with self._sync_lock:
            return list(self._io_handler.processes())

    def process_names(self):
        """
        Get the name of all unique processes that generated IO.

        :returns [string]:
        """
        with self._sync_lock:
            return list(self._io_handler.process_names())

    def __getitem__(self, key):
        """
        Get the output for a given process or process name.

        :param key: The process to get the output for
        :type key: String, or launch.actions.ExecuteProcess
        """
        with self._sync_lock:
            return self._io_handler[key]

    def assertWaitFor(self, *args, **kwargs):
        success = self.waitFor(*args, **kwargs)
        assert success, 'Waiting for output timed out'

    def waitFor(
        self,
        expected_output,
        process=None,  # Will wait for IO from all procs by default
        cmd_args=None,
        *,
        strict_proc_matching=True,
        output_filter=None,
        timeout=10,
        strip_ansi_escape_sequences=True,
        stream='stderr',
    ):
        success = False

        def msg_found():
            try:
                assertInStream(
                    self._io_handler,  # Use unsynchronized, since this is called from a lock
                    expected_output=expected_output,
                    process=process,
                    cmd_args=cmd_args,
                    output_filter=output_filter,
                    strict_proc_matching=strict_proc_matching,
                    strip_ansi_escape_sequences=strip_ansi_escape_sequences,
                    stream=stream,
                )
                return True
            except NoMatchingProcessException:
                # This can happen if no processes have generated any output yet.  It's not fatal.
                return False
            except AssertionError:
                return False

        with self._sync_lock:
            # TODO(pete.baughman): Searching through all of the IO can be time consuming/wasteful.
            # We can optimize this by making a note of where we left off searching and only
            # searching new messages when we return from the wait.
            success = self._sync_lock.wait_for(
                msg_found,
                timeout=timeout
            )

        if not success:
            # Help the user a little.  It's possible that they gave us a bad process name and no
            # had no hope of matching anything.
            matches = resolveProcesses(self,
                                       process=process,
                                       cmd_args=cmd_args,
                                       strict_proc_matching=False)
            if len(matches) == 0:
                raise Exception(
                    "After timeout, found no processes matching '{}'  "
                    "It either doesn't exist, was never launched, "
                    "or didn't generate any output".format(
                        process
                    )
                )

        return success
