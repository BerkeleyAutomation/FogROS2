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

import contextlib

import launch
import launch.actions
import launch.events
from launch.utilities import ensure_argument_type

from ..io_handler import ActiveIoHandler
from ..proc_info_handler import ActiveProcInfoHandler


class ProcessProxy:
    """A proxy to interact with `launch.actions.ExecuteProcess` instances."""

    def __init__(self, process_action, proc_info, proc_output, *, output_filter=None):
        """
        Construct a proxy for the given ``process_action``.

        :param process_action: `launch.actions.ExecuteProcess` instance to proxy.
        :param proc_info: `ActiveProcInfoHandler` tracking process state.
        :param proc_output: `ActiveIoHandler` tracking process output.
        :param output_filter: an optional callable to filter output text.
        """
        ensure_argument_type(
            process_action, types=launch.actions.ExecuteProcess, argument_name='process_action'
        )
        ensure_argument_type(proc_info, types=ActiveProcInfoHandler, argument_name='proc_info')
        ensure_argument_type(proc_output, types=ActiveIoHandler, argument_name='proc_output')
        if output_filter is not None and not callable(output_filter):
            raise TypeError(
                "Expected 'output_filter' to be callable but got '{!r}'".format(output_filter)
            )
        self._process_action = process_action
        self._proc_info = proc_info
        self._proc_output = proc_output
        self._output_filter = output_filter

    def wait_for_shutdown(self, timeout=None):
        """
        Wait for the target process to shutdown.

        :param timeout: time in seconds to wait, or None to block indefinitely.
        :return: whether the target process shut down or not.
        """
        with self._proc_info.proc_event:
            return self._proc_info.proc_event.wait_for(
                lambda: self.terminated, timeout=timeout
            )

    def wait_for_output(self, condition=None, timeout=None):
        """
        Wait for the target process to produce any output, either over stdout or stderr.

        :param condition: a callable to wait on a specific output condition to be satisfied,
            or ``None`` (default) to wake on any output.
        :param timeout: time in seconds to wait, or ``None`` (default) to block indefinitely.
        :return: whether the condition has been satisfied or not.
        """
        if condition is None:
            condition = (lambda output: True)
        if not callable(condition):
            raise TypeError(
                "Expected 'condition' to be callable but got '{!r}'".format(condition)
            )

        actual_output = None

        def remember_output():
            nonlocal actual_output
            actual_output = self.output
            return actual_output

        class BoolWithText:

            def __init__(self, result, output):
                self._result = result
                self._output = output

            def __bool__(self):
                return self._result

            def __repr__(self):
                return f'<BoolWithText({self._result}): {repr(self._output)}>'

        with self._proc_output.io_event:
            bool_result = self._proc_output.io_event.wait_for(
                lambda: self.running and condition(remember_output()), timeout=timeout
            )
            return BoolWithText(bool_result, actual_output)

    @property
    def target_process_action(self):
        return self._process_action

    @property
    def stderr(self):
        output_events = self._proc_output[self._process_action]
        output_text = ''.join(ev.text.decode() for ev in output_events if ev.from_stderr)
        if self._output_filter is not None:
            output_text = self._output_filter(output_text)
        return output_text

    @property
    def stdout(self):
        output_events = self._proc_output[self._process_action]
        output_text = ''.join(ev.text.decode() for ev in output_events if ev.from_stdout)
        if self._output_filter is not None:
            output_text = self._output_filter(output_text)
        return output_text

    @property
    def output(self):
        output_events = self._proc_output[self._process_action]
        output_text = ''.join(ev.text.decode() for ev in output_events)
        if self._output_filter is not None:
            output_text = self._output_filter(output_text)
        return output_text

    @property
    def running(self):
        if self._process_action not in self._proc_info.processes():
            return False
        return isinstance(
            self._proc_info[self._process_action],
            launch.events.process.ProcessStarted
        )

    @property
    def terminated(self):
        if self._process_action not in self._proc_info.processes():
            return False
        return isinstance(
            self._proc_info[self._process_action],
            launch.events.process.ProcessExited
        )

    @property
    def exit_code(self):
        return self._proc_info[self._process_action].returncode


@contextlib.contextmanager
def launch_process(launch_service, process_action, proc_info, proc_output, **kwargs):
    """
    Launch and interact with a process.

    On context entering, start execution of a ``process_action`` using the given ``launch_service``
    and yield a `ProcessProxy` to that ``process_action``.
    On context exiting, shut the process down if it has not been terminated yet.
    All additional arguments are forwarded to `ProcessProxy` on construction.
    """
    ensure_argument_type(
        process_action, types=launch.actions.ExecuteProcess, argument_name='process_action'
    )

    launch_service.emit_event(
        event=launch.events.IncludeLaunchDescription(
            launch_description=launch.LaunchDescription([process_action])
        )
    )
    process_proxy = ProcessProxy(process_action, proc_info, proc_output, **kwargs)
    try:
        yield process_proxy
    finally:
        if not process_proxy.terminated:
            launch_service.emit_event(
                event=launch.events.process.ShutdownProcess(
                    process_matcher=launch.events.matches_action(process_action)
                )
            )
