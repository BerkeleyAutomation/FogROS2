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
A module providing process output assertions.

To prevent pytest from rewriting this module assertions, please PYTEST_DONT_REWRITE.
See https://docs.pytest.org/en/latest/assert.html#disabling-assert-rewriting for
further reference.
"""

from osrf_pycommon.terminal_color import remove_ansi_escape_sequences

from ..tools.text import build_text_match
from ..util import resolveProcesses


def assertInStream(proc_output,
                   expected_output,
                   process,
                   cmd_args=None,
                   *,
                   output_filter=None,
                   strict_proc_matching=True,
                   strip_ansi_escape_sequences=True,
                   stream='stderr'):
    """
    Assert that 'output' was found in a stream of a process.

    :param proc_output: The process output captured by launch_test.  This is usually injected
    into test cases as self._proc_output
    :type proc_output: An launch_testing.IoHandler

    :param expected_output: The output to search for
    :type expected_output: string or regex pattern or a list of the aforementioned types

    :param process: The process whose output will be searched
    :type process: A string (search by process name) or a launch.actions.ExecuteProcess object

    :param cmd_args: Optional.  If 'process' is a string, cmd_args will be used to disambiguate
    processes with the same name.  Pass launch_testing.asserts.NO_CMD_ARGS to match a proc without
    command arguments
    :type cmd_args: string

    :param output_filter: Optional. A function to filter output before attempting any assertion.
    :type output_filter: callable

    :param strict_proc_matching: Optional (default True), If proc is a string and the combination
    of proc and cmd_args matches multiple processes, then strict_proc_matching=True will raise
    an error.
    :type strict_proc_matching: bool

    :param strip_ansi_escape_sequences: If True (default), strip ansi escape
    sequences from actual output before comparing with the output filter or
    expected output.
    :type strip_ansi_escape_sequences: bool

    :param stream: Which stream to examine.  This must be one of 'stderr' or 'stdout'.
    :type stream: string
    """
    resolved_procs = resolveProcesses(
        info_obj=proc_output,
        process=process,
        cmd_args=cmd_args,
        strict_proc_matching=strict_proc_matching
    )
    if output_filter is not None:
        if not callable(output_filter):
            raise ValueError('output_filter is not callable')
    output_match = build_text_match(expected_output)

    for proc in resolved_procs:  # Nominally just one matching proc
        if stream == 'stdout':
            full_output = ''.join(
                output.text.decode() for output in proc_output[proc] if output.from_stdout
            )
        elif stream == 'stderr':
            full_output = ''.join(
                output.text.decode() for output in proc_output[proc] if output.from_stderr
            )
        else:
            raise ValueError("Invalid value for stream; must be 'stdout' or 'stderr'")

        if strip_ansi_escape_sequences:
            full_output = remove_ansi_escape_sequences(full_output)
        if output_filter is not None:
            full_output = output_filter(full_output)
        if output_match(full_output) is not None:
            break
    else:
        names = ', '.join(sorted(p.process_details['name'] for p in resolved_procs))
        assert False, "Did not find '{}' in output for any of the matching processes: {}".format(
            expected_output, names
        )


def assertInStdout(proc_output,
                   expected_output,
                   process,
                   cmd_args=None,
                   *,
                   output_filter=None,
                   strict_proc_matching=True,
                   strip_ansi_escape_sequences=True):
    """
    Assert that 'output' was found in the standard output of a process.

    See the documentation for 'assertInStream' for full details.
    """
    assertInStream(proc_output, expected_output, process, cmd_args, output_filter=output_filter,
                   strict_proc_matching=strict_proc_matching,
                   strip_ansi_escape_sequences=strip_ansi_escape_sequences, stream='stdout')


def assertInStderr(proc_output,
                   expected_output,
                   process,
                   cmd_args=None,
                   *,
                   output_filter=None,
                   strict_proc_matching=True,
                   strip_ansi_escape_sequences=True):
    """
    Assert that 'output' was found in the standard error of a process.

    See the documentation for 'assertInStream' for full details.
    """
    assertInStream(proc_output, expected_output, process, cmd_args, output_filter=output_filter,
                   strict_proc_matching=strict_proc_matching,
                   strip_ansi_escape_sequences=strip_ansi_escape_sequences, stream='stderr')


def assertDefaultStream():
    """
    Return the stream that is used by default for 'assertInStream'.

    This is useful for writing tests that are compatible with both Eloquent and newer releases.
    """
    return 'stderr'
