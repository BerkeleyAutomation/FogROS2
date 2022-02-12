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

import re

import launch.actions
import launch.events
import launch.launch_context
import launch_testing.io_handler
import launch_testing.proc_info_handler

from launch_testing.tools import basic_output_filter
from launch_testing.tools import expect_output
from launch_testing.tools import ProcessProxy


def test_basic_output_filter():
    filter_fn = basic_output_filter(
        filtered_patterns=[r'.*\[listener\].*']
    )

    assert filter_fn('[listener] I heard: foo') == ''

    assert filter_fn('[talker] I said: foo') == '[talker] I said: foo'

    input_content = """\
        [listener] I heard: foo
        [listener] I heard: bar
        [listener] I heard: foobar
    """.replace(' ' * 4, '')
    output_content = ''
    assert filter_fn(input_content) == output_content

    input_content = """\
        [talker] I said: foo
        [listener] I heard: bar
        [listener] I heard: foobar
    """.replace(' ' * 4, '')
    output_content = """\
        [talker] I said: foo
    """.replace(' ' * 4, '')
    assert filter_fn(input_content) == output_content

    input_content = """\
        [talker] I said: foo
        [talker] I said: bar
        [talker] I said: foobar
    """.replace(' ' * 4, '')
    output_content = input_content
    assert filter_fn(input_content) == output_content


def test_expect_output():
    output_text = """\
        [talker] I said: foo
        [listener] I heard: bar
        [talker] I said: foo!
        [listener] I heard: baz""".replace(' ' * 4, '')
    output_lines = output_text.splitlines()

    assert expect_output(expected_text=output_text, text=output_text)
    assert expect_output(expected_text=output_text, text=output_text, strict=True)
    assert expect_output(expected_lines=output_lines, text=output_text)
    assert expect_output(expected_lines=output_lines, text=output_text, strict=True)
    assert expect_output(expected_text=output_text, lines=output_lines)
    assert expect_output(expected_text=output_text, lines=output_lines, strict=True)
    assert expect_output(expected_lines=output_lines, lines=output_lines)
    assert expect_output(expected_lines=output_lines, lines=output_lines, strict=True)

    assert expect_output(
        expected_text=re.compile(r'^\[talker\].*$', re.M), text=output_text
    )
    assert not expect_output(
        expected_text=re.compile(r'^\[listener\].*$', re.M),
        text=output_text, strict=True
    )
    assert expect_output(
        expected_lines=[
            re.compile(r'^\[talker\].*$', re.M),
            re.compile(r'^\[listener\].*$', re.M)
        ] * 2,
        text=output_text,
        strict=True
    )


def test_process_proxy():
    proc_output = launch_testing.io_handler.ActiveIoHandler()
    proc_info = launch_testing.proc_info_handler.ActiveProcInfoHandler()
    process_action = launch.actions.ExecuteProcess(cmd=['ls', '-las'], name='ls')
    proxy = ProcessProxy(process_action, proc_info, proc_output)

    context = launch.launch_context.LaunchContext()
    process_action.prepare(context)

    assert not proxy.running
    assert not proxy.terminated

    proc_info.append(launch.events.process.ProcessStarted(
        action=process_action,
        name=process_action.name,
        cmd=process_action.cmd,
        cwd=process_action.cwd,
        env=process_action.env,
        pid=1001
    ))
    proc_output.track(process_action.name)

    assert proxy.running
    assert not proxy.terminated
    assert proxy.output == ''
    assert proxy.stdout == ''
    assert proxy.stderr == ''

    proc_output.append(launch.events.process.ProcessStdout(
        action=process_action,
        text='Foobar\n'.encode('utf-8'),
        name=process_action.name,
        cmd=process_action.cmd,
        cwd=process_action.cwd,
        env=process_action.env,
        pid=1001
    ))

    assert proxy.running
    assert not proxy.terminated
    assert proxy.output == 'Foobar\n'
    assert proxy.stdout == 'Foobar\n'
    assert proxy.stderr == ''

    proc_output.append(launch.events.process.ProcessStderr(
        action=process_action,
        text='Warning!\n'.encode('utf-8'),
        name=process_action.name,
        cmd=process_action.cmd,
        cwd=process_action.cwd,
        env=process_action.env,
        pid=1001
    ))

    assert proxy.running
    assert not proxy.terminated
    assert proxy.output == 'Foobar\nWarning!\n'
    assert proxy.stdout == 'Foobar\n'
    assert proxy.stderr == 'Warning!\n'

    proc_info.append(launch.events.process.ProcessExited(
        action=process_action,
        returncode=0,
        name=process_action.name,
        cmd=process_action.cmd,
        cwd=process_action.cwd,
        env=process_action.env,
        pid=1001
    ))

    assert not proxy.running
    assert proxy.terminated
    assert proxy.exit_code == 0
    assert proxy.output == 'Foobar\nWarning!\n'
    assert proxy.stdout == 'Foobar\n'
    assert proxy.stderr == 'Warning!\n'
