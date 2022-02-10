# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import sys

import launch
import launch_pytest
from launch_pytest import tools

import pytest

PYTHON_SCRIPT = """\
import sys
import time

print('hello')
print('world', file=sys.stderr)
time.sleep(5)
"""


@pytest.fixture
def dut():
    return launch.actions.ExecuteProcess(
        cmd=[sys.executable, '-c', PYTHON_SCRIPT],
        cached_output=True,
        output='screen'
    )


@launch_pytest.fixture
def launch_description(dut):
    return launch.LaunchDescription([
        dut,
    ])


@pytest.mark.launch(fixture=launch_description)
async def test_async_process_tools(dut, launch_context):
    assert await tools.wait_for_start(launch_context, dut, timeout=10)

    def check_output(output):
        assert output.splitlines() == ['hello']
    await tools.assert_output(
        launch_context, dut, check_output, timeout=10)

    def check_stderr(err):
        assert err.splitlines() == ['world']
    await tools.assert_stderr(
        launch_context, dut, check_stderr, timeout=10)
    assert await tools.wait_for_exit(launch_context, dut, timeout=10)


@pytest.mark.launch(fixture=launch_description)
def test_sync_process_tools(dut, launch_context):
    tools.wait_for_start_sync(launch_context, dut, timeout=10)

    def check_output(output):
        assert output.splitlines() == ['hello']
    tools.assert_output_sync(
        launch_context, dut, check_output, timeout=10)

    def check_stderr(err):
        assert err.splitlines() == ['world']
    tools.assert_stderr_sync(
        launch_context, dut, check_stderr, timeout=10)
    assert tools.wait_for_exit_sync(launch_context, dut, timeout=10)
