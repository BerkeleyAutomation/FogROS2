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


import launch

import launch_pytest
from launch_pytest.tools import process as process_tools

import pytest


@pytest.fixture
def hello_world_proc():
    # Launch a process to test
    return launch.actions.ExecuteProcess(
        cmd=['echo', 'hello_world'],
        shell=True,
        cached_output=True,
    )


# This function specifies the processes to be run for our test.
@launch_pytest.fixture
def launch_description(hello_world_proc):
    """Launch a simple process to print 'hello_world'."""
    return launch.LaunchDescription([
        hello_world_proc,
        # Tell launch when to start the test
        # If no ReadyToTest action is added, one will be appended automatically.
        launch_pytest.actions.ReadyToTest()
    ])


@pytest.mark.launch(fixture=launch_description)
def test_read_stdout(hello_world_proc, launch_context):
    """Check if 'hello_world' was found in the stdout."""
    def validate_output(output):
        # this function can use assertions to validate the output or return a boolean.
        # pytest generates easier to understand failures when assertions are used.
        assert output.splitlines() == ['hello_world'], 'process never printed hello_world'
    process_tools.assert_output_sync(
        launch_context, hello_world_proc, validate_output, timeout=5)

    def validate_output(output):
        return output == 'this will never happen'
    assert not process_tools.wait_for_output_sync(
        launch_context, hello_world_proc, validate_output, timeout=0.1)
    yield
    # this is executed after launch service shutdown
    assert hello_world_proc.return_code == 0
