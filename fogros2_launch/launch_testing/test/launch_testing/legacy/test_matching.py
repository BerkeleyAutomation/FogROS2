# Copyright 2015 Open Source Robotics Foundation, Inc.
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

import os
import sys
import tempfile

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess
from launch_testing.legacy import LaunchTestService
from launch_testing.legacy.output import create_output_test_from_file


def test_matching():
    # This temporary directory and files contained in it
    # will be deleted when the process ends.
    tempdir = tempfile.mkdtemp()
    output_file = tempdir + os.sep + 'testfile'
    full_output_file = output_file + '.regex'
    with open(full_output_file, 'w+') as f:
        f.write(r'this is line \d\nthis is line [a-z]')

    executable_command = [
        sys.executable, os.path.join(
            os.path.abspath(os.path.dirname(__file__)), 'matching.py'
        )
    ]

    ld = LaunchDescription()
    launch_test = LaunchTestService()
    action = launch_test.add_fixture_action(
        ld, ExecuteProcess(cmd=executable_command, output='screen')
    )
    launch_test.add_output_test(
        ld, action, create_output_test_from_file(output_file)
    )

    launch_service = LaunchService()
    launch_service.include_launch_description(ld)
    return_code = launch_test.run(launch_service)
    assert return_code == 0, 'Launch failed with exit code %r' % (return_code,)


if __name__ == '__main__':
    test_matching()
