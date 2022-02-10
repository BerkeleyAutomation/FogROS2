# Copyright 2016 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess
from launch_testing.legacy import LaunchTestService


def test_env():
    ld = LaunchDescription()
    launch_test = LaunchTestService()

    env = os.environ.copy()
    try:
        sub_env = os.environ.copy()
        sub_env['testenv1'] = 'testval1'
        os.environ['testenv2'] = 'testval2'
        launch_test.add_test_action(ld, ExecuteProcess(
            cmd=[
                sys.executable,
                os.path.join(
                    os.path.abspath(
                        os.path.dirname(__file__)),
                    'check_env.py')],
            name='test_env',
            env=sub_env,
        ))
        launch_service = LaunchService()
        launch_service.include_launch_description(ld)
        return_code = launch_test.run(launch_service)
    finally:
        os.environ = env
    assert return_code == 0, 'Launch failed with exit code %r' % (return_code,)


if __name__ == '__main__':
    test_env()
