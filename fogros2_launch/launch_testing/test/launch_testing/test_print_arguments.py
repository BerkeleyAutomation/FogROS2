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

import os
import subprocess

import ament_index_python


# Run the 'args.test.py' example with --show-args and verify the arguments are printed
# but the test does not run
def test_print_args():

    testpath = os.path.join(
        ament_index_python.get_package_share_directory('launch_testing'),
        'examples',
        'args_launch_test.py',
    )

    completed_process = subprocess.run(
        args=[
            'launch_test',
            testpath,
            '--show-args',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert 0 == completed_process.returncode
    # Take a look at examples/args.test.py to see where this expected output comes from
    assert 'dut_arg' in completed_process.stdout.decode()
    assert 'Passed to the terminating process' in completed_process.stdout.decode()


def test_no_args_to_print():

    testpath = os.path.join(
        ament_index_python.get_package_share_directory('launch_testing'),
        'examples',
        'good_proc_launch_test.py',
    )

    completed_process = subprocess.run(
        args=[
            'launch_test',
            testpath,
            '--show-args',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert 0 == completed_process.returncode
    assert 'No arguments.' in completed_process.stdout.decode()
