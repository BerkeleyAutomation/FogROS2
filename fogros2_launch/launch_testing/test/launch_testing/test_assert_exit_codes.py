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

import unittest

from launch.events.process import ProcessExited

from launch_testing import ProcInfoHandler
from launch_testing.asserts import assertExitCodes


class TestExitCodes(unittest.TestCase):

    def setUp(self):
        self.dummy_proc_info = ProcInfoHandler()

        for n in range(4):
            proc_data = ProcessExited(
                action=object(),
                name='process_{}'.format(n),
                cmd=['process'],
                pid=n,
                returncode=0,
                cwd=None,
                env=None,
            )
            self.dummy_proc_info.append(proc_data)

    def test_assert_exit_codes_no_error(self):
        assertExitCodes(self.dummy_proc_info)

    def test_assert_exit_codes_notices_error(self):
        self.dummy_proc_info.append(
            ProcessExited(
                action=object(),
                name='test_process_1',
                cmd=['test_process'],
                pid=10,
                returncode=-1,
                cwd=None,
                env=None,
            )
        )

        with self.assertRaises(AssertionError) as cm:
            assertExitCodes(self.dummy_proc_info)

        # Check that the process name made it into the error message
        self.assertIn('test_process_1', str(cm.exception))

    def test_assert_exit_code_allows_specific_codes(self):
        self.dummy_proc_info.append(
            ProcessExited(
                action=object(),
                name='test_process_1',
                cmd=['test_process'],
                pid=10,
                returncode=131,
                cwd=None,
                env=None,
            )
        )

        assertExitCodes(self.dummy_proc_info, allowable_exit_codes=[0, 131])
