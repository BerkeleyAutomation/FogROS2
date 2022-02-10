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
import re
import sys
import unittest

import ament_index_python

import launch
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.event_handlers import OnProcessStart

from launch_testing import ActiveIoHandler
from launch_testing.asserts import assertInStdout
# from launch_testing.asserts import NO_CMD_ARGS


TEST_PROC_PATH = os.path.join(
    ament_index_python.get_package_prefix('launch_testing'),
    'lib/launch_testing',
    'terminating_proc.py'
)
TEST_CMD = [sys.executable, TEST_PROC_PATH]


class TestIoHandlerAndAssertions(unittest.TestCase):

    EXPECTED_TEXT = 'Ready'  # Expected to be in every test run
    NOT_FOUND_TEXT = 'Zazzlefraz'  # Not expected to be in the output anywhere

    @classmethod
    def setUpClass(cls):
        # It's easier to actually capture some IO from the launch system than it is to fake it
        # but it takes a few seconds.  We'll do it once and run tests on the same captured
        # IO

        proc_env = os.environ.copy()
        proc_env['PYTHONUNBUFFERED'] = '1'

        cls.proc_output = ActiveIoHandler()

        cls.proc_1 = launch.actions.ExecuteProcess(
            cmd=TEST_CMD,
            name='terminating_proc.py',
            env=proc_env
        )

        # This process should be distinguishable by its cmd line args
        cls.proc_2 = launch.actions.ExecuteProcess(
            cmd=TEST_CMD + ['--extra'],
            name='terminating_proc.py',
            env=proc_env
        )

        # This process should be distinguishable by its different name
        cls.proc_3 = launch.actions.ExecuteProcess(
            cmd=TEST_CMD + ['node:=different_name'],
            name='terminating_proc.py',
            env=proc_env
        )

        launch_description = launch.LaunchDescription([
            cls.proc_1,
            cls.proc_2,
            cls.proc_3,
            # This plumbs all the output to our IoHandler just like the LaunchTestRunner does
            RegisterEventHandler(
                OnProcessStart(
                    on_start=lambda event, _: cls.proc_output.track(event.process_name)
                )
            ),
            RegisterEventHandler(
                OnProcessIO(
                    on_stdout=cls.proc_output.append,
                    on_stderr=cls.proc_output.append,
                )
            )
        ])

        launch_service = launch.LaunchService()
        launch_service.include_launch_description(launch_description)
        launch_service.run()

    def test_all_processes_had_io(self):
        # Should have three processes
        self.assertEqual(3, len(self.proc_output.processes()))

    def test_only_one_process_had_arguments(self):
        text_lines = [t.text.decode() for t in self.proc_output]
        print('All text: {}'.format(text_lines))

        matches = [t for t in text_lines if 'Called with arguments' in t]
        print('Called with arguments: {}'.format(matches))

        # Two process have args, because thats how process names are passed down
        self.assertEqual(2, len(matches))

        matches_extra = [t for t in matches if '--extra' in t]
        self.assertEqual(1, len(matches_extra))

        matches_proc_name = [t for t in matches if 'node:=different_name' in t]
        self.assertEqual(1, len(matches_proc_name))

    def test_assert_wait_for_returns_immediately(self):
        # If the output has already been seen, ensure that assertWaitsFor returns right away
        self.proc_output.assertWaitFor('Starting Up', timeout=1, stream='stdout')

    def test_EXPECTED_TEXT_is_present(self):
        # Sanity check - makes sure the EXPECTED_TEXT is somewhere in the test run
        text_lines = [t.text.decode() for t in self.proc_output]
        contains_ready = [self.EXPECTED_TEXT in t for t in text_lines]
        self.assertTrue(any(contains_ready))

    def test_process_names(self):
        self.assertIn('terminating_proc.py-1', self.proc_output.process_names())
        self.assertIn('terminating_proc.py-2', self.proc_output.process_names())
        self.assertIn('terminating_proc.py-3', self.proc_output.process_names())

    def test_processes(self):
        self.assertIn(self.proc_1, self.proc_output.processes())
        self.assertIn(self.proc_2, self.proc_output.processes())
        self.assertIn(self.proc_3, self.proc_output.processes())

    # ---------- Tests for assertInStdout below this line ----------
    def test_assertInStdout_notices_no_matching_proc(self):
        with self.assertRaisesRegex(Exception, 'Did not find any process') as cm:
            assertInStdout(self.proc_output, self.EXPECTED_TEXT, 'bad_proc_name')

        print(cm.exception)

        # Make sure the assertion method lists the names of the process it does have:
        self.assertIn('terminating_proc.py-1', str(cm.exception))
        self.assertIn('terminating_proc.py-2', str(cm.exception))
        self.assertIn('terminating_proc.py-3', str(cm.exception))

    def test_assertInStdout_notices_too_many_matching_procs(self):
        with self.assertRaisesRegex(Exception, 'Found multiple processes') as cm:
            assertInStdout(self.proc_output, self.EXPECTED_TEXT, 'terminating_proc.py')

        # Make sure the assertion method lists the names of the duplicate procs:
        self.assertIn('terminating_proc.py-1', str(cm.exception))
        self.assertIn('terminating_proc.py-2', str(cm.exception))
        self.assertIn('terminating_proc.py-3', str(cm.exception))

    def test_strict_proc_matching_false(self):
        assertInStdout(
            self.proc_output,
            self.EXPECTED_TEXT,
            'terminating_proc.py',
            strict_proc_matching=False
        )

    def test_regex_matching(self):
        assertInStdout(
            self.proc_output,
            re.compile(r'Called with arguments \S+'),
            'terminating_proc.py-2'
        )

    def test_arguments_disambiguate_processes(self):
        txt = self.EXPECTED_TEXT
        assertInStdout(self.proc_output, txt, 'terminating_proc.py', '--extra')
        assertInStdout(self.proc_output, txt, 'terminating_proc.py', 'node:=different_name')
        # TODO(hidmic): add NO_CMD_ARGS test again when and if its implementation
        # improves beyond checking argument count.
        # assertInStdout(self.proc_output, txt, 'terminating_proc.py', NO_CMD_ARGS)

    def test_asserts_on_missing_text(self):
        with self.assertRaisesRegex(AssertionError, self.NOT_FOUND_TEXT):
            assertInStdout(self.proc_output, self.NOT_FOUND_TEXT, 'terminating_proc.py-1')

    def test_asserts_on_missing_text_by_proc(self):
        with self.assertRaisesRegex(AssertionError, self.NOT_FOUND_TEXT):
            assertInStdout(self.proc_output, self.NOT_FOUND_TEXT, self.proc_2)
