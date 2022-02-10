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
import sys
import types
import unittest

import ament_index_python
import launch.actions
import launch.substitutions
import launch_testing
import launch_testing.actions
from launch_testing.loader import LoadTestsFromPythonModule
from launch_testing.test_runner import LaunchTestRunner
import launch_testing.util


class TestResolveProcess(unittest.TestCase):

    def test_unlaunched_process_lookup(self):
        info_obj = launch_testing.ProcInfoHandler()

        lookup_obj = launch.actions.ExecuteProcess(
            cmd=[
                'python',
                '-c',
                '',
            ]
        )

        with self.assertRaises(launch_testing.util.NoMatchingProcessException) as cm:
            launch_testing.util.resolveProcesses(
                info_obj,
                process=lookup_obj
            )

        # We'll get a good error mesasge here because there were no substitutions in
        # the execute process cmd - it's all text
        self.assertIn('python -c', str(cm.exception))

    def test_unlaunched_process_lookup_with_substitutions(self):
        info_obj = launch_testing.ProcInfoHandler()

        lookup_obj = launch.actions.ExecuteProcess(
            cmd=[
                launch.substitutions.LocalSubstitution('foo'),
                'python',
                '-c',
                '',
            ]
        )

        with self.assertRaises(launch_testing.util.NoMatchingProcessException) as cm:
            launch_testing.util.resolveProcesses(
                info_obj,
                process=lookup_obj
            )

        # Since the process wasn't launched yet, and it has substitutions that need to be
        # resolved by the launch system, we won't be able to take a guess at the command
        self.assertIn('Unknown', str(cm.exception))


class TestStringProcessResolution(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Run a launch so we can get some real looking proc_info and proc_output objects to test
        # against
        proc_command = os.path.join(
            ament_index_python.get_package_prefix('launch_testing'),
            'lib/launch_testing',
            'good_proc.py',
        )
        proc_env = os.environ.copy()
        proc_env['PYTHONUNBUFFERED'] = '1'

        def generate_test_description():
            no_arg_proc = launch.actions.ExecuteProcess(
                cmd=[sys.executable],
                env=proc_env
            )

            one_arg_proc = launch.actions.ExecuteProcess(
                cmd=[sys.executable, proc_command, '--one-arg'],
                env=proc_env
            )

            two_arg_proc = launch.actions.ExecuteProcess(
                cmd=[sys.executable, proc_command, '--two-arg', 'arg_two'],
                env=proc_env
            )

            ld = launch.LaunchDescription([
                no_arg_proc,
                one_arg_proc,
                two_arg_proc,
                launch_testing.actions.ReadyToTest(),
            ])

            return (ld, locals())

        arr = []

        class PreShutdownTests(unittest.TestCase):

            def test_wait_self(self,
                               proc_output,
                               proc_info,
                               no_arg_proc,
                               one_arg_proc,
                               two_arg_proc):
                proc_output.assertWaitFor('--one-arg', stream='stdout')
                proc_output.assertWaitFor('--two-arg', stream='stdout')
                proc_output.assertWaitFor('arg_two', stream='stdout')

                arr.append(proc_info)

        # Set up a fake module containing the test data:
        test_module = types.ModuleType('test_module')
        test_module.generate_test_description = generate_test_description
        test_module.FakePreShutdownTests = PreShutdownTests

        # Run the test:
        runner = LaunchTestRunner(
            LoadTestsFromPythonModule(test_module)
        )

        runner.run()

        # Grab the very realistic proc_info object to use for tests below
        cls.proc_info = arr[0]

    def test_proc_string_lookup_multiple_args(self):
        found_proc = launch_testing.util.resolveProcesses(
            self.proc_info,
            process=os.path.basename(sys.executable),
            cmd_args=['--two-arg', 'arg_two']
        )

        assert found_proc

    def test_proc_string_lookup_no_args(self):
        found_proc = launch_testing.util.resolveProcesses(
            self.proc_info,
            process=os.path.basename(sys.executable),
            cmd_args=launch_testing.util.NO_CMD_ARGS
        )

        assert found_proc

    def test_strict_proc_matching(self):
        with self.assertRaisesRegex(Exception, 'Found multiple processes'):
            launch_testing.util.resolveProcesses(
                self.proc_info,
                process=os.path.basename(sys.executable),
            )

        found_proc = launch_testing.util.resolveProcesses(
            self.proc_info,
            process=os.path.basename(sys.executable),
            strict_proc_matching=False
        )

        assert len(found_proc) == 3

    def test_string_cmd_args(self):
        # Old versions let you pass a string to cmd_args instead of a list of strings
        # but that made it impossible to match multiple command line arguments.
        # This test checks the old way still works

        found_proc = launch_testing.util.resolveProcesses(
            self.proc_info,
            process=os.path.basename(sys.executable),
            cmd_args='--one-arg',
        )

        assert found_proc
