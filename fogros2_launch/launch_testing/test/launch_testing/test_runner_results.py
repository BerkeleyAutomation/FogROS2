# Copyright 2018 Apex.AI, Inc.
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
import unittest
import unittest.mock as mock

import ament_index_python
import launch
import launch.actions
import launch_testing
import launch_testing.actions
from launch_testing.loader import TestRun as TR
from launch_testing.test_runner import LaunchTestRunner


# Run tests on processes that die early with an exit code and make sure the results returned
# indicate failure
def test_dut_that_shuts_down(capsys):

    def generate_test_description():
        TEST_PROC_PATH = os.path.join(
            ament_index_python.get_package_prefix('launch_testing'),
            'lib/launch_testing',
            'terminating_proc.py'
        )

        return launch.LaunchDescription([
            launch.actions.ExecuteProcess(
                cmd=[sys.executable, TEST_PROC_PATH]
            ),

            launch_testing.actions.ReadyToTest(),
        ])

    with mock.patch('launch_testing.test_runner._RunnerWorker._run_test'):
        runner = LaunchTestRunner(
            [TR('', generate_test_description, {}, [], [])]
        )

        results = runner.run()

        for result in results.values():
            assert not result.wasSuccessful()

    # This is the negative version of the test below.  If no exit code, no extra output
    # is generated
    out, err = capsys.readouterr()
    assert 'Starting Up' not in out


def test_dut_that_has_exception(capsys):
    # This is the same as above, but we also want to check we get extra output from processes
    # that had an exit code

    def generate_test_description():
        TEST_PROC_PATH = os.path.join(
            ament_index_python.get_package_prefix('launch_testing'),
            'lib/launch_testing',
            'terminating_proc.py'
        )

        EXIT_PROC_PATH = os.path.join(
            ament_index_python.get_package_prefix('launch_testing'),
            'lib/launch_testing',
            'exit_code_proc.py'
        )

        return launch.LaunchDescription([
            launch.actions.ExecuteProcess(
                cmd=[sys.executable, TEST_PROC_PATH, '--exception']
            ),

            # This process makes sure we can handle processes that exit with a code but don't
            # generate any output.  This is a regression test for an issue fixed in PR31
            launch.actions.ExecuteProcess(
                cmd=[sys.executable, EXIT_PROC_PATH, '--silent']
            ),

            launch_testing.actions.ReadyToTest(),
        ])

    with mock.patch('launch_testing.test_runner._RunnerWorker._run_test'):
        runner = LaunchTestRunner(
            [TR('', generate_test_description, {}, [], [])]
        )

        results = runner.run()

        for result in results.values():
            assert not result.wasSuccessful()

    # Make sure some information about WHY the process died shows up in the output
    out, err = capsys.readouterr()
    assert 'Starting Up' in out
    assert 'Process had a pretend error' in out  # This is the exception text from exception_node


# Run some known good tests to check the nominal-good test path
def test_nominally_good_dut(source_test_loader):

    def test_ok(self):
        pass

    TEST_PROC_PATH = os.path.join(
        ament_index_python.get_package_prefix('launch_testing'),
        'lib/launch_testing',
        'good_proc.py'
    )

    def generate_test_description():
        return launch.LaunchDescription([
            launch.actions.ExecuteProcess(
                cmd=[sys.executable, TEST_PROC_PATH]
            ),

            launch_testing.actions.ReadyToTest(),
        ])

    runner = LaunchTestRunner(
        source_test_loader(
            generate_test_description,
            pre_shutdown_tests=[test_ok],
            post_shutdown_tests=[test_ok],
        )
    )

    results = runner.run()

    for result in results.values():
        assert result.wasSuccessful()


def test_parametrized_run_with_one_failure(source_test_loader):

    # Test Data
    @launch_testing.parametrize('arg_val', [1, 2, 3, 4, 5])
    def generate_test_description(arg_val):
        TEST_PROC_PATH = os.path.join(
            ament_index_python.get_package_prefix('launch_testing'),
            'lib/launch_testing',
            'good_proc.py'
        )

        # This is necessary to get unbuffered output from the process under test
        proc_env = os.environ.copy()
        proc_env['PYTHONUNBUFFERED'] = '1'

        return launch.LaunchDescription([
            launch.actions.ExecuteProcess(
                cmd=[sys.executable, TEST_PROC_PATH],
                env=proc_env,
            ),
            launch_testing.actions.ReadyToTest(),
        ])

    def test_fail_on_two(self, proc_output, arg_val):
        proc_output.assertWaitFor('Starting Up', stream='stdout')
        assert arg_val != 2

    def test_fail_on_three(self, arg_val):
        assert arg_val != 3

    # Run the test:
    runner = LaunchTestRunner(
        source_test_loader(
            generate_test_description,
            pre_shutdown_tests=[test_fail_on_two],
            post_shutdown_tests=[test_fail_on_three],
        )
    )
    results = runner.run()

    passes = [result for result in results.values() if result.wasSuccessful()]
    fails = [result for result in results.values() if not result.wasSuccessful()]

    assert len(passes) == 3  # 1, 4, and 5 should pass
    assert len(fails) == 2  # 2 fails in an active test, 3 fails in a post-shutdown test


def test_skipped_launch_description(source_test_loader):

    @unittest.skip('skip reason string')
    def generate_test_description():
        raise Exception('This should never be invoked')  # pragma: no cover

    def test_fail_always(self):
        assert False  # pragma: no cover

    def test_pass_always(self):
        pass  # pragma: no cover

    # Run the test:
    runner = LaunchTestRunner(
        source_test_loader(
            generate_test_description,
            pre_shutdown_tests=[test_fail_always],
            post_shutdown_tests=[test_fail_always, test_pass_always],
        )
    )

    results = runner.run()
    # Just one test run expected
    assert len(results.values()) == 1

    skip_result = next(iter(results.values()))
    # Make sure it looks like all three tests were skipped
    assert len(skip_result.skipped) == 3
    assert skip_result.testsRun == 3

    # XML Structure is checked in test_xml_output.py
