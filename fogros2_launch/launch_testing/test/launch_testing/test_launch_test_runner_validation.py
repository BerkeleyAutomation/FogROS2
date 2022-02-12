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

import importlib
import unittest

import launch
import launch.actions
import launch_testing
from launch_testing.actions import ReadyToTest
from launch_testing.loader import LoadTestsFromPythonModule
from launch_testing.test_runner import LaunchTestRunner


def make_test_run_for_dut(generate_test_description_function):
    test_spec = importlib.util.spec_from_loader('test_module', loader=None)
    module = importlib.util.module_from_spec(test_spec)
    module.generate_test_description = generate_test_description_function
    return LoadTestsFromPythonModule(module)


class TestLaunchTestRunnerValidation(unittest.TestCase):

    def test_catches_bad_signature(self):

        # A `generate_test_description` function without a ready_fn argument is allowed because
        # it might be a new style function that uses a ReadyToTest action to signal when it's time
        # for tests to start.
        # If there's no ReadyToTest action, we won't catch that until later because dut.validate()
        # doesn't actually invoke the function.
        # We will still expect to reject functions with wrong name arguments

        dut = LaunchTestRunner(
            make_test_run_for_dut(
                lambda misspelled_ready_fn: None
            )
        )

        with self.assertRaisesRegex(Exception, "unexpected extra argument 'misspelled_ready_fn'"):
            dut.validate()

        dut = LaunchTestRunner(
            make_test_run_for_dut(
                lambda ready_fn: None
            )
        )

        dut.validate()

    def test_too_many_arguments(self):

        dut = LaunchTestRunner(
            make_test_run_for_dut(lambda ready_fn, extra_arg: None)
        )

        with self.assertRaisesRegex(Exception, "unexpected extra argument 'extra_arg'"):
            dut.validate()

    def test_bad_parametrization_argument(self):

        @launch_testing.parametrize('bad_argument', [1, 2, 3])
        def bad_launch_description(ready_fn):
            pass  # pragma: no cover

        dut = LaunchTestRunner(
            make_test_run_for_dut(bad_launch_description)
        )

        with self.assertRaisesRegex(Exception, 'Could not find an argument') as cm:
            dut.validate()
        self.assertIn('bad_argument', str(cm.exception))


class TestNewStyleTestDescriptions(unittest.TestCase):
    # Tests for `generate_test_description` functions that include a ReadyToTest action in
    # the test description

    def test_good_launch_description(self):

        def generate_test_description():
            return launch.LaunchDescription([
                ReadyToTest()
            ])

        runs = make_test_run_for_dut(generate_test_description)
        dut = LaunchTestRunner(
            runs
        )

        dut.validate()  # Make sure this passes initial validation (probably redundant with above)
        runs[0].normalized_test_description(ready_fn=lambda: None)

    def test_launch_description_with_missing_ready_action(self):

        def generate_test_description():
            return launch.LaunchDescription([
            ])

        runs = make_test_run_for_dut(generate_test_description)
        dut = LaunchTestRunner(
            runs
        )

        dut.validate()  # Make sure this passes initial validation (probably redundant with above)

        with self.assertRaisesRegex(Exception, 'containing a ReadyToTest action'):
            runs[0].normalized_test_description(ready_fn=lambda: None)

    def test_launch_description_with_conditional_ready_action(self):

        def generate_test_description():
            return launch.LaunchDescription([
                launch.actions.TimerAction(
                    period=10.0,
                    actions=[ReadyToTest()]
                )
            ])

        runs = make_test_run_for_dut(generate_test_description)
        dut = LaunchTestRunner(
            runs
        )

        dut.validate()  # Make sure this passes initial validation (probably redundant with above)
        runs[0].normalized_test_description(ready_fn=lambda: None)

    def test_launch_description_with_multiple_conditionals_and_deeper_nesting(self):

        def generate_test_description():
            return launch.LaunchDescription([
                launch.actions.LogInfo(msg='Dummy Action'),
                launch.actions.TimerAction(
                    period=10.0,
                    actions=[
                        launch.actions.OpaqueFunction(function=lambda context: None),
                        launch.actions.TimerAction(
                            period=5.0,
                            actions=[
                                launch.actions.LogInfo(msg='Deeply Nested Action'),
                                ReadyToTest()
                            ]
                        )
                    ]
                )
            ])

        runs = make_test_run_for_dut(generate_test_description)
        dut = LaunchTestRunner(
            runs
        )

        dut.validate()  # Make sure this passes initial validation (probably redundant with above)
        runs[0].normalized_test_description(ready_fn=lambda: None)

    def test_parametrized_launch_description(self):

        @launch_testing.parametrize('my_param', [1, 2, 3])
        def generate_test_description(my_param):
            return launch.LaunchDescription([
                ReadyToTest()
            ])

        runs = make_test_run_for_dut(generate_test_description)
        dut = LaunchTestRunner(
            runs
        )

        dut.validate()  # Make sure this passes initial validation (probably redundant with above)
        runs[0].normalized_test_description(ready_fn=lambda: None)
