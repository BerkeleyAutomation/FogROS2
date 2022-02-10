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
import signal
import sys

import ament_index_python
import launch
import launch.actions
import launch.events.process
import launch_testing.actions
from launch_testing.test_runner import LaunchTestRunner
import launch_testing.util


def test_wait_for_shutdown(source_test_loader):

    def generate_test_description():
        TEST_PROC_PATH = os.path.join(
            ament_index_python.get_package_prefix('launch_testing'),
            'lib/launch_testing',
            'good_proc.py'
        )

        good_process = launch.actions.ExecuteProcess(
                cmd=[sys.executable, TEST_PROC_PATH],
        )

        # Let 'good_process' run for 10 seconds, then terminate it
        return launch.LaunchDescription([
            good_process,
            launch_testing.util.KeepAliveProc(),
            launch.actions.TimerAction(
                period=10.0,
                actions=[
                    launch.actions.EmitEvent(
                        event=launch.events.process.SignalProcess(
                            signal_number=signal.SIGINT,
                            process_matcher=lambda proc: proc is good_process
                        )
                    )
                ]
            ),
            launch_testing.actions.ReadyToTest(),
        ]), {'good_process': good_process}

    # This is kind of a weird test-within-a-test, but it's the easiest way to get
    # all of the proc_info handlers hooked up correctly without digging deep into
    # the guts of the runner
    def test_01_check_running_process(self, proc_info, good_process):
        with self.assertRaisesRegex(AssertionError, 'Timed out waiting for process'):
            proc_info.assertWaitForShutdown(good_process, timeout=3)  # Should raise

    def test_02_check_when_process_exits(self, proc_info, good_process):
        proc_info.assertWaitForShutdown(good_process, timeout=15)

    runner = LaunchTestRunner(
        source_test_loader(
            generate_test_description,
            pre_shutdown_tests=[
                test_01_check_running_process,
                test_02_check_when_process_exits,
            ]
        )
    )
    results = runner.run()

    for result in results.values():
        assert result.wasSuccessful()


def test_wait_for_startup(source_test_loader):

    def generate_test_description():
        TEST_PROC_PATH = os.path.join(
            ament_index_python.get_package_prefix('launch_testing'),
            'lib/launch_testing',
            'good_proc.py'
        )

        good_process = launch.actions.ExecuteProcess(
                cmd=[sys.executable, TEST_PROC_PATH],
        )

        # Start 'good_proc.py' after a ten second timer elapses
        return launch.LaunchDescription([
            launch.actions.TimerAction(
                period=10.0,
                actions=[good_process]
            ),
            launch_testing.actions.ReadyToTest(),
        ]), {'good_process': good_process}

    # This is kind of a weird test-within-a-test, but it's the easiest way to get
    # all of the proc_info handlers hooked up correctly without digging deep into
    # the guts of the runner
    def test_01_check_for_non_running_process(self, proc_info, good_process):
        with self.assertRaisesRegex(AssertionError, 'Timed out waiting for process'):
            proc_info.assertWaitForStartup(good_process, timeout=3)  # Should raise

    def test_02_check_when_process_runs(self, proc_info, good_process):
        # The process we're waiting for should start about 7 seconds into this wait
        proc_info.assertWaitForStartup(good_process, timeout=15)

    runner = LaunchTestRunner(
        source_test_loader(
            generate_test_description,
            pre_shutdown_tests=[
                test_01_check_for_non_running_process,
                test_02_check_when_process_runs
            ]
        )
    )
    results = runner.run()

    for result in results.values():
        assert result.wasSuccessful()
