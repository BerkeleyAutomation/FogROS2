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


"""Tests for the TimerAction Action."""
import sys

import launch
import launch.actions
import launch.event_handlers


def test_multiple_launch_with_timers():
    # Regression test for https://github.com/ros2/launch/issues/183
    # Unfortunately, when things aren't working this test just hangs on the second call to
    # ls.run

    def generate_launch_description():
        return launch.LaunchDescription([

            launch.actions.ExecuteProcess(
                cmd=[sys.executable, '-c', 'while True: pass'],
            ),

            launch.actions.TimerAction(
                period=1.,
                actions=[
                    launch.actions.Shutdown(reason='Timer expired')
                ]
            )
        ])

    ls = launch.LaunchService()
    ls.include_launch_description(generate_launch_description())
    assert 0 == ls.run()  # Always works

    ls = launch.LaunchService()
    ls.include_launch_description(generate_launch_description())
    # Next line hangs forever before https://github.com/ros2/launch/issues/183 was fixed.
    assert 0 == ls.run()


def _shutdown_listener_factory(reasons_arr):
    return launch.actions.RegisterEventHandler(
        launch.event_handlers.OnShutdown(
            on_shutdown=lambda event, context: reasons_arr.append(event)
        )
    )


def test_timer_action_sanity_check():
    """Test that timer actions work (sanity check)."""
    # This test is structured like test_shutdown_preempts_timers and
    # test_timer_can_block_preemption as a sanity check that the shutdown listener
    # and other launch related infrastructure works as expected
    shutdown_reasons = []

    ld = launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=[sys.executable, '-c', 'while True: pass'],
        ),

        launch.actions.TimerAction(
            period=1.,
            actions=[
                launch.actions.Shutdown(reason='One second timeout')
            ]
        ),

        _shutdown_listener_factory(shutdown_reasons),
    ])

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert shutdown_reasons[0].reason == 'One second timeout'


def test_shutdown_preempts_timers():
    shutdown_reasons = []

    ld = launch.LaunchDescription([

        launch.actions.ExecuteProcess(
            cmd=[sys.executable, '-c', 'while True: pass'],
        ),

        launch.actions.TimerAction(
            period=1.,
            actions=[
                launch.actions.Shutdown(reason='fast shutdown')
            ]
        ),

        launch.actions.TimerAction(
            period=2.,
            actions=[
                launch.actions.Shutdown(reason='slow shutdown')
            ]
        ),

        _shutdown_listener_factory(shutdown_reasons),
    ])

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert len(shutdown_reasons) == 1
    assert shutdown_reasons[0].reason == 'fast shutdown'


def test_timer_can_block_preemption():
    shutdown_reasons = []

    ld = launch.LaunchDescription([

        launch.actions.ExecuteProcess(
            cmd=[sys.executable, '-c', 'while True: pass'],
        ),

        launch.actions.TimerAction(
            period=1.,
            actions=[
                launch.actions.Shutdown(reason='fast shutdown')
            ]
        ),

        launch.actions.TimerAction(
            period=2.,
            actions=[
                launch.actions.Shutdown(reason='slow shutdown')
            ],
            cancel_on_shutdown=False  # Preempted in test_shutdown_preempts_timers, but not here
        ),

        _shutdown_listener_factory(shutdown_reasons),
    ])

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert len(shutdown_reasons) == 2  # Should see 'shutdown' event twice because
    assert shutdown_reasons[0].reason == 'fast shutdown'
    assert shutdown_reasons[1].reason == 'slow shutdown'
