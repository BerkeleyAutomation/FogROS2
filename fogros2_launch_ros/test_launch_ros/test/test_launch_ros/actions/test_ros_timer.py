# Copyright 2021 Open Source Robotics Foundation, Inc.
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


"""Tests for the RosTimer Action."""
from functools import partial
import threading
import time

from builtin_interfaces.msg import Time
import launch
from launch.actions import DeclareLaunchArgument
import launch.event_handlers
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import RosTimer
from launch_ros.actions import SetUseSimTime
import pytest
import rclpy
from rclpy.clock import Clock, ClockType
from rosgraph_msgs.msg import Clock as ClockMsg


start_time = None
end_time = None


def set_start_time(context):
    global start_time
    start_time = time.time()


def set_end_time(context):
    global end_time
    end_time = time.time()


def _shutdown_listener_factory(reasons_arr):
    return launch.actions.RegisterEventHandler(
        launch.event_handlers.OnShutdown(
            on_shutdown=lambda event, context: reasons_arr.append(event)
        )
    )


def test_multiple_launch_with_timers():
    def generate_launch_description():
        return launch.LaunchDescription([

            RosTimer(
                period=1.,
                actions=[]
            ),
        ])

    ls = launch.LaunchService()
    ls.include_launch_description(generate_launch_description())
    assert 0 == ls.run()

    ls = launch.LaunchService()
    ls.include_launch_description(generate_launch_description())
    assert 0 == ls.run()


def test_timer_with_launch_configuration():
    def generate_launch_description():
        return launch.LaunchDescription([
            DeclareLaunchArgument('my_period', default_value='0.1'),
            RosTimer(
                period=LaunchConfiguration('my_period'),
                actions=[]
            ),
        ])

    ls = launch.LaunchService()
    ls.include_launch_description(generate_launch_description())
    assert 0 == ls.run()


def test_timer_action_sanity_check():
    """Test that timer actions work (sanity check)."""
    # This test is structured like test_shutdown_preempts_timers
    # as a sanity check that the shutdown listener
    # and other launch related infrastructure works as expected
    shutdown_reasons = []

    ld = launch.LaunchDescription([

        launch.actions.OpaqueFunction(function=set_start_time),

        RosTimer(
            period=1.,
            actions=[
                launch.actions.OpaqueFunction(function=set_end_time),
                launch.actions.Shutdown(reason='One second timeout')
            ]
        ),

        _shutdown_listener_factory(shutdown_reasons),
    ])

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert shutdown_reasons[0].reason == 'One second timeout'

    # Verify that 1 sec has passed between start of test and timeout
    tolerance = 0.1
    assert (end_time - start_time) > 1 - tolerance
    assert (end_time - start_time) < 1 + tolerance


def test_shutdown_preempts_timers():
    shutdown_reasons = []

    ld = launch.LaunchDescription([

        RosTimer(
            period=1.,
            actions=[
                launch.actions.Shutdown(reason='fast shutdown')
            ]
        ),

        RosTimer(
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


@pytest.fixture
def rclpy_node():
    rclpy.init()
    node = rclpy.create_node('test_ros_timer_action_node')
    yield node
    rclpy.shutdown()


def test_timer_uses_sim_time(rclpy_node):
    """Test that timer uses time from /clock topic."""
    # Create clock publisher node
    publisher = rclpy_node.create_publisher(ClockMsg, '/clock', 10)

    # Increment sim time by 100 every time callback is called
    def timer_callback(publisher, time_msg):
        time_msg.sec += 100
        publisher.publish(ClockMsg(clock=time_msg))

    # For every second of system time, publish new sim time value
    callback_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
    time_msg = Time(sec=0, nanosec=0)
    rclpy_node.create_timer(1, partial(timer_callback, publisher, time_msg), clock=callback_clock)

    thread = threading.Thread(target=rclpy.spin, args=(rclpy_node, ), daemon=True)
    thread.start()

    ld = launch.LaunchDescription([

        launch.actions.OpaqueFunction(function=set_start_time),

        RosTimer(
            period=200.,
            actions=[
                launch.actions.OpaqueFunction(function=set_end_time)
            ],
        ),

        SetUseSimTime(True)  # Must be set to allow timer action to use sim time
    ])

    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()

    # Timer is using sim time which is 100x faster than system time,
    # so 200 sec timer should finish in 2 sec
    tolerance = 0.1
    assert (end_time - start_time) > 2 - tolerance
    assert (end_time - start_time) < 2 + tolerance
