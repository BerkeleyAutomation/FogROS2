# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Tests for the Shutdown action class."""

from launch import LaunchContext
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.events import Shutdown as ShutdownEvent


def test_shutdown_execute():
    """Test the execute (or visit) of the Shutdown class."""
    action = Shutdown()
    context = LaunchContext()
    assert context._event_queue.qsize() == 0
    assert action.visit(context) is None
    assert context._event_queue.qsize() == 1
    event = context._event_queue.get_nowait()
    assert isinstance(event, ShutdownEvent)


def test_shutdown_execute_conditional():
    """Test the conditional execution (or visit) of the Shutdown class."""
    true_action = Shutdown(condition=IfCondition('True'))
    false_action = Shutdown(condition=IfCondition('False'))
    context = LaunchContext()

    assert context._event_queue.qsize() == 0
    assert false_action.visit(context) is None
    assert context._event_queue.qsize() == 0
    assert true_action.visit(context) is None
    assert context._event_queue.qsize() == 1
    event = context._event_queue.get_nowait()
    assert isinstance(event, ShutdownEvent)


def test_shutdown_reason():
    """Test the execute (or visit) of a Shutdown class that has a reason."""
    action = Shutdown(reason='test reason')
    context = LaunchContext()
    assert action.visit(context) is None
    assert context._event_queue.qsize() == 1
    event = context._event_queue.get_nowait()
    assert isinstance(event, ShutdownEvent)
    assert event.reason == 'test reason'
