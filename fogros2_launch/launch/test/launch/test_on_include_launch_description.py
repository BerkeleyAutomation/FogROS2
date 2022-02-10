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

"""Tests for the OnIncludeLaunchDescription event handler."""

from unittest.mock import Mock

from launch import LaunchContext
from launch import LaunchDescription
from launch.action import Action
from launch.event_handlers.on_include_launch_description import OnIncludeLaunchDescription
from launch.events import IncludeLaunchDescription
from launch.events.process import ProcessStarted

phony_process_started = ProcessStarted(
    action=Mock(spec=Action), name='PhonyProcessStarted', cmd=['ls'], cwd=None, env=None, pid=1)
phony_include_launch_description = IncludeLaunchDescription(
    launch_description=Mock(spec=LaunchDescription))
phony_context = Mock(spec=LaunchContext)


def test_matches_include_launch_description():
    handler = OnIncludeLaunchDescription()
    assert handler.matches(phony_include_launch_description)
    assert not handler.matches(phony_process_started)


def test_event_added_to_context():
    context = Mock(spec=LaunchContext)
    extend_locals_mock = context.extend_locals
    unregister_event_handler_mock = context.unregister_event_handler

    handler = OnIncludeLaunchDescription()
    handler.handle(phony_include_launch_description, context)
    extend_locals_mock.assert_called_once_with({'event': phony_include_launch_description})
    unregister_event_handler_mock.assert_not_called()


def test_handle_once():
    context = Mock(spec=LaunchContext)
    unregister_event_handler_mock = context.unregister_event_handler

    handler = OnIncludeLaunchDescription(handle_once=True)
    handler.handle(phony_include_launch_description, context)
    unregister_event_handler_mock.assert_called_once_with(handler)
