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

"""Tests for the OnShutdown event handler."""

from unittest.mock import Mock
from unittest.mock import NonCallableMock

from launch import LaunchContext
from launch.action import Action
from launch.event_handlers.on_shutdown import OnShutdown
from launch.events import Shutdown
from launch.events.process import ProcessStarted

phony_process_started = ProcessStarted(
    action=Mock(spec=Action), name='PhonyProcessStarted', cmd=['ls'], cwd=None, env=None, pid=1)
phony_shutdown = Shutdown()
phony_context = Mock(spec=LaunchContext)


def test_matches_shutdown():
    handler = OnShutdown(on_shutdown=Mock())
    assert handler.matches(phony_shutdown)
    assert not handler.matches(phony_process_started)


def test_handle_callable():
    mock_callable = Mock()
    handler = OnShutdown(on_shutdown=mock_callable)
    handler.handle(phony_shutdown, phony_context)
    mock_callable.assert_called_once_with(phony_shutdown, phony_context)


def test_handle_action():
    mock_action = NonCallableMock(spec=Action)
    handler = OnShutdown(on_shutdown=mock_action)
    assert mock_action == handler.handle(phony_shutdown, phony_context)


def test_event_added_to_context():
    context = Mock(spec=LaunchContext)
    extend_locals_mock = context.extend_locals
    unregister_event_handler_mock = context.unregister_event_handler

    handler = OnShutdown(on_shutdown=Mock())
    handler.handle(phony_shutdown, context)
    extend_locals_mock.assert_called_once_with({'event': phony_shutdown})
    unregister_event_handler_mock.assert_not_called()


def test_handle_once():
    context = Mock(spec=LaunchContext)
    unregister_event_handler_mock = context.unregister_event_handler

    handler = OnShutdown(on_shutdown=Mock(), handle_once=True)
    handler.handle(phony_shutdown, context)
    unregister_event_handler_mock.assert_called_once_with(handler)
