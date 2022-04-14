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

"""Tests for the OnProcessExit event handler."""

from unittest.mock import Mock
from unittest.mock import NonCallableMock

from launch import LaunchContext
from launch.action import Action
from launch.actions.execute_process import ExecuteProcess
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.events.process import ProcessExited
from launch.events.process import ProcessStarted

import pytest


phony_process_started = ProcessStarted(
    action=Mock(spec=Action), name='PhonyProcessStarted', cmd=['ls'], cwd=None, env=None, pid=1)
phony_process_exited = ProcessExited(
    action=Mock(spec=Action), name='PhonyProcessExited', cmd=['ls'], cwd=None, env=None, pid=2,
    returncode=0)
phony_context = Mock(spec=LaunchContext)


def test_non_execute_process_target():
    with pytest.raises(TypeError):
        OnProcessExit(
            target_action=NonCallableMock(),
            on_exit=NonCallableMock(spec=Action))


def test_callable_target():
    handler = OnProcessExit(
        target_action=Mock(spec=ExecuteProcess),
        on_exit=NonCallableMock(spec=Action))
    assert not handler.matches(phony_process_started)
    assert handler.matches(phony_process_exited)


def test_non_action_on_exit():
    with pytest.raises(TypeError):
        OnProcessExit(
            target_action=Mock(spec=ExecuteProcess),
            on_start=NonCallableMock())


def test_matches_process_exited():
    handler = OnProcessExit(on_exit=Mock())
    assert handler.matches(phony_process_exited)
    assert not handler.matches(phony_process_started)


def test_matches_single_process():
    target_action = NonCallableMock(spec=ExecuteProcess)
    handler = OnProcessExit(
        target_action=target_action,
        on_exit=Mock())
    assert handler.matches(ProcessExited(
        action=target_action, name='foo', cmd=['ls'], cwd=None, env=None, pid=3,
        returncode=0))
    assert not handler.matches(phony_process_started)
    assert not handler.matches(phony_process_exited)


def test_handle_callable():
    mock_callable = Mock()
    handler = OnProcessExit(on_exit=mock_callable)
    handler.handle(phony_process_exited, phony_context)
    mock_callable.assert_called_once_with(phony_process_exited, phony_context)


def test_handle_action():
    mock_action = NonCallableMock(spec=Action)
    handler = OnProcessExit(on_exit=mock_action)
    assert [mock_action] == handler.handle(phony_process_exited, phony_context)


def test_handle_list_of_actions():
    mock_actions = [NonCallableMock(spec=Action), NonCallableMock(spec=Action)]
    handler = OnProcessExit(on_exit=mock_actions)
    assert mock_actions == handler.handle(phony_process_exited, phony_context)


def test_event_added_to_context():
    context = Mock(spec=LaunchContext)
    extend_locals_mock = context.extend_locals
    unregister_event_handler_mock = context.unregister_event_handler

    handler = OnProcessExit(on_exit=Mock())
    handler.handle(phony_process_exited, context)
    extend_locals_mock.assert_called_once_with({'event': phony_process_exited})
    unregister_event_handler_mock.assert_not_called()


def test_handle_once():
    context = Mock(spec=LaunchContext)
    unregister_event_handler_mock = context.unregister_event_handler

    handler = OnProcessExit(on_exit=Mock(), handle_once=True)
    handler.handle(phony_process_exited, context)
    unregister_event_handler_mock.assert_called_once_with(handler)
