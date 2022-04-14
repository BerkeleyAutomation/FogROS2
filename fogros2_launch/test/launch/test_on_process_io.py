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

"""Tests for the OnProcessIO event handler."""

from unittest.mock import Mock
from unittest.mock import NonCallableMock

from launch import LaunchContext
from launch.action import Action
from launch.actions.execute_process import ExecuteProcess
from launch.event_handlers.on_process_io import OnProcessIO
from launch.events.process import ProcessIO
from launch.events.process import ProcessStarted

import pytest


phony_process_started = ProcessStarted(
    action=Mock(spec=Action), name='PhonyProcessStarted', cmd=['ls'], cwd=None, env=None, pid=1)
phony_process_io = ProcessIO(
    action=Mock(spec=Action), name='PhonyProcessIO', cmd=['ls'], cwd=None, env=None, pid=1,
    text=b'phony io', fd=0)
phony_context = Mock(spec=LaunchContext)


def test_non_execute_target():
    with pytest.raises(TypeError):
        OnProcessIO(target_action=NonCallableMock())


def test_matches_process_io():
    handler = OnProcessIO()
    assert handler.matches(phony_process_io)
    assert not handler.matches(phony_process_started)


def test_matches_single_process_output():
    target_action = NonCallableMock(spec=ExecuteProcess)
    handler = OnProcessIO(
        target_action=target_action)
    assert handler.matches(ProcessIO(
        action=target_action, name='foo', cmd=['ls'], cwd=None, env=None, pid=3,
        text=b'phony io', fd=0))
    assert not handler.matches(phony_process_started)
    assert not handler.matches(phony_process_io)


def test_matches_with_callable():
    target_action = Mock(spec=ExecuteProcess)
    handler = OnProcessIO(
        target_action=target_action)
    assert handler.matches(ProcessIO(
        action=target_action, name='foo', cmd=['ls'], cwd=None, env=None, pid=3,
        text=b'phony io', fd=0))
    assert not handler.matches(phony_process_started)
    assert handler.matches(phony_process_io)


def test_handle_callable_stdin():
    mock_stdin_callable = Mock()
    mock_stdout_callable = Mock()
    mock_stderr_callable = Mock()
    handler = OnProcessIO(
        on_stdin=mock_stdin_callable,
        on_stdout=mock_stdout_callable,
        on_stderr=mock_stderr_callable)

    event = ProcessIO(
        action=Mock(spec=Action), name='stdin', cmd=['ls'], cwd=None, env=None, pid=1,
        text=b'stdin', fd=0)
    handler.handle(event, phony_context)
    mock_stdin_callable.assert_called_once_with(event)
    mock_stdout_callable.assert_not_called()
    mock_stderr_callable.assert_not_called()


def test_handle_callable_stdout():
    mock_stdin_callable = Mock()
    mock_stdout_callable = Mock()
    mock_stderr_callable = Mock()
    handler = OnProcessIO(
        on_stdin=mock_stdin_callable,
        on_stdout=mock_stdout_callable,
        on_stderr=mock_stderr_callable)

    event = ProcessIO(
        action=Mock(spec=Action), name='stdout', cmd=['ls'], cwd=None, env=None, pid=1,
        text=b'stdout', fd=1)
    handler.handle(event, phony_context)
    mock_stdout_callable.assert_called_once_with(event)
    mock_stdin_callable.assert_not_called()
    mock_stderr_callable.assert_not_called()


def test_handle_callable_stderr():
    mock_stdin_callable = Mock()
    mock_stdout_callable = Mock()
    mock_stderr_callable = Mock()
    handler = OnProcessIO(
        on_stdin=mock_stdin_callable,
        on_stdout=mock_stdout_callable,
        on_stderr=mock_stderr_callable)

    event = ProcessIO(
        action=Mock(spec=Action), name='stderr', cmd=['ls'], cwd=None, env=None, pid=1,
        text=b'stderr', fd=2)
    handler.handle(event, phony_context)
    mock_stderr_callable.assert_called_once_with(event)
    mock_stdin_callable.assert_not_called()
    mock_stdout_callable.assert_not_called()


def test_event_added_to_context():
    context = Mock(spec=LaunchContext)
    extend_locals_mock = context.extend_locals
    unregister_event_handler_mock = context.unregister_event_handler

    handler = OnProcessIO()
    handler.handle(phony_process_io, context)
    extend_locals_mock.assert_called_once_with({'event': phony_process_io})
    unregister_event_handler_mock.assert_not_called()


def test_handle_once():
    context = Mock(spec=LaunchContext)
    unregister_event_handler_mock = context.unregister_event_handler

    handler = OnProcessIO(handle_once=True)
    handler.handle(phony_process_io, context)
    unregister_event_handler_mock.assert_called_once_with(handler)
