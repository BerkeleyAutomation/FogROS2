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

import asyncio
import contextlib
import threading
import time

from launch import event_handlers


@contextlib.contextmanager
def register_event_handler(context, event_handler):
    # Code to acquire resource, e.g.:
    try:
        yield context.register_event_handler(event_handler, append=True)
    finally:
        context.unregister_event_handler(event_handler)


async def _wait_for_event(
    launch_context, execute_process_action, get_launch_event_handler, timeout=None
):
    pyevent = asyncio.Event()
    event_handler = get_launch_event_handler(execute_process_action, pyevent)
    with register_event_handler(launch_context, event_handler):
        try:
            await asyncio.wait_for(pyevent.wait(), timeout)
        except asyncio.TimeoutError:
            return False
    return True


async def _wait_for_event_with_condition(
    launch_context, execute_process_action, get_launch_event_handler, condition, timeout=None
):
    pyevent = asyncio.Event()
    event_handler = get_launch_event_handler(execute_process_action, pyevent)
    cond_value = condition()
    with register_event_handler(launch_context, event_handler):
        start = time.time()
        now = start
        while not cond_value and (timeout is None or now < start + timeout):
            try:
                await asyncio.wait_for(pyevent.wait(), start - now + timeout)
            except asyncio.TimeoutError:
                break
            pyevent.clear()
            cond_value = condition()
            now = time.time()
    return cond_value


def _wait_for_event_sync(
    launch_context, execute_process_action, get_launch_event_handler, timeout=None
):
    pyevent = threading.Event()
    event_handler = get_launch_event_handler(execute_process_action, pyevent)
    with register_event_handler(launch_context, event_handler):
        return pyevent.wait(timeout)


def _wait_for_event_with_condition_sync(
    launch_context, execute_process_action, get_launch_event_handler, condition, timeout=None
):
    pyevent = threading.Event()
    event_handler = get_launch_event_handler(execute_process_action, pyevent)
    cond_value = condition()
    with register_event_handler(launch_context, event_handler):
        start = time.time()
        now = start
        while not cond_value and (timeout is None or now < start + timeout):
            pyevent.wait(start - now + timeout)
            pyevent.clear()
            cond_value = condition()
            now = time.time()
    return cond_value


def _get_stdout_event_handler(action, pyevent):
    return event_handlers.OnProcessIO(
        target_action=action, on_stdout=lambda _1: pyevent.set())


async def wait_for_output(
    launch_context, execute_process_action, validate_output, timeout=None
):
    return await _wait_for_event_with_condition(
        launch_context,
        execute_process_action,
        _get_stdout_event_handler,
        lambda: validate_output(execute_process_action.get_stdout()),
        timeout)


async def assert_output(
    launch_context, execute_process_action, validate_output, timeout=None
):
    def condition():
        try:
            validate_output(execute_process_action.get_stdout())
        except AssertionError:
            return False
        return True
    cond_value = await _wait_for_event_with_condition(
        launch_context,
        execute_process_action,
        _get_stdout_event_handler,
        condition,
        timeout)
    if not cond_value:
        validate_output(execute_process_action.get_stdout())


def wait_for_output_sync(
    launch_context, execute_process_action, validate_output, timeout=None
):
    return _wait_for_event_with_condition_sync(
        launch_context,
        execute_process_action,
        _get_stdout_event_handler,
        lambda: validate_output(execute_process_action.get_stdout()),
        timeout)


def assert_output_sync(
    launch_context, execute_process_action, validate_output, timeout=None
):
    def condition():
        try:
            validate_output(execute_process_action.get_stdout())
        except AssertionError:
            return False
        return True
    cond_value = _wait_for_event_with_condition_sync(
        launch_context,
        execute_process_action,
        _get_stdout_event_handler,
        condition,
        timeout)
    if not cond_value:
        validate_output(execute_process_action.get_stdout())


def _get_stderr_event_handler(action, pyevent):
    return event_handlers.OnProcessIO(
        target_action=action, on_stderr=lambda _1: pyevent.set())


async def wait_for_stderr(
    launch_context, execute_process_action, validate_output, timeout=None
):
    return await _wait_for_event_with_condition(
        launch_context,
        execute_process_action,
        _get_stderr_event_handler,
        lambda: validate_output(execute_process_action.get_stderr()),
        timeout)


async def assert_stderr(
    launch_context, execute_process_action, validate_output, timeout=None
):
    def condition():
        try:
            validate_output(execute_process_action.get_stderr())
        except AssertionError:
            return False
        return True
    cond_value = await _wait_for_event_with_condition(
        launch_context,
        execute_process_action,
        _get_stderr_event_handler,
        condition,
        timeout)
    if not cond_value:
        validate_output(execute_process_action.get_stderr())


def wait_for_stderr_sync(
    launch_context, execute_process_action, validate_output, timeout=None
):
    return _wait_for_event_with_condition_sync(
        launch_context,
        execute_process_action,
        _get_stderr_event_handler,
        lambda: validate_output(execute_process_action.get_stderr()),
        timeout)


def assert_stderr_sync(
    launch_context, execute_process_action, validate_output, timeout=None
):
    def condition():
        try:
            validate_output(execute_process_action.get_stderr())
        except AssertionError:
            return False
        return True
    cond_value = _wait_for_event_with_condition_sync(
        launch_context,
        execute_process_action,
        _get_stderr_event_handler,
        condition,
        timeout)
    if not cond_value:
        validate_output(execute_process_action.get_stderr())


def _get_on_process_start_event_handler(execute_process_action, pyevent):
    return event_handlers.OnProcessStart(
        target_action=execute_process_action, on_start=lambda _1, _2: pyevent.set())


async def wait_for_start(
    launch_context, execute_process_action, timeout=None
):
    return await _wait_for_event(
        launch_context, execute_process_action, _get_on_process_start_event_handler, timeout)


def wait_for_start_sync(
    launch_context, execute_process_action, timeout=None
):
    return _wait_for_event_sync(
        launch_context, execute_process_action, _get_on_process_start_event_handler, timeout)


def _get_on_process_exit_event_handler(execute_process_action, pyevent):
    return event_handlers.OnProcessExit(
        target_action=execute_process_action, on_exit=lambda _1, _2: pyevent.set())


async def wait_for_exit(
    launch_context, execute_process_action, timeout=None
):
    return await _wait_for_event(
        launch_context, execute_process_action, _get_on_process_exit_event_handler, timeout)


def wait_for_exit_sync(
    launch_context, execute_process_action, timeout=None
):
    return _wait_for_event_sync(
        launch_context, execute_process_action, _get_on_process_exit_event_handler, timeout)
