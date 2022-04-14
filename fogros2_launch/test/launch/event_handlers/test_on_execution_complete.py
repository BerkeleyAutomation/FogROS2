# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Tests for the OnExecutionComplete class."""

from launch.actions import LogInfo
from launch.event_handlers import OnExecutionComplete
from launch.events import ExecutionComplete


import pytest


def test_bad_construction():
    """Test bad construction parameters."""
    with pytest.raises(TypeError):
        OnExecutionComplete(
            target_action='not-an-action',
            on_completion=lambda *args: None
        )

    with pytest.raises(TypeError):
        OnExecutionComplete(
            target_action=LogInfo(msg='some message'),
            on_completion='not-a-callable-nor-an-action-iterable'
        )


def test_single_action_is_matched():
    """Test that only the target action execution complete event is matched."""
    an_action = LogInfo(msg='some message')
    event_handler = OnExecutionComplete(
        target_action=an_action,
        on_completion=lambda *args: None
    )
    other_action = LogInfo(msg='other message')
    assert event_handler.matches(ExecutionComplete(action=an_action))
    assert not event_handler.matches(ExecutionComplete(action=other_action))


def test_all_actions_are_matched():
    """Test that all execution complete events are matched."""
    an_action = LogInfo(msg='some message')
    other_action = LogInfo(msg='other message')
    event_handler = OnExecutionComplete(
        on_completion=lambda *args: None
    )
    assert event_handler.matches(ExecutionComplete(action=an_action))
    assert event_handler.matches(ExecutionComplete(action=other_action))
