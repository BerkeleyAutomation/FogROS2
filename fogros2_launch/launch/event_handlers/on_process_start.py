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

"""Module for OnProcessStart class."""

from typing import Callable
from typing import cast
from typing import Optional
from typing import TYPE_CHECKING
from typing import Union

from .on_action_event_base import OnActionEventBase
from ..event import Event
from ..events.process import ProcessStarted
from ..launch_context import LaunchContext
from ..some_actions_type import SomeActionsType

if TYPE_CHECKING:
    from ..actions import Action  # noqa: F401
    from ..actions import ExecuteProcess  # noqa: F401


class OnProcessStart(OnActionEventBase):
    """
    Convenience class for handling a process started event.

    It may be configured to only handle the starting of a specific action,
    or to handle all started processes.
    """

    def __init__(
        self,
        *,
        target_action:
            Optional[Union[Callable[['ExecuteProcess'], bool], 'ExecuteProcess']] = None,
        on_start:
            Union[
                SomeActionsType,
                Callable[[ProcessStarted, LaunchContext], Optional[SomeActionsType]]],
        **kwargs
    ) -> None:
        """Create an OnProcessStart event handler."""
        from ..actions import ExecuteProcess  # noqa: F811
        target_action = cast(
            Optional[Union[Callable[['Action'], bool], 'Action']],
            target_action)
        on_start = cast(
            Union[
                SomeActionsType,
                Callable[[Event, LaunchContext], Optional[SomeActionsType]]],
            on_start)
        super().__init__(
            action_matcher=target_action,
            on_event=on_start,
            target_event_cls=ProcessStarted,
            target_action_cls=ExecuteProcess,
            **kwargs,
        )
