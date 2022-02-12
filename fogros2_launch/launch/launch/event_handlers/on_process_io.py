# Copyright 2018-2021 Open Source Robotics Foundation, Inc.
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

"""Module for OnProcessIO class."""

from typing import Callable
from typing import cast
from typing import Optional
from typing import TYPE_CHECKING
from typing import Union

from .on_action_event_base import OnActionEventBase
from ..event import Event
from ..events.process import ProcessIO
from ..launch_context import LaunchContext
from ..some_actions_type import SomeActionsType

if TYPE_CHECKING:
    from ..actions import Action  # noqa: F401
    from ..actions import ExecuteLocal  # noqa: F401


class OnProcessIO(OnActionEventBase):
    """Convenience class for handling I/O from processes via events."""

    # TODO(wjwwood): make the __init__ more flexible like OnProcessExit, so
    # that it can take SomeActionsType directly or a callable that returns it.
    def __init__(
        self,
        *,
        target_action:
            Optional[Union[Callable[['ExecuteLocal'], bool], 'ExecuteLocal']] = None,
        on_stdin: Callable[[ProcessIO], Optional[SomeActionsType]] = None,
        on_stdout: Callable[[ProcessIO], Optional[SomeActionsType]] = None,
        on_stderr: Callable[[ProcessIO], Optional[SomeActionsType]] = None,
        **kwargs
    ) -> None:
        """Create an OnProcessIO event handler."""
        from ..actions import ExecuteLocal  # noqa: F811
        target_action = cast(
            Optional[Union[Callable[['Action'], bool], 'Action']],
            target_action)

        def handle(event: Event, _: LaunchContext) -> Optional[SomeActionsType]:
            event = cast(ProcessIO, event)
            if event.from_stdout and on_stdout is not None:
                return on_stdout(event)
            elif event.from_stderr and on_stderr is not None:
                return on_stderr(event)
            elif event.from_stdin and on_stdin is not None:
                return on_stdin(event)
            return None

        super().__init__(
            action_matcher=target_action,
            on_event=handle,
            target_event_cls=ProcessIO,
            target_action_cls=ExecuteLocal,
            **kwargs,
        )
