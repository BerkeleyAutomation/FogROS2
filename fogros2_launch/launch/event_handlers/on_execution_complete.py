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

from typing import Callable
from typing import cast
from typing import Optional
from typing import TYPE_CHECKING
from typing import Union

from .on_action_event_base import OnActionEventBase
from ..event import Event
from ..events import ExecutionComplete
from ..launch_context import LaunchContext
from ..some_actions_type import SomeActionsType

if TYPE_CHECKING:
    from .. import Action  # noqa: F401


class OnExecutionComplete(OnActionEventBase):
    """
    Convenience class for handling an action completion event.

    It may be configured to only handle the completion of a specific action,
    or to handle them all.
    """

    def __init__(
        self,
        *,
        target_action:
            Optional[Union[Callable[['Action'], bool], 'Action']] = None,
        on_completion:
            Union[
                SomeActionsType,
                Callable[[ExecutionComplete, LaunchContext], Optional[SomeActionsType]]],
        **kwargs
    ) -> None:
        """Create an OnExecutionComplete event handler."""
        from ..action import Action  # noqa: F811
        on_completion = cast(
            Union[
                SomeActionsType,
                Callable[[Event, LaunchContext], Optional[SomeActionsType]]],
            on_completion)
        super().__init__(
            action_matcher=target_action,
            on_event=on_completion,
            target_event_cls=ExecutionComplete,
            target_action_cls=Action,
            **kwargs,
        )
