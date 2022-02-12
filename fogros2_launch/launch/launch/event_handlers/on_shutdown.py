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

"""Module for OnShutdown class."""

from typing import Callable
from typing import cast
from typing import Optional
from typing import Text
from typing import TYPE_CHECKING
from typing import Union

from ..event import Event
from ..event_handler import BaseEventHandler
from ..events import Shutdown
from ..some_actions_type import SomeActionsType
from ..utilities import is_a_subclass

if TYPE_CHECKING:
    from ..launch_context import LaunchContext  # noqa: F401


class OnShutdown(BaseEventHandler):
    """Convenience class for handling the launch shutdown event."""

    def __init__(
        self,
        *,
        on_shutdown: Union[SomeActionsType,
                           Callable[[Shutdown, 'LaunchContext'], Optional[SomeActionsType]]],
        **kwargs
    ) -> None:
        """Create an OnShutdown event handler."""
        super().__init__(
            matcher=lambda event: is_a_subclass(event, Shutdown),
            **kwargs,
        )
        # TODO(wjwwood) check that it is not only callable, but also a callable that matches
        # the correct signature for a handler in this case
        self.__on_shutdown = on_shutdown
        if not callable(on_shutdown):
            self.__on_shutdown = (lambda event, context: on_shutdown)

    def handle(self, event: Event, context: 'LaunchContext') -> Optional[SomeActionsType]:
        """Handle the given event."""
        super().handle(event, context)
        return self.__on_shutdown(cast(Shutdown, event), context)

    @property
    def handler_description(self) -> Text:
        """Return the string description of the handler."""
        # TODO(dhood): print known actions if they were passed in, like in OnProcessExit
        return '{}'.format(self.__on_shutdown)

    @property
    def matcher_description(self):
        """Return the string description of the matcher."""
        return 'event issubclass of launch.events.Shutdown'
