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

"""Module for EventHandler class."""

from typing import Callable
from typing import List
from typing import Optional
from typing import Text
from typing import Tuple
from typing import TYPE_CHECKING

from .event import Event
from .some_actions_type import SomeActionsType

if TYPE_CHECKING:
    from .launch_context import LaunchContext  # noqa: F401


class BaseEventHandler:
    """
    Base class for event handlers, which handle events in the launch system.

    Entities yielded by the event handler can access the event being handled
    via the context's locals, e.g. `context.locals.event`
    As another example, getting the name of the event as a Substitution:
    `launch.substitutions.LocalSubstitution('event.name')`.
    """

    def __init__(self, *, matcher: Callable[[Event], bool], handle_once: bool = False):
        """
        Create a BaseEventHandler.

        :param: matcher is a callable that takes an event and returns True if
            the event should be handled by this event handler, False otherwise.
        :param: handle_once is a flag that, if True, unregisters this EventHandler
            after being handled once.
        """
        self.__matcher = matcher
        self.__handle_once = handle_once

    @property
    def handle_once(self):
        """Getter for handle_once flag."""
        return self.__handle_once

    @property
    def handler_description(self):
        """
        Return the string description of the handler.

        This should be overridden.
        """
        return None

    @property
    def matcher_description(self):
        """
        Return the string description of the matcher.

        This should be overridden.
        """
        return None

    def matches(self, event: Event) -> bool:
        """Return True if the given event should be handled by this event handler."""
        return self.__matcher(event)

    def describe(self) -> Tuple[Text, List[SomeActionsType]]:
        """Return the description list with 0 as a string, and then LaunchDescriptionEntity's."""
        return (
            "{}(matcher='{}', handler='{}', handle_once={})".format(
                type(self).__name__,
                self.matcher_description,
                self.handler_description,
                self.handle_once
            ),
            []
        )

    def handle(self, event: Event, context: 'LaunchContext') -> Optional[SomeActionsType]:
        """
        Handle the given event.

        This implementation should always be called by child classes in order to properly
        support common event handler functionality.
        """
        context.extend_locals({'event': event})
        if self.handle_once:
            context.unregister_event_handler(self)


class EventHandler(BaseEventHandler):
    def __init__(
        self,
        *,
        matcher: Callable[[Event], bool],
        entities: Optional[SomeActionsType] = None,
        handle_once: bool = False
    ) -> None:
        """
        Create an EventHandler.

        :param: matcher is a callable that takes an event and returns True if
            the event should be handled by this event handler, False otherwise.
        :param: entities is an LaunchDescriptionEntity or list of them, and is
            returned by handle() unconditionally if matcher returns True.
        :param: handle_once is a flag that, if True, unregisters this EventHandler
            after being handled once.
        """
        super().__init__(matcher=matcher, handle_once=handle_once)

        self.__entities = entities

    @property
    def entities(self):
        """Getter for entities."""
        return self.__entities

    def describe(self) -> Tuple[Text, List[SomeActionsType]]:
        """Return the description list with 0 as a string, and then LaunchDescriptionEntity's."""
        text, actions = super().describe()
        if self.entities:
            actions.extend(self.entities)
        return (text, actions)

    def handle(self, event: Event, context: 'LaunchContext') -> Optional[SomeActionsType]:
        """Handle the given event."""
        super().handle(event, context)
        return self.entities
