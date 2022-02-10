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

"""Module for the TimerAction action."""

import asyncio
import collections.abc
from typing import Any  # noqa: F401
from typing import cast
from typing import Dict  # noqa: F401
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text
from typing import Tuple
from typing import Union
import warnings

import launch.logging

from .opaque_function import OpaqueFunction

from ..action import Action
from ..event_handler import EventHandler
from ..events import Shutdown
from ..events import TimerEvent
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser
from ..launch_context import LaunchContext
from ..launch_description_entity import LaunchDescriptionEntity
from ..some_actions_type import SomeActionsType
from ..some_substitutions_type import SomeSubstitutionsType
from ..some_substitutions_type import SomeSubstitutionsType_types_tuple
from ..utilities import create_future
from ..utilities import ensure_argument_type
from ..utilities import is_a_subclass
from ..utilities import type_utils


@expose_action('timer')
class TimerAction(Action):
    """
    Action that defers other entities until a period of time has passed, unless canceled.

    All timers are "one-shot", in that they only fire one time and never again.
    """

    def __init__(
        self,
        *,
        period: Union[float, SomeSubstitutionsType],
        actions: Iterable[LaunchDescriptionEntity],
        cancel_on_shutdown: Union[bool, SomeSubstitutionsType] = True,
        **kwargs
    ) -> None:
        """
        Create a TimerAction.

        :param period: the time (in seconds) to set the timer for
        :param actions: an iterable containing actions to be executed on timeout
        :param cancel_on_shutdown: whether to cancel the timer on launch shutdown
        """
        super().__init__(**kwargs)
        period_types = list(SomeSubstitutionsType_types_tuple) + [float]
        ensure_argument_type(period, period_types, 'period', 'TimerAction')
        ensure_argument_type(actions, collections.abc.Iterable, 'actions', 'TimerAction')
        if isinstance(period, str):
            period = float(period)
            warnings.warn(
                "The parameter 'period' must be a float or substitution,"
                'passing a string literal was deprecated',
                stacklevel=2)
        self.__period = type_utils.normalize_typed_substitution(period, float)
        self.__actions = actions
        self.__context_locals: Dict[Text, Any] = {}
        self._completed_future: Optional[asyncio.Future] = None
        self.__canceled = False
        self._canceled_future: Optional[asyncio.Future] = None
        self.__cancel_on_shutdown = type_utils.normalize_typed_substitution(
            cancel_on_shutdown, bool)
        self.__logger = launch.logging.get_logger(__name__)

    async def _wait_to_fire_event(self, context):
        done, pending = await asyncio.wait(
            [self._canceled_future],
            timeout=type_utils.perform_typed_substitution(context, self.__period, float),
        )
        if not self._canceled_future.done():
            await context.emit_event(TimerEvent(timer_action=self))
        self._completed_future.set_result(None)

    @classmethod
    def parse(
        cls,
        entity: Entity,
        parser: Parser,
    ):
        """Return the `Timer` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        kwargs['period'] = parser.parse_if_substitutions(
            entity.get_attr('period', data_type=float, can_be_str=True))
        kwargs['actions'] = [parser.parse_action(child) for child in entity.children]
        cancel_on_shutdown = entity.get_attr(
            'cancel_on_shutdown', optional=True, data_type=bool, can_be_str=True)
        if cancel_on_shutdown is not None:
            kwargs['cancel_on_shutdown'] = parser.parse_if_substitutions(cancel_on_shutdown)
        return cls, kwargs

    @property
    def period(self):
        return self.__period

    @property
    def actions(self):
        return self.__actions

    def describe(self) -> Text:
        """Return a description of this TimerAction."""
        return 'TimerAction(period={}, actions=<actions>)'.format(self.__period)

    def describe_conditional_sub_entities(self) -> List[Tuple[
        Text,
        Iterable['LaunchDescriptionEntity'],
    ]]:
        """Return the actions that will result when the timer expires, but was not canceled."""
        return [('{} seconds pass without being canceled'.format(self.__period), self.__actions)]

    def handle(self, context: LaunchContext) -> Optional[SomeActionsType]:
        """Handle firing of timer."""
        context.extend_locals(self.__context_locals)
        return self.__actions

    def cancel(self) -> None:
        """
        Cancel this TimerAction.

        Calling cancel will not fail if the timer has already finished or
        already been canceled or if the timer has not been started yet.

        This function is not thread-safe and should be called only from under
        another coroutine.
        """
        self.__canceled = True
        if self._canceled_future is not None and not self._canceled_future.done():
            self._canceled_future.set_result(True)
        return None

    def execute(self, context: LaunchContext) -> Optional[List['Action']]:
        """
        Execute the action.

        This does the following:
        - register a global event handler for TimerAction's if not already done
        - create a task for the coroutine that waits until canceled or timeout
        - coroutine asynchronously fires event after timeout, if not canceled
        """
        self._completed_future = create_future(context.asyncio_loop)
        self._canceled_future = create_future(context.asyncio_loop)

        if self.__canceled:
            # In this case, the action was canceled before being executed.
            self.__logger.debug(
                'timer {} not waiting because it was canceled before being executed'.format(self),
            )
            self._completed_future.set_result(None)
            return None

        # Once per context, install the general purpose OnTimerEvent event handler.
        if not hasattr(context, '_TimerAction__event_handler_has_been_installed'):
            context.register_event_handler(EventHandler(
                matcher=lambda event: is_a_subclass(event, TimerEvent),
                entities=OpaqueFunction(
                    function=lambda context: (
                        cast(TimerEvent, context.locals.event).timer_action.handle(context)
                    )
                ),
            ))
            setattr(context, '_TimerAction__event_handler_has_been_installed', True)

        # Capture the current context locals so the yielded actions can make use of them too.
        self.__context_locals = dict(context.get_locals_as_dict())  # Capture a copy
        context.asyncio_loop.create_task(self._wait_to_fire_event(context))

        # By default, the 'shutdown' event will cause timers to cancel so they don't hold up the
        # launch process
        if type_utils.perform_typed_substitution(context, self.__cancel_on_shutdown, bool):
            context.register_event_handler(
                EventHandler(
                    matcher=lambda event: is_a_subclass(event, Shutdown),
                    entities=OpaqueFunction(function=lambda context: self.cancel())
                )
            )

        return None

    def get_asyncio_future(self) -> Optional[asyncio.Future]:
        """Return an asyncio Future, used to let the launch system know when we're done."""
        return self._completed_future
