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

"""Module for the RosTimer action."""

import asyncio
import collections.abc
from functools import partial
from typing import Iterable
from typing import Optional
from typing import Text
from typing import Union

from launch.actions import TimerAction
from launch.events import TimerEvent
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.some_substitutions_type import SomeSubstitutionsType_types_tuple
from launch.utilities import create_future
from launch.utilities import ensure_argument_type
from launch.utilities import type_utils

from launch_ros.ros_adapters import get_ros_node


@expose_action('ros_timer')
class RosTimer(TimerAction):
    """
    Action that defers other entities until a period of time has passed, unless canceled.

    This timer uses ROS time instead of wall clock time.
    To enable the use of sim time, you must also use the SetUseSimTime action.
    All timers are "one-shot", in that they only fire one time and never again.
    """

    def __init__(
        self,
        *,
        period: Union[float, SomeSubstitutionsType],
        actions: Iterable[LaunchDescriptionEntity],
        **kwargs
    ) -> None:
        """
        Create a RosTimer.

        :param period: is the time (in seconds) to set the timer for.
        :param actions: is an iterable containing actions to be executed upon on timeout.
        """
        super().__init__(period=period, actions=actions, **kwargs)
        period_types = list(SomeSubstitutionsType_types_tuple) + [float]
        ensure_argument_type(period, period_types, 'period', 'RosTimer')
        ensure_argument_type(actions, collections.abc.Iterable, 'actions', 'RosTimer')
        self.__period = type_utils.normalize_typed_substitution(period, float)
        self.__timer_future: Optional[asyncio.Future] = None

    def __timer_callback(self):
        if not self.__timer_future.done():
            self.__timer_future.set_result(True)

    async def _wait_to_fire_event(self, context):
        node = get_ros_node(context)
        node.create_timer(
            type_utils.perform_typed_substitution(context, self.__period, float),
            partial(context.asyncio_loop.call_soon_threadsafe, self.__timer_callback),
        )

        done, pending = await asyncio.wait(
            [self._canceled_future, self.__timer_future],
            return_when=asyncio.FIRST_COMPLETED
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
        """Return the `RosTimer` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        kwargs['period'] = parser.parse_if_substitutions(
            entity.get_attr('period', data_type=float, can_be_str=True))
        kwargs['actions'] = [parser.parse_action(child) for child in entity.children]
        return cls, kwargs

    def describe(self) -> Text:
        """Return a description of this RosTimer."""
        return 'RosTimer(period={}, actions=<actions>)'.format(self.__period)

    def execute(self, context: LaunchContext):
        self.__timer_future = create_future(context.asyncio_loop)
        return super().execute(context)
