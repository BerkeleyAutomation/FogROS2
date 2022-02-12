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

"""Module for the OpaqueCoroutine action."""

import asyncio
import collections.abc
from typing import Any
from typing import Coroutine
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text

from ..action import Action
from ..event import Event
from ..event_handlers import OnShutdown
from ..launch_context import LaunchContext
from ..launch_description_entity import LaunchDescriptionEntity
from ..some_actions_type import SomeActionsType
from ..utilities import ensure_argument_type


class OpaqueCoroutine(Action):
    """
    Action that adds a Python coroutine to the launch run loop.

    The signature of a coroutine should be:

    .. code-block:: python

        async def coroutine(
            context: LaunchContext,
            *args,
            **kwargs
        ):
            ...

    if ignore_context is False on construction (currently the default), or

    .. code-block:: python

        async def coroutine(
            *args,
            **kwargs
        ):
            ...

    if ignore_context is True on construction.
    """

    def __init__(
        self, *,
        coroutine: Coroutine,
        args: Optional[Iterable[Any]] = None,
        kwargs: Optional[Dict[Text, Any]] = None,
        ignore_context: bool = False,
        **left_over_kwargs
    ) -> None:
        """Create an OpaqueCoroutine action."""
        super().__init__(**left_over_kwargs)
        if not asyncio.iscoroutinefunction(coroutine):
            raise TypeError(
                "OpaqueCoroutine expected a coroutine for 'coroutine', got '{}'".format(
                    type(coroutine)
                )
            )
        ensure_argument_type(
            args, (collections.abc.Iterable, type(None)), 'args', 'OpaqueCoroutine'
        )
        ensure_argument_type(kwargs, (dict, type(None)), 'kwargs', 'OpaqueCoroutine')
        ensure_argument_type(ignore_context, bool, 'ignore_context', 'OpaqueCoroutine')
        self.__coroutine = coroutine
        self.__args = []  # type: Iterable
        if args is not None:
            self.__args = args
        self.__kwargs = {}  # type: Dict[Text, Any]
        if kwargs is not None:
            self.__kwargs = kwargs
        self.__ignore_context = ignore_context  # type: bool
        self.__future = None  # type: Optional[asyncio.Future]

    def __on_shutdown(self, event: Event, context: LaunchContext) -> Optional[SomeActionsType]:
        """Cancel ongoing coroutine upon shutdown."""
        if self.__future is not None:
            self.__future.cancel()
        return None

    def execute(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        """Execute the action."""
        args = self.__args
        if not self.__ignore_context:
            args = [context, *self.__args]
        self.__future = context.asyncio_loop.create_task(
            self.__coroutine(*args, **self.__kwargs)
        )
        context.register_event_handler(
            OnShutdown(on_shutdown=self.__on_shutdown)
        )
        return None

    def get_asyncio_future(self) -> Optional[asyncio.Future]:
        """Return an asyncio Future, used to let the launch system know when we're done."""
        return self.__future
