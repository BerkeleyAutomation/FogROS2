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

"""Module for Action class."""

from typing import Iterable
from typing import List
from typing import Optional
from typing import Text
from typing import Tuple

from .condition import Condition
from .launch_context import LaunchContext
from .launch_description_entity import LaunchDescriptionEntity

if False:
    from .frontend import Entity  # noqa: F401
    from .frontend import Parser  # noqa: F401


class Action(LaunchDescriptionEntity):
    """
    LaunchDescriptionEntity that represents a user intention to do something.

    The action describes the intention to do something, but also can be
    executed given a :class:`launch.LaunchContext` at runtime.
    """

    def __init__(self, *, condition: Optional[Condition] = None) -> None:
        """
        Create an Action.

        If the conditions argument is not None, the condition object will be
        evaluated while being visited and the action will only be executed if
        the condition evaluates to True.

        :param condition: Either a :py:class:`Condition` or None
        """
        self.__condition = condition

    @staticmethod
    def parse(entity: 'Entity', parser: 'Parser'):
        """
        Return the `Action` action and kwargs for constructing it.

        This is only intended for code reuse.
        This class is not exposed with `expose_action`.
        """
        # Import here for avoiding cyclic imports.
        from .conditions import IfCondition
        from .conditions import UnlessCondition
        if_cond = entity.get_attr('if', optional=True)
        unless_cond = entity.get_attr('unless', optional=True)
        kwargs = {}
        if if_cond is not None and unless_cond is not None:
            raise RuntimeError("if and unless conditions can't be used simultaneously")
        if if_cond is not None:
            kwargs['condition'] = IfCondition(
                predicate_expression=parser.parse_substitution(if_cond)
            )
        if unless_cond is not None:
            kwargs['condition'] = UnlessCondition(
                predicate_expression=parser.parse_substitution(unless_cond)
            )
        return Action, kwargs

    @property
    def condition(self) -> Optional[Condition]:
        """Getter for condition."""
        return self.__condition

    def describe(self) -> Text:
        """Return a description of this Action."""
        return self.__repr__()

    def get_sub_entities(self) -> List[LaunchDescriptionEntity]:
        """Return subentities."""
        return []

    def describe_sub_entities(self) -> List[LaunchDescriptionEntity]:
        """Override describe_sub_entities from LaunchDescriptionEntity."""
        return self.get_sub_entities() if self.condition is None else []

    def describe_conditional_sub_entities(self) -> List[Tuple[
        Text,  # text description of the condition
        Iterable[LaunchDescriptionEntity],  # list of conditional sub-entities
    ]]:
        """Override describe_conditional_sub_entities from LaunchDescriptionEntity."""
        return [
            ('Conditionally included by {}'.format(self.describe()), self.get_sub_entities())
        ] if self.condition is not None else []

    def visit(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        """Override visit from LaunchDescriptionEntity so that it executes."""
        if self.__condition is None or self.__condition.evaluate(context):
            try:
                return self.execute(context)
            finally:
                from .events import ExecutionComplete  # noqa
                event = ExecutionComplete(action=self)
                if context.would_handle_event(event):
                    future = self.get_asyncio_future()
                    if future is not None:
                        future.add_done_callback(
                            lambda _: context.emit_event_sync(event)
                        )
                    else:
                        context.emit_event_sync(event)
        return None

    def execute(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        """
        Execute the action.

        Should be overridden by derived class, but by default does nothing.
        """
        pass
