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

"""Module for the LogInfo action."""

from typing import List

import launch.logging

from ..action import Action
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser  # noqa: F401
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions


@expose_action('log')
class LogInfo(Action):
    """Action that logs a message when executed."""

    def __init__(self, *, msg: SomeSubstitutionsType, **kwargs):
        """Create a LogInfo action."""
        super().__init__(**kwargs)

        self.__msg = normalize_to_list_of_substitutions(msg)
        self.__logger = launch.logging.get_logger('launch.user')

    @classmethod
    def parse(
        cls,
        entity: Entity,
        parser: 'Parser'
    ):
        """Parse `log` tag."""
        _, kwargs = super().parse(entity, parser)
        kwargs['msg'] = parser.parse_substitution(entity.get_attr('message'))
        return cls, kwargs

    @property
    def msg(self) -> List[Substitution]:
        """Getter for self.__msg."""
        return self.__msg

    def execute(self, context: LaunchContext) -> None:
        """Execute the action."""
        self.__logger.info(
            ''.join([context.perform_substitution(sub) for sub in self.msg])
        )
        return None
