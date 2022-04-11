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

"""Module for the SetEnvironmentVariable action."""

import os
from typing import List

from ..action import Action
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions


@expose_action('set_env')
class SetEnvironmentVariable(Action):
    """Action that sets an environment variable."""

    def __init__(
        self,
        name: SomeSubstitutionsType,
        value: SomeSubstitutionsType,
        **kwargs
    ) -> None:
        """Create a SetEnvironmentVariable action."""
        super().__init__(**kwargs)
        self.__name = normalize_to_list_of_substitutions(name)
        self.__value = normalize_to_list_of_substitutions(value)

    @classmethod
    def parse(
        cls,
        entity: Entity,
        parser: Parser,
    ):
        """Parse a 'set_env' entity."""
        _, kwargs = super().parse(entity, parser)
        kwargs['name'] = parser.parse_substitution(entity.get_attr('name'))
        kwargs['value'] = parser.parse_substitution(entity.get_attr('value'))
        return cls, kwargs

    @property
    def name(self) -> List[Substitution]:
        """Getter for the name of the environment variable to be set."""
        return self.__name

    @property
    def value(self) -> List[Substitution]:
        """Getter for the value of the environment variable to be set."""
        return self.__value

    def execute(self, context: LaunchContext) -> None:
        """Execute the action."""
        os.environ[perform_substitutions(context, self.name)] = \
            perform_substitutions(context, self.value)
        return None
