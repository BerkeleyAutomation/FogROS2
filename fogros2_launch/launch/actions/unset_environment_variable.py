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

"""Module for the UnsetEnvironmentVariable action."""

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


@expose_action('unset_env')
class UnsetEnvironmentVariable(Action):
    """Action that unsets an environment variable if it is set, otherwise does nothing."""

    def __init__(
        self,
        name: SomeSubstitutionsType,
        **kwargs
    ) -> None:
        """Create an UnsetEnvironmentVariable action."""
        super().__init__(**kwargs)
        self.__name = normalize_to_list_of_substitutions(name)

    @classmethod
    def parse(
        cls,
        entity: Entity,
        parser: Parser,
    ):
        """Parse a 'set_env' entity."""
        _, kwargs = super().parse(entity, parser)
        kwargs['name'] = parser.parse_substitution(entity.get_attr('name'))
        return cls, kwargs

    @property
    def name(self) -> List[Substitution]:
        """Getter for the name of the environment variable to be unset."""
        return self.__name

    def execute(self, context: LaunchContext) -> None:
        """Execute the action."""
        name = perform_substitutions(context, self.name)
        if name in os.environ:
            del os.environ[name]
        return None
