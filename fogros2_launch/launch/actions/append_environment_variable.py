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

"""Module for the AppendEnvironmentVariable action."""

import os
from typing import List
from typing import Union

from ..action import Action
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions
from ..utilities.type_utils import normalize_typed_substitution
from ..utilities.type_utils import NormalizedValueType
from ..utilities.type_utils import perform_typed_substitution


@expose_action('append_env')
class AppendEnvironmentVariable(Action):
    """
    Action that appends to an environment variable if it exists and sets it if it does not.

    It can optionally prepend instead of appending.
    It can also optionally use a custom separator, with the default being a platform-specific
    separator, `os.pathsep`.
    """

    def __init__(
        self,
        name: SomeSubstitutionsType,
        value: SomeSubstitutionsType,
        prepend: Union[bool, SomeSubstitutionsType] = False,
        separator: SomeSubstitutionsType = os.pathsep,
        **kwargs,
    ) -> None:
        """
        Create an AppendEnvironmentVariable action.

        All parameters can be provided as substitutions.
        A substitution for the prepend parameter will be coerced to `bool` following YAML rules.

        :param name: the name of the environment variable
        :param value: the value to set or append
        :param prepend: whether the value should be prepended instead
        :param separator: the separator to use, defaulting to a platform-specific separator
        """
        super().__init__(**kwargs)
        self.__name = normalize_to_list_of_substitutions(name)
        self.__value = normalize_to_list_of_substitutions(value)
        self.__prepend = normalize_typed_substitution(prepend, bool)
        self.__separator = normalize_to_list_of_substitutions(separator)

    @classmethod
    def parse(
        cls,
        entity: Entity,
        parser: Parser,
    ):
        """Parse an 'append_env' entity."""
        _, kwargs = super().parse(entity, parser)
        kwargs['name'] = parser.parse_substitution(entity.get_attr('name'))
        kwargs['value'] = parser.parse_substitution(entity.get_attr('value'))
        prepend = entity.get_attr('prepend', optional=True, data_type=bool, can_be_str=True)
        if prepend is not None:
            kwargs['prepend'] = parser.parse_if_substitutions(prepend)
        separator = entity.get_attr('separator', optional=True)
        if separator is not None:
            kwargs['separator'] = parser.parse_substitution(separator)
        return cls, kwargs

    @property
    def name(self) -> List[Substitution]:
        """Getter for the name of the environment variable to be set or appended to."""
        return self.__name

    @property
    def value(self) -> List[Substitution]:
        """Getter for the value of the environment variable to be set or appended."""
        return self.__value

    @property
    def prepend(self) -> NormalizedValueType:
        """Getter for the prepend flag."""
        return self.__prepend

    @property
    def separator(self) -> List[Substitution]:
        """Getter for the separator."""
        return self.__separator

    def execute(self, context: LaunchContext) -> None:
        """Execute the action."""
        name = perform_substitutions(context, self.name)
        value = perform_substitutions(context, self.value)
        prepend = perform_typed_substitution(context, self.prepend, bool)
        separator = perform_substitutions(context, self.separator)
        if name in os.environ:
            os.environ[name] = \
                os.environ[name] + separator + value \
                if not prepend \
                else value + separator + os.environ[name]
        else:
            os.environ[name] = value
        return None
