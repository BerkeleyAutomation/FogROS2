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

"""Module for the EnvironmentVariable substitution."""

import os
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text

from .substitution_failure import SubstitutionFailure
from ..frontend.expose import expose_substitution
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution


@expose_substitution('env')
class EnvironmentVariable(Substitution):
    """
    Substitution that gets an environment variable value as a string.

    If the environment variable is not found, it returns empty string.

    Note that the environment variable is resolved based on the launch context's environment (i.e.,
    `os.environ`) even though the substitution may be associated with an entity that might have a
    different set of environment variables.
    """

    def __init__(
        self,
        name: SomeSubstitutionsType,
        *,
        default_value: Optional[SomeSubstitutionsType] = None
    ) -> None:
        """
        Construct an enviroment variable substitution.

        :param name: name of the environment variable.
        :param default_value: used when the environment variable doesn't exist.
            If `None`, the substitution is not optional.
        :raise `SubstitutionFailure`:
            If the environment variable doesn't exist and `default_value` is `None`.
        """
        super().__init__()

        from ..utilities import normalize_to_list_of_substitutions  # import here to avoid loop
        self.__name = normalize_to_list_of_substitutions(name)
        if default_value is not None:
            default_value = normalize_to_list_of_substitutions(default_value)
        self.__default_value = default_value

    @classmethod
    def parse(cls, data: Iterable[SomeSubstitutionsType]):
        """Parse `EnviromentVariable` substitution."""
        if len(data) < 1 or len(data) > 2:
            raise TypeError('env substitution expects 1 or 2 arguments')
        kwargs = {'name': data[0]}
        if len(data) == 2:
            kwargs['default_value'] = data[1]
        return cls, kwargs

    @property
    def name(self) -> List[Substitution]:
        """Getter for name."""
        return self.__name

    @property
    def default_value(self) -> List[Substitution]:
        """Getter for default_value."""
        return self.__default_value

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'EnvVar({})'.format(' + '.join([sub.describe() for sub in self.name]))

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by looking up the environment variable."""
        from ..utilities import perform_substitutions  # import here to avoid loop
        default_value = self.default_value
        if default_value is not None:
            default_value = perform_substitutions(context, self.default_value)
        name = perform_substitutions(context, self.name)
        value = os.environ.get(
            name,
            default_value
        )
        if value is None:
            raise SubstitutionFailure(
                "environment variable '{}' does not exist".format(name)
            )
        return value
