# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Module for the anonymous name substitution."""

from typing import Iterable
from typing import List
from typing import Text

from ..frontend import expose_substitution
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution


@expose_substitution('anon')
class AnonName(Substitution):
    """
    Generates an anonymous id based on name.

    Name itself is a unique identifier: multiple uses of anon with
    the same parameter name will create the same "anonymized" name
    """

    def __init__(self, name: SomeSubstitutionsType) -> None:
        """Construct an `AnonName` substitution."""
        super().__init__()

        from ..utilities import normalize_to_list_of_substitutions
        self.__name = normalize_to_list_of_substitutions(name)

    @classmethod
    def parse(cls, data: Iterable[SomeSubstitutionsType]):
        """Parse `AnonName` substitution."""
        if len(data) != 1:
            raise TypeError('anon substitution expects 1 argument')
        return cls, {'name': data[0]}

    @property
    def name(self) -> List[Substitution]:
        """Getter for name."""
        return self.__name

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'AnonName({})'.format(' + '.join([sub.describe() for sub in self.name]))

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by creating/getting the anonymous name."""
        from ..utilities import perform_substitutions
        name = perform_substitutions(context, self.name)

        if 'anon' not in context.launch_configurations:
            context.launch_configurations['anon'] = {}
        anon_context = context.launch_configurations['anon']

        if name not in anon_context:
            anon_context[name] = self.compute_name(name)

        return anon_context[name]

    def compute_name(self, id_value: Text) -> Text:
        """Get anonymous name based on id value."""
        import os
        import random
        import socket
        import sys
        name = f'{id_value}_{socket.gethostname()}_{os.getpid()}_{random.randint(0, sys.maxsize)}'
        name = name.replace('.', '_')
        name = name.replace('-', '_')
        return name.replace(':', '_')
