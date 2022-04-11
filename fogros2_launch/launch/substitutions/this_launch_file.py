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

"""Module for the ThisLaunchFile substitution."""

from typing import Iterable
from typing import Text

from .substitution_failure import SubstitutionFailure
from ..frontend.expose import expose_substitution
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution


@expose_substitution('filename')
class ThisLaunchFile(Substitution):
    """Substitution that returns the absolute path to the current launch file."""

    def __init__(self) -> None:
        """Create a ThisLaunchFile substitution."""
        super().__init__()

    @classmethod
    def parse(cls, data: Iterable[SomeSubstitutionsType]):
        """Parse `EnviromentVariable` substitution."""
        if len(data) != 0:
            raise TypeError("filename substitution doesn't expect arguments")
        return cls, {}

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'ThisLaunchFile()'

    def perform(self, context: LaunchContext) -> Text:
        """
        Perform the substitution by returning the path to the current launch file.

        If there is no current launch file, i.e. if run from a script, then an
        error is raised.

        :raises: SubstitutionFailure if not in a launch file
        """
        subst_failure = SubstitutionFailure(
                'ThisLaunchFile used outside of a launch file (in a script)')
        if 'current_launch_file_path' not in context.get_locals_as_dict():
            raise subst_failure
        return context.locals.current_launch_file_path
