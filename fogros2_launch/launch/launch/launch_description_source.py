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

"""Module for the LaunchDescriptionSource class."""

import traceback
from typing import Optional
from typing import Text

import launch.logging

from .launch_context import LaunchContext
from .launch_description import LaunchDescription
from .some_substitutions_type import SomeSubstitutionsType
from .utilities import normalize_to_list_of_substitutions
from .utilities import perform_substitutions


class LaunchDescriptionSource:
    """Encapsulation of a launch description, where it comes from, and how it was generated."""

    def __init__(
        self,
        launch_description: Optional[LaunchDescription] = None,
        location: SomeSubstitutionsType = '<string>',
        method: str = 'unspecified mechanism from a script',
    ) -> None:
        """
        Create a LaunchDescriptionSource.

        For example, loading a file called ``example.launch.py`` for inclusion
        might end up setting `location` to ``/path/to/example.launch.py`` and
        the ``method`` to be ``interpreted python launch file``.

        :param launch_description: the launch description that this source represents
        :param location: the location from where this launch description was loaded if applicable
        :param method: the method by which the launch description was generated
        """
        self.__launch_description: Optional[LaunchDescription] = launch_description
        self.__expanded_location: Optional[Text] = None
        self.__location: SomeSubstitutionsType = normalize_to_list_of_substitutions(location)
        self.__method: str = method
        self.__logger = launch.logging.get_logger(__name__)

    def try_get_launch_description_without_context(self) -> Optional[LaunchDescription]:
        """
        Attempt to load the LaunchDescription without a context, return None if unsuccessful.

        This method is useful for trying to introspect the included launch
        description without visiting the user of this source.
        """
        if self.__launch_description is None:
            # Try to expand the launch file path and load the launch file with a local context.
            try:
                context = LaunchContext()
                expanded_location = \
                    perform_substitutions(context, self.__location)
                return self._get_launch_description(expanded_location)
            except Exception as exc:
                self.__logger.debug(traceback.format_exc())
                self.__logger.debug(
                    'Failed to load the launch file without a context: ' + str(exc),
                )
        return self.__launch_description

    def get_launch_description(self, context: LaunchContext) -> LaunchDescription:
        """Get the LaunchDescription, loading it if necessary."""
        if self.__expanded_location is None:
            self.__expanded_location = \
                perform_substitutions(context, self.__location)
        if self.__launch_description is None:
            self.__launch_description = \
                self._get_launch_description(self.__expanded_location)
        return self.__launch_description

    def _get_launch_description(self, location):
        """Get the LaunchDescription from location."""
        if self.__launch_description is None:
            raise RuntimeError(
                'LaunchDescriptionSource.get_launch_description(): '
                'called without launch description being set'
            )

    @property
    def location(self) -> str:
        """
        Get the location of the launch description source as a string.

        The string is either a list of Substitution instances converted to
        strings or the expanded path if :py:meth:`get_launch_description` has
        been called.
        """
        if self.__expanded_location is None:
            # get_launch_description() has not been called yet
            return ' + '.join([str(sub) for sub in self.__location])
        return self.__expanded_location

    @property
    def method(self) -> str:
        """Getter for self.__method."""
        return self.__method
