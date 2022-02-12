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

"""Module for the FrontendLaunchDescriptionSource class."""

from typing import Type

from .frontend_launch_file_utilities import get_launch_description_from_frontend_launch_file
from ..frontend import Parser
from ..launch_description_source import LaunchDescriptionSource
from ..some_substitutions_type import SomeSubstitutionsType


class FrontendLaunchDescriptionSource(LaunchDescriptionSource):
    """
    Encapsulation of a declarative (markup based) launch file.

    It can be loaded during launch using an `IncludeLaunchDescription` action.
    """

    def __init__(
        self,
        launch_file_path: SomeSubstitutionsType,
        *,
        method: str = 'interpreted frontend launch file',
        parser: Type[Parser] = Parser
    ) -> None:
        """
        Create a FrontendLaunchDescriptionSource.

        The given file path should be to a launch frontend style file (like xml or yaml).
        If a relative path is passed, it will be relative to the current working
        directory wherever the launch file was run from.

        :param launch_file_path: the path to the launch file. It can be made up of Substitution
            instances which are expanded when :py:meth:`get_launch_description()` is called.
        :param parser: an specific parser implementation
        """
        super().__init__(
            None,
            launch_file_path,
            method
        )
        self._parser = Parser

    def _get_launch_description(self, location):
        """Get the LaunchDescription from location."""
        return get_launch_description_from_frontend_launch_file(location, parser=self._parser)
