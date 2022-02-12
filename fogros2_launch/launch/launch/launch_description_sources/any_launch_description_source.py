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

"""Module for the AnyLaunchDescriptionSource class."""

from .any_launch_file_utilities import get_launch_description_from_any_launch_file
from ..launch_description_source import LaunchDescriptionSource
from ..some_substitutions_type import SomeSubstitutionsType


class AnyLaunchDescriptionSource(LaunchDescriptionSource):
    """
    Encapsulation of a launch file, which can be loaded during launch.

    This launch description source will attempt to load a launch file based on its extension
    first, then it will try to load the file as a python launch file, and then as a declarative
    (markup based) launch file.
    It is recommended to use specific `LaunchDescriptionSource` subclasses when possible.
    """

    def __init__(
        self,
        launch_file_path: SomeSubstitutionsType,
    ) -> None:
        """
        Create an AnyLaunchDescriptionSource.

        If a relative path is passed, it will be relative to the current working
        directory wherever the launch file was run from.

        :param launch_file_path: the path to the launch file. It can be made up of Substitution
            instances which are expanded when :py:meth:`get_launch_description()` is called.
        """
        super().__init__(
            None,
            launch_file_path,
            'interpreted launch file',
        )

    def _get_launch_description(self, location):
        """Get the LaunchDescription from location."""
        return get_launch_description_from_any_launch_file(location)
