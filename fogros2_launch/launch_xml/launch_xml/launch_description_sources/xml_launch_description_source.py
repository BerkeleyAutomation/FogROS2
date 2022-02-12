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

"""Module for the XMLLaunchDescriptionSource class."""

from launch import SomeSubstitutionsType
from launch.launch_description_sources import FrontendLaunchDescriptionSource

from ..parser import Parser


class XMLLaunchDescriptionSource(FrontendLaunchDescriptionSource):
    """Encapsulation of a XML launch file, which can be loaded during launch."""

    def __init__(
        self,
        launch_file_path: SomeSubstitutionsType,
    ) -> None:
        """
        Create an XMLLaunchDescriptionSource.

        The given file path should be to a launch XML style file (`.launch.xml`).
        If a relative path is passed, it will be relative to the current working
        directory wherever the launch file was run from.

        :param launch_file_path: the path to the launch file. It can be made up of Substitution
            instances which are expanded when :py:meth:`get_launch_description()` is called.
        """
        super().__init__(
            launch_file_path,
            method='interpreted XML launch file',
            parser=Parser
        )
