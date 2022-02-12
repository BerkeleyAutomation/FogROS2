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

"""Module for the YAMLLaunchDescriptionSource class."""

from launch import SomeSubstitutionsType
from launch.launch_description_sources import FrontendLaunchDescriptionSource

from ..parser import Parser


class YAMLLaunchDescriptionSource(FrontendLaunchDescriptionSource):
    """Encapsulation of a YAML launch file, which can be loaded during launch."""

    def __init__(
        self,
        launch_file_path: SomeSubstitutionsType,
    ) -> None:
        """
        Create a YAMLLaunchDescriptionSource.

        The given file path should be to a launch YAML style file (`.launch.yaml`).
        If a relative path is passed, it will be relative to the current working
        directory wherever the launch file was run from.

        :param launch_file_path: the path to the launch file. It path can be made up of
            Substitution instances which are expanded when :py:meth:`get_launch_description()`
            is called.
        """
        super().__init__(
            launch_file_path,
            method='interpreted YAML launch file',
            parser=Parser
        )
