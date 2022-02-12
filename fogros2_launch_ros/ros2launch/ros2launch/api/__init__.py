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

"""Python package for the ros2 launch api."""

from launch.launch_description_sources import InvalidLaunchFileError
from launch.launch_description_sources import InvalidPythonLaunchFileError

from .api import get_share_file_path_from_package
from .api import is_launch_file
from .api import launch_a_launch_file
from .api import LaunchFileNameCompleter
from .api import MultipleLaunchFilesError
from .api import print_a_launch_file
from .api import print_arguments_of_launch_description
from .api import print_arguments_of_launch_file


__all__ = [
    'get_share_file_path_from_package',
    'is_launch_file',
    'InvalidLaunchFileError',
    'InvalidPythonLaunchFileError',
    'LaunchFileNameCompleter',
    'launch_a_launch_file',
    'MultipleLaunchFilesError',
    'print_a_launch_file',
    'print_arguments_of_launch_description',
    'print_arguments_of_launch_file',
]
