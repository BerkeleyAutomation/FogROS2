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

"""Python package utility functions related to loading Frontend Launch Files."""

import os
from typing import Text
from typing import Type

from .frontend_launch_file_utilities import get_launch_description_from_frontend_launch_file
from .python_launch_file_utilities import get_launch_description_from_python_launch_file
from ..frontend import Parser
from ..invalid_launch_file_error import InvalidLaunchFileError
from ..launch_description import LaunchDescription


def get_launch_description_from_any_launch_file(
    launch_file_path: Text,
    *,
    parser: Type[Parser] = Parser
) -> LaunchDescription:
    """
    Load a given launch file (by path), and return the launch description from it.

    :raise `InvalidLaunchFileError`: Failed to load launch file.
        It's only showed with launch files without extension (or not recognized extensions).
    :raise `SyntaxError`: Invalid file. The file may have a syntax error in it.
    :raise `ValueError`: Invalid file. The file may not be a text file.
    """
    loaders = [get_launch_description_from_frontend_launch_file]
    launch_file_name = os.path.basename(launch_file_path)
    extension = os.path.splitext(launch_file_name)[1]
    if extension:
        extension = extension[1:]
    if extension == 'py':
        loaders.insert(0, get_launch_description_from_python_launch_file)
    else:
        loaders.append(get_launch_description_from_python_launch_file)
        extension = '' if not Parser.is_filename_valid(launch_file_name) else extension
    exceptions = []
    for loader in loaders:
        try:
            return loader(launch_file_path)
        except Exception as ex:
            exceptions.append(ex)
    raise InvalidLaunchFileError(extension, likely_errors=exceptions)
