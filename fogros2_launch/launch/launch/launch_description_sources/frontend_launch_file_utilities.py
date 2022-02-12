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

from typing import Text
from typing import Type

from ..frontend import InvalidFrontendLaunchFileError
from ..frontend import Parser
from ..launch_description import LaunchDescription

# Re-export name
InvalidFrontendLaunchFileError = InvalidFrontendLaunchFileError


def get_launch_description_from_frontend_launch_file(
    frontend_launch_file_path: Text,
    *,
    parser: Type[Parser] = Parser
) -> LaunchDescription:
    """Load a `LaunchDescription` from a declarative (markup based) launch file."""
    root_entity, parser = parser.load(frontend_launch_file_path)
    return parser.parse_description(root_entity)
