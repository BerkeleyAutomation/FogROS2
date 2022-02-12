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

"""Module for the GTest action."""

import sys

from launch import SomeSubstitutionsType

from .test import Test


class PyTest(Test):
    """Action that runs a Pytest test."""

    def __init__(
        self,
        *,
        path: SomeSubstitutionsType,
        **kwargs
    ) -> None:
        """
        Create a PyTest test action.

        timeout argument is passed to :class:`launch_testing.Test`.
        The other arguments are passed to :class:`launch.ExecuteProcess`, so
        see the documentation for the class for additional details.

        :param: path to the test to be executed.
        """
        cmd = [sys.executable, '-m', 'pytest', path]
        super().__init__(cmd=cmd, **kwargs)
        self.__path = path

    @property
    def path(self):
        """Getter for path."""
        return self.__path
