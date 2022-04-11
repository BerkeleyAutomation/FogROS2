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

"""Implementation of `InvalidLaunchFileError` class."""


class InvalidLaunchFileError(Exception):
    """Exception raised when the given launch file is not valid."""

    def __init__(self, extension='', *, likely_errors=None):
        """Create an InvalidLaunchFileError."""
        self._extension = extension
        self._likely_errors = likely_errors
        if self._extension == '' or not self._likely_errors:
            self._error_message = (
                'The launch file may have a syntax error, or its format is unknown'
            )
        else:
            self._error_message = (
                'Caught exception when trying to load file of format [{}]: {}'
            ).format(self._extension, self._likely_errors[0])
            self.__cause__ = self._likely_errors[0]

    def __str__(self):
        """Pretty print."""
        return self._error_message
