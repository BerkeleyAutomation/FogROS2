# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Module for LaunchConfigurationNotEquals class."""

from typing import Optional
from typing import Text

from .launch_configuration_equals import LaunchConfigurationEquals
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType


class LaunchConfigurationNotEquals(LaunchConfigurationEquals):
    """
    Condition on the value of a launch configuration.

    This condition takes an optional string expression that is compared with the value of
    a launch configuration.
    If the value is not equal to the launch configuration value, then this ``Condition``
    evaluates to ``True``.
    The expression may consist of :py:class:`launch.Substitution` instances.

    If ``None`` is provided instead of a string expression, then the condition
    evaluates to ``True`` if the launch configuration is set.
    """

    def __init__(
        self,
        launch_configuration_name: Text,
        expected_value: Optional[SomeSubstitutionsType]
    ) -> None:
        super().__init__(launch_configuration_name, expected_value)

    def _predicate_func(self, context: LaunchContext) -> bool:
        return not super()._predicate_func(context)

    def describe(self) -> Text:
        """Return a description of this Condition."""
        return self.__repr__()
