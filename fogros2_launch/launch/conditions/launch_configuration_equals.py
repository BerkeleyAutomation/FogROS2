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

"""Module for LaunchConfigurationEquals class."""

from typing import Optional
from typing import Text

from ..condition import Condition
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions


class LaunchConfigurationEquals(Condition):
    """
    Condition on the value of a launch configuration.

    This condition takes an optional string expression that is compared with the value of
    a launch configuration.
    If the value is equal to the launch configuration value, then this ``Condition``
    evaluates to ``True``.
    The expression may consist of :py:class:`launch.Substitution` instances.

    If ``None`` is provided instead of a string expression, then the condition
    evaluates to ``True`` if the launch configuration is not set.
    """

    def __init__(
        self,
        launch_configuration_name: Text,
        expected_value: Optional[SomeSubstitutionsType]
    ) -> None:
        self.__launch_configuration_name = launch_configuration_name
        if expected_value is not None:
            self.__expected_value = normalize_to_list_of_substitutions(expected_value)
        else:
            self.__expected_value = None
        super().__init__(predicate=self._predicate_func)

    def _predicate_func(self, context: LaunchContext) -> bool:
        expanded_expected_value = None
        if self.__expected_value is not None:
            expanded_expected_value = perform_substitutions(context, self.__expected_value)
        try:
            value = context.launch_configurations[self.__launch_configuration_name]
            return value == expanded_expected_value
        except KeyError:
            if expanded_expected_value is None:
                return True
        return False

    def describe(self) -> Text:
        """Return a description of this Condition."""
        return self.__repr__()
