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

"""Module for the `SetParameter` action."""

from launch import Action
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions

from launch_ros.parameters_type import ParameterName
from launch_ros.parameters_type import ParameterValue
from launch_ros.parameters_type import SomeParameterValue
from launch_ros.utilities.evaluate_parameters import evaluate_parameter_dict
from launch_ros.utilities.normalize_parameters import normalize_parameter_dict


@expose_action('set_parameter')
class SetParameter(Action):
    """
    Action that sets a parameter in the current context.

    This parameter will be set in all the nodes launched in the same scope.
    e.g.:
    ```python3
        LaunchDescription([
            ...,
            GroupAction(
                actions = [
                    ...,
                    SetParameter(name='my_param', value='2'),
                    ...,
                    Node(...),  // the param will be passed to this node
                    ...,
                ]
            ),
            Node(...),  // here it won't be passed, as it's not in the same scope
            ...
        ])
    ```
    """

    def __init__(
        self,
        name: SomeSubstitutionsType,
        value: SomeParameterValue,
        **kwargs
    ) -> None:
        """Create a SetParameter action."""
        super().__init__(**kwargs)
        normalized_name = normalize_to_list_of_substitutions(name)
        self.__param_dict = normalize_parameter_dict({tuple(normalized_name): value})

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `SetParameter` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        kwargs['name'] = parser.parse_substitution(entity.get_attr('name'))
        kwargs['value'] = parser.parse_substitution(entity.get_attr('value'))
        return cls, kwargs

    @property
    def name(self) -> ParameterName:
        """Getter for name."""
        return self.__param_dict.keys()[0]

    @property
    def value(self) -> ParameterValue:
        """Getter for value."""
        return self.__param_dict.values()[0]

    def execute(self, context: LaunchContext):
        """Execute the action."""
        eval_param_dict = evaluate_parameter_dict(context, self.__param_dict)
        global_param_list = context.launch_configurations.get('global_params', [])
        global_param_list.extend(eval_param_dict.items())
        context.launch_configurations['global_params'] = global_param_list
