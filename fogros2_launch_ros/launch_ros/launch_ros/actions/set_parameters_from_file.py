# Copyright 2021 Open Source Robotics Foundation, Inc.
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

"""Module for the `SetParametersFromFile` action."""

from launch import Action
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions


@expose_action('set_parameters_from_file')
class SetParametersFromFile(Action):
    """
    Action that sets parameters for all nodes in scope based on a given yaml file.

    e.g.
    ```python3
        LaunchDescription([
            ...,
            GroupAction(
                actions = [
                    ...,
                    SetParametersFromFile('path/to/file.yaml'),
                    ...,
                    Node(...),  // the params will be passed to this node
                    ...,
                ]
            ),
            Node(...),  // here it won't be passed, as it's not in the same scope
            ...
        ])
    ```
    ```xml
    <launch>
        <group>
            <set_parameters_from_file filename='/path/to/file.yaml'/>
            <node .../>    // Node in scope, params will be passed
        </group>
        <node .../>  // Node not in scope, params won't be passed
    </launch>

    ```
    """

    def __init__(
        self,
        filename: SomeSubstitutionsType,
        **kwargs
    ) -> None:
        """Create a SetParameterFromFile action."""
        super().__init__(**kwargs)
        self._input_file = normalize_to_list_of_substitutions(filename)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `SetParameterFromFile` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        kwargs['filename'] = parser.parse_substitution(entity.get_attr('filename'))
        return cls, kwargs

    def execute(self, context: LaunchContext):
        """Execute the action."""
        filename = perform_substitutions(context, self._input_file)
        global_param_list = context.launch_configurations.get('global_params', [])
        global_param_list.append(filename)
        context.launch_configurations['global_params'] = global_param_list
