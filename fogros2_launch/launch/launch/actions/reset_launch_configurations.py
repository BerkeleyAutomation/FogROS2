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

"""Module for the ResetLaunchConfigurations action."""

from typing import Dict
from typing import List
from typing import Optional

from ..action import Action
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions


@expose_action('reset')
class ResetLaunchConfigurations(Action):
    """
    Action that resets launch configurations in the current context.

    This action can be used to clear the launch configurations from the
    context it was called in.
    It optionally can be given a dictionary with launch configurations
    to be set after clearing.
    Launch configurations given in the dictionary are evaluated before
    the context launch configurations are cleared.
    This allows launch configurations to be passed through the clearing
    of the context.

    If launch_configurations is None or an empty dict then all launch configurations
    will be cleared.

    If launch_configurations has entries (i.e. {'foo': 'FOO'}) then these will be
    set after the clearing operation.
    """

    def __init__(
        self,
        launch_configurations: Optional[Dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        **kwargs
    ) -> None:
        """Create an ResetLaunchConfigurations action."""
        super().__init__(**kwargs)
        self.__launch_configurations = launch_configurations

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `ResetLaunchConfigurations` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        keeps = entity.get_attr('keep', data_type=List[Entity], optional=True)
        if keeps is not None:
            kwargs['launch_configurations'] = {
                    tuple(parser.parse_substitution(e.get_attr('name'))):
                    parser.parse_substitution(e.get_attr('value')) for e in keeps
            }
            for e in keeps:
                e.assert_entity_completely_parsed()
        return cls, kwargs

    def execute(self, context: LaunchContext):
        """Execute the action."""
        if self.__launch_configurations is None:
            context.launch_configurations.clear()
        else:
            evaluated_configurations = {}
            for k, v in self.__launch_configurations.items():
                evaluated_k = perform_substitutions(context, normalize_to_list_of_substitutions(k))
                evaluated_v = perform_substitutions(context, normalize_to_list_of_substitutions(v))
                evaluated_configurations[evaluated_k] = evaluated_v

            context.launch_configurations.clear()
            context.launch_configurations.update(evaluated_configurations)
