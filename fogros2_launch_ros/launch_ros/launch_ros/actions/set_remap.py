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

"""Module for the `SetRemap` action."""

from typing import List

from launch import Action
from launch import Substitution
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions


@expose_action('set_remap')
class SetRemap(Action):
    """
    Action that sets a remapping rule in the current context.

    This remapping rule will be passed to all the nodes launched in the same scope, overriding
    the ones specified in the `Node` action constructor.
    e.g.:
    ```python3
        LaunchDescription([
            ...,
            GroupAction(
                actions = [
                    ...,
                    SetRemap(src='asd', dst='bsd'),
                    ...,
                    Node(...),  // the remap rule will be passed to this node
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
        src: SomeSubstitutionsType,
        dst: SomeSubstitutionsType,
        **kwargs
    ) -> None:
        """Create a SetRemap action."""
        super().__init__(**kwargs)
        self.__src = normalize_to_list_of_substitutions(src)
        self.__dst = normalize_to_list_of_substitutions(dst)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `SetRemap` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        kwargs['src'] = parser.parse_substitution(entity.get_attr('from'))
        kwargs['dst'] = parser.parse_substitution(entity.get_attr('to'))
        return cls, kwargs

    @property
    def src(self) -> List[Substitution]:
        """Getter for src."""
        return self.__src

    @property
    def dst(self) -> List[Substitution]:
        """Getter for dst."""
        return self.__dst

    def execute(self, context: LaunchContext):
        """Execute the action."""
        src = perform_substitutions(context, self.__src)
        dst = perform_substitutions(context, self.__dst)
        global_remaps = context.launch_configurations.get('ros_remaps', [])
        global_remaps.append((src, dst))
        context.launch_configurations['ros_remaps'] = global_remaps
