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

"""Module for the DeclareLaunchArgument action."""

from typing import Iterable
from typing import List
from typing import Optional
from typing import Text

import launch.logging

from ..action import Action
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser  # noqa: F401
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions


@expose_action('arg')
class DeclareLaunchArgument(Action):
    """
    Action that declares a new launch argument.

    A launch argument is stored in a "launch configuration" of the same name.
    See :py:class:`launch.actions.SetLaunchConfiguration` and
    :py:class:`launch.substitutions.LaunchConfiguration`.

    Any launch arguments declared within a :py:class:`launch.LaunchDescription`
    will be exposed as arguments when that launch description is included, e.g.
    as additional parameters in the
    :py:class:`launch.actions.IncludeLaunchDescription` action or as
    command-line arguments when launched with ``ros2 launch ...``.

    In addition to the name, which is also where the argument result is stored,
    launch arguments may have a default value, a list of valid value choices, and a description.
    If a default value is given, then the argument becomes optional and the
    default value is placed in the launch configuration instead.
    If no default value is given and no value is given when including the
    launch description, then an error occurs.
    If a choice list is given, and the given value is not in it, an error
    occurs.

    The default value may use Substitutions, but the name and description can
    only be Text, since they need a meaningful value before launching, e.g.
    when listing the command-line arguments.

    Note that declaring a launch argument needs to be in a part of the launch
    description that is describable without launching.
    For example, if you declare a launch argument with this action from within
    a condition group or as a callback to an event handler, then it may not be
    possible for a tool like ``ros2 launch`` to know about the argument before
    launching the launch description.
    In such cases, the argument will not be visible on the command line but
    may raise an exception if that argument is not satisfied once visited (and
    has no default value).

    Put another way, the post-condition of this action being visited is either
    that a launch configuration of the same name is set with a value or an
    exception is raised because none is set and there is no default value.
    However, the pre-condition does not guarantee that the argument was visible
    if behind condition or situational inclusions.

    .. doctest::

        >>> ld = LaunchDescription([
        ...     DeclareLaunchArgument('simple_argument'),
        ...     DeclareLaunchArgument('with_default_value', default_value='default'),
        ...     DeclareLaunchArgument(
        ...         'with_default_and_description',
        ...         default_value='some_default',
        ...         description='this argument is used to configure ...'),
        ...     DeclareLaunchArgument(
        ...         'mode',
        ...         default_value='A',
        ...         description='Choose between mode A and mode B',
        ...         choices=['A', 'B']),
        ...     # other actions here, ...
        ... ])

    .. code-block:: xml

        <launch>
            <arg name="simple_argument"/>
            <arg name="with_default_value" default_value="default"/>
            <arg name="with_default_and_description" default_value="some_default"
                description="this argument is used to configure ..."/>
        </launch>
    """

    def __init__(
        self,
        name: Text,
        *,
        default_value: Optional[SomeSubstitutionsType] = None,
        description: Optional[Text] = None,
        choices: Iterable[Text] = None,
        **kwargs
    ) -> None:
        """Create a DeclareLaunchArgument action."""
        super().__init__(**kwargs)
        self.__name = name
        self.__logger = launch.logging.get_logger(__name__)
        if default_value is None:
            self.__default_value = default_value
        else:
            self.__default_value = normalize_to_list_of_substitutions(default_value)
        if choices is not None:
            if len(choices) == 0:
                self.__logger.error(
                    'Provided choices arg is empty. Use None to ignore the choice list.')
                raise RuntimeError(
                    'Provided choices arg is empty. Use None to ignore the choice list.')

            # Check if a non substitution default value is provided and is a valid choice
            if default_value is not None and not isinstance(default_value, (Substitution, list)):
                if default_value not in choices:
                    self.__logger.error(
                        'Provided default_value "{}" is not in provided choices "{}".'.format(
                            default_value, choices)
                    )
                    raise RuntimeError(
                        'Provided default_value "{}" is not in provided choices "{}".'.format(
                            default_value, choices))

        if description is None:
            if choices is None:
                self.__description = 'no description given'
            else:
                self.__description = 'One of: ' + str(choices)
        else:
            self.__description = description
            if choices is not None:
                if not self.__description.endswith('.'):
                    self.__description += '.'
                self.__description += ' Valid choices are: ' + str(choices)

        self.__choices = choices

        # This is used later to determine if this launch argument will be
        # conditionally visited.
        # Its value will be read and set at different times and so the value
        # may change depending at different times based on the context.
        self._conditionally_included = False

    @classmethod
    def parse(
        cls,
        entity: Entity,
        parser: 'Parser'
    ):
        """Parse `arg` tag."""
        _, kwargs = super().parse(entity, parser)
        kwargs['name'] = parser.escape_characters(entity.get_attr('name'))
        default_value = entity.get_attr('default', optional=True)
        if default_value is not None:
            kwargs['default_value'] = parser.parse_substitution(default_value)
        description = entity.get_attr('description', optional=True)
        if description is not None:
            kwargs['description'] = parser.escape_characters(description)
        choices = entity.get_attr('choice', data_type=List[Entity], optional=True)
        if choices is not None:
            kwargs['choices'] = [
                parser.escape_characters(choice.get_attr('value')) for choice in choices
            ]
        return cls, kwargs

    @property
    def name(self) -> Text:
        """Getter for self.__name."""
        return self.__name

    @property
    def default_value(self) -> Optional[List[Substitution]]:
        """Getter for self.__default_value."""
        return self.__default_value

    @property
    def description(self) -> Text:
        """Getter for self.__description."""
        return self.__description

    @property
    def choices(self) -> List[Text]:
        """Getter for self.__choices."""
        return self.__choices

    def execute(self, context: LaunchContext):
        """Execute the action."""
        if self.name not in context.launch_configurations:
            if self.default_value is None:
                # Argument not already set and no default value given, error.
                self.__logger.error(
                    'Required launch argument "{}" (description: "{}") was not provided'
                    .format(self.name, self.description)
                )
                raise RuntimeError(
                    'Required launch argument "{}" was not provided.'.format(self.name))
            context.launch_configurations[self.name] = \
                perform_substitutions(context, self.default_value)

        if self.__choices is not None:
            value = context.launch_configurations[self.name]
            if value not in self.__choices:
                error_msg = ('Argument "{}" provided value "{}" is not valid. Valid options '
                             'are: {}'.format(self.name, value, self.__choices))
                self.__logger.error(error_msg)
                raise RuntimeError(error_msg)
