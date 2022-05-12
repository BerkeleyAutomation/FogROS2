# Copyright 2022 The Regents of the University of California (Regents)
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
#
# Copyright ©2022. The Regents of the University of California (Regents).
# All Rights Reserved. Permission to use, copy, modify, and distribute this
# software and its documentation for educational, research, and not-for-profit
# purposes, without fee and without a signed licensing agreement, is hereby
# granted, provided that the above copyright notice, this paragraph and the
# following two paragraphs appear in all copies, modifications, and
# distributions. Contact The Office of Technology Licensing, UC Berkeley, 2150
# Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-7201,
# otl@berkeley.edu, http://ipira.berkeley.edu/industry-info for commercial
# licensing opportunities. IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE. REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY,
# PROVIDED HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
# MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

from typing import TYPE_CHECKING, Iterable, List, Optional, Text, Tuple

import launch.logging
from launch.action import Action
from launch.actions import DeclareLaunchArgument
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity

if TYPE_CHECKING:
    from launch.actions.include_launch_description import (
        IncludeLaunchDescription,
    )  # noqa: F401

import pickle
from collections import defaultdict
from threading import Thread
from time import sleep

from .vpn import VPN


class FogROSLaunchDescription(LaunchDescriptionEntity):
    """
    Description of a launch-able system.

    The description is expressed by a collection of entities which represent
    the system architect's intentions.

    The description may also have arguments, which are declared by
    :py:class:`launch.actions.DeclareLaunchArgument` actions within this
    launch description.

    Arguments for this description may be accessed via the
    :py:meth:`get_launch_arguments` method.
    The arguments are gathered by searching through the entities in this
    launch description and the descriptions of each entity (which may include
    entities yielded by those entities).
    """

    def __init__(
        self,
        initial_entities: Optional[Iterable[LaunchDescriptionEntity]] = None,
        *,
        deprecated_reason: Optional[Text] = None,
    ) -> None:
        """Create a LaunchDescription."""
        self.__entities = []
        self.__to_cloud_entities = defaultdict(list)
        self.__streamed_topics = []
        if initial_entities:
            for entity in initial_entities:
                self.add_entity_with_filter(entity)

        self.__deprecated_reason = deprecated_reason

    def visit(
        self, context: LaunchContext
    ) -> Optional[List[LaunchDescriptionEntity]]:
        """Override LaunchDescriptionEntity to visit contained entities."""
        # dump the to cloud nodes into different files
        for key, value in self.__to_cloud_entities.items():
            with open(f"/tmp/to_cloud_{key}", "wb+") as f:
                print(f"{key}: to be dumped")
                dumped_node_str = pickle.dumps(value)
                f.write(dumped_node_str)

        # create VPN credentials to all of the machines
        machines = [
            self.__to_cloud_entities[n][0].machine
            for n in self.__to_cloud_entities
        ]
        vpn = VPN()
        vpn.generate_wg_config_files(machines)
        vpn.start_robot_vpn()

        # tell remote machine to push the to cloud nodes and
        # wait here until all the nodes are done
        for machine in machines:
            while not machine.is_created:
                print(f"Waiting for machine {machine.name}")
                sleep(1)
            # machine is ready, # push to_cloud and setup vpn
            machine.push_to_cloud_nodes()
            machine.push_and_setup_vpn()
            machine.configure_DDS()
            machine.launch_cloud_dockers()
            thread = Thread(target=machine.launch_cloud_node, args=[])
            thread.start()

        if self.__deprecated_reason is not None:
            if "current_launch_file_path" in context.get_locals_as_dict():
                message = "launch file [{}] is deprecated: {}".format(
                    context.locals.current_launch_file_path,
                    self.__deprecated_reason,
                )
            else:
                message = "deprecated launch description: {}".format(
                    self.__deprecated_reason
                )
            launch.logging.get_logger().warning(message)
        return self.__entities

    def describe_sub_entities(self) -> List[LaunchDescriptionEntity]:
        """Override from LaunchDescriptionEntity to return sub entities."""
        return self.__entities

    def get_launch_arguments(
        self, conditional_inclusion=False
    ) -> List[DeclareLaunchArgument]:
        """
        Return list of :py:class:`launch.actions.DeclareLaunchArgument`.

        See
        :py:method:`get_launch_arguments_with_include_launch_description_actions()`
        for more details.
        """
        return [
            item[0]
            for item in
            self.get_launch_arguments_with_include_launch_description_actions(
                conditional_inclusion
            )
        ]

    def get_launch_arguments_with_include_launch_description_actions(
        self, conditional_inclusion=False
    ) -> List[Tuple[DeclareLaunchArgument, List["IncludeLaunchDescription"]]]:
        """
        Return list of launch args with associated actions.

        The first element of the tuple is a declare launch argument action.
        The second is `None` if the argument was declared at the top level of
        this launch description, if not it's a list with all the nested
        include launch description actions involved.

        This list is generated (never cached) by searching through this launch
        description for any instances of the action that declares launch
        arguments.

        It will use
        :py:meth:`launch.LaunchDescriptionEntity.describe_sub_entities`
        and
        :py:meth:`launch.LaunchDescriptionEntity.describe_conditional_sub_entities`
        in order to discover as many instances of the declare launch argument
        actions as is possible.
        Also, specifically in the case of the
        :py:class:`launch.actions.IncludeLaunchDescription` action, the method
        :py:meth:`launch.LaunchDescriptionSource.try_get_launch_description_without_context`
        is used to attempt to load launch descriptions without the "runtime"
        context available.
        This function may fail, e.g. if the path to the launch file to include
        uses the values of launch configurations that have not been set yet,
        and in that case the failure is ignored and the arugments defined in
        those launch files will not be seen either.

        Duplicate declarations of an argument are ignored, therefore the
        default value and description from the first instance of the argument
        declaration is used.
        """
        from launch.actions import IncludeLaunchDescription  # noqa: F811

        declared_launch_arguments: List[
            Tuple[DeclareLaunchArgument, List[IncludeLaunchDescription]]
        ] = []
        from launch.actions import ResetLaunchConfigurations

        def process_entities(
            entities, *, _conditional_inclusion, nested_ild_actions=None
        ):
            for entity in entities:
                if isinstance(entity, DeclareLaunchArgument):
                    # Avoid duplicate entries with the same name.
                    if entity.name in (
                        e.name for e, _ in declared_launch_arguments
                    ):
                        continue
                    # Stuff this contextual information into the class for
                    # potential use in command-line descriptions or errors.
                    entity._conditionally_included = _conditional_inclusion
                    entity._conditionally_included |= (
                        entity.condition is not None
                    )
                    declared_launch_arguments.append(
                        (entity, nested_ild_actions)
                    )
                if isinstance(entity, ResetLaunchConfigurations):
                    # Launch arguments after this cannot be set directly
                    # by top level arguments
                    return
                else:
                    next_nested_ild_actions = nested_ild_actions
                    if isinstance(entity, IncludeLaunchDescription):
                        if next_nested_ild_actions is None:
                            next_nested_ild_actions = []
                        next_nested_ild_actions.append(entity)
                    process_entities(
                        entity.describe_sub_entities(),
                        _conditional_inclusion=False,
                        nested_ild_actions=next_nested_ild_actions,
                    )
                    for (
                        conditional_sub_entity
                    ) in entity.describe_conditional_sub_entities():
                        process_entities(
                            conditional_sub_entity[1],
                            _conditional_inclusion=True,
                            nested_ild_actions=next_nested_ild_actions,
                        )

        process_entities(
            self.entities, _conditional_inclusion=conditional_inclusion
        )

        return declared_launch_arguments

    @property
    def entities(self) -> List[LaunchDescriptionEntity]:
        """Getter for the entities."""
        return self.__entities

    def add_entity(self, entity: LaunchDescriptionEntity) -> None:
        """Add an entity to the LaunchDescription."""
        # self.__entities.append(entity)
        self.add_entity_with_filter(entity)

    def add_entity_with_filter(self, entity):
        if entity.__class__.__name__ == "CloudNode":
            self.__to_cloud_entities[entity.unique_id].append(entity)
            if entity.stream_topics:
                for stream_topic in entity.stream_topics:
                    self.add_image_transport_entities(
                        stream_topic[0], stream_topic[1], entity.machine
                    )
        else:
            self.__entities.append(entity)

    def add_image_transport_entities(
        self, topic_name, intermediate_transport, machine
    ):
        """Add image transport nodes to the cloud and robot."""
        from launch_ros.actions import Node

        import fogros2

        self.__streamed_topics.append(topic_name)
        new_cloud_topic_name = f"{topic_name}/cloud"
        print(
            f"Added {intermediate_transport} transport decoder/subscriber "
            f"for topic {topic_name}"
        )
        decoder_node = fogros2.CloudNode(
            machine=machine,
            package="image_transport",
            executable="republish",
            output="screen",
            name="republish_node",
            arguments=[
                intermediate_transport,  # Input
                "raw",  # Output
            ],
            remappings=[
                (
                    f"in/{intermediate_transport}",
                    f"{topic_name}/{intermediate_transport}",
                ),
                ("out", new_cloud_topic_name),
            ],
        )

        self.__to_cloud_entities[decoder_node.unique_id].append(decoder_node)

        print(
            f"Added {intermediate_transport} transport encoder/publisher "
            f"for topic {topic_name}"
        )
        encoder_node = Node(
            package="image_transport",
            executable="republish",
            output="screen",
            name="republish_node2",
            arguments=[
                "raw",  # Input
                intermediate_transport,  # Output
            ],
            remappings=[
                ("in", topic_name),
                (
                    f"out/{intermediate_transport}",
                    f"{topic_name}/{intermediate_transport}",
                ),
            ],
        )
        self.__entities.append(encoder_node)

    def add_action(self, action: Action) -> None:
        """Add an action to the LaunchDescription."""
        self.add_entity(action)

    @property
    def deprecated(self) -> bool:
        """Getter for deprecated."""
        return self.__deprecated_reason is not None

    @property
    def deprecated_reason(self) -> Optional[Text]:
        """
        Getter for deprecated.

        Returns `None` if the launch description is not deprecated.
        """
        return self.__deprecated_reason
