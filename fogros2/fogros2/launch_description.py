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

"""Module for LaunchDescription class."""

from typing import TYPE_CHECKING, Iterable, List, Optional, Text, Tuple

import launch.logging
from launch.action import Action
from launch.actions import DeclareLaunchArgument
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity

if TYPE_CHECKING:
    from launch.actions.include_launch_description import IncludeLaunchDescription  # noqa: F401

import os
import pickle
from collections import defaultdict
from threading import Thread
from time import sleep

import wgconfig
import wgconfig.wgexec as wgexec


class VPN:
    def __init__(
        self,
        cloud_key_path="/tmp/fogros-cloud.conf",
        robot_key_path="/tmp/fogros-local.conf",
    ):
        self.cloud_key_path = cloud_key_path
        self.robot_key_path = robot_key_path

        self.cloud_name_to_pub_key_path = dict()
        self.cloud_name_to_priv_key_path = dict()

        self.robot_private_key = wgexec.generate_privatekey()
        self.robot_public_key = wgexec.get_publickey(self.robot_private_key)

    def generate_key_pairs(self, machines):
        """
        @param machines: List<machine>
        """
        for machine in machines:
            name = machine.get_name()
            cloud_private_key = wgexec.generate_privatekey()
            self.cloud_name_to_priv_key_path[name] = cloud_private_key
            cloud_public_key = wgexec.get_publickey(cloud_private_key)
            self.cloud_name_to_pub_key_path[name] = cloud_public_key

    def generate_wg_config_files(self, machines):
        self.generate_key_pairs(machines)

        # generate cloud configs
        counter = 2  # start the static ip addr counter from 2
        for machine in machines:
            name = machine.get_name()
            machine_config_pwd = self.cloud_key_path + name
            machine_priv_key = self.cloud_name_to_priv_key_path[name]
            aws_config = wgconfig.WGConfig(machine_config_pwd)
            aws_config.add_attr(None, "PrivateKey", machine_priv_key)
            aws_config.add_attr(None, "ListenPort", 51820)
            aws_config.add_attr(None, "Address", "10.0.0." + str(counter) + "/24")
            aws_config.add_peer(self.robot_public_key, "# fogROS Robot")
            aws_config.add_attr(self.robot_public_key, "AllowedIPs", "10.0.0.1/32")
            aws_config.write_file()
            counter += 1

        # generate robot configs
        robot_config = wgconfig.WGConfig(self.robot_key_path)
        robot_config.add_attr(None, "PrivateKey", self.robot_private_key)
        robot_config.add_attr(None, "ListenPort", 51820)
        robot_config.add_attr(None, "Address", "10.0.0.1/24")
        for machine in machines:
            name = machine.get_name()
            ip = machine.get_ip()
            cloud_pub_key = self.cloud_name_to_pub_key_path[name]
            robot_config.add_peer(cloud_pub_key, "# AWS" + name)
            robot_config.add_attr(cloud_pub_key, "AllowedIPs", "10.0.0.2/32")
            robot_config.add_attr(cloud_pub_key, "Endpoint", f"{ip}:51820")
            robot_config.add_attr(cloud_pub_key, "PersistentKeepalive", 3)
        robot_config.write_file()

    def start_robot_vpn(self):
        # Copy /tmp/fogros-local.conf to /etc/wireguard/wg0.conf locally.
        # TODO: This needs root. Move this to a separate script with setuid.

        os.system("sudo cp /tmp/fogros-local.conf /etc/wireguard/wg0.conf")
        os.system("sudo chmod 600 /etc/wireguard/wg0.conf")
        os.system("sudo wg-quick down wg0")
        os.system("sudo wg-quick up wg0")


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
        launch.logging.get_logger().info("init")
        # self.__entities = list(initial_entities) if initial_entities is not None else []
        self.__entities = []
        self.__to_cloud_entities = defaultdict(list)
        self.__streamed_topics = []
        if initial_entities:
            for entity in initial_entities:
                self.add_entity_with_filter(entity)

        self.__deprecated_reason = deprecated_reason

    def visit(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        """Override visit from LaunchDescriptionEntity to visit contained entities."""

        # dump the to cloud nodes into different files
        for key, value in self.__to_cloud_entities.items():
            with open("/tmp/to_cloud_" + key, "wb+") as f:
                print(key + ": to be dumped")
                dumped_node_str = pickle.dumps(value)
                f.write(dumped_node_str)

        # create VPN credentials to all of the machines
        machines = [self.__to_cloud_entities[n][0].machine for n in self.__to_cloud_entities]
        vpn = VPN()
        vpn.generate_wg_config_files(machines)
        vpn.start_robot_vpn()

        # tell remote machine to push the to cloud nodes and
        # wait here until all the nodes are done
        for machine in machines:
            machine_name = machine.get_name()
            while not machine.get_ready_state():
                print("Waiting for machine " + machine_name)
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
                message = "deprecated launch description: {}".format(self.__deprecated_reason)
            launch.logging.get_logger().warning(message)
        return self.__entities

    def describe_sub_entities(self) -> List[LaunchDescriptionEntity]:
        """Override describe_sub_entities from LaunchDescriptionEntity to return sub entities."""
        return self.__entities

    def get_launch_arguments(self, conditional_inclusion=False) -> List[DeclareLaunchArgument]:
        """
        Return a list of :py:class:`launch.actions.DeclareLaunchArgument` actions.

        See :py:method:`get_launch_arguments_with_include_launch_description_actions()`
        for more details.
        """
        return [
            item[0]
            for item in self.get_launch_arguments_with_include_launch_description_actions(conditional_inclusion)
        ]

    def get_launch_arguments_with_include_launch_description_actions(
        self, conditional_inclusion=False
    ) -> List[Tuple[DeclareLaunchArgument, List["IncludeLaunchDescription"]]]:
        """
        Return a list of launch arguments with its associated include launch descriptions actions.

        The first element of the tuple is a declare launch argument action.
        The second is `None` if the argument was declared at the top level of this
        launch description, if not it's a list with all the nested include launch description
        actions involved.

        This list is generated (never cached) by searching through this launch
        description for any instances of the action that declares launch
        arguments.

        It will use :py:meth:`launch.LaunchDescriptionEntity.describe_sub_entities`
        and :py:meth:`launch.LaunchDescriptionEntity.describe_conditional_sub_entities`
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

        declared_launch_arguments: List[Tuple[DeclareLaunchArgument, List[IncludeLaunchDescription]]] = []
        from launch.actions import ResetLaunchConfigurations

        def process_entities(entities, *, _conditional_inclusion, nested_ild_actions=None):
            for entity in entities:
                if isinstance(entity, DeclareLaunchArgument):
                    # Avoid duplicate entries with the same name.
                    if entity.name in (e.name for e, _ in declared_launch_arguments):
                        continue
                    # Stuff this contextual information into the class for
                    # potential use in command-line descriptions or errors.
                    entity._conditionally_included = _conditional_inclusion
                    entity._conditionally_included |= entity.condition is not None
                    declared_launch_arguments.append((entity, nested_ild_actions))
                if isinstance(entity, ResetLaunchConfigurations):
                    # Launch arguments after this cannot be set directly by top level arguments
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
                    for conditional_sub_entity in entity.describe_conditional_sub_entities():
                        process_entities(
                            conditional_sub_entity[1],
                            _conditional_inclusion=True,
                            nested_ild_actions=next_nested_ild_actions,
                        )

        process_entities(self.entities, _conditional_inclusion=conditional_inclusion)

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
            self.__to_cloud_entities[entity.get_unique_id()].append(entity)
            if entity.stream_topics:
                for stream_topic in entity.stream_topics:
                    self.add_image_transport_entities(stream_topic[0], stream_topic[1], entity.machine)
        else:
            self.__entities.append(entity)

    def add_image_transport_entities(self, topic_name, intermediate_transport, machine):
        """Adds image transport nodes to the cloud and robot."""
        from launch_ros.actions import Node

        import fogros2

        self.__streamed_topics.append(topic_name)
        new_cloud_topic_name = topic_name + "/cloud"
        print(f"Added {intermediate_transport} transport decoder/subscriber for topic {topic_name}")
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
                ("in/" + intermediate_transport, topic_name + "/" + intermediate_transport),
                ("out", new_cloud_topic_name),
            ],
        )

        self.__to_cloud_entities[decoder_node.get_unique_id()].append(decoder_node)

        print(f"Added {intermediate_transport} transport encoder/publisher for topic {topic_name}")
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
                ("out/" + intermediate_transport, topic_name + "/" + intermediate_transport),
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
