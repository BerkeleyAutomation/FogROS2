# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

"""Utility functions to help detect non-unique node names."""

from collections import defaultdict

from launch.launch_context import LaunchContext


def add_node_name(context: LaunchContext, node_name: str) -> None:
    """
    Add a node name to the context, indicating an occurrence of the node name.

    :param context: the context that keeps track of the node names
    :param node_name: the node name to keep track
    """
    try:
        unique_node_names = context.locals.unique_ros_node_names
    except AttributeError:
        context.extend_globals({'unique_ros_node_names': defaultdict(int)})
        unique_node_names = context.locals.unique_ros_node_names
    unique_node_names[node_name] += 1


def get_node_name_count(context: LaunchContext, node_name: str) -> int:
    """
    Get the number of times the node name has occurred, according to the context.

    :param context: the context that keeps track of the node names
    :param node_name: the node name to keep track
    :returns: number of times the node name has occurred
    """
    try:
        unique_node_names = context.locals.unique_ros_node_names
    except AttributeError:
        return 0
    return unique_node_names[node_name]
