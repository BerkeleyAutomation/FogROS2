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

"""Tests for the LifecycleNode Action."""

from launch import LaunchContext
from launch_ros.actions import LifecycleNode

import pytest


def test_lifecycle_node_constructor():
    # Construction without namespace
    with pytest.raises(TypeError):
        LifecycleNode(
            package='asd',
            executable='bsd',
            name='my_node',
        )
    with pytest.raises(TypeError):
        LifecycleNode(
            package='asd',
            executable='bsd',
            namespace='my_ns',
        )
    # Successfull construction
    LifecycleNode(
        package='asd',
        executable='bsd',
        name='my_node',
        namespace='my_ns',
    )


def test_node_name():
    node_object = LifecycleNode(
        package='asd',
        executable='bsd',
        name='my_node',
        namespace='my_ns',
    )
    lc = LaunchContext()
    node_object._perform_substitutions(lc)
    assert node_object.is_node_name_fully_specified() is True
