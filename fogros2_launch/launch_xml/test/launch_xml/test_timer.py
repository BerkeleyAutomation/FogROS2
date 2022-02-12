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

"""Test parsing a `DeclareLaunchArgument` action."""

import io
import math
import textwrap

from launch.actions import TimerAction
from launch.frontend import Parser
from launch.substitutions import LaunchConfiguration


def test_timer():
    xml_file = \
        """\
        <launch>
            <timer period="5">
                <executable cmd="ls -las"/>
            </timer>
        </launch>
        """
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = Parser.load(io.StringIO(xml_file))
    ld = parser.parse_description(root_entity)
    timer = ld.entities[0]
    assert isinstance(timer, TimerAction)
    assert isinstance(timer.period, float)
    assert math.isclose(timer.period, 5.)
    assert len(timer.actions) == 1


def test_timer_period_is_substitution():
    xml_file = \
        """\
        <launch>
            <timer period="$(var my_period 5)">
                <executable cmd="ls -las"/>
            </timer>
        </launch>
        """
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = Parser.load(io.StringIO(xml_file))
    ld = parser.parse_description(root_entity)
    timer = ld.entities[0]
    assert isinstance(timer, TimerAction)
    assert isinstance(timer.period, list)
    assert len(timer.period) == 1
    assert isinstance(timer.period[0], LaunchConfiguration)
    assert len(timer.actions) == 1
