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

"""Test parsing a `LogInfo` action."""

import io
import textwrap

from launch import LaunchContext
from launch.actions import LogInfo
from launch.frontend import Parser
from launch.utilities import perform_substitutions


def test_log():
    launch_context = LaunchContext()
    xml_file = \
        """\
        <launch>
            <log message="Hello world!" />
        </launch>
        """
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = Parser.load(io.StringIO(xml_file))
    launch_description = parser.parse_description(root_entity)
    log_info = launch_description.entities[0]
    assert isinstance(log_info, LogInfo)
    assert perform_substitutions(launch_context, log_info.msg) == 'Hello world!'
