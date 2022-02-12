# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Test parsing let and var actions."""

import io
import textwrap

from launch import LaunchContext
from launch.frontend import Parser


def test_let_var():
    """Parse let and var example."""
    xml_file = \
        """\
        <launch>
            <let name="var1" value="asd"/>
            <let name="var2" value="2 $(var var1)"/>
        </launch>
        """
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = Parser.load(io.StringIO(xml_file))
    ld = parser.parse_description(root_entity)
    context = LaunchContext()
    assert len(ld.entities) == 2
    ld.entities[0].execute(context)
    ld.entities[1].execute(context)
    assert context.launch_configurations['var1'] == 'asd'
    assert context.launch_configurations['var2'] == '2 asd'


if __name__ == '__main__':
    test_let_var()
