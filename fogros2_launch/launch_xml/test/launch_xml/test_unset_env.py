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

"""Test parsing a unset_env action."""

import io
import textwrap

from launch.actions import UnsetEnvironmentVariable
from launch.frontend import Parser


def test_unset_env():
    xml_file = \
        """\
        <launch>
            <unset_env name="my_env_var"/>
        </launch>
        """
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = Parser.load(io.StringIO(xml_file))
    ld = parser.parse_description(root_entity)
    assert len(ld.entities) == 1
    unset_env = ld.entities[0]
    assert isinstance(unset_env, UnsetEnvironmentVariable)
    assert 'my_env_var' == ''.join([x.perform(None) for x in unset_env.name])
