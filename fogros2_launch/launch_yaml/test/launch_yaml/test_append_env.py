# Copyright 2021 Open Source Robotics Foundation, Inc.
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

"""Test parsing an append_env action."""

import io
import os
import textwrap

from launch.actions import AppendEnvironmentVariable
from launch.frontend import Parser


def test_append_env():
    yaml_file = \
        """\
        launch:
        -   append_env:
                name: my_env_var
                value: asd
        -   append_env:
                name: my_env_var
                value: zxc
                separator: "|"
        -   append_env:
                name: my_other_env_var
                value: fgh
        -   append_env:
                name: my_other_env_var
                value: jkl
                prepend: false
        -   append_env:
                name: my_other_env_var
                value: qwe
                prepend: yes
        -   append_env:
                name: my_other_env_var
                value: rty
                prepend: true
                separator: "|"
        """
    yaml_file = textwrap.dedent(yaml_file)
    root_entity, parser = Parser.load(io.StringIO(yaml_file))
    ld = parser.parse_description(root_entity)
    assert len(ld.entities) == 6
    assert isinstance(ld.entities[0], AppendEnvironmentVariable)
    assert isinstance(ld.entities[1], AppendEnvironmentVariable)
    assert isinstance(ld.entities[2], AppendEnvironmentVariable)
    assert isinstance(ld.entities[3], AppendEnvironmentVariable)
    assert isinstance(ld.entities[4], AppendEnvironmentVariable)
    assert isinstance(ld.entities[5], AppendEnvironmentVariable)
    assert 'my_env_var' == ''.join([x.perform(None) for x in ld.entities[0].name])
    assert 'my_env_var' == ''.join([x.perform(None) for x in ld.entities[0].name])
    assert 'my_other_env_var' == ''.join([x.perform(None) for x in ld.entities[2].name])
    assert 'my_other_env_var' == ''.join([x.perform(None) for x in ld.entities[3].name])
    assert 'my_other_env_var' == ''.join([x.perform(None) for x in ld.entities[4].name])
    assert 'my_other_env_var' == ''.join([x.perform(None) for x in ld.entities[5].name])
    assert 'asd' == ''.join([x.perform(None) for x in ld.entities[0].value])
    assert 'zxc' == ''.join([x.perform(None) for x in ld.entities[1].value])
    assert 'fgh' == ''.join([x.perform(None) for x in ld.entities[2].value])
    assert 'jkl' == ''.join([x.perform(None) for x in ld.entities[3].value])
    assert 'qwe' == ''.join([x.perform(None) for x in ld.entities[4].value])
    assert 'rty' == ''.join([x.perform(None) for x in ld.entities[5].value])
    assert not ld.entities[0].prepend
    assert not ld.entities[1].prepend
    assert not ld.entities[2].prepend
    assert not ld.entities[3].prepend
    assert ld.entities[4].prepend
    assert ld.entities[5].prepend
    assert os.pathsep == ''.join([x.perform(None) for x in ld.entities[0].separator])
    assert '|' == ''.join([x.perform(None) for x in ld.entities[1].separator])
    assert os.pathsep == ''.join([x.perform(None) for x in ld.entities[2].separator])
    assert os.pathsep == ''.join([x.perform(None) for x in ld.entities[3].separator])
    assert os.pathsep == ''.join([x.perform(None) for x in ld.entities[4].separator])
    assert '|' == ''.join([x.perform(None) for x in ld.entities[5].separator])
