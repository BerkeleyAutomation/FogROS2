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

"""Test parsing a group action."""

import io
import textwrap

from launch.actions import GroupAction, PopLaunchConfigurations, PushLaunchConfigurations
from launch.actions import ResetLaunchConfigurations, SetLaunchConfiguration
from launch.frontend import Parser
from launch.launch_context import LaunchContext


def test_group():
    yaml_file = \
        """\
        launch:
            - let:
                name: 'foo'
                value: 'FOO'
            - let:
                name: 'bar'
                value: 'BAR'
            - group:
                scoped: True
                forwarding: False
                keep:
                    -   name: 'bar'
                        value: $(var bar)
                    -   name: 'baz'
                        value: 'BAZ'
                children:
                    - let:
                        name: 'var1'
                        value: 'asd'
                    - let:
                        name: 'var2'
                        value: 'asd'
        """  # noqa: E501
    yaml_file = textwrap.dedent(yaml_file)
    root_entity, parser = Parser.load(io.StringIO(yaml_file))
    ld = parser.parse_description(root_entity)

    assert isinstance(ld.entities[0], SetLaunchConfiguration)
    assert isinstance(ld.entities[1], SetLaunchConfiguration)
    assert isinstance(ld.entities[2], GroupAction)

    lc = LaunchContext()
    assert 0 == len(lc.launch_configurations)
    ld.entities[0].visit(lc)
    ld.entities[1].visit(lc)
    assert 2 == len(lc.launch_configurations)
    assert 'foo' in lc.launch_configurations.keys()
    assert 'FOO' == lc.launch_configurations['foo']
    assert 'bar' in lc.launch_configurations.keys()
    assert 'BAR' == lc.launch_configurations['bar']
    actions = ld.entities[2].execute(lc)
    assert 5 == len(actions)
    assert isinstance(actions[0], PushLaunchConfigurations)
    assert isinstance(actions[1], ResetLaunchConfigurations)
    assert isinstance(actions[2], SetLaunchConfiguration)
    assert isinstance(actions[3], SetLaunchConfiguration)
    assert isinstance(actions[4], PopLaunchConfigurations)
    actions[0].visit(lc)
    actions[1].visit(lc)
    assert 'foo' not in lc.launch_configurations.keys()
    assert 'bar' in lc.launch_configurations.keys()
    assert 'BAR' == lc.launch_configurations['bar']
    assert 'baz' in lc.launch_configurations.keys()
    assert 'BAZ' == lc.launch_configurations['baz']
    actions[2].visit(lc)
    actions[3].visit(lc)
    actions[4].visit(lc)


if __name__ == '__main__':
    test_group()
