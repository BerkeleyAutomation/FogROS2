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

"""Test parsing a reset action."""

import io
import textwrap

from launch.actions import ResetLaunchConfigurations, SetLaunchConfiguration
from launch.frontend import Parser
from launch.launch_context import LaunchContext


def test_reset():
    yaml_file = \
        """\
        launch:
            - let:
                name: 'foo'
                value: 'FOO'
            - let:
                name: 'bar'
                value: 'BAR'
            - reset:
                keep:
                    -   name: 'bar'
                        value: $(var bar)
                    -   name: 'baz'
                        value: 'BAZ'
        """  # noqa: E501
    print('Load YAML')
    yaml_file = textwrap.dedent(yaml_file)
    print('Load Parser')
    root_entity, parser = Parser.load(io.StringIO(yaml_file))
    print('Parse Description')
    ld = parser.parse_description(root_entity)

    assert isinstance(ld.entities[0], SetLaunchConfiguration)
    assert isinstance(ld.entities[1], SetLaunchConfiguration)
    assert isinstance(ld.entities[2], ResetLaunchConfigurations)

    lc = LaunchContext()
    assert len(lc.launch_configurations) == 0
    ld.entities[0].visit(lc)
    ld.entities[1].visit(lc)
    assert len(lc.launch_configurations) == 2
    assert 'foo' in lc.launch_configurations.keys()
    assert lc.launch_configurations['foo'] == 'FOO'
    assert 'bar' in lc.launch_configurations.keys()
    assert lc.launch_configurations['bar'] == 'BAR'
    ld.entities[2].visit(lc)
    assert 'foo' not in lc.launch_configurations.keys()
    assert 'bar' in lc.launch_configurations.keys()
    assert lc.launch_configurations['bar'] == 'BAR'
    assert 'baz' in lc.launch_configurations.keys()
    assert lc.launch_configurations['baz'] == 'BAZ'


if __name__ == '__main__':
    test_reset()
