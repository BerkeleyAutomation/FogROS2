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

"""Test launch description with deprecated attributte."""

import io
import textwrap

from launch import LaunchDescription
from launch.frontend import Parser


def test_deprecated_launch_file():
    xml_file = \
        """\
        <launch deprecated="MY_DEPRECATED_MESSAGE"/>
        """
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = Parser.load(io.StringIO(xml_file))
    ld = parser.parse_description(root_entity)
    assert isinstance(ld, LaunchDescription)
    assert 'MY_DEPRECATED_MESSAGE' == ld.deprecated_reason
