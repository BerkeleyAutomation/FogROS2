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

"""Test parsing list attributes."""

import io
import textwrap
from typing import List

from launch.frontend import Parser


def test_list():
    """Parse tags with list attributes."""
    xml_file = \
        """\
        <root>
            <tag attr="1,2,3" attr-sep=","/>
            <tag attr="1 2 3" attr-sep=" "/>
            <tag attr="1, 2, 3" attr-sep=", "/>
        </root>
        """
    xml_file = textwrap.dedent(xml_file)
    root_entity, parser = Parser.load(io.StringIO(xml_file))
    tags = root_entity.children
    assert tags[0].get_attr('attr', data_type=List[str]) == ['1', '2', '3']
    assert tags[0].get_attr('attr', data_type=List[int]) == [1, 2, 3]
    assert tags[0].get_attr('attr', data_type=List[float]) == [1., 2., 3.]


if __name__ == '__main__':
    test_list()
