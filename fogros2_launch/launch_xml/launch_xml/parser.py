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

"""Module for XML Parser class."""

import io
from typing import Set
from typing import Text
from typing import Union
import xml.etree.ElementTree as ET

from launch import frontend

from .entity import Entity


class Parser(frontend.Parser):
    """XML parser implementation."""

    @classmethod
    def load(
        cls,
        file: Union[str, io.TextIOBase],
    ) -> (Entity, 'Parser'):
        """Return entity loaded from XML file."""
        return (Entity(ET.parse(file).getroot()), cls())

    @classmethod
    def get_file_extensions(cls) -> Set[Text]:
        """Return the set of file extensions known to this parser."""
        return {'launch.xml', 'xml', 'launch'}
