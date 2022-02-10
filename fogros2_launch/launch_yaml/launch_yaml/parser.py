# Copyright 2019 Open Source Robotics Foundation, Inc.
# Copyright 2020 Open Avatar Inc.
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

"""Module for YAML Parser class."""

from typing import Set
from typing import Text
from typing import TextIO
from typing import Union

from launch import frontend
from launch.utilities.typing_file_path import FilePath

import yaml

from .entity import Entity


class Parser(frontend.Parser):
    """YAML parser implementation."""

    @classmethod
    def load(
        cls,
        file: Union[FilePath, TextIO],
    ) -> (Entity, 'Parser'):
        """Return entity loaded from YAML file."""
        try:
            fileobj = open(file, 'r')
            didopen = True
        except TypeError:
            fileobj = file
            didopen = False

        try:
            tree = yaml.safe_load(fileobj)
            if len(tree) != 1:
                raise RuntimeError('Expected only one root')
            type_name = list(tree.keys())[0]
            return (Entity(tree[type_name], type_name), cls())
        finally:
            if didopen:
                fileobj.close()

    @classmethod
    def get_file_extensions(cls) -> Set[Text]:
        """Return the set of file extensions known to this parser."""
        return {'launch.yaml', 'launch.yml', 'yaml', 'yml', 'launch'}
