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

"""Module for Entity class."""

from typing import List
from typing import Optional
from typing import Text
from typing import Union

from launch.utilities.type_utils import AllowedTypesType
from launch.utilities.type_utils import AllowedValueType


class Entity:
    """Single item in the intermediate front_end representation."""

    @property
    def type_name(self) -> Text:
        """Get Entity type."""
        raise NotImplementedError()

    @property
    def parent(self) -> Optional['Entity']:
        """Get Entity parent."""
        raise NotImplementedError()

    @property
    def children(self) -> List['Entity']:
        """Get the Entity's children."""
        raise NotImplementedError()

    def get_attr(
        self,
        name: Text,
        *,
        data_type: AllowedTypesType = str,
        optional: bool = False,
        can_be_str: bool = True,
    ) -> Optional[Union[
        AllowedValueType,
        List['Entity'],
    ]]:
        """
        Access an attribute of the entity.

        By default, it will try to return it as an string.
        `data_type` is the expected type of the attribute. Type coercion or type checking is
        applied depending on the particular frontend.

        See :py:obj:`launch.utilities.AllowedTypesTuple` to see what types are allowed.

        `data_type = None` will result in yaml parsing of the attribute value as a string.

        `List[Entity]` allows accessing a list of subentities with an specific name.

        Check the documentation of each specific frontend implementation to see how a list of
        attributes or `List[Entity]` look like.

        If `optional` is `True` and the attribute cannot be found, `None` will be returned
        instead of raising `AttributeError`.

        :param name: name of the attribute
        :param data_type: type of the attribute to be read. Defaults to 'str'
        :param optional: when `True`, it doesn't raise an error when the attribute is not found.
            It returns `None` instead. Defaults to `False`
        :raises `AttributeError`: Attribute not found. Only possible if `optional` is `False`
        :raises `TypeError`: Attribute found but it is not of the correct type.
            Only happens in frontend implementations that do type checking
        :raises `ValueError`: Attribute found but can't be coerced to one of the specified types.
            Only happens in frontend implementations that do type coercion.
        """
        raise NotImplementedError()

    def assert_entity_completely_parsed(self):
        """
        Assert that all attributes and children of the entity were parsed.

        This function is automatically called after the `parse(entity, parser)`
        function completed.
        """
        raise NotImplementedError()
