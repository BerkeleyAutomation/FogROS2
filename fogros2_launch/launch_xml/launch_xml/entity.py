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
import xml.etree.ElementTree as ET

from launch.frontend import Entity as BaseEntity
from launch.frontend.type_utils import check_is_list_entity
from launch.utilities.type_utils import AllowedTypesType
from launch.utilities.type_utils import AllowedValueType
from launch.utilities.type_utils import get_typed_value


class Entity(BaseEntity):
    """Single item in the intermediate XML front_end representation."""

    def __init__(
        self,
        xml_element: ET.Element = None,
        *,
        parent: 'Entity' = None
    ) -> Text:
        """Construnctor."""
        self.__xml_element = xml_element
        self.__parent = parent
        self.__read_attributes = set()
        self.__read_children = set()

    @property
    def type_name(self) -> Text:
        """Get Entity type."""
        return self.__xml_element.tag

    @property
    def parent(self) -> Optional['Entity']:
        """Get Entity parent."""
        return self.__parent

    @property
    def children(self) -> List['Entity']:
        """Get the Entity's children."""
        self.__read_children = {item.tag for item in self.__xml_element}
        return [Entity(item) for item in self.__xml_element]

    def assert_entity_completely_parsed(self):
        unparsed_nested_tags = {item.tag for item in self.__xml_element} - self.__read_children
        if unparsed_nested_tags:
            raise ValueError(
                f'Unexpected nested tag(s) found in `{self.__xml_element.tag}`: '
                f'{unparsed_nested_tags}'
            )
        unparsed_attributes = set(self.__xml_element.attrib.keys()) - self.__read_attributes
        if unparsed_attributes:
            raise ValueError(
                f'Unexpected attribute(s) found in `{self.__xml_element.tag}`: '
                f'{unparsed_attributes}'
            )

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

        See :ref:meth:`launch.frontend.Entity.get_attr`.
        `launch_xml` uses type coercion.
        If coercion fails, `ValueError` will be raised.
        """
        attr_error = AttributeError(
            'Attribute {} of type {} not found in Entity {}'.format(
                name, data_type, self.type_name
            )
        )
        if check_is_list_entity(data_type):
            return_list = [x for x in self.__xml_element if x.tag == name]
            if not return_list:
                if optional:
                    return None
                else:
                    raise attr_error
            self.__read_children.add(return_list[0].tag)
            return [Entity(item) for item in return_list]
        value = None
        if name in self.__xml_element.attrib:
            name_sep = name + '-sep'
            if name_sep not in self.__xml_element.attrib:
                value = self.__xml_element.attrib[name]
            else:
                self.__read_attributes.add(name_sep)
                sep = self.__xml_element.attrib[name_sep]
                value = self.__xml_element.attrib[name].split(sep)
            self.__read_attributes.add(name)
        if value is None:
            if not optional:
                raise attr_error
            else:
                return None
        try:
            value = get_typed_value(value, data_type, can_be_str=can_be_str)
        except ValueError:
            raise TypeError(
                'Attribute {} of Entity {} expected to be of type {}.'
                '`{}` can not be converted to one of those types'.format(
                    name, self.type_name, data_type, value
                )
            )
        return value
