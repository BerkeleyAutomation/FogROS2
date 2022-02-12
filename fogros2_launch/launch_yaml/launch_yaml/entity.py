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

"""Module for YAML Entity class."""

from typing import List
from typing import Optional
from typing import Text
from typing import Union

from launch.frontend import Entity as BaseEntity
from launch.frontend.type_utils import check_is_list_entity
from launch.utilities.type_utils import AllowedTypesType
from launch.utilities.type_utils import AllowedValueType
from launch.utilities.type_utils import is_instance_of


class Entity(BaseEntity):
    """Single item in the intermediate YAML front_end representation."""

    def __init__(
        self,
        element: dict,
        type_name: Text = None,
        *,
        parent: 'Entity' = None
    ) -> Text:
        """Create an Entity."""
        self.__type_name = type_name
        self.__element = element
        self.__parent = parent
        self.__read_keys = set()
        self.__children_called = False

    @property
    def type_name(self) -> Text:
        """Get Entity type."""
        return self.__type_name

    @property
    def parent(self) -> Optional['Entity']:
        """Get Entity parent."""
        return self.__parent

    @property
    def children(self) -> List['Entity']:
        """Get the Entity's children."""
        self.__children_called = True
        if not isinstance(self.__element, (dict, list)):
            raise TypeError(
                f'Expected a dict or list, got {type(self.element)}:'
                f'\n---\n{self.__element}\n---'
            )
        if isinstance(self.__element, dict):
            if 'children' not in self.__element:
                raise ValueError(
                    f'Expected entity `{self.__type_name}` to have children entities.'
                    f'That can be a list of subentities or a dictionary with a `children` '
                    'list element')
            self.__read_keys.add('children')
            children = self.__element['children']
        else:
            children = self.__element
        entities = []
        for child in children:
            if len(child) != 1:
                raise RuntimeError(
                    'Subentities must be a dictionary with only one key'
                    ', which is the entity type')
            type_name = list(child.keys())[0]
            entities.append(Entity(child[type_name], type_name))
        return entities

    def assert_entity_completely_parsed(self):
        if isinstance(self.__element, list):
            if not self.__children_called:
                raise ValueError(
                    f'Unexpected nested entity(ies) found in `{self.__type_name}`: '
                    f'{self.__element}')
            return
        unparsed_keys = set(self.__element.keys()) - self.__read_keys
        if unparsed_keys:
            raise ValueError(
                f'Unexpected key(s) found in `{self.__type_name}`: {unparsed_keys}'
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
        `launch_yaml` does not apply type coercion,
        it only checks if the read value is of the correct type.
        """
        if name not in self.__element:
            if not optional:
                raise AttributeError(
                    'Can not find attribute {} in Entity {}'.format(
                        name, self.type_name))
            else:
                return None
        self.__read_keys.add(name)
        data = self.__element[name]
        if check_is_list_entity(data_type):
            if isinstance(data, list) and isinstance(data[0], dict):
                return [Entity(child, name) for child in data]
            raise TypeError(
                'Attribute {} of Entity {} expected to be a list of dictionaries.'.format(
                    name, self.type_name
                )
            )
        if not is_instance_of(data, data_type, can_be_str=can_be_str):
            raise TypeError(
                'Attribute {} of Entity {} expected to be of type {}, got {}'.format(
                    name, self.type_name, data_type, type(data)
                )
            )
        return data
