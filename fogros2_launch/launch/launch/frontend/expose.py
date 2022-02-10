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

"""Module which adds methods for exposing parsing methods."""

import functools
import inspect
from typing import Iterable
from typing import Optional
from typing import Text

from ..action import Action
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution

if False:
    from .entity import Entity  # noqa: F401
    from .parser import Parser  # noqa: F401

action_parse_methods = {}
substitution_parse_methods = {}


def instantiate_action(entity: 'Entity', parser: 'Parser') -> Action:
    """Call the registered parsing method for the `Entity`."""
    if entity.type_name not in action_parse_methods:
        raise RuntimeError('Unrecognized entity of the type: {}'.format(entity.type_name))
    action_type, kwargs = action_parse_methods[entity.type_name](entity, parser)
    return action_type(**kwargs)


def instantiate_substitution(
    type_name: Text,
    args: Optional[Iterable[SomeSubstitutionsType]] = None
) -> Substitution:
    """Call the registered substitution parsing method, according to `args`."""
    if type_name not in substitution_parse_methods:
        raise RuntimeError(
            'Unknown substitution: {}'.format(type_name))
    args = [] if args is None else args
    subst_type, kwargs = substitution_parse_methods[type_name](args)
    return subst_type(**kwargs)


def __expose_impl(name: Text, parse_methods_map: dict, exposed_type: Text):
    """
    Return a decorator for exposing a parsing method in a dictionary.

    The returned decorator will check the following things in order:
    - If it is decorating a class, it will look for a method called `parse` and store it
        as the parsing method. The `parse` method is supposed to be static. If the class
        doesn't have a `parse` method, it will raise a `RuntimeError`.
    - If it is decorating a callable, it will store it as the parsing method.
    - If not, it will raise a `RuntimeError`.

    If two different parsing methods are exposed using the same `name`, a `RuntimeError`
    will be raised.

    :param name: a string which specifies the key used for storing the parsing
        method in the dictionary.
    :param parse_methods_map: a dict where the parsing method will be stored.
    :param exposed_type: A string specifing the parsing function type.
    """
    # TODO(ivanpauno): Check signature of the registered method/parsing function.
    # TODO(ivanpauno): Infer a parsing function from the constructor annotations.
    #   That should be done in case a method called 'parse' is not found in the decorated class.
    def expose_impl_decorator(exposed):
        found_parse_method = None
        if inspect.isclass(exposed):
            if 'parse' in dir(exposed) and inspect.ismethod(exposed.parse):
                found_parse_method = exposed.parse
            else:
                raise RuntimeError(
                    "Did not find a class method called 'parse' in the class being decorated."
                )
        elif inspect.isfunction(exposed):
            found_parse_method = exposed
        if not found_parse_method:
            raise RuntimeError(
                'Exposed {} parser for {} is not a callable or a class'
                ' containg a parse method'.format(exposed_type, name)
            )
        if name in parse_methods_map and found_parse_method != parse_methods_map[name]:
            raise RuntimeError(
                'Two {} parsing methods exposed with the same name: [{}].'.format(
                    exposed_type,
                    name
                )
            )
        if exposed_type == 'action':
            # For actions, validate that the user didn't provide unknown attributes or children
            @functools.wraps(found_parse_method)
            def wrapper(entity, parser):
                ret = found_parse_method(entity, parser)
                entity.assert_entity_completely_parsed()
                return ret
            parse_methods_map[name] = wrapper
        else:
            parse_methods_map[name] = found_parse_method
        return exposed
    return expose_impl_decorator


def expose_substitution(name: Text):
    """
    Return a decorator for exposing a substitution.

    Read __expose_impl documentation.
    """
    return __expose_impl(name, substitution_parse_methods, 'substitution')


def expose_action(name: Text):
    """
    Return a decorator for exposing an action.

    Read __expose_impl documentation.
    """
    return __expose_impl(name, action_parse_methods, 'action')
