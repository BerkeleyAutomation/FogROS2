# Copyright 2019-2020 Open Source Robotics Foundation, Inc.
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

"""Module that implements type coercion and checking utilities."""

import collections.abc
from typing import Any
from typing import cast
from typing import List
from typing import Optional
from typing import Sequence
from typing import Set
from typing import Text
from typing import Tuple
from typing import Type
from typing import Union

import yaml

from .ensure_argument_type_impl import ensure_argument_type
from .normalize_to_list_of_substitutions_impl import normalize_to_list_of_substitutions
from .perform_substitutions_impl import perform_substitutions
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution

"""Tuple of valid scalar types."""
ScalarTypesTuple = (
    int, float, bool, str
)

"""Tuple of valid list types."""
ListTypesTuple = (
    List[int], List[float], List[bool], List[str]
)

AllowedTypesTuple = ScalarTypesTuple + ListTypesTuple

"""Allowed scalar types."""
ScalarTypesType = Type[Union[int, float, bool, str]]
"""Allowed uniform list types."""
ListTypesType = Type[Union[
    List[int], List[float], List[bool], List[str]
]]
"""Allowed types."""
AllowedTypesType = Union[ScalarTypesType, ListTypesType]

"""Allowed scalar values."""
ScalarValueType = Union[int, float, bool, str]
"""Allowed uniform list values."""
ListValueType = Union[List[int], List[float], List[bool], List[str]]
"""Allowed uniform sequence values."""
SequenceValueType = Union[Sequence[int], Sequence[float], Sequence[bool], Sequence[str]]
"""Allowed values."""
AllowedValueType = Union[ScalarValueType, SequenceValueType]

"""Substitution to a scalar type or a scalar type."""
SomeScalarType = Union[SomeSubstitutionsType, ScalarValueType]
"""Substitution that can be performed to a uniform list."""
SomeSequenceType = Union[(
    Sequence[Union[int, SomeSubstitutionsType]],
    Sequence[Union[float, SomeSubstitutionsType]],
    Sequence[Union[bool, SomeSubstitutionsType]],
    Sequence[Union[str, SomeSubstitutionsType]],
)]
"""Union of SomeScalarType and SomeSequenceType."""
SomeValueType = Union[SomeScalarType, SomeSequenceType]

"""Normalized version of SomeScalarType."""
NormalizedScalarType = Union[List[Substitution], ScalarValueType]
"""Normalized version of SomeSequenceType."""
NormalizedSequenceType = Union[
    List[Union[int, List[Substitution]]],
    List[Union[float, List[Substitution]]],
    List[Union[bool, List[Substitution]]],
    List[Union[str, List[Substitution]]],
]
"""Normalized version of SomeValueType."""
NormalizedValueType = Union[NormalizedScalarType, NormalizedSequenceType]

"""String embedded substitution version of SomeScalarType"""
StrSomeScalarType = SomeScalarType
"""String embedded substitution version of SomeSequenceType"""
StrSomeSequenceType = Union[(
    Sequence[Union[int, str]],
    Sequence[Union[float, str]],
    Sequence[Union[bool, str]],
    Sequence[Union[str]],
)]
"""String embedded substitution version of SomeValueType"""
StrSomeValueType = Union[StrSomeScalarType, StrSomeSequenceType]


def is_typing_list(data_type: Any) -> bool:
    """
    Return `True` if data_type is `typing.List` or a subscription of it.

    Examples
    --------
    ```python3
    assert is_typing_list(typing.List)
    assert is_typing_list(typing.List[int])
    assert not is_typing_list(int)
    ```

    """
    return data_type is List or (
        hasattr(data_type, '__origin__') and
        hasattr(data_type, '__args__') and
        data_type.__origin__ in  # type: ignore
        # This has changed in newer Python implementations to `List`,
        # `list` is checked for compatibility.
        (list, List)
    )


def is_valid_scalar_type(data_type: AllowedTypesType) -> bool:
    """
    Return `True` if `data_type` is an instance of a valid scalar type.

    See :py:data:`ScalarTypesTuple` for further reference.
    """
    return data_type in ScalarTypesTuple


def extract_type(data_type: AllowedTypesType) -> Tuple[ScalarTypesType, bool]:
    """
    Extract type information from type object.

    :param data_type: It can be:
        - a scalar type i.e. `str`, `int`, `float`, `bool`;
        - a uniform list i.e `List[str]`, `List[int]`, `List[float]`, `List[bool]`;

    :returns: a tuple (type_obj, is_list).
        `is_list` is `True` for the supported list types, if not is `False`.
        `type_obj` is the object representing that type in Python. In the case of a list,
        it's the type of the items.
        e.g.:
            `data_type = List[int]` -> `(int, True)`
            `data_type = bool` -> `(bool, False)`
    """
    is_list = False
    scalar_type: ScalarTypesType = cast(ScalarTypesType, data_type)
    if is_typing_list(data_type) and getattr(data_type, '__args__', None):
        is_list = True
        scalar_type = data_type.__args__[0]  # type: ignore
    if is_valid_scalar_type(scalar_type) is False:
        raise ValueError(f'Unrecognized data type: {data_type}')
    return (scalar_type, is_list)


def is_instance_of_valid_type(value: Any, *, can_be_str: bool = False) -> bool:
    """
    Return `True` if value is an instance of an allowed type.

    See :py:data:`AllowedTypesTuple` definition for further reference.

    :param value: variable to be checked.
    :param can_be_str: when True, non-uniform lists mixed with strings are allowed.
    :return: `True` when value is an instance of a valid type, `False` if not.
    """
    if isinstance(value, list):
        if not value:
            return True  # Accept empty lists.
        member_type = next((type(x) for x in value if type(x) is not str), str)
        if member_type not in ScalarTypesTuple:
            return False
        valid_types = (member_type, str) if can_be_str else member_type
        return all(isinstance(x, valid_types) for x in value)
    return isinstance(value, ScalarTypesTuple)


def is_instance_of(
    value: Any,
    data_type: Optional[AllowedTypesType] = None,
    *,
    can_be_str: bool = False,
) -> bool:
    """
    Check if `value` is instance of `data_type`.

    :param value: variable to check.
    :param data_type: type that `value` should be an instance of.
        When `None` the function behaves like :py:func:`is_instance_of_valid_type`.
    :param can_be_str: if `True`, strings will also be accepted.
      :py:mod:`launch.frontend` makes use of this for string embedded substitutions.
    :raise: `ValueError` if `data_type` is invalid.
    :return: `True` if `value` is an instance of `data_type`, else `False`.
    """
    if data_type is None:
        return is_instance_of_valid_type(value, can_be_str=can_be_str)
    type_obj, is_list = extract_type(data_type)
    if can_be_str:
        type_obj = (str, type_obj)
    if not is_list:
        return isinstance(value, type_obj)
    if not isinstance(value, list):
        return False
    return all(isinstance(x, type_obj) for x in value)


def coerce_to_type(
    value: Text,
    data_type: Optional[AllowedTypesType] = None,
    *,
    can_be_str: bool = False,
) -> StrSomeValueType:
    """
    Coerce `value` type to `data_type`.

    :param value: string to be coerced.
    :param data_type: value will be coerced to data_type.
    :param can_be_str: if `True`, the result will be kept as an string if it cannot be coerced.
      In the case of lists, it will also accept strings as items.
      :py:mod:`launch.frontend` makes use of this for string embedded substitutions.
    :raise: `ValueError` if the coercion failed or `data_type` is invalid.
    :raise: `TypeError` if `value` is not a `str`.
    :return: `value` coerced to `data_type`.
    """
    def convert_as_yaml(value, error_msg):
        # Forward empty strings
        if not value:
            return value

        try:
            output = yaml.safe_load(value)
        except Exception as err:
            if can_be_str:
                return value
            raise ValueError(f'{error_msg}: yaml.safe_load() failed\n{err}')

        if not is_instance_of_valid_type(output, can_be_str=can_be_str):
            raise ValueError(
                f'{error_msg}: output type is not allowed, got {type(output)}'
            )
        return output

    ensure_argument_type(value, str, 'value', 'coerce_to_type')
    if data_type is None:
        # No type specified, return best conversion.
        return convert_as_yaml(value, f"Failed to convert '{value}' using yaml rules")

    type_obj, is_list = extract_type(data_type)
    valid_types = (str, type_obj) if can_be_str else type_obj

    if is_list:
        output = convert_as_yaml(
            value, f"Cannot convert value '{value}' to a list of '{valid_types}'")
        if not isinstance(output, list):
            raise ValueError(f"Cannot convert value '{value}' to a list of '{valid_types}'")
        if not all(isinstance(x, valid_types) for x in output):
            raise ValueError(
                f"Cannot convert value '{value}' to a list of '{valid_types}', got {output}")
        return output

    if type_obj is str:
        return value
    if type_obj in (int, float):
        try:
            return type_obj(value)
        except ValueError:
            if can_be_str:
                return value
            else:
                raise

    if type_obj is not bool:
        raise ValueError(
            'data_type is invalid. Expected one of: '
            'int, float, str, bool, List[int], List[float], List[str], List[bool]'
            f'. Got {data_type}')
    output = convert_as_yaml(value, f"Failed to convert '{value}' to '{type_obj}'")
    if isinstance(output, valid_types):
        return output
    raise ValueError(f"Cannot convert value '{value}' to '{type_obj}'")


def coerce_list(
    value: List[Text],
    data_type: Optional[ScalarTypesType] = None,
    *,
    can_be_str: bool = False,
) -> StrSomeSequenceType:
    """
    Coerce a list of strings to a list of scalars.

    :param value: list of strings to be coerced.
    :param data_type: value will be coerced to data_type.
    :param can_be_str: if `True`, strings will be kept in case coercion fails.
      launch.frontend makes use of this for string embedded substitutions.
    :raise: `ValueError` if the coercion failed.
    :raise: `TypeError` if `value` is not a list of `str`.
    :return: `value` coerced to `data_type`.
    """
    ensure_argument_type(value, list, 'value', 'coerce_list')
    output = [coerce_to_type(i, data_type, can_be_str=can_be_str) for i in value]
    if not is_instance_of_valid_type(output, can_be_str=can_be_str):
        raise ValueError(f'cannot convert value to {data_type}. Got value=`{value}`')
    return cast(ListValueType, output)


def get_typed_value(
    value: Union[Text, List[Text]],
    data_type: Optional[AllowedTypesType],
    *,
    can_be_str: bool = False,
) -> StrSomeValueType:
    """
    Try to convert `value` to the type specified in `data_type`.

    :param value: string or list of strings to be coerced.
    :param data_type: value will be coerced to data_type.
    :param can_be_str: if `True`, strings will be kept in case coercion fails.
      In the case of lists, it will also accepts strings as items.
      launch.frontend makes use of this for string embedded substitutions.
    :raise: `ValueError` if the coercion failed.
    :raise: `TypeError` if `value` is a `list` and `data_type` is not a `typing.List[x]` object.
    :raise: `TypeError` if `value` is neither a `str` of a list of `str`.
    :return: `value` coerced to `data_type`.
    """
    if isinstance(value, list):
        if data_type is not None:
            data_type, is_list = extract_type(data_type)
            if not is_list:
                raise TypeError(
                    f"Cannot convert input '{value}' of type '{type(value)}' to"
                    f" '{data_type}'"
                )
        output: AllowedValueType = coerce_list(value, data_type, can_be_str=can_be_str)
    else:
        output = coerce_to_type(value, data_type, can_be_str=can_be_str)
    return output


def is_substitution(x):
    """
    Return `True` if `x` is some substitution.

    This can be:
    - A :py:class:`launch.Substitution` instance.
    - A list of mixed `launch.Substitution` and `str` instances,
        with at least one instance of the former.
    """
    return (
        isinstance(x, Substitution) or
        (
            isinstance(x, collections.abc.Iterable) and
            len(x) > 0 and
            all(isinstance(item, (Substitution, str)) for item in x) and
            any(isinstance(item, Substitution) for item in x)
        )
    )


def normalize_typed_substitution(
    value: SomeValueType, data_type: Optional[AllowedTypesType]
) -> NormalizedValueType:
    """
    Normalize a mix of substitution and values.

    This function becomes handy when you need a substitution whose result will be coerced to a
    specific type.

    Example:
    -------
    ```python3
    class MyAction(Action):
        def __init__(self, my_int: Union[int, SomeSubstitutionsType]):
            self.__my_int_normalized = normalize_typed_substitution(some_int, int)
            ...

        def execute(self, context):
            my_int = perform_typed_substitution(context, self.__my_int_normalized, int)
            ...
    ```

    List of substitutions coerced a list to strings can be confused with a single substitution
    that is coerced to a list of strings.
    e.g.: `['asd', TextSubstitution(text='bsd')]`
    To avoid confusions, the passed value will always be interpreted as a single substitution
    if possible, like in the above example (which will resolved to `'asdbsd'`).
    To make it a list of substitutions:
    `['asd', [TextSubstitution(text='bsd')]]`

    :param value: value to be normalized.
    :param data_type: `value` can be either an instance of `data_type` or a substitution.
        In the case of lists, `value` can be either a substitution or a list.
        In the later case, its items should match the type specified by `data_type` or be a
        substitution.
        If `None`, it should be possible to perform the resulting normalized `value` to a valid
        type.
        See :py:func:`is_substitution`.
    :return: the normalized `value`.
    :raise: `TypeError` if the normalized `value` cannot later be resolved to an instance
        of `data_type`, or to a valid type when `data_type is `None`.
    :raise: `ValueError` if `data_type` is not valid.
        See :py:obj:`AllowedTypesTuple`.

    """
    # Resolve scalar types immediately
    if isinstance(value, ScalarTypesTuple):
        if not is_instance_of(value, data_type):
            raise TypeError(f"value='{value}' is not an instance of {data_type}")
        return value
    # Resolve substitutions and list of substitutions immediately
    if is_substitution(value):
        return normalize_to_list_of_substitutions(cast(SomeSubstitutionsType, value))
    # For the other cases, the input must be an iterable
    if not isinstance(value, collections.abc.Iterable):
        raise TypeError(
            'value should be either a scalar, a substitutions,'
            ' or a mixed list of scalars and substitutions. '
            f'Got `value={value}` of type `{type(value)}`. '
        )
    # Collect the types of the items of the list
    types_in_list: Set[Optional[Type[Union[str, int, float, bool, Substitution]]]] = set()
    for x in value:
        if isinstance(x, ScalarTypesTuple):
            types_in_list.add(type(x))
        elif is_substitution(x):
            types_in_list.add(Substitution)
        else:
            raise TypeError(
                'value is a list, and one of the items is not a scalar, a Substitution '
                f"or a list of substitutions. Got value='{value}'"
            )
    # Extract expected type information
    is_list = True
    if data_type is not None:
        data_type, is_list = extract_type(data_type)
    # Must be expecting a list
    if not is_list:
        raise TypeError(
            'The provided value resolves to a list, though the required type is a scalar. '
            f"Got value='{value}', data_type='{data_type}'."
        )

    # Normalize each specific uniform list input
    err_msg = (
        "Got a list of '{}'"
        f", expected a list of '{data_type}'. value='{value}'"
    )
    if types_in_list == {Substitution}:
        # list of substitutions, can be coerced later to anything
        return cast(List[Union[List[Substitution], str]], [
            normalize_to_list_of_substitutions(cast(SomeSubstitutionsType, x)) for x in value
        ])
    if types_in_list.issubset({str, Substitution}):
        # list of mixed strings and substitutions
        if data_type not in (None, str):
            raise TypeError(err_msg.format(str))
        return cast(List[Union[List[Substitution], str]], [
            x if isinstance(x, str) else  # Don't convert strings to TextSubstitution
            normalize_to_list_of_substitutions(cast(SomeSubstitutionsType, x)) for x in value
        ])
    if types_in_list.issubset({bool, Substitution}):
        # list of booleans and substitutions
        if data_type not in (None, bool):
            raise TypeError(err_msg.format(bool))
        return cast(List[Union[List[Substitution], bool]], [
            normalize_to_list_of_substitutions(cast(SomeSubstitutionsType, x))
            if is_substitution(x) else x for x in value
        ])
    if types_in_list.issubset({int, Substitution}):
        # list of ints and substitutions
        if data_type not in (None, int):
            raise TypeError(err_msg.format(int))
        return cast(List[Union[List[Substitution], int]], [
            normalize_to_list_of_substitutions(cast(SomeSubstitutionsType, x))
            if is_substitution(x) else x for x in value
        ])
    if types_in_list.issubset({int, float, Substitution}):
        # list of floats and substitutions
        if data_type not in (None, float):
            raise TypeError(err_msg.format(float))
        return cast(List[Union[List[Substitution], float]], [
            normalize_to_list_of_substitutions(cast(SomeSubstitutionsType, x))
            if is_substitution(x) else float(cast(Union[int, float], x)) for x in value
        ])
    # Invalid input
    raise TypeError(
        "Input value is not an acceptable 'SomeValueType'."
        f"Got value='{value}'"
    )


def is_normalized_substitution(x):
    """
    Return `True` if `x` is a normalized substitution.

    A normalized substitution is a list with :py:class:`launch.Substitution` instances as items.
    """
    return (
        isinstance(x, list) and
        len(x) > 0 and
        all(isinstance(item, Substitution) for item in x)
    )


def perform_typed_substitution(
    context: LaunchContext,
    value: NormalizedValueType,
    data_type: Optional[AllowedTypesType]
) -> AllowedValueType:
    """
    Perform a normalized typed substitution.

    This function becomes handy when you need a substitution whose result will be coerced to a
    specific type.

    See :py:obj:`normalize_typed_substitution` for an example.

    :param context: A :py:class:`launch.LaunchContext` instance.
        It will be used to perform the substitutions in `value`, if there are any.
    :param value: normalized typed substitution to be performed.
    :param data_type: The return value will satisfy `is_instance_of(output) == True`.
        If after trying to coerce the performed substitution that isn't achieved,
        `ValueError` is raised.
        See :py:func:`is_instance_of`.
    :return: the result of performing all the substitutions in value and coercing the result.
    :raise: `TypeError` if the normalized `value` cannot later be resolved to an instance
        of `data_type`, or to a valid type when `data_type is `None`.
    :raise: `ValueError` if `data_type` is not valid.
        See :py:obj:`AllowedTypesTuple`.
    :raise: `ValueError` if after coercing the substitutions the end result is not of the
        expected type.
    """
    if isinstance(value, ScalarTypesTuple):
        if data_type is not None and not is_instance_of(value, data_type):
            raise TypeError(
                f'value=`{value}` is a scalar and not an instance of `{data_type}`')
        return value
    elif is_normalized_substitution(value):
        return coerce_to_type(
            perform_substitutions(context, cast(List[Substitution], value)),
            data_type
        )
    elif isinstance(value, list):
        scalar_type: Optional[ScalarTypesType] = None
        if data_type is not None:
            scalar_type, is_list = extract_type(data_type)
            if not is_list:
                raise ValueError(
                    'The input value is a list, cannot convert it to a scalar. '
                    f"Got value='{value}', expected type {scalar_type}."
                )
        output = [
            coerce_to_type(
                perform_substitutions(context, cast(List[Substitution], x)), scalar_type)
            if is_normalized_substitution(x) else x for x in value
        ]
        if not is_instance_of(output, data_type):
            raise ValueError(
                    'The output list does not match the expected type '
                    f"Got value='{value}', expected type {scalar_type}."
                    if scalar_type is not None else
                    'The output list is not uniform'
                )
        return cast(ListValueType, output)
    # Invalid input
    raise TypeError(
        "Input value is not an acceptable 'NormalizedValueType'."
        f"Got value='{value}'"
    )
