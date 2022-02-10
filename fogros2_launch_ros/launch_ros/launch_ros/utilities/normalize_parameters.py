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

"""Module with utility for normalizing parameters to a node."""

from collections.abc import Mapping
from collections.abc import Sequence
from typing import cast
from typing import List
from typing import Optional
from typing import Sequence as SequenceTypeHint
from typing import Set  # noqa: F401
from typing import Union

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.some_substitutions_type import SomeSubstitutionsType_types_tuple
from launch.substitution import Substitution
from launch.substitutions import TextSubstitution
from launch.utilities import ensure_argument_type
from launch.utilities import normalize_to_list_of_substitutions

import yaml

from ..parameter_descriptions import Parameter as ParameterDescription
from ..parameter_descriptions import ParameterFile
from ..parameter_descriptions import ParameterValue as ParameterValueDescription
from ..parameters_type import Parameters
from ..parameters_type import ParametersDict
from ..parameters_type import ParameterValue
from ..parameters_type import SomeParameters
from ..parameters_type import SomeParametersDict
from ..parameters_type import SomeParameterValue


def _normalize_parameter_array_value(value: SomeParameterValue) -> ParameterValue:
    """Normalize substitutions while preserving the type if it's not a substitution."""
    if not isinstance(value, Sequence):
        raise TypeError('Value {} must be a sequence'.format(repr(value)))

    # Figure out what type the list should be
    has_types = set()  # type: Set[type]
    for subvalue in value:
        allowed_subtypes = (float, int, str, bool) + SomeSubstitutionsType_types_tuple
        ensure_argument_type(subvalue, allowed_subtypes, 'subvalue')

        if isinstance(subvalue, Substitution):
            has_types.add(Substitution)
        elif isinstance(subvalue, str):
            has_types.add(str)
        elif isinstance(subvalue, bool):
            has_types.add(bool)
        elif isinstance(subvalue, int):
            has_types.add(int)
        elif isinstance(subvalue, float):
            has_types.add(float)
        elif isinstance(subvalue, Sequence):
            has_types.add(tuple)
        else:
            raise RuntimeError('Failed to handle type {}'.format(repr(subvalue)))

    if {int} == has_types:
        # everything is an integer
        make_mypy_happy_int = cast(List[int], value)
        return tuple(int(e) for e in make_mypy_happy_int)
    elif has_types in ({float}, {int, float}):
        # all were floats or ints, so return floats
        make_mypy_happy_float = cast(List[Union[int, float]], value)
        return tuple(float(e) for e in make_mypy_happy_float)
    elif Substitution in has_types and has_types.issubset({str, Substitution}):
        # make a list of substitutions forming a single string
        return tuple(normalize_to_list_of_substitutions(cast(SomeSubstitutionsType, value)))
    elif {bool} == has_types:
        # all where bools
        return tuple(bool(e) for e in value)
    else:
        # Should evaluate to a list of strings
        # Normalize to a list of lists of substitutions
        new_value = []  # type: List[SomeSubstitutionsType]
        for element in value:
            if isinstance(element, (float, int, bool, str)):
                new_value.append(yaml.dump(element))
            else:
                new_value.append(element)
        return tuple(normalize_to_list_of_substitutions(e) for e in new_value)


def normalize_parameter_dict(
    parameters: SomeParametersDict, *,
    _prefix: Optional[SequenceTypeHint[Substitution]] = None
) -> ParametersDict:
    """
    Normalize types used to store parameters in a dictionary.

    The parameters are passed as a dictionary that specifies parameter rules.
    Keys of the dictionary can be strings, a Substitution, or an iterable of Substitution.
    A normalized key will be a tuple of substitutions.
    Values in the dictionary can be strings, integers, floats, substitutions, lists of
    the previous types, or another dictionary with the same properties.

    Normalized values that were lists will have all subvalues converted to the same type.
    If all subvalues are int or float, then the normalized subvalues will all be float.
    If the subvalues otherwise do not all have the same type, then the normalized subvalues
    will be lists of Substitution that will result in a list of strings.

    Values that are a list of strings will become a list of strings when normalized and evaluated.
    Values that are a list which has at least one :class:`Substitution` and all other elements
    are either strings or a list of substitutions will become a single list of substitutions that
    will evaluate to a single string.
    To make a list of strings from substitutions, each item in the list must be a list or tuple.

    Normalized values that contained nested dictionaries will be collapsed into a single
    layer with parameter names concatenated with the parameter namespace separator ".".

    :param parameters: Parameters to be normalized
    :param _prefix: internal parameter used for flatening recursive dictionaries
    :return: Normalized parameters
    """
    if not isinstance(parameters, Mapping):
        raise TypeError('expected dict')

    normalized = {}  # type: ParametersDict
    for name, value in parameters.items():
        # First make name a list of substitutions
        name = normalize_to_list_of_substitutions(name)
        if _prefix:
            # Prefix name if there is a recursive dictionary
            # weird looking logic to combine into one list to appease mypy
            combined = list(_prefix)
            combined.append(TextSubstitution(text='.'))
            combined.extend(name)
            name = combined

        # Normalize the value next
        if isinstance(value, Mapping):
            # Flatten recursive dictionaries
            sub_dict = normalize_parameter_dict(value, _prefix=name)
            normalized.update(sub_dict)
        elif isinstance(value, ParameterValueDescription):
            normalized[tuple(name)] = value
        elif isinstance(value, str):
            normalized[tuple(name)] = tuple(normalize_to_list_of_substitutions(yaml.dump(value)))
        elif isinstance(value, Substitution):
            normalized[tuple(name)] = tuple(normalize_to_list_of_substitutions(value))
        elif isinstance(value, (float, bool, int)):
            # Keep some types as is
            normalized[tuple(name)] = value
        elif isinstance(value, bytes):
            # Keep bytes as is
            normalized[tuple(name)] = value
        elif isinstance(value, Sequence):
            # try to make the parameter types uniform
            normalized[tuple(name)] = _normalize_parameter_array_value(value)
        else:
            raise TypeError('Unexpected type for parameter value {}'.format(repr(value)))
    return normalized


def normalize_parameters(parameters: SomeParameters) -> Parameters:
    """
    Normalize the types used to store parameters to substitution types.

    The passed parameters must be an iterable where each element is
    a path to a yaml file or a dict.
    The normalized parameters will have all paths converted to a list of :class:`Substitution`,
    and dictionaries normalized using :meth:`normalize_parameter_dict`.
    """
    if isinstance(parameters, str) or not isinstance(parameters, Sequence):
        raise TypeError('Expecting list of parameters, got {}'.format(parameters))

    normalized_params: List[Union[ParameterFile, ParametersDict, ParameterDescription]] = []
    for param in parameters:
        if isinstance(param, Mapping):
            normalized_params.append(normalize_parameter_dict(param))
        elif isinstance(param, ParameterDescription):
            normalized_params.append(param)
        elif isinstance(param, ParameterFile):
            normalized_params.append(param)
        else:
            # It's a path
            normalized_params.append(ParameterFile(param))
    return tuple(normalized_params)
