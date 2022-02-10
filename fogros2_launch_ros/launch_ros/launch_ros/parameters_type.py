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

"""Module for ROS parameter types."""

import pathlib

from typing import Any
from typing import Dict
from typing import Mapping
from typing import Sequence
from typing import Union

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.some_substitutions_type import SomeSubstitutionsType_types_tuple
from launch.substitution import Substitution

from .parameter_descriptions import Parameter as ParameterDescription
from .parameter_descriptions import ParameterFile
from .parameter_descriptions import ParameterValue as ParameterValueDescription


# Parameter value types used below
_SingleValueType_types_tuple = (str, int, float, bool)
_SingleValueType = Union[str, int, float, bool]
_MultiValueType = Union[
    Sequence[str], Sequence[int], Sequence[float], Sequence[bool], bytes]

SomeParameterFile = Union[SomeSubstitutionsType, pathlib.Path, ParameterFile]
SomeParameterName = Sequence[Union[Substitution, str]]
SomeParameterValue = Union[
    ParameterValueDescription,
    SomeSubstitutionsType,
    Sequence[SomeSubstitutionsType],
    _SingleValueType,
    _MultiValueType
]
SomeParameterValue_types_tuple = (
    SomeSubstitutionsType_types_tuple +
    _SingleValueType_types_tuple +
    (bytes,)
)

# TODO(sloretz) Recursive type when mypy supports them python/mypy#731
_SomeParametersDict = Mapping[SomeParameterName, Any]
SomeParametersDict = Mapping[SomeParameterName, Union[SomeParameterValue, _SomeParametersDict]]

# Paths to yaml files with parameters, or dictionaries of parameters, or pairs of
# parameter names and values
SomeParameters = Sequence[Union[SomeParameterFile, ParameterDescription, SomeParametersDict]]

ParameterFile = ParameterFile  # re-export
ParameterName = Sequence[Substitution]
ParameterValue = Union[
    Sequence[Substitution],
    Sequence[Sequence[Substitution]],
    _SingleValueType,
    _MultiValueType,
    ParameterValueDescription]

# Normalized (flattened to avoid having a recursive type) parameter dict
ParametersDict = Dict[ParameterName, ParameterValue]

# Normalized parameters
Parameters = Sequence[Union[ParameterFile, ParametersDict, ParameterDescription]]

EvaluatedParameterValue = Union[_SingleValueType, _MultiValueType]
# Evaluated parameters: filenames or dictionary after substitutions have been evaluated
EvaluatedParameters = Sequence[
    Union[pathlib.Path, ParameterDescription, Dict[str, EvaluatedParameterValue]]
]
