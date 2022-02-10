# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Module for a description of a Parameter."""

import os
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import List
from typing import Optional
from typing import Text
from typing import Tuple
from typing import TYPE_CHECKING
from typing import Union

from launch import LaunchContext
from launch import SomeSubstitutionsType
from launch import SomeSubstitutionsType_types_tuple
from launch.frontend.parse_substitution import parse_substitution
from launch.substitution import Substitution
from launch.substitutions import SubstitutionFailure
from launch.utilities import ensure_argument_type
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch.utilities.type_utils import AllowedTypesType
from launch.utilities.type_utils import normalize_typed_substitution
from launch.utilities.type_utils import perform_typed_substitution
from launch.utilities.type_utils import SomeValueType
from launch.utilities.typing_file_path import FilePath

import yaml

if TYPE_CHECKING:
    from .parameters_type import EvaluatedParameterValue


class ParameterValue:
    """Describes a ROS parameter value."""

    def __init__(
        self,
        value: SomeValueType,
        *,
        value_type: Optional[AllowedTypesType] = None
    ) -> None:
        """
        Construct a parameter value description.

        :param value: Value or substitution that can be resolved to a value.
        :param value_type: Used when `value` is a substitution, to coerce the result.
            Can be one of:
                - Scalar types: int, bool, float, str
                - List types: List[int], List[bool], List[float], List[str]
                - None: will use yaml rules, and check that the result matches one of the above.
        """
        self.__value = normalize_typed_substitution(value, value_type)
        self.__value_type = value_type
        self.__evaluated_parameter_value: Optional['EvaluatedParameterValue'] = None

    @property
    def value(self) -> SomeValueType:
        """Getter for parameter value."""
        if self.__evaluated_parameter_value is not None:
            return self.__evaluated_parameter_value
        return self.__value

    @property
    def value_type(self) -> AllowedTypesType:
        """Getter for parameter value type."""
        return self.__value_type

    def __str__(self) -> Text:
        return (
            'launch_ros.description.ParameterValue'
            f'(value={self.value}, value_type={self.value_type})'
        )

    def evaluate(self, context: LaunchContext) -> 'EvaluatedParameterValue':
        """Evaluate and return parameter rule."""
        self.__evaluated_parameter_value = perform_typed_substitution(
            context, self.value, self.value_type)
        return self.__evaluated_parameter_value


class Parameter:
    """Describes a ROS parameter."""

    def __init__(
        self,
        name: SomeSubstitutionsType,
        value: SomeValueType,
        *,
        value_type: Optional[AllowedTypesType] = None
    ) -> None:
        """
        Construct a parameter description.

        :param name: Name of the parameter.
        :param value: Value of the parameter.
        :param value_type: Used when `value` is a substitution, to coerce the result.
            Can be one of:
                - A scalar type: `int`, `str`, `float`, `bool`.
                  `bool` are written like in `yaml`.
                  Both `1` and `1.` are valid floats.
                - An uniform list: `List[int]`, `List[str]`, `List[float]`, `List[bool]`.
                  Lists are written like in `yaml`.
                - `None`, which means that yaml rules will be used.
                  The result of the convertion must be one of the above types,
                  if not `ValueError` is raised.
            If value is not a substitution and this parameter is provided,
            it will be used to check `value` type.
        """
        ensure_argument_type(name, SomeSubstitutionsType_types_tuple, 'name')

        self.__name = normalize_to_list_of_substitutions(name)
        self.__parameter_value = ParameterValue(value, value_type=value_type)
        self.__evaluated_parameter_name: Optional[Text] = None
        self.__evaluated_parameter_rule: Optional[Tuple[Text, 'EvaluatedParameterValue']] = None

    @property
    def name(self) -> Union[List[Substitution], Text]:
        """Getter for parameter name."""
        if self.__evaluated_parameter_name is not None:
            return self.__evaluated_parameter_name
        return self.__name

    @property
    def value(self) -> SomeValueType:
        """Getter for parameter value."""
        return self.__parameter_value.value

    @property
    def value_type(self) -> AllowedTypesType:
        """Getter for parameter value type."""
        return self.__parameter_value.value_type

    def __str__(self) -> Text:
        return (
            'launch_ros.description.Parameter'
            f'(name={self.name}, value={self.value}, value_type={self.value_type})'
        )

    def evaluate(self, context: LaunchContext) -> Tuple[Text, 'EvaluatedParameterValue']:
        """Evaluate and return parameter rule."""
        if self.__evaluated_parameter_rule is not None:
            return self.__evaluated_parameter_rule

        name = perform_substitutions(context, self.name)
        value = self.__parameter_value.evaluate(context)

        self.__evaluated_parameter_name = name
        self.__evaluated_parameter_rule = (name, value)
        return (name, value)


class ParameterFile:
    """Describes a ROS parameter file."""

    def __init__(
        self,
        param_file: Union[FilePath, SomeSubstitutionsType],
        *,
        allow_substs: [bool, SomeSubstitutionsType] = False
    ) -> None:
        """
        Construct a parameter file description.

        :param param_file: Path to a parameter file.
        :param allow_subst: Allow substitutions in the parameter file.
        """
        # In Python, __del__ is called even if the constructor throws an
        # exception.  It is possible for ensure_argument_type() below to
        # throw an exception and try to access these member variables
        # during cleanup, so make sure to initialize them here.
        self.__evaluated_param_file: Optional[Path] = None
        self.__created_tmp_file = False

        ensure_argument_type(
            param_file,
            SomeSubstitutionsType_types_tuple + (os.PathLike, bytes),
            'param_file',
            'ParameterFile()'
        )
        ensure_argument_type(
            allow_substs,
            bool,
            'allow_subst',
            'ParameterFile()'
        )
        self.__param_file: Union[List[Substitution], FilePath] = param_file
        if isinstance(param_file, SomeSubstitutionsType_types_tuple):
            self.__param_file = normalize_to_list_of_substitutions(param_file)
        self.__allow_substs = normalize_typed_substitution(allow_substs, data_type=bool)
        self.__evaluated_allow_substs: Optional[bool] = None

    @property
    def param_file(self) -> Union[FilePath, List[Substitution]]:
        """Getter for parameter file."""
        if self.__evaluated_param_file is not None:
            return self.__evaluated_param_file
        return self.__param_file

    @property
    def allow_substs(self) -> Union[bool, List[Substitution]]:
        """Getter for allow substitutions argument."""
        if self.__evaluated_allow_substs is not None:
            return self.__evaluated_allow_substs
        return self.__allow_substs

    def __str__(self) -> Text:
        return (
            'launch_ros.description.ParameterFile'
            f'(param_file={self.param_file}, allow_substs={self.allow_substs})'
        )

    def evaluate(self, context: LaunchContext) -> Path:
        """Evaluate and return a parameter file path."""
        if self.__evaluated_param_file is not None:
            return self.__evaluated_param_file

        param_file = self.__param_file
        if isinstance(param_file, list):
            # list of substitutions
            param_file = perform_substitutions(context, self.__param_file)

        allow_substs = perform_typed_substitution(context, self.__allow_substs, data_type=bool)
        param_file_path: Path = Path(param_file)
        if allow_substs:
            with open(param_file_path, 'r') as f, NamedTemporaryFile(
                mode='w', prefix='launch_params_', delete=False
            ) as h:
                parsed = perform_substitutions(context, parse_substitution(f.read()))
                try:
                    yaml.safe_load(parsed)
                except Exception:
                    raise SubstitutionFailure(
                        'The substituted parameter file is not a valid yaml file')
                h.write(parsed)
                param_file_path = Path(h.name)
                self.__created_tmp_file = True
        self.__evaluated_param_file = param_file_path
        return param_file_path

    def cleanup(self):
        """Delete created temporary files."""
        if self.__created_tmp_file and self.__evaluated_param_file is not None:
            try:
                os.unlink(self.__evaluated_param_file)
            except FileNotFoundError:
                pass
            self.__evaluated_param_file = None

    def __del__(self):
        self.cleanup()
