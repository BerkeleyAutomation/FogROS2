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

"""Module with utility for evaluating parameters in a launch context."""


from collections.abc import Mapping
from collections.abc import Sequence
import pathlib
from typing import cast
from typing import Dict
from typing import List
from typing import Optional
from typing import Union

from launch.launch_context import LaunchContext
from launch.substitution import Substitution
from launch.utilities import ensure_argument_type
from launch.utilities import perform_substitutions

import yaml

from ..parameter_descriptions import Parameter as ParameterDescription
from ..parameter_descriptions import ParameterFile
from ..parameter_descriptions import ParameterValue as ParameterValueDescription
from ..parameters_type import EvaluatedParameters
from ..parameters_type import EvaluatedParameterValue
from ..parameters_type import Parameters
from ..parameters_type import ParametersDict


def evaluate_parameter_dict(
    context: LaunchContext,
    parameters: ParametersDict
) -> Dict[str, EvaluatedParameterValue]:
    def check_sequence_type_is_allowed(sequence):
        # Check if the items of the sequence aren't dissimilar.
        # Also, check if the item type is one of bool, int, float, str.
        if not sequence:  # Don't allow empty sequences
            return False
        subtype = type(sequence[0])
        if subtype not in (bool, int, float, str):
            return False
        for item in sequence:
            if not isinstance(item, subtype):
                return False
        return True

    if not isinstance(parameters, Mapping):
        raise TypeError('expected dict')
    output_dict: Dict[str, EvaluatedParameterValue] = {}
    for name, value in parameters.items():
        if not isinstance(name, tuple):
            raise TypeError('Expecting tuple of substitutions got {}'.format(repr(name)))
        evaluated_name: str = perform_substitutions(context, list(name))
        evaluated_value: Optional[EvaluatedParameterValue] = None

        if isinstance(value, tuple) and len(value):
            if isinstance(value[0], Substitution):
                # Value is a list of substitutions, so perform them to make a string
                evaluated_value = perform_substitutions(context, list(value))

                try:
                    yaml_evaluated_value = yaml.safe_load(evaluated_value)
                except yaml.YAMLError:
                    raise TypeError(
                        'Unable to parse the value of parameter {} as yaml. '
                        'If the parameter is meant to be a string, try wrapping it in '
                        'launch_ros.parameter_descriptions.ParameterValue'
                        '(value, value_type=str)'.format(evaluated_name)
                    )

                if type(yaml_evaluated_value) in (bool, int, float, str, bytes):
                    evaluated_value = yaml_evaluated_value
                elif isinstance(yaml_evaluated_value, Sequence):
                    # str and bytes were already handled in the previous case
                    # If it is a list with dissimilar types, don't evaluate the value as yaml.
                    if not check_sequence_type_is_allowed(yaml_evaluated_value):
                        raise TypeError(
                            'Expected a non-empty sequence, with items of uniform type. '
                            'Allowed sequence item types are bool, int, float, str.'
                        )
                    evaluated_value = tuple(yaml_evaluated_value)
                else:
                    raise TypeError(
                        'Allowed value types are bytes, bool, int, float, str, Sequence[bool]'
                        ', Sequence[int], Sequence[float], Sequence[str]. Got {}.'
                        'If the parameter is meant to be a string, try wrapping it in '
                        'launch_ros.parameter_descriptions.ParameterValue'
                        '(value, value_type=str)'.format(type(yaml_evaluated_value))
                    )
            elif isinstance(value[0], Sequence):
                # Value is an array of a list of substitutions
                output_subvalue: List[str] = []
                for subvalue in value:
                    value = perform_substitutions(context, list(subvalue))
                    output_subvalue.append(value)
                evaluated_value = tuple(output_subvalue)
                # All values in a list must have the same type.
                # If they don't then assume it is a list of strings
                yaml_evaluated_value = [
                    yaml.safe_load(item) for item in evaluated_value
                ]
                if not check_sequence_type_is_allowed(yaml_evaluated_value):
                    raise TypeError(
                        'Expected a non-empty sequence, with items of uniform type. '
                        'Allowed sequence item types are bool, int, float, str.'
                    )
                evaluated_value = tuple(yaml_evaluated_value)
            else:
                # Value is an array of the same type, so nothing to evaluate.
                output_value = []
                target_type = type(value[0])
                for i, subvalue in enumerate(value):
                    output_value.append(target_type(subvalue))
                    evaluated_value = tuple(output_value)
        elif isinstance(value, ParameterValueDescription):
            evaluated_value = value.evaluate(context)
        else:
            # Value is a singular type, so nothing to evaluate
            ensure_argument_type(value, (float, int, str, bool, bytes), 'value')
            evaluated_value = cast(Union[float, int, str, bool, bytes], value)
        if evaluated_value is None:
            raise TypeError('given unnormalized parameters %r, %r' % (name, value))
        output_dict[evaluated_name] = evaluated_value
    return output_dict


def evaluate_parameters(context: LaunchContext, parameters: Parameters) -> EvaluatedParameters:
    """
    Evaluate substitutions to produce paths and name/value pairs.

    The parameters must have been normalized with normalize_parameters() prior to calling this.
    Substitutions for parameter values in dictionaries will be evaluated according to yaml rules.
    If you want the substitution to stay a string, the output of the substition must have quotes.

    :param parameters: normalized parameters
    :returns: values after evaluating lists of substitutions
    """
    output_params = []  # type: List[Union[pathlib.Path, Dict[str, EvaluatedParameterValue]]]
    for param in parameters:
        if isinstance(param, ParameterFile):
            # Evaluate a list of Substitution to a file path
            output_params.append(param.evaluate(context))
        elif isinstance(param, ParameterDescription):
            output_params.append(param)
        elif isinstance(param, Mapping):
            # It's a list of name/value pairs
            output_params.append(evaluate_parameter_dict(context, param))
    return tuple(output_params)
