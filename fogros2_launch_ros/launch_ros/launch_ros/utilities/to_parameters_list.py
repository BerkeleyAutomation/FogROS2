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

"""Module with utility to transform evaluated parameters into parameter lists."""

import pathlib
from typing import List
import warnings

from launch.launch_context import LaunchContext
import rclpy.parameter

import yaml

from .evaluate_parameters import evaluate_parameter_dict
from .normalize_parameters import normalize_parameter_dict

from ..parameters_type import EvaluatedParameters


def __normalize_parameters_dict(dictionary):
    """
    Combine namespaces and their node names into one key and remove the "ros__parameters" key.

    Return an empty dict if the "ros__parameters" key is not found.
    """
    # Recursive function
    def normalize_parameters_dict(dictionary, keys, result_dict):
        for key, value in dictionary.items():
            # Base case: found 'ros__parameters' key
            # Return
            if key == 'ros__parameters':
                node_name = '/'.join(keys)
                result_dict[node_name] = value
                return result_dict

            if isinstance(value, dict):
                keys.append(key.lstrip('/'))
                result_dict = normalize_parameters_dict(value, keys, result_dict)
                # Reset keys in case there are multiple ros__parameter entries
                keys = []

        return result_dict

    return normalize_parameters_dict(dictionary, [], {})


def to_parameters_list(
    context: LaunchContext,
    node_name: str,
    namespace: str,
    evaluated_parameters: EvaluatedParameters
) -> List[rclpy.parameter.Parameter]:
    """
    Transform evaluated parameters into a list of rclpy.parameter.Parameter objects.

    :param context: to carry out substitutions during normalization of parameter files.
        See `normalize_parameters()` documentation for further reference.
    :param node_name: node name
    :param namespace: node namespace
    :param parameters: parameters as either paths to parameter files or name/value pairs.
        See `evaluate_parameters()` documentation for further reference.
    :returns: a list of parameters
    """
    parameters = []  # type: List[rclpy.parameter.Parameter]
    node_name = node_name.lstrip('/')
    if namespace:
        namespace = namespace.strip('/')
        node_name = f'{namespace}/{node_name}'

    params_set = {}
    warned_once = False
    for params_set_or_path in evaluated_parameters:
        if isinstance(params_set_or_path, pathlib.Path):
            with open(str(params_set_or_path), 'r') as f:
                param_dict = yaml.safe_load(f)
                normalized_param_dict = __normalize_parameters_dict(param_dict)

                if normalized_param_dict:
                    param_dict.clear()
                    if '**' in normalized_param_dict:
                        param_dict = normalized_param_dict['**']
                    if node_name in normalized_param_dict:
                        param_dict.update(normalized_param_dict[node_name])
                    if not warned_once and not node_name:
                        warnings.warn(
                            'node name not provided to launch; parameter files will not apply',
                            UserWarning
                        )
                        warned_once = True

                if param_dict:
                    params_set = evaluate_parameter_dict(
                        context, normalize_parameter_dict(param_dict)
                    )
        else:
            params_set = params_set_or_path
        if not isinstance(params_set, dict):
            raise RuntimeError('invalid evaluated parameters {}'.format(repr(params_set)))
        for name, value in params_set.items():
            type_ = next((
                type_ for type_ in rclpy.parameter.Parameter.Type if type_.check(value)
            ), None)  # type: rclpy.parameter.Parameter.Type
            if type_ is None:
                raise RuntimeError('invalid parameter value {}'.format(repr(value)))
            parameters.append(rclpy.parameter.Parameter(name, type_, value))
    return parameters
