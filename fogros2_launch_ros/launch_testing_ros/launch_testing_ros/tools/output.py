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

import ament_index_python


def get_rmw_output_filter(rmw_implementation, filter_type):
    supported_filter_types = ['prefixes', 'patterns']
    if filter_type not in supported_filter_types:
        raise TypeError(
            'Unsupported filter_type "{0}". Supported types: {1}'
            .format(filter_type, supported_filter_types))
    resource_name = 'rmw_output_' + filter_type
    prefix_with_resource = ament_index_python.has_resource(
        resource_name, rmw_implementation)
    if not prefix_with_resource:
        return []

    # Treat each line of the resource as an independent filter.
    rmw_output_filter, _ = ament_index_python.get_resource(resource_name, rmw_implementation)
    return rmw_output_filter.splitlines()


def basic_output_filter(
    filtered_prefixes=None,
    filtered_patterns=None,
    filtered_rmw_implementation=None
):
    """
    Create a line filtering function to help output testing.

    :param filtered_prefixes: A list of byte strings representing prefixes that will cause
    output lines to be ignored if they start with one of the prefixes. By default lines
    starting with the process ID (`b'pid'`) and return code (`b'rc'`) will be ignored.
    :param filtered_patterns: A list of byte strings representing regexes that will cause
    output lines to be ignored if they match one of the regexes.
    :param filtered_rmw_implementation: RMW implementation for which the output will be
    ignored in addition to the `filtered_prefixes`/`filtered_patterns`.
    """
    from launch_testing.tools.output import basic_output_filter as base_output_filter
    from launch_testing.tools.output import get_default_filtered_prefixes
    from launch_testing.tools.output import get_default_filtered_patterns
    if filtered_prefixes is None:
        filtered_prefixes = get_default_filtered_prefixes()
    if filtered_patterns is None:
        filtered_patterns = get_default_filtered_patterns()
    if filtered_rmw_implementation:
        filtered_prefixes = filtered_prefixes + get_rmw_output_filter(
            filtered_rmw_implementation, 'prefixes'
        )
        filtered_patterns = filtered_patterns + get_rmw_output_filter(
            filtered_rmw_implementation, 'patterns'
        )
    return base_output_filter(filtered_prefixes, filtered_patterns)
