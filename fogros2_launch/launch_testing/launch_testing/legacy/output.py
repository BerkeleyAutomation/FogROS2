# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import io
import os
import re

import ament_index_python


def get_default_filtered_prefixes():
    return [
        b'pid', b'rc',
    ]


def get_default_filtered_patterns():
    return []


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
    return [str.encode(line) for line in rmw_output_filter.splitlines()]


def create_output_lines_filter(
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
    filtered_prefixes = filtered_prefixes or get_default_filtered_prefixes()
    filtered_patterns = filtered_patterns or get_default_filtered_patterns()
    if filtered_rmw_implementation:
        filtered_prefixes.extend(get_rmw_output_filter(
            filtered_rmw_implementation, 'prefixes'
        ))
        filtered_patterns.extend(get_rmw_output_filter(
            filtered_rmw_implementation, 'patterns'
        ))
    filtered_patterns = map(re.compile, filtered_patterns)
    encoded_line_sep = os.linesep.encode('ascii')

    def _filter(output):
        filtered_output = []
        for line in output.splitlines():
            # Filter out stdout that comes from underlying DDS implementation
            # Note: we do not currently support matching filters across multiple stdout lines.
            if any(line.startswith(prefix) for prefix in filtered_prefixes):
                continue
            if any(pattern.match(line) for pattern in filtered_patterns):
                continue
            filtered_output.append(line)
        if output.endswith(encoded_line_sep):
            filtered_output.append(encoded_line_sep)
        return encoded_line_sep.join(filtered_output)
    return _filter


def create_output_lines_test(expected_lines):
    """Create output test given a list of expected lines."""
    def _collate(output, addendum):
        output.write(addendum)
        return output

    def _match(output, pattern):
        return any(pattern in line for line in output.getvalue().splitlines())

    return io.BytesIO(), _collate, _match, expected_lines


def create_output_regex_test(expected_patterns):
    """Create output test given a list of expected matching regular expressions."""
    def _collate(output, addendum):
        output.write(b'\n'.join(addendum.splitlines()))
        return output

    def _match(output, pattern):
        return pattern.search(output.getvalue()) is not None

    return io.BytesIO(), _collate, _match, expected_patterns


def create_output_test_from_file(output_file):
    """
    Create output test using the given file content.

    :param output_file: basename (i.e. w/o extension) of either a .txt file containing the
    lines to be matched or a .regex file containing patterns to be searched for.
    """
    literal_file = output_file + '.txt'
    if os.path.isfile(literal_file):
        with open(literal_file, 'rb') as f:
            expected_output = f.read().splitlines()
        return create_output_lines_test(expected_output)

    regex_file = output_file + '.regex'
    if os.path.isfile(regex_file):
        with open(regex_file, 'rb') as f:
            patterns = [re.compile(regex) for regex in f.read().splitlines()]
        return create_output_regex_test(patterns)

    raise RuntimeError('could not find output check file: {}'.format(output_file))
