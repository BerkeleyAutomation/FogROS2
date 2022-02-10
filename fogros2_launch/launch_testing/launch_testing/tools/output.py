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

import os
import re

from osrf_pycommon.terminal_color import remove_ansi_escape_sequences

from .text import build_line_match
from .text import build_text_match


def get_default_filtered_prefixes():
    return [
        'pid', 'rc',
    ]


def get_default_filtered_patterns():
    return []


def basic_output_filter(
    filtered_prefixes=None,
    filtered_patterns=None,
):
    """
    Create a line filtering function to help output testing.

    :param filtered_prefixes: A list of byte strings representing prefixes that will cause
    output lines to be ignored if they start with one of the prefixes. By default lines
    starting with the process ID (`'pid'`) and return code (`'rc'`) will be ignored.
    :param filtered_patterns: A list of byte strings representing regexes that will cause
    output lines to be ignored if they match one of the regexes.
    """
    if filtered_prefixes is None:
        filtered_prefixes = get_default_filtered_prefixes()
    if filtered_patterns is None:
        filtered_patterns = get_default_filtered_patterns()
    filtered_patterns = list(map(re.compile, filtered_patterns))

    def _filter(output):
        filtered_output_lines = []
        for line in output.splitlines(keepends=True):
            # Filter out stdout that comes from underlying DDS implementation
            # Note: we do not currently support matching filters across multiple stdout lines.
            if any(line.startswith(prefix) for prefix in filtered_prefixes):
                continue
            if any(pattern.match(line) for pattern in filtered_patterns):
                continue
            filtered_output_lines.append(line)
        return ''.join(filtered_output_lines)
    return _filter


def expected_output_from_file(path):
    """
    Get expected output lines from a file.

    :param path: path w/o extension of either a .txt file containing the lines
    to be matched or a .regex file containing patterns to be searched for.
    """
    literal_file = path + '.txt'
    if os.path.isfile(literal_file):
        with open(literal_file, 'r') as f:
            return f.read().splitlines()

    regex_file = path + '.regex'
    if os.path.isfile(regex_file):
        with open(regex_file, 'r') as f:
            return [re.compile(regex) for regex in f.read().splitlines()]

    raise RuntimeError('could not find output check file: {}'.format(path))


def expect_output(
    text=None,
    *,
    lines=None,
    expected_text=None,
    expected_lines=None,
    strip_ansi_escape_sequences=True,
    strict=False
):
    r"""
    Match output text or lines with expected text or lines.

    Either (expected) text or (expected) lines can be provided but giving both results
    in a ValueError.
    If lines are given but a text is expected, these lines are joined using '\n'.
    Likewise, if text is given but lines are expected, text is split into lines.

    :param expected_text: output text expectation, as supported
        by `launch_testing.tools.text.build_text_match`
    :param expected_lines: output lines expectation, as supported
        by `launch_testing.tools.text.build_line_match`
    :param text: output text to be matched
    :param lines: output text lines to be matched
    :param strip_ansi_escape_sequences: If True (default), strip
        ansi escape sequences from actual output before comparing
    """
    if (text is not None) == (lines is not None):
        raise ValueError('Either lines or text, but not both, must be specified')

    if (expected_text is not None) == (expected_lines is not None):
        raise ValueError('Either expected lines or text, but not both, must be specified')

    if expected_text is not None:
        if text is None:
            text = '\n'.join(lines)
        match = build_text_match(expected_text, strict=strict)
        if strip_ansi_escape_sequences:
            text = remove_ansi_escape_sequences(text)
        return match(text) is not None

    match = build_line_match(expected_lines, strict=strict)
    if lines is None:
        lines = text.splitlines()
    if strip_ansi_escape_sequences:
        lines = [remove_ansi_escape_sequences(line) for line in lines]
    return match(lines) is not None
