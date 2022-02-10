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

from collections.abc import Iterable

import os


def normalize_lineseps(lines):
    r"""Normalize and then return the given lines to all use '\n'."""
    lines = lines.replace(os.linesep, '\n')
    # This happens (even on Linux and macOS) when capturing I/O from an
    # emulated tty.
    lines = lines.replace('\r\n', '\n')
    return lines


def build_line_match(expected_lines, *, strict=False):
    """
    Make a callable to match lines with ``expected_lines``.

    :param expected_lines: line expectation to match against
    :type expected_text: str or List[str] or List[Union[str, regex pattern]]
    :return: a callable that matches text against the expectation.
    :rtype: Callable[bool, [str]]
    """
    if isinstance(expected_lines, str):
        def _match(actual_lines, start=0):
            if strict:
                if all(expected_lines == line for line in actual_lines[start:]):
                    return start, start + len(actual_lines[start:])
                return None
            return next((
                (i, i + 1) for i, line in enumerate(
                    actual_lines[start:], start=start
                ) if expected_lines in line
            ), None)
        return _match

    if hasattr(expected_lines, 'match'):
        def _match(actual_lines, start):
            if strict:
                if all(expected_lines.match(line) for line in actual_lines[start:]):
                    return start, start + len(actual_lines[start:])
                return None
            return next((
                (i, i + 1) for i, line in enumerate(
                    actual_lines[start:], start=start
                ) if expected_lines.match(line)
            ), None)
        return _match

    if isinstance(expected_lines, Iterable):
        head_match, *tail_matches = [
            build_line_match(line, strict=False) for line in expected_lines
        ]

        def _match(actual_lines, start=0):
            next_start, end = head_match(actual_lines, start) or (-1, -1)
            if next_start < start or (strict and next_start != start):
                return None
            start = next_start
            for match in tail_matches:
                next_start, next_end = match(actual_lines, end) or (-1, -1)
                if next_start < end or (strict and next_start != end):
                    return None
                end = next_end
            return start, end
        return _match

    raise ValueError('Unknown format for expected lines')


def build_text_match(expected_text, *, strict=False):
    """
    Make a callable to match text with ``expected_text``.

    :param expected_text: text expectation to match against
    :type expected_text: str or regex pattern or List[Union[str, regex pattern]]
    :return: a callable that matches text against the expectation.
    :rtype: Callable[bool, [str]]
    """
    if isinstance(expected_text, str):
        def _match(actual_text, start=0):
            actual_text = normalize_lineseps(actual_text)
            if strict:
                if actual_text[start:] != expected_text:
                    return None
            else:
                start = actual_text.find(expected_text, start)
                if start < 0:
                    return None
            return start, start + len(expected_text)
        return _match

    if hasattr(expected_text, 'search') and hasattr(expected_text, 'match'):
        def _match(actual_text, start=0):
            actual_text = normalize_lineseps(actual_text)
            if strict:
                match = expected_text.match(actual_text, start)
            else:
                match = expected_text.search(actual_text, start)
            if match is not None:
                match = match.start(), match.end()
            return match
        return _match

    if isinstance(expected_text, Iterable):
        head_match, *tail_matches = [
            build_text_match(text, strict=False) for text in expected_text
        ]

        def _match(actual_text, start=0):
            actual_text = normalize_lineseps(actual_text)
            next_start, end = head_match(actual_text, start) or (-1, -1)
            if next_start < start or (strict and next_start != start):
                return None
            start = next_start
            for match in tail_matches:
                next_start, next_end = match(actual_text, end) or (-1, -1)
                if next_start < end or (strict and next_start != end):
                    return None
                end = next_end
            return start, end
        return _match

    raise ValueError('Unknown format for expected text')
