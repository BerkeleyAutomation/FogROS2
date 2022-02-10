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

"""Test the abstract Parser class."""

from unittest.mock import patch
import warnings

import launch.frontend.parser
from launch.frontend.parser import importlib_metadata
from launch.frontend.parser import Parser

import pytest


class InvalidEntryPoint:

    name = 'RefusesToLoad'

    def load(self):
        raise ValueError('I dont want to load!')


def skip_if_warned_already(warn_text):
    # TODO(sloretz) clear warning registry instead of skipping
    if hasattr(launch.frontend.parser, '__warningregistry__'):
        for key in launch.frontend.parser.__warningregistry__.keys():
            if warn_text in key[0]:
                pytest.skip('Skip because warnings can only be raised once')


def test_invalid_launch_extension():
    skip_if_warned_already('Failed to load the launch')
    with patch(importlib_metadata.__name__ + '.entry_points') as mock_ep:
        mock_ep.return_value = {
            'launch.frontend.launch_extension': [InvalidEntryPoint()]
        }
        with warnings.catch_warnings(record=True) as caught_warnings:
            Parser.load_launch_extensions()
            assert(caught_warnings)
            assert('Failed to load the launch' in str(caught_warnings[0]))


def test_invalid_parser_implementations():
    skip_if_warned_already('Failed to load the parser')
    with patch(importlib_metadata.__name__ + '.entry_points') as mock_ep:
        mock_ep.return_value = {
            'launch.frontend.parser': [InvalidEntryPoint()]
        }

        with warnings.catch_warnings(record=True) as caught_warnings:
            Parser.load_parser_implementations()
            assert(caught_warnings)
            assert('Failed to load the parser' in str(caught_warnings[0]))
