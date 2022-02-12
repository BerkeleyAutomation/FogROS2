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

"""Tests for the perform_substitutions() function."""

from launch import LaunchContext, Substitution
from launch.utilities import perform_substitutions

import pytest


def test_valid_substitutions():
    """Test the perform_substitutions() function with valid input."""
    context = LaunchContext()

    class MockSubstitution(Substitution):

        def perform(self, context):
            return 'Mock substitution'

    mock_sub = MockSubstitution()
    sub_mock_sub = perform_substitutions(context, [mock_sub])
    assert 'Mock substitution' == sub_mock_sub
    sub_mock_sub_multi = perform_substitutions(context, [mock_sub, mock_sub, mock_sub])
    assert 'Mock substitutionMock substitutionMock substitution' == sub_mock_sub_multi


def test_invalid_substitutions():
    """Test the perform_substitutions() function with invalid input."""
    context = LaunchContext()

    class MockSubstitution(Substitution):

        def perform(self, context):
            return 'Mock substitution'

    mock_sub = MockSubstitution()
    with pytest.raises(TypeError):
        perform_substitutions(context, None)
    with pytest.raises(TypeError):
        # Not iterable
        perform_substitutions(context, mock_sub)
    with pytest.raises(TypeError):
        perform_substitutions(context, 1)
    with pytest.raises(AttributeError):
        perform_substitutions(context, 'foo')
    with pytest.raises(AttributeError):
        perform_substitutions(context, [1.4142, mock_sub])
    with pytest.raises(AttributeError):
        perform_substitutions(context, [mock_sub, [mock_sub]])
    with pytest.raises(NotImplementedError):
        perform_substitutions(context, [Substitution()])
