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

"""Tests for the normalize_to_list_of_substitutions() function."""

from launch import Substitution
from launch.utilities import normalize_to_list_of_substitutions

import pytest


def test_valid_substitutions():
    """Test the normalize_to_list_of_substitutions() function with valid input."""
    class MockSubstitution(Substitution):
        pass

    sub = MockSubstitution()
    norm_sub = normalize_to_list_of_substitutions(sub)
    assert isinstance(norm_sub, list)
    assert len(norm_sub) == 1
    assert isinstance(norm_sub[0], Substitution)
    text_sub = 'foo'
    norm_text_sub = normalize_to_list_of_substitutions(text_sub)
    assert isinstance(norm_text_sub, list)
    assert len(norm_text_sub) == 1
    assert isinstance(norm_text_sub[0], Substitution)
    sub_list = ['bar', MockSubstitution()]
    norm_sub_list = normalize_to_list_of_substitutions(sub_list)
    assert isinstance(norm_sub_list, list)
    assert len(norm_sub_list) == 2
    assert isinstance(norm_sub_list[0], Substitution)
    assert isinstance(norm_sub_list[1], Substitution)
    sub_tuple = (MockSubstitution(), '')
    norm_sub_tuple = normalize_to_list_of_substitutions(sub_tuple)
    assert isinstance(norm_sub_tuple, list)
    assert len(norm_sub_tuple) == 2
    assert isinstance(norm_sub_tuple[0], Substitution)
    assert isinstance(norm_sub_tuple[1], Substitution)
    norm_sub_empty = normalize_to_list_of_substitutions([])
    assert isinstance(norm_sub_empty, list)
    assert len(norm_sub_empty) == 0


def test_invalid_substitutions():
    """Test the normalize_to_list_of_substitutions() function with invalid input."""
    with pytest.raises(TypeError):
        normalize_to_list_of_substitutions(None)
    with pytest.raises(TypeError):
        normalize_to_list_of_substitutions(1)
    with pytest.raises(TypeError):
        normalize_to_list_of_substitutions([1.4142, 'bar'])
    with pytest.raises(TypeError):
        normalize_to_list_of_substitutions(['foo', ['bar']])
