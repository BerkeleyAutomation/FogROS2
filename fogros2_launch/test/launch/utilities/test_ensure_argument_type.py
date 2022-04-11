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

"""Tests for the ensure_argument_type() function."""

from launch.utilities import ensure_argument_type

import pytest


def test_valid_argument_types():
    """Test the ensure_argument_type function with valid input."""
    ensure_argument_type('foo', str, 'arg_foo')
    ensure_argument_type(1, [str, int], 'arg_bar')
    ensure_argument_type([3.14159], [list, str, int], '')
    ensure_argument_type(3.14159, [float], ' ')

    class MockClass:
        pass

    mock_class_obj = MockClass()
    ensure_argument_type(mock_class_obj, MockClass, 'MockClass')

    # With inheritence
    class MockChildClass(MockClass):
        pass

    mock_child_obj = MockChildClass()
    ensure_argument_type(mock_child_obj, MockClass, 'MockChildClass')


def test_invalid_argument_types():
    """Test the ensure_argument_type function with invalid input."""
    with pytest.raises(TypeError):
        ensure_argument_type(None, None, 'none')
    with pytest.raises(TypeError):
        ensure_argument_type(1, str, 'foo')
    with pytest.raises(TypeError):
        ensure_argument_type('bar', [int, float, list, tuple], 'arg_bar')

    class MockClass:
        pass

    class MockChildClass(MockClass):
        pass

    mock_class_obj = MockClass()
    with pytest.raises(TypeError):
        ensure_argument_type(mock_class_obj, MockChildClass, 'MockChildClass')


def test_invalid_argument_types_with_caller():
    """Test the ensure_argument_type function with invalid input and caller name."""
    with pytest.raises(TypeError) as ex:
        ensure_argument_type(1, [float, complex, list, tuple], 'test arg name', 'My test caller')
    assert 'test arg name' in str(ex.value)
    assert 'My test caller' in str(ex.value)
