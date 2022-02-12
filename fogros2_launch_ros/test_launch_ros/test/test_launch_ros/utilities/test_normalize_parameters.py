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

"""Tests for the normalizing parameters utility."""

import pathlib
from typing import List

from launch import LaunchContext
from launch.substitutions import TextSubstitution

from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.utilities import evaluate_parameters
from launch_ros.utilities import normalize_parameters

import pytest


def test_not_a_list():
    with pytest.raises(TypeError):
        normalize_parameters({'foo': 'bar'})
    with pytest.raises(TypeError):
        normalize_parameters('foobar')


def test_single_str_path():
    norm = normalize_parameters(['/foo/bar'])
    expected = (pathlib.Path('/foo/bar'),)
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_single_pathlib_path():
    norm = normalize_parameters([pathlib.Path('/foo/bar')])
    expected = (pathlib.Path('/foo/bar'),)
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_multiple_paths():
    norm = normalize_parameters([pathlib.Path('/foo/bar'), '/bar/baz'])
    expected = (pathlib.Path('/foo/bar'), pathlib.Path('/bar/baz'))
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_path_with_substitutions():
    orig = [(TextSubstitution(text='/foo'), TextSubstitution(text='/bar'))]
    norm = normalize_parameters(orig)
    expected = (pathlib.Path('/foo/bar'),)
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_single_dictionary():
    orig = [{'foo': 1, 'bar': 2.0}]
    norm = normalize_parameters(orig)
    expected = ({'foo': 1, 'bar': 2.0},)
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_multiple_dictionaries():
    orig = [{'foo': 1, 'bar': 2.0}, {'baz': 'asdf'}]
    norm = normalize_parameters(orig)
    expected = ({'foo': 1, 'bar': 2.0}, {'baz': 'asdf'})
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_dictionary_with_substitution():
    orig = [{TextSubstitution(text='bar'): TextSubstitution(text='baz')}]
    norm = normalize_parameters(orig)
    expected = ({'bar': 'baz'},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    # substitutions get yamlfied
    orig = [{TextSubstitution(text='false'): TextSubstitution(text='off')}]
    norm = normalize_parameters(orig)
    expected = ({'false': False},)
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_dictionary_with_substitution_list_name():
    orig = [{(TextSubstitution(text='bar'), TextSubstitution(text='foo')): 1}]
    norm = normalize_parameters(orig)
    expected = ({'barfoo': 1},)
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_dictionary_with_substitution_list_value():
    orig = [{'foo': [
        [TextSubstitution(text='fiz'), TextSubstitution(text='buz')],
        TextSubstitution(text='fiz')
    ]}]
    norm = normalize_parameters(orig)
    expected = ({'foo': ('fizbuz', 'fiz')},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'foo': [TextSubstitution(text='fiz'), TextSubstitution(text='buz')]}]
    norm = normalize_parameters(orig)
    expected = ({'foo': 'fizbuz'},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'foo': [[TextSubstitution(text='fiz')], [TextSubstitution(text='buz')]]}]
    norm = normalize_parameters(orig)
    expected = ({'foo': ('fiz', 'buz')},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'bools': [[TextSubstitution(text='True')], [TextSubstitution(text='False')]]}]
    norm = normalize_parameters(orig)
    expected = ({'bools': (True, False)},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'ints': [[TextSubstitution(text='1')], [TextSubstitution(text='2')]]}]
    norm = normalize_parameters(orig)
    expected = ({'ints': (1, 2)},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'floats': [[TextSubstitution(text='1.0')], [TextSubstitution(text='2.0')]]}]
    norm = normalize_parameters(orig)
    expected = ({'floats': (1.0, 2.0)},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'strings': ['True', '1', '1.0']}]
    norm = normalize_parameters(orig)
    expected = ({'strings': ('True', '1', '1.0')},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'int_sequence': TextSubstitution(text='[2, 3, 4]')}]
    norm = normalize_parameters(orig)
    expected = ({'int_sequence': (2, 3, 4)},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'float_sequence': TextSubstitution(text='[2.0, 3.0, 4.0]')}]
    norm = normalize_parameters(orig)
    expected = ({'float_sequence': (2., 3., 4.)},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'bool_sequence': TextSubstitution(text='[True, False, True]')}]
    norm = normalize_parameters(orig)
    expected = ({'bool_sequence': (True, False, True)},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'string_sequence': TextSubstitution(text="['True', '1', 'asd', '2.0']")}]
    norm = normalize_parameters(orig)
    expected = ({'string_sequence': ('True', '1', 'asd', '2.0')},)
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_dictionary_with_mixed_substitutions_and_strings():
    orig = [{'foo': [TextSubstitution(text='fiz'), 'bar']}]
    norm = normalize_parameters(orig)
    expected = ({'foo': 'fizbar'},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'foo': [[TextSubstitution(text='fiz')], 'bar']}]
    norm = normalize_parameters(orig)
    expected = ({'foo': ('fiz', 'bar')},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'foo': ['bar', TextSubstitution(text='fiz')]}]
    norm = normalize_parameters(orig)
    expected = ({'foo': 'barfiz'},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'foo': ['bar', [TextSubstitution(text='fiz')]]}]
    norm = normalize_parameters(orig)
    expected = ({'foo': ('bar', 'fiz')},)
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_dictionary_with_str():
    orig = [{'foo': 'bar', 'fiz': ['b', 'u', 'z']}]
    norm = normalize_parameters(orig)
    expected = ({'foo': 'bar', 'fiz': ('b', 'u', 'z')},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'foo': 'False', 'fiz': ['True', 'False', 'True']}]
    norm = normalize_parameters(orig)
    expected = ({'foo': 'False', 'fiz': ('True', 'False', 'True')},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'foo': '1.2', 'fiz': ['2.3', '3.4', '4.5']}]
    norm = normalize_parameters(orig)
    expected = ({'foo': '1.2', 'fiz': ('2.3', '3.4', '4.5')},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'foo': '1', 'fiz': ['2', '3', '4']}]
    norm = normalize_parameters(orig)
    expected = ({'foo': '1', 'fiz': ('2', '3', '4')},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'strs': ['True', '2.0', '3']}]
    norm = normalize_parameters(orig)
    expected = ({'strs': ('True', '2.0', '3')},)
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_dictionary_with_bool():
    orig = [{'foo': False, 'fiz': [True, False, True]}]
    norm = normalize_parameters(orig)
    expected = ({'foo': False, 'fiz': (True, False, True)},)
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_dictionary_with_float():
    orig = [{'foo': 1.2, 'fiz': [2.3, 3.4, 4.5]}]
    norm = normalize_parameters(orig)
    expected = ({'foo': 1.2, 'fiz': (2.3, 3.4, 4.5)},)
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_dictionary_with_int():
    orig = [{'foo': 1, 'fiz': [2, 3, 4]}]
    norm = normalize_parameters(orig)
    expected = ({'foo': 1, 'fiz': (2, 3, 4)},)
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_dictionary_with_int_and_float():
    orig = [{'foo': 1, 'fiz': [2, 3.1, 4]}]
    norm = normalize_parameters(orig)
    expected = ({'foo': 1, 'fiz': (2.0, 3.1, 4.0)},)
    evaluated = evaluate_parameters(LaunchContext(), norm)
    assert evaluated == expected
    # pytest doesn't check int vs float type
    assert tuple(map(type, evaluated[0]['fiz'])) == (float, float, float)

    orig = [{'foo': 1, 'fiz': [2.0, 3, 4]}]
    norm = normalize_parameters(orig)
    expected = ({'foo': 1, 'fiz': (2.0, 3.0, 4.0)},)
    evaluated = evaluate_parameters(LaunchContext(), norm)
    assert evaluated == expected
    # pytest doesn't check int vs float type
    assert tuple(map(type, evaluated[0]['fiz'])) == (float, float, float)


def test_dictionary_with_bytes():
    orig = [{'foo': 1, 'fiz': bytes([0xff, 0x5c, 0xaa])}]
    norm = normalize_parameters(orig)
    expected = ({'foo': 1, 'fiz': bytes([0xff, 0x5c, 0xaa])},)
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_dictionary_with_dissimilar_array():
    with pytest.raises(TypeError) as exc:
        orig = [{'foo': 1, 'fiz': [True, 2.0, 3]}]
        norm = normalize_parameters(orig)
        evaluate_parameters(LaunchContext(), norm)
    assert 'Expected a non-empty' in str(exc.value)

    with pytest.raises(TypeError) as exc:
        orig = [{'foo': 1, 'fiz': [True, 1, TextSubstitution(text='foo')]}]
        norm = normalize_parameters(orig)
        evaluate_parameters(LaunchContext(), norm)
    assert 'Expected a non-empty' in str(exc.value)

    with pytest.raises(TypeError) as exc:
        orig = [{'foo': 1, 'fiz': [TextSubstitution(text='foo'), True, 1]}]
        norm = normalize_parameters(orig)
        evaluate_parameters(LaunchContext(), norm)
    assert 'Expected a non-empty' in str(exc.value)

    with pytest.raises(TypeError) as exc:
        orig = [{'foo': 1, 'fiz': [True, 1, [TextSubstitution(text='foo')]]}]
        norm = normalize_parameters(orig)
        evaluate_parameters(LaunchContext(), norm)
    assert 'Expected a non-empty' in str(exc.value)

    with pytest.raises(TypeError) as exc:
        orig = [{'foo': 1, 'fiz': [[TextSubstitution(text='foo')], True, 1]}]
        norm = normalize_parameters(orig)
        evaluate_parameters(LaunchContext(), norm)
    assert 'Expected a non-empty' in str(exc.value)

    with pytest.raises(TypeError) as exc:
        orig = [{'foo': [
            [TextSubstitution(text='True')],
            [TextSubstitution(text='2.0')],
            [TextSubstitution(text='3')],
        ]}]
        norm = normalize_parameters(orig)
        evaluate_parameters(LaunchContext(), norm)
    assert 'Expected a non-empty' in str(exc.value)


def test_nested_dictionaries():
    orig = [{'foo': {'bar': 'baz'}, 'fiz': {'buz': 3}}]
    norm = normalize_parameters(orig)
    expected = ({'foo.bar': 'baz', 'fiz.buz': 3},)
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_mixed_path_dicts():
    orig = ['/foo/bar', {'fiz': {'buz': 3}}, pathlib.Path('/tmp/baz')]
    norm = normalize_parameters(orig)
    expected = (pathlib.Path('/foo/bar'), {'fiz.buz': 3}, pathlib.Path('/tmp/baz'))
    assert evaluate_parameters(LaunchContext(), norm) == expected


def test_unallowed_yaml_types_in_substitutions():
    with pytest.raises(TypeError) as exc:
        orig = [{'foo': 1, 'fiz': TextSubstitution(text="{'asd': 3}")}]
        norm = normalize_parameters(orig)
        evaluate_parameters(LaunchContext(), norm)
    assert 'Allowed value types' in str(exc.value)
    assert 'dict' in str(exc.value)

    with pytest.raises(TypeError) as exc:
        orig = [{'foo': 1, 'fiz': TextSubstitution(text='[1, 2.0, 3]')}]
        norm = normalize_parameters(orig)
        evaluate_parameters(LaunchContext(), norm)
    assert 'Expected a non-empty sequence' in str(exc.value)

    with pytest.raises(TypeError) as exc:
        orig = [{'foo': 1, 'fiz': TextSubstitution(text='[[2, 3], [2, 3], [2, 3]]')}]
        norm = normalize_parameters(orig)
        evaluate_parameters(LaunchContext(), norm)
    assert 'Expected a non-empty sequence' in str(exc.value)

    with pytest.raises(TypeError) as exc:
        orig = [{'foo': 1, 'fiz': TextSubstitution(text='[]')}]
        norm = normalize_parameters(orig)
        evaluate_parameters(LaunchContext(), norm)
    assert 'Expected a non-empty sequence' in str(exc.value)

    with pytest.raises(TypeError) as exc:
        orig = [{
            'foo': 1,
            'fiz': [
                [TextSubstitution(text="['asd', 'bsd']")],
                [TextSubstitution(text="['asd', 'csd']")]
            ]
        }]
        norm = normalize_parameters(orig)
        evaluate_parameters(LaunchContext(), norm)
    assert 'Expected a non-empty sequence' in str(exc.value)

    with pytest.raises(TypeError) as exc:
        orig = [{'foo': 1, 'fiz': TextSubstitution(text='Text That : Cannot Be Parsed As : Yaml')}]
        norm = normalize_parameters(orig)
        evaluate_parameters(LaunchContext(), norm)
    assert 'Unable to parse' in str(exc.value)


def test_unallowed_yaml_types_as_strings():
    # All the tests from test_unallowed_yaml_types_in_substitutions
    # but coerced to the proper type with ParameterValue
    orig = [{'foo': 1, 'fiz': ParameterValue(TextSubstitution(text="{'asd': 3}"), value_type=str)}]
    norm = normalize_parameters(orig)
    expected = ({'foo': 1, 'fiz': "{'asd': 3}"},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'foo': 1, 'fiz': ParameterValue(TextSubstitution(text='[1, 2.0, 3]'),
                                             value_type=str)}]
    norm = normalize_parameters(orig)
    evaluate_parameters(LaunchContext(), norm)
    expected = ({'foo': 1, 'fiz': '[1, 2.0, 3]'},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'foo': 1, 'fiz': ParameterValue(TextSubstitution(text='[[2, 3], [2, 3], [2, 3]]'),
                                             value_type=str)}]
    norm = normalize_parameters(orig)
    evaluate_parameters(LaunchContext(), norm)
    expected = ({'foo': 1, 'fiz': '[[2, 3], [2, 3], [2, 3]]'},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'foo': 1, 'fiz': ParameterValue(TextSubstitution(text='[]'), value_type=str)}]
    norm = normalize_parameters(orig)
    evaluate_parameters(LaunchContext(), norm)
    expected = ({'foo': 1, 'fiz': '[]'},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{
        'foo': 1,
        'fiz': ParameterValue([
            [TextSubstitution(text="['asd', 'bsd']")],
            [TextSubstitution(text="['asd', 'csd']")]
        ], value_type=List[str])
    }]
    norm = normalize_parameters(orig)
    evaluate_parameters(LaunchContext(), norm)
    expected = ({'foo': 1, 'fiz': ["['asd', 'bsd']", "['asd', 'csd']"]},)
    assert evaluate_parameters(LaunchContext(), norm) == expected

    orig = [{'foo': 1,
             'fiz': ParameterValue(TextSubstitution(text='Text That : Cannot Be Parsed As : Yaml'),
                                   value_type=str)}]
    norm = normalize_parameters(orig)
    evaluate_parameters(LaunchContext(), norm)
    expected = ({'foo': 1, 'fiz': 'Text That : Cannot Be Parsed As : Yaml'},)
    assert evaluate_parameters(LaunchContext(), norm) == expected
