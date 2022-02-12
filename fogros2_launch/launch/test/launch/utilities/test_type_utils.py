# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Test type checking/coercion utils."""

from typing import List
from typing import Union

from launch.substitutions import TextSubstitution
from launch.utilities.type_utils import coerce_list
from launch.utilities.type_utils import coerce_to_type
from launch.utilities.type_utils import extract_type
from launch.utilities.type_utils import get_typed_value
from launch.utilities.type_utils import is_instance_of
from launch.utilities.type_utils import is_instance_of_valid_type
from launch.utilities.type_utils import is_normalized_substitution
from launch.utilities.type_utils import is_substitution
from launch.utilities.type_utils import is_typing_list
from launch.utilities.type_utils import is_valid_scalar_type
from launch.utilities.type_utils import normalize_typed_substitution
from launch.utilities.type_utils import perform_typed_substitution

import pytest


def test_is_typing_list():
    assert is_typing_list(List)
    assert is_typing_list(List[int])
    assert is_typing_list(List[float])
    assert is_typing_list(List[Union[int, float]])
    assert not is_typing_list(list)
    assert not is_typing_list(int)
    assert not is_typing_list(Union[int, float])
    assert not is_typing_list(None)


def test_is_valid_scalar_type():
    assert is_valid_scalar_type(int)
    assert is_valid_scalar_type(float)
    assert is_valid_scalar_type(bool)
    assert is_valid_scalar_type(str)
    assert not is_valid_scalar_type(bytes)
    assert not is_valid_scalar_type(None)
    assert not is_valid_scalar_type(List[float])
    assert not is_valid_scalar_type(list)
    assert not is_typing_list(int)


def test_extract_type():
    assert extract_type(int) == (int, False)
    assert extract_type(float) == (float, False)
    assert extract_type(bool) == (bool, False)
    assert extract_type(str) == (str, False)

    assert extract_type(List[int]) == (int, True)
    assert extract_type(List[float]) == (float, True)
    assert extract_type(List[bool]) == (bool, True)
    assert extract_type(List[str]) == (str, True)

    with pytest.raises(ValueError):
        extract_type(List)
    with pytest.raises(ValueError):
        extract_type(bytes)


@pytest.mark.parametrize(
    'is_instance_of_valid_type_impl',
    (
        is_instance_of_valid_type,
        lambda x, can_be_str=False: is_instance_of(x, None, can_be_str=can_be_str),
    ),
    ids=[
        'testing is_instance_of_valid_type implementation',
        'testing is_instance_of implementation',
    ]
)
def test_is_instance_of_valid_type(is_instance_of_valid_type_impl):
    assert is_instance_of_valid_type_impl(1)
    assert is_instance_of_valid_type_impl(1.)
    assert is_instance_of_valid_type_impl('asd')
    assert is_instance_of_valid_type_impl(True)

    assert is_instance_of_valid_type_impl([1, 2])
    assert is_instance_of_valid_type_impl([1., 2.])
    assert is_instance_of_valid_type_impl(['asd', 'bsd'])
    assert is_instance_of_valid_type_impl([True, False])

    assert not is_instance_of_valid_type_impl([1, '2'])
    assert not is_instance_of_valid_type_impl(object)
    assert not is_instance_of_valid_type_impl(test_is_instance_of_valid_type)
    assert not is_instance_of_valid_type_impl({'key': 'value'})

    assert is_instance_of_valid_type_impl(1, can_be_str=True)
    assert is_instance_of_valid_type_impl(1., can_be_str=True)
    assert is_instance_of_valid_type_impl('asd', can_be_str=True)
    assert is_instance_of_valid_type_impl(True, can_be_str=True)

    assert is_instance_of_valid_type_impl([1, 2], can_be_str=True)
    assert is_instance_of_valid_type_impl([1, '2'], can_be_str=True)
    assert is_instance_of_valid_type_impl([1., '2.'], can_be_str=True)
    assert is_instance_of_valid_type_impl(['asd', 'bsd'], can_be_str=True)
    assert is_instance_of_valid_type_impl([True, 'False'], can_be_str=True)

    assert not is_instance_of_valid_type_impl([1, '2', 1.], can_be_str=True)
    assert not is_instance_of_valid_type_impl(object, can_be_str=True)
    assert not is_instance_of_valid_type_impl(test_is_instance_of_valid_type, can_be_str=True)
    assert not is_instance_of_valid_type_impl({'key': 'value'}, can_be_str=True)


def test_is_instance_of():
    assert is_instance_of(1, int)
    assert is_instance_of(1., float)
    assert is_instance_of('asd', str)
    assert is_instance_of(True, bool)

    assert is_instance_of([1, 2], List[int])
    assert is_instance_of([1., 2.], List[float])
    assert is_instance_of(['asd', 'bsd'], List[str])
    assert is_instance_of([True, False], List[bool])

    assert not is_instance_of(1., int)
    assert not is_instance_of(1, float)
    assert not is_instance_of(True, str)
    assert not is_instance_of('asd', bool)

    assert not is_instance_of([1, 2.], List[int])
    assert not is_instance_of([1, 2.], List[float])
    assert not is_instance_of([True, False], List[str])
    assert not is_instance_of(['True', 'False'], List[bool])

    assert not is_instance_of(1, List[int])
    assert not is_instance_of(['1', 2], List[str])
    assert not is_instance_of(['1', '2'], str)

    assert is_instance_of(1, int, can_be_str=True)
    assert is_instance_of('1', int, can_be_str=True)
    assert is_instance_of([1, 2], List[int], can_be_str=True)
    assert is_instance_of(['1', 2], List[int], can_be_str=True)
    assert not is_instance_of(['1', 2.], List[int], can_be_str=True)
    assert not is_instance_of([1, 2.], List[int], can_be_str=True)
    assert not is_instance_of([1, 2], int, can_be_str=True)
    assert not is_instance_of([1, '2'], List[str], can_be_str=True)

    with pytest.raises(ValueError):
        is_instance_of(1, bytes)
    with pytest.raises(ValueError):
        is_instance_of([1], List)
    with pytest.raises(ValueError):
        is_instance_of([True, False], list)
    with pytest.raises(ValueError):
        is_instance_of([True, False], Union[int, str])


@pytest.mark.parametrize(
    'coerce_to_type_impl',
    (
        coerce_to_type,
        lambda value, data_type=None, can_be_str=False: get_typed_value(
            value, data_type, can_be_str=can_be_str),
    ),
    ids=[
        'testing coerce_to_type implementation',
        'testing get_typed_value implementation',
    ]
)
def test_coercions_using_yaml_rules(coerce_to_type_impl):
    assert coerce_to_type_impl('') == ''
    assert coerce_to_type_impl('asd') == 'asd'
    assert coerce_to_type_impl('tRuE') == 'tRuE'
    assert coerce_to_type_impl("'1'") == '1'
    assert coerce_to_type_impl("'off'") == 'off'

    assert coerce_to_type_impl('1') == 1
    assert coerce_to_type_impl('1000') == 1000

    assert coerce_to_type_impl('1.') == 1.0
    assert coerce_to_type_impl('1000.0') == 1000.
    assert coerce_to_type_impl('0.2') == .2
    assert coerce_to_type_impl('.3') == 0.3

    assert coerce_to_type_impl('on') is True
    assert coerce_to_type_impl('off') is False
    assert coerce_to_type_impl('True') is True

    assert coerce_to_type_impl('[2, 1, 1]') == [2, 1, 1]
    assert coerce_to_type_impl('[.2, .1, .1]') == [.2, .1, .1]
    assert coerce_to_type_impl('[asd, bsd, csd]') == ['asd', 'bsd', 'csd']
    assert coerce_to_type_impl('[on, false, no]') == [True, False, False]

    assert coerce_to_type_impl('[asd, 2.0]', can_be_str=True) == ['asd', 2.0]


@pytest.mark.parametrize(
    'coerce_to_type_impl',
    (
        coerce_to_type,
        lambda value, data_type=None, can_be_str=False: get_typed_value(
            value, data_type, can_be_str=can_be_str),
    ),
    ids=[
        'testing coerce_to_type implementation',
        'testing get_typed_value implementation',
    ]
)
def test_coercions_given_specific_type(coerce_to_type_impl):
    assert coerce_to_type_impl('', data_type=str) == ''
    assert coerce_to_type_impl('asd', data_type=str) == 'asd'
    assert coerce_to_type_impl('tRuE', data_type=str) == 'tRuE'
    assert coerce_to_type_impl("'1'", data_type=str) == "'1'"
    assert coerce_to_type_impl("'off'", data_type=str) == "'off'"
    assert coerce_to_type_impl("''1''", data_type=str) == "''1''"
    assert coerce_to_type_impl('{1}', data_type=str) == '{1}'

    assert coerce_to_type_impl('1', data_type=int) == 1
    assert coerce_to_type_impl('1000', data_type=int) == 1000

    assert coerce_to_type_impl('1', data_type=float) == 1.0
    assert coerce_to_type_impl('1.', data_type=float) == 1.0
    assert coerce_to_type_impl('1000.0', data_type=float) == 1000.
    assert coerce_to_type_impl('0.2', data_type=float) == .2
    assert coerce_to_type_impl('.3', data_type=float) == 0.3

    assert coerce_to_type_impl('on', data_type=bool) is True
    assert coerce_to_type_impl('off', data_type=bool) is False
    assert coerce_to_type_impl('True', data_type=bool) is True

    assert coerce_to_type_impl('[.2, .1, .1]', data_type=List[float]) == [.2, .1, .1]
    assert coerce_to_type_impl('[asd, bsd, csd]', data_type=List[str]) == ['asd', 'bsd', 'csd']
    assert coerce_to_type_impl('[on, false, no]', data_type=List[bool]) == [True, False, False]

    assert coerce_to_type_impl(
        'asd', data_type=float, can_be_str=True) == 'asd'
    assert coerce_to_type_impl(
        '[asd, 2.0]', data_type=List[float], can_be_str=True) == ['asd', 2.0]


@pytest.mark.parametrize(
    'coerce_to_type_impl',
    (
        coerce_to_type,
        lambda value, data_type=None, can_be_str=False: get_typed_value(
            value, data_type, can_be_str=can_be_str),
    ),
    ids=[
        'testing coerce_to_type implementation',
        'testing get_typed_value implementation',
    ]
)
def test_coercion_raises_value_error(coerce_to_type_impl):
    with pytest.raises(ValueError):
        coerce_to_type_impl("''1''")
    with pytest.raises(ValueError):
        coerce_to_type_impl('{1}')
    with pytest.raises(ValueError):
        coerce_to_type_impl('[1, 2.0]')
    with pytest.raises(ValueError):
        coerce_to_type_impl('[asd, 2.0]')

    with pytest.raises(ValueError):
        coerce_to_type_impl('', data_type=int)
    with pytest.raises(ValueError):
        coerce_to_type_impl('1000.5', data_type=int)
    with pytest.raises(ValueError):
        coerce_to_type_impl('Bsd', data_type=int)

    with pytest.raises(ValueError):
        coerce_to_type_impl('', data_type=float)
    with pytest.raises(ValueError):
        coerce_to_type_impl('Bsd', data_type=float)

    with pytest.raises(ValueError):
        coerce_to_type_impl('', data_type=bool)
    with pytest.raises(ValueError):
        coerce_to_type_impl('Bsd', data_type=bool)
    with pytest.raises(ValueError):
        coerce_to_type_impl('1', data_type=bool)

    with pytest.raises(ValueError):
        coerce_to_type_impl('', data_type=List[float])
    with pytest.raises(ValueError):
        coerce_to_type_impl('[1, 2.0]', data_type=List[float])
    with pytest.raises(ValueError):
        coerce_to_type_impl('[asd, 2.0]', data_type=List[float])


def coerce_to_type_raises_type_error():
    with pytest.raises(TypeError):
        coerce_to_type(['a', 'b'])


@pytest.mark.parametrize(
    'coerce_list_impl',
    (
        coerce_list,
        lambda value, data_type=None, can_be_str=False: get_typed_value(
            value, data_type, can_be_str=can_be_str),
    ),
    ids=[
        'testing coerce_list implementation',
        'testing get_typed_value implementation',
    ]
)
def test_coercing_list_using_yaml_rules(coerce_list_impl):
    assert coerce_list_impl(['']) == ['']
    assert coerce_list_impl(['asd', 'bsd']) == ['asd', 'bsd']
    assert coerce_list_impl(['1', '1000']) == [1, 1000]
    assert coerce_list_impl(['1.', '1000.']) == [1., 1000.]
    assert coerce_list_impl(['on', 'off', 'True']) == [True, False, True]

    assert coerce_list_impl(['', '1000'], data_type=None, can_be_str=True) == ['', 1000]
    assert coerce_list_impl(['asd', '1000'], data_type=None, can_be_str=True) == ['asd', 1000]
    assert coerce_list_impl(['asd', '1000.'], data_type=None, can_be_str=True) == ['asd', 1000.]
    assert coerce_list_impl(
        ['asd', 'off', 'True'], data_type=None, can_be_str=True) == ['asd', False, True]

    with pytest.raises(ValueError):
        coerce_list_impl(['', '1000'])
    with pytest.raises(ValueError):
        coerce_list_impl(['1000.', '1000'])
    with pytest.raises(ValueError):
        coerce_list_impl(['asd', '1000'])
    with pytest.raises(ValueError):
        coerce_list_impl(['asd', '1000.'])
    with pytest.raises(ValueError):
        coerce_list_impl(['asd', 'True'])


@pytest.mark.parametrize(
    'coerce_list_impl',
    (
        coerce_list,
        lambda value, data_type=None, can_be_str=False: get_typed_value(
            value, List[data_type], can_be_str=can_be_str),
    ),
    ids=[
        'testing coerce_list implementation',
        'testing get_typed_value implementation',
    ]
)
def test_coercing_list_given_specific_type(coerce_list_impl):
    assert coerce_list_impl([''], str) == ['']
    assert coerce_list_impl(['asd', 'bsd'], str) == ['asd', 'bsd']
    assert coerce_list_impl(['1', '1000'], int) == [1, 1000]
    assert coerce_list_impl(['1.', '1000.'], float) == [1., 1000.]
    assert coerce_list_impl(['on', 'off', 'True'], bool) == [True, False, True]
    assert coerce_list_impl(['1000.', '1000'], float) == [1000., 1000.]

    assert coerce_list_impl(['', '1000'], int, can_be_str=True) == ['', 1000]
    assert coerce_list_impl(['asd', '1000'], int, can_be_str=True) == ['asd', 1000]
    assert coerce_list_impl(['asd', '1000.'], float, can_be_str=True) == ['asd', 1000.]
    assert coerce_list_impl(['asd', 'off', 'True'], bool, can_be_str=True) == ['asd', False, True]

    with pytest.raises(ValueError):
        coerce_list_impl(['1', '2'], bool)
    with pytest.raises(ValueError):
        coerce_list_impl(['asd', '1000'], int)
    with pytest.raises(ValueError):
        coerce_list_impl(['asd', '1000.'], float)
    with pytest.raises(ValueError):
        coerce_list_impl(['asd', 'True'], bool)


def test_coerce_list_raises_type_error():
    with pytest.raises(TypeError):
        coerce_list('a')
    with pytest.raises(TypeError):
        coerce_list([1, 2])


def test_get_typed_value_raises_type_error():
    with pytest.raises(TypeError):
        get_typed_value({'a'})
    with pytest.raises(TypeError):
        get_typed_value(['1', '2'], int)
    with pytest.raises(TypeError):
        get_typed_value([1, 2])


def test_is_substitution():
    assert is_substitution(TextSubstitution(text='asd'))
    assert is_substitution([
        TextSubstitution(text='asd'),
        'bsd'
    ])
    assert is_substitution([
        'asd',
        TextSubstitution(text='bsd'),
    ])
    assert is_substitution([
        TextSubstitution(text='asd'),
        TextSubstitution(text='bsd'),
    ])
    assert not is_substitution([])
    assert not is_substitution('asd')
    assert not is_substitution(['asd', 'bsd'])
    assert not is_substitution(['asd', 'bsd'])
    assert not is_substitution(1)


def test_normalize_typed_substitution():
    assert normalize_typed_substitution(1, int) == 1

    nts = normalize_typed_substitution(TextSubstitution(text='bsd'), int)
    assert isinstance(nts, list)
    assert len(nts) == 1
    assert isinstance(nts[0], TextSubstitution)

    nts = normalize_typed_substitution(
        [
            TextSubstitution(text='asd'),
            TextSubstitution(text='bsd'),
        ],
        int
    )
    assert len(nts) == 2
    assert isinstance(nts[0], TextSubstitution)
    assert isinstance(nts[1], TextSubstitution)

    nts = normalize_typed_substitution(TextSubstitution(text='bsd'), int)
    assert isinstance(nts, list)
    assert len(nts) == 1
    assert isinstance(nts[0], TextSubstitution)

    nts = normalize_typed_substitution(
        [
            TextSubstitution(text='asd'),
            TextSubstitution(text='bsd'),
        ],
        List[int]
    )
    assert len(nts) == 2
    assert isinstance(nts[0], TextSubstitution)
    assert isinstance(nts[1], TextSubstitution)

    assert normalize_typed_substitution([1, 2], List[int]) == [1, 2]
    nts = normalize_typed_substitution([TextSubstitution(text='bsd'), 2], List[int])
    assert len(nts) == 2
    assert isinstance(nts[0], list)
    assert len(nts[0]) == 1
    assert isinstance(nts[0][0], TextSubstitution)
    assert nts[1] == 2

    nts = normalize_typed_substitution([[TextSubstitution(text='bsd')], 2], List[int])
    assert len(nts) == 2
    assert len(nts[0]) == 1
    assert isinstance(nts[0][0], TextSubstitution)
    assert nts[1] == 2

    nts = normalize_typed_substitution([[TextSubstitution(text='asd')], 'bsd'], List[str])
    assert len(nts) == 2
    assert len(nts[0]) == 1
    assert isinstance(nts[0][0], TextSubstitution)
    assert nts[1] == 'bsd'

    nts = normalize_typed_substitution(
        [
            [TextSubstitution(text='asd')],
            [TextSubstitution(text='bsd'), 'bsd'],
        ],
        List[int]
    )
    assert len(nts) == 2
    assert isinstance(nts[0], list)
    assert len(nts[0]) == 1
    assert isinstance(nts[0][0], TextSubstitution)
    assert isinstance(nts[1], list)
    assert len(nts[1]) == 2
    assert isinstance(nts[1][0], TextSubstitution)
    assert isinstance(nts[1][1], TextSubstitution)  # should have been normalized

    assert normalize_typed_substitution(1, data_type=None) == 1

    nts = normalize_typed_substitution(TextSubstitution(text='bsd'), data_type=None)
    assert isinstance(nts, list)
    assert len(nts) == 1
    assert isinstance(nts[0], TextSubstitution)

    nts = normalize_typed_substitution(
        [
            TextSubstitution(text='asd'),
            TextSubstitution(text='bsd'),
        ],
        data_type=None
    )
    assert len(nts) == 2
    assert isinstance(nts[0], TextSubstitution)
    assert isinstance(nts[1], TextSubstitution)

    nts = normalize_typed_substitution([TextSubstitution(text='bsd'), 2], data_type=None)
    assert len(nts) == 2
    assert isinstance(nts[0], list)
    assert len(nts[0]) == 1
    assert isinstance(nts[0][0], TextSubstitution)
    assert nts[1] == 2

    with pytest.raises(ValueError):
        normalize_typed_substitution(1, bytes)
    with pytest.raises(TypeError):
        normalize_typed_substitution(1, List[int])
    with pytest.raises(TypeError):
        normalize_typed_substitution(['asd', 2], List[int])


def test_is_normalized_substitution():
    assert is_normalized_substitution([TextSubstitution(text='asd')])
    assert is_normalized_substitution(
        [TextSubstitution(text='asd'), TextSubstitution(text='bsd')])

    assert not is_normalized_substitution(TextSubstitution(text='asd'))
    assert not is_normalized_substitution([TextSubstitution(text='asd'), 'bsd'])
    assert not is_normalized_substitution(['bsd'])
    assert not is_normalized_substitution('bsd')


class MockContext:

    def __init__(self):
        self.launch_configurations = {}

    def perform_substitution(self, sub):
        return sub.perform(None)


def test_perform_typed_substitution():
    lc = MockContext()

    assert perform_typed_substitution(lc, 1, int) == 1
    assert perform_typed_substitution(lc, [TextSubstitution(text='1')], int) == 1
    assert perform_typed_substitution(
        lc, [TextSubstitution(text='[1, 2, 3]')], List[int]) == [1, 2, 3]
    assert perform_typed_substitution(
        lc, [TextSubstitution(text='[1, 2, 3]')], None) == [1, 2, 3]

    assert perform_typed_substitution(
        lc,
        [
            TextSubstitution(text='100'),
            TextSubstitution(text='.'),
            TextSubstitution(text='5'),
        ],
        float,
    ) == 100.5

    assert perform_typed_substitution(
        lc,
        [
            [TextSubstitution(text='100')],
            [TextSubstitution(text='1')],
            [TextSubstitution(text='5')],
        ],
        List[int],
    ) == [100, 1, 5]

    assert perform_typed_substitution(
        lc,
        [
            'asd',
            [TextSubstitution(text='bsd')],
            [TextSubstitution(text='csd')],
        ],
        List[str],
    ) == ['asd', 'bsd', 'csd']

    assert perform_typed_substitution(
        lc,
        [
            'asd',
            [TextSubstitution(text='bsd')],
            [TextSubstitution(text='csd')],
        ],
        data_type=None,
    ) == ['asd', 'bsd', 'csd']

    with pytest.raises(TypeError):
        perform_typed_substitution(lc, object, int)
    with pytest.raises(TypeError):
        perform_typed_substitution(lc, 1., int)
    with pytest.raises(ValueError):
        perform_typed_substitution(lc, 'asd', bytes)
    with pytest.raises(ValueError):
        perform_typed_substitution(lc, [TextSubstitution(text='1.')], int)
    with pytest.raises(ValueError):
        perform_typed_substitution(lc, [TextSubstitution(text='1')], List[int])
    with pytest.raises(ValueError):
        perform_typed_substitution(
            lc, [[TextSubstitution(text='1.')], [TextSubstitution(text='1')]], data_type=None)
