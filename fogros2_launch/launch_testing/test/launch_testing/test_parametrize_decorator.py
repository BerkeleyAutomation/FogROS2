# Copyright 2019 Apex.AI, Inc.
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

import launch_testing


def test_parametrize_attribute():

    @launch_testing.parametrize('val', [1, 2, 3])
    def fake_test_description(arg):
        pass  # pragma: no cover

    assert hasattr(fake_test_description, '__parametrized__')


def test_binding_arguments():

    results = []

    @launch_testing.parametrize('val', [1, 2, 3])
    def fake_test_description(val):
        results.append(val)

    for func, params in fake_test_description():
        func()

    assert results == [1, 2, 3]


def test_binding_one_tuples():

    results = []

    @launch_testing.parametrize('val', [(1,), (2,), (3,)])
    def fake_test_description(val):
        results.append(val)

    for func, params in fake_test_description():
        func()

    assert results == [1, 2, 3]


def test_partial_binding():

    results = []

    @launch_testing.parametrize('val', ['x', 'y', 'z'])
    def fake_test_description(val, arg):
        results.append((val, arg))

    for index, (func, params) in enumerate(fake_test_description()):
        func(arg=index)

    assert results == [('x', 0), ('y', 1), ('z', 2)]


def test_multiple_args():

    results = []

    @launch_testing.parametrize('arg_1, arg_2', [(5, 10), (15, 20), (25, 30)])
    def fake_test_description(arg_1, arg_2):
        results.append((arg_1, arg_2))

    for index, (func, params) in enumerate(fake_test_description()):
        func()

    assert results == [(5, 10), (15, 20), (25, 30)]
