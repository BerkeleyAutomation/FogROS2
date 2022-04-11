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

from launch.frontend.expose import __expose_impl

import pytest


class ToBeExposed:

    @classmethod
    def parse(cls, entity, parser):
        return ToBeExposed(), ()


def to_be_exposed(entity, parser):
    return ToBeExposed(), ()


def test_expose_decorators():
    register = {}

    def expose_test(name):
        return __expose_impl(name, register, 'test')
    expose_test('ToBeExposed')(ToBeExposed)
    assert 'ToBeExposed' in register
    if 'ToBeExposed' in register:
        assert register['ToBeExposed'] == ToBeExposed.parse
    expose_test('to_be_exposed')(to_be_exposed)
    assert 'to_be_exposed' in register
    if 'to_be_exposed' in register:
        assert register['to_be_exposed'] == to_be_exposed
    NotACallable = 5
    with pytest.raises(
        RuntimeError,
        match='Exposed test parser for NotACallable is not a callable or a class'
              ' containg a parse method'
    ):
        expose_test('NotACallable')(NotACallable)
