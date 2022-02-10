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

"""Tests for the EnvironmentVariable substitution class."""

import os

from launch import LaunchContext
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch.substitutions import SubstitutionFailure

import pytest


def test_this_launch_file_path():
    if 'MY_ENVIRONMENT_VARIABLE' in os.environ:
        del os.environ['MY_ENVIRONMENT_VARIABLE']
    lc = LaunchContext()
    sub1 = EnvironmentVariable('MY_ENVIRONMENT_VARIABLE')
    with pytest.raises(SubstitutionFailure) as ex:
        sub1.perform(lc)
    ex.match("environment variable 'MY_ENVIRONMENT_VARIABLE' does not exist")

    sub2 = EnvironmentVariable('MY_ENVIRONMENT_VARIABLE', default_value='')
    assert '' == sub2.perform(lc)

    sub3 = EnvironmentVariable('MY_ENVIRONMENT_VARIABLE', default_value='MY_DEFAULT_VALUE')
    assert 'MY_DEFAULT_VALUE' == sub3.perform(lc)

    SetEnvironmentVariable('MY_ENVIRONMENT_VARIABLE', 'SOME_VALUE').visit(lc)
    assert 'SOME_VALUE' == sub1.perform(lc)
    assert 'SOME_VALUE' == sub2.perform(lc)
    assert 'SOME_VALUE' == sub3.perform(lc)
    del os.environ['MY_ENVIRONMENT_VARIABLE']
