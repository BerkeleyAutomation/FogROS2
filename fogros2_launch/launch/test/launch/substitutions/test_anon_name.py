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

"""Tests for the AnonName substitution class."""

import re

from launch import LaunchContext
from launch.substitutions import AnonName


def test_this_launch_file_path():
    lc = LaunchContext()
    sub1 = AnonName('foo')
    result1 = sub1.perform(lc)
    assert result1
    assert re.match(r'foo_[a-zA-Z0-9_]+_[0-9]+_[0-9]+', result1)

    sub2 = AnonName('foo')
    result2 = sub2.perform(lc)
    assert result2
    assert result1 == result2

    sub3 = AnonName('car')
    result3 = sub3.perform(lc)
    assert result3
    assert re.match(r'car_[a-zA-Z0-9_]+_[0-9]+_[0-9]+', result3)
