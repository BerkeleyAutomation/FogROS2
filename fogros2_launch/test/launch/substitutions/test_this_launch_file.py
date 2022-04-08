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

"""Tests for the ThisLaunchFile substitution class."""

from launch import LaunchContext
from launch.substitutions import SubstitutionFailure
from launch.substitutions import ThisLaunchFile

import pytest


def test_this_launch_file_path():
    """Test the constructors for ThisLaunchFileDir class."""
    ThisLaunchFile()


def test_this_launch_file_path_methods():
    """Test the methods of the ThisLaunchFileDir class."""
    tlf = ThisLaunchFile()

    lc = LaunchContext()
    with pytest.raises(SubstitutionFailure):
        tlf.perform(lc)
    lc.extend_locals({'current_launch_file_path': 'foo'})
    assert tlf.perform(lc) == 'foo'
