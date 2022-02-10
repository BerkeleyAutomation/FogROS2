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

"""Tests for the GTest Action."""

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import EmitEvent
from launch.events import Shutdown

from launch_testing.actions import GTest


def launch_gtest(test_path):
    """Launch a gtest."""
    ld = LaunchDescription([
        GTest(path=str(test_path), timeout=5.0, on_exit=[EmitEvent(event=Shutdown())])
    ])
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()


def test_gtest_locking():
    """Test running a locking gtest with timeout."""
    launch_gtest('locking')


def test_gtest_non_locking():
    """Test running a non-locking gtest with timeout."""
    launch_gtest('dummy')
