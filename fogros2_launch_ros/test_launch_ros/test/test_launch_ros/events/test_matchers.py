# Copyright 2021 Open Source Robotics Foundation, Inc.
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

"""Tests for the matchers module."""

from launch_ros.events import matches_node_name


class MockNode:
    """Mock node action."""

    def __init__(self, node_name):
        self.node_name = node_name


def test_matches_node_name():
    """Test the matches_node_name function."""
    matcher = matches_node_name('/foo')

    assert matcher(MockNode('/foo'))
    assert not matcher(MockNode('/foo/bar'))

    # Without '/' prefix
    matcher = matches_node_name('foo')

    assert matcher(MockNode('/foo'))
    assert not matcher(MockNode('/foo/bar'))
