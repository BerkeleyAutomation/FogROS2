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

"""Tests for the LaunchConfigurationEquals condition class."""

from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import TextSubstitution


def test_launch_configuration_equals():
    """Test LaunchConfigurationEquals class."""
    class MockLaunchContext:

        def perform_substitution(self, substitution):
            return substitution.perform(self)

        @property
        def launch_configurations(self):
            return {
                'foo': 'foo_value',
                'bar': 'bar_value',
                'empty': '',
            }

    lc = MockLaunchContext()
    test_cases = [
        ('foo', 'foo_value', True),
        ('bar', 'bar_value', True),
        ('bar', 'foo_value', False),
        ('bar', None, False),
        ('empty', '', True),
        ('empty', 'foo_value', False),
        ('empty', None, False),
        ('baz', None, True),
        ('baz', 'foo_value', False),
    ]

    for name, value, expected in test_cases:
        assert LaunchConfigurationEquals(
            name,
            [TextSubstitution(text=value)] if value is not None else None
        ).evaluate(lc) is expected
