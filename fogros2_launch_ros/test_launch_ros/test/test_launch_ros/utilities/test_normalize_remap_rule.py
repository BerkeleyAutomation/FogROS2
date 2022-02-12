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

"""Tests for the Node Action."""

from launch import LaunchContext
from launch.substitutions import TextSubstitution
from launch.utilities import perform_substitutions
from launch_ros.utilities import normalize_remap_rule
from launch_ros.utilities import normalize_remap_rules
import pytest


def test_not_a_tuple():
    with pytest.raises(TypeError):
        normalize_remap_rule(['foo', 'bar'])


def test_wrong_rule_length():
    with pytest.raises(TypeError):
        normalize_remap_rule(('foo'))

    with pytest.raises(TypeError):
        normalize_remap_rule(('foo', 'bar', 'baz'))


def test_plain_text():
    lc = LaunchContext()
    rule = ('foo', 'bar')
    output_rule = normalize_remap_rule(rule)
    assert isinstance(output_rule, tuple)
    assert len(output_rule) == 2
    assert rule[0] == perform_substitutions(lc, output_rule[0])
    assert rule[1] == perform_substitutions(lc, output_rule[1])


def test_mixed_substitutions():
    lc = LaunchContext()
    rule = (('foo', 'bar'), ['bar', TextSubstitution(text='baz')])
    output_rule = normalize_remap_rule(rule)
    assert isinstance(output_rule, tuple)
    assert len(output_rule) == 2
    assert 'foobar' == perform_substitutions(lc, output_rule[0])
    assert 'barbaz' == perform_substitutions(lc, output_rule[1])


def test_multiple_rules():
    lc = LaunchContext()
    rules = [('ping', 'pong'), (('baz', 'foo'), ['bar', TextSubstitution(text='baz')])]
    output_rules = list(normalize_remap_rules(rules))
    assert len(rules) == len(output_rules)
    assert 'ping' == perform_substitutions(lc, output_rules[0][0])
    assert 'pong' == perform_substitutions(lc, output_rules[0][1])
    assert 'bazfoo' == perform_substitutions(lc, output_rules[1][0])
    assert 'barbaz' == perform_substitutions(lc, output_rules[1][1])
