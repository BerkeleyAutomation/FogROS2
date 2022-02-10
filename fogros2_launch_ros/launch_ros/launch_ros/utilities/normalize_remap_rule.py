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

"""Module for normalize_remap_rule utility."""

from launch.utilities import ensure_argument_type
from launch.utilities import normalize_to_list_of_substitutions

from ..remap_rule_type import RemapRule
from ..remap_rule_type import RemapRules
from ..remap_rule_type import SomeRemapRule
from ..remap_rule_type import SomeRemapRules


def normalize_remap_rule(remap_rule: SomeRemapRule) -> RemapRule:
    """Normalize a remap rule to a specific type."""
    ensure_argument_type(remap_rule, (tuple), 'remap_rule')
    if len(remap_rule) != 2:
        raise TypeError(
            'remap_rule must be a tuple of length 2, got length {}'.format(len(remap_rule)))
    from_sub = tuple(normalize_to_list_of_substitutions(remap_rule[0]))
    to_sub = tuple(normalize_to_list_of_substitutions(remap_rule[1]))
    return from_sub, to_sub


def normalize_remap_rules(remap_rules: SomeRemapRules) -> RemapRules:
    """Normalize multiple remap rules."""
    yield from map(normalize_remap_rule, remap_rules)
