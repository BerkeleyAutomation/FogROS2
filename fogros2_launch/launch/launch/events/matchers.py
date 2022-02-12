# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Module for standard "matchers", which are used with Events."""

from typing import Callable

from ..action import Action


def matches_action(target_action: Action) -> Callable[[Action], bool]:
    """Return a matcher which matches based on an exact given ExecuteProcess action."""
    return lambda action: action == target_action
