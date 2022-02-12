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

"""Matchers for node names."""

from typing import Callable
from typing import Text

if False:
    # imports here would cause loops, but are only used as forward-references for type-checking
    from ..actions import Node  # noqa: F401


def matches_node_name(node_name: Text) -> Callable[['Node'], bool]:
    """Return a matcher which matches based on the name of the node itself."""
    if not node_name.startswith('/'):
        node_name = f'/{node_name}'
    return lambda action: action.node_name == node_name
