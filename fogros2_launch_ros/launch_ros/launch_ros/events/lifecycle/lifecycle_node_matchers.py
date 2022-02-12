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

"""Module for standard "lifecycle_node_matchers", which are used with targeted lifecycle events."""

from typing import Callable
from typing import Text
import warnings

from ..matchers import matches_node_name as _matches_node_name

if False:
    # imports here would cause loops, but are only used as forward-references for type-checking
    from ...actions import LifecycleNode  # noqa: F401


def matches_node_name(node_name: Text) -> Callable[['LifecycleNode'], bool]:
    """Return a matcher which matches based on the name of the node itself."""
    warnings.warn(
        "'matches_node_name' has been moved into the 'launch.events' module and will be removed "
        "from the 'lifecycle' module in the future"
    )
    return _matches_node_name(node_name)
