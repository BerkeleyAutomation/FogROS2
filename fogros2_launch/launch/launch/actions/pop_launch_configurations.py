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

"""Module for the PopLaunchConfigurations action."""

from ..action import Action
from ..launch_context import LaunchContext


class PopLaunchConfigurations(Action):
    """
    Action that pops the state of launch configurations from a stack.

    The state can be stored initially by pushing onto the stack with the
    :py:class:`launch.actions.PushLaunchConfigurations` action.
    """

    def __init__(self, **kwargs) -> None:
        """Create a PopLaunchConfigurations action."""
        super().__init__(**kwargs)

    def execute(self, context: LaunchContext):
        """Execute the action."""
        context._pop_launch_configurations()
