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

"""Module for ExecutionComplete event."""

from ..action import Action
from ..event import Event


class ExecutionComplete(Event):
    """Event that is emitted on action execution completion."""

    name = 'launch.events.ExecutionComplete'

    def __init__(self, *, action: Action) -> None:
        """Create an ExecutionComplete event."""
        self.__action = action

    @property
    def action(self):
        """Getter for action."""
        return self.__action
