# Copyright 2019 Apex.AI, Inc.
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

from typing import List
from typing import Optional
from typing import Text
from typing import Tuple

from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessIO
from launch.some_actions_type import SomeActionsType


class StdoutReadyListener(OnProcessIO):
    """
    Part of a LaunchDescription that can wait for processes to signal ready with stdout.

    Some processes signal that they're ready by printing a message to stdout.  This listener
    can be added to a launch description to wait for a particular process to output a particular
    bit of text
    """

    def __init__(
        self,
        *,
        target_action: Optional[ExecuteProcess] = None,
        ready_txt: Text,
        actions: [SomeActionsType]
    ):
        self.__ready_txt = ready_txt
        self.__actions = actions

        super().__init__(
            target_action=target_action,
            on_stdout=self.__on_stdout
        )

    def __on_stdout(self, process_io):
        if self.__ready_txt in process_io.text.decode():
            return self.__actions

    def describe(self) -> Tuple[Text, List[SomeActionsType]]:
        """Return the description list with 0 as a string, and then LaunchDescriptionEntity's."""
        description = super().describe()[0]
        return (
            description,
            self.__actions
        )
