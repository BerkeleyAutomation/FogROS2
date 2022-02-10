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

"""Module for the GTest action."""

from typing import List
from typing import Optional
from typing import Union

from launch import LaunchContext
from launch import SomeActionsType
from launch import SomeSubstitutionsType
from launch.action import Action
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.event import Event
from launch.event_handlers import OnProcessExit


class Test(ExecuteProcess):
    """Action that runs a test."""

    def __init__(
        self,
        *,
        timeout: Optional[Union[float, SomeSubstitutionsType]] = None,
        kill_timeout: Union[float, SomeSubstitutionsType] = 5.0,
        **kwargs
    ) -> None:
        """
        Create a Test action.

        Many arguments are passed to :class:`launch.ExecuteProcess`, so
        see the documentation for the class for additional details.

        :param: timeout the test will be killed after timeout seconds.
        """
        super().__init__(**kwargs)
        self.__timeout = timeout
        self.__timer = None

    @property
    def timeout(self):
        """Getter for timeout."""
        return self.__timeout

    def __on_process_exit(self, event: Event, context: LaunchContext) -> Optional[SomeActionsType]:
        """On shutdown event."""
        if self.__timer:
            self.__timer.cancel()

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """
        Execute the action.

        Delegated to :meth:`launch.actions.ExecuteProcess.execute`.
        """
        actions = super().execute(context)
        if not self.__timeout:
            return actions
        self.__timer = TimerAction(
            period=self.__timeout,
            actions=[OpaqueFunction(
                function=self._shutdown_process,
                kwargs={'send_sigint': True})])
        on_process_exit_event = OnProcessExit(
                on_exit=self.__on_process_exit,
                target_action=self
            )
        context.register_event_handler(on_process_exit_event)
        if not actions:
            return [self.__timer]

        return actions.append(self.__timer)
