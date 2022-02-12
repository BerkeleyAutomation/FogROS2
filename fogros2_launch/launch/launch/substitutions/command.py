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

"""Module for the Command substitution."""

import os
import shlex
import subprocess
from typing import Iterable
from typing import List
from typing import Text

import launch.logging

from .substitution_failure import SubstitutionFailure
from ..frontend.expose import expose_substitution
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution


@expose_substitution('command')
class Command(Substitution):
    """
    Substitution that gets the output of a command as a string.

    If the command is not found or fails a `SubstitutionFailure` error is raised.
    Behavior on stderr output is configurable, see constructor.
    """

    def __init__(
        self,
        command: SomeSubstitutionsType,
        *,
        on_stderr: SomeSubstitutionsType = 'fail'
    ) -> None:
        """
        Construct a command substitution.

        :param command: command to be executed. The substitutions will be performed, and
            `shlex.split` will be used on the result.
        :param on_stderr: specifies what to do when there is stderr output.
            Can be one of:
            - 'fail': raises `SubstitutionFailere` when stderr output is detected.
            - 'ignore': `stderr` output is ignored.
            - 'warn': The `stderr` output is ignored, but a warning is logged if detected.
            - 'capture': The `stderr` output will be captured, together with stdout.
            It can also be a substitution, that results in one of those four options.
        """
        super().__init__()

        from ..utilities import normalize_to_list_of_substitutions  # import here to avoid loop
        self.__command = normalize_to_list_of_substitutions(command)
        self.__on_stderr = normalize_to_list_of_substitutions(on_stderr)

    @classmethod
    def parse(cls, data: Iterable[SomeSubstitutionsType]):
        """Parse `Command` substitution."""
        if len(data) < 1 or len(data) > 2:
            raise ValueError('command substitution expects 1 or 2 arguments')
        kwargs = {'command': data[0]}
        if len(data) == 2:
            kwargs['on_stderr'] = data[1]
        return cls, kwargs

    @property
    def command(self) -> List[Substitution]:
        """Getter for command."""
        return self.__command

    @property
    def on_stderr(self) -> List[Substitution]:
        """Getter for on_stderr."""
        return self.__on_stderr

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'Command({})'.format(' + '.join([sub.describe() for sub in self.command]))

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by running the command and capturing its output."""
        from ..utilities import perform_substitutions  # import here to avoid loop
        command_str = perform_substitutions(context, self.command)
        if os.name != 'nt':
            command = shlex.split(command_str)
        else:
            command = command_str
        on_stderr = perform_substitutions(context, self.on_stderr)
        if on_stderr not in ('fail', 'ignore', 'warn', 'capture'):
            raise SubstitutionFailure(
                "expected 'on_stderr' to be one of: 'fail', 'ignore', 'warn' or 'capture'")
        stderr = subprocess.PIPE
        if on_stderr == 'capture':
            stderr = subprocess.STDOUT

        try:
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=stderr,
                universal_newlines=True)
        except FileNotFoundError as ex:
            raise SubstitutionFailure(f'file not found: {ex}')
        if result.returncode != 0:
            on_error_message = f'executed command failed. Command: {command_str}'
            if result.stderr:
                on_error_message += f'\nCaptured stderr output: {result.stderr}'
            raise SubstitutionFailure(on_error_message)
        if result.stderr:
            on_stderr_message = f'executed command showed stderr output.' \
                f' Command: {command_str}\n' \
                f'Captured stderr output:\n{result.stderr}'
            if on_stderr == 'fail':
                raise SubstitutionFailure(on_stderr_message)
            elif on_stderr == 'warn':
                launch.logging.get_logger().warning(on_stderr_message)

        return result.stdout
