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

"""Module for the ExecuteProcess action."""

import shlex
import threading
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text

from .execute_local import ExecuteLocal

from ..descriptions import Executable
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitutions import TextSubstitution

_global_process_counter_lock = threading.Lock()
_global_process_counter = 0  # in Python3, this number is unbounded (no rollover)


@expose_action('executable')
class ExecuteProcess(ExecuteLocal):
    """
    Action that begins executing a process and sets up event handlers for it.

    Simple example:

        .. doctest::

            >>> ld = LaunchDescription([
            ...     ExecuteProcess(
            ...         cmd=['ls', '-las'],
            ...         name='my_ls_process',  # this is optional
            ...         output='both',
            ...     ),
            ... ])

        .. code-block:: xml

            <launch>
                <executable cmd="ls -las" name="my_ls_process" output="both"/>
            </launch>

    Substitutions in the command:

        .. doctest::

            >>> ld = LaunchDescription([
            ...     DeclareLaunchArgument(name='file_path', description='file path to cat'),
            ...     ExecuteProcess(
            ...         # each item of the command arguments' list can be:
            ...         # a string ('cat'),
            ...         # a substitution (`LaunchConfiguration('file_path')`),
            ...         # or a list of string/substitutions
            ...         # (`[LaunchConfiguration('directory'), '/file.txt']`)
            ...         cmd=['cat', LaunchConfiguration('file_path')],
            ...     ),
            ... ])

        .. code-block:: xml

            <launch>
                <arg name="file_path" description="path of the file to cat"/>
                <executable cmd="cat $(var file_path)"/>
            </launch>

    Optional cli argument:

        .. doctest::

            >>> ld = LaunchDescription([
            ...     DeclareLaunchArgument(name='open_gui', default_value='False'),
            ...     ExecuteProcess(
            ...         cmd=['my_cmd', '--open-gui'],
            ...         condition=IfCondition(LaunchConfiguration('open_gui')),
            ...     ),
            ...     ExecuteProcess(
            ...         cmd=['my_cmd'],
            ...         condition=UnlessCondition(LaunchConfiguration('open_gui')),
            ...     ),
            ... ])

        .. code-block:: xml

            <launch>
                <arg name="open_gui" description="when truthy, the gui will be opened"/>
                <executable cmd="my_cmd --open-gui" if="$(var open_gui)"/>
                <executable cmd="my_cmd" unless="$(var open_gui)"/>
            </launch>

    Environment variables:

        .. doctest::

            >>> ld = LaunchDescription([
            ...     ExecuteProcess(
            ...         cmd=['my_cmd'],
            ...         additional_env={'env_variable': 'env_var_value'},
            ...     ),
            ... ])

        .. code-block:: xml

            <launch>
                <executable cmd="my_cmd">
                    <env name="env_variable" value="env_var_value"/>
                </executable>
            </launch>
    """

    def __init__(
            self,
            *,
            cmd: Iterable[SomeSubstitutionsType],
            prefix: Optional[SomeSubstitutionsType] = None,
            name: Optional[SomeSubstitutionsType] = None,
            cwd: Optional[SomeSubstitutionsType] = None,
            env: Optional[Dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
            additional_env: Optional[Dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
            **kwargs
    ) -> None:
        """
        Construct an ExecuteProcess action.

        Many arguments are passed eventually to :class:`subprocess.Popen`, so
        see the documentation for the class for additional details.

        This action, once executed, registers several event handlers for
        various process related events and will also emit events asynchronously
        when certain events related to the process occur.

        Handled events include:

        - launch.events.process.ShutdownProcess:

          - begins standard shutdown procedure for a running executable

        - launch.events.process.SignalProcess:

          - passes the signal provided by the event to the running process

        - launch.events.process.ProcessStdin:

          - passes the text provided by the event to the stdin of the process

        - launch.events.Shutdown:

          - same as ShutdownProcess

        Emitted events include:

        - launch.events.process.ProcessStarted:

            - emitted when the process starts

        - launch.events.process.ProcessExited:

            - emitted when the process exits
            - event contains return code

        - launch.events.process.ProcessStdout and launch.events.process.ProcessStderr:

            - emitted when the process produces data on either the stdout or stderr pipes
            - event contains the data from the pipe

        Note that output is just stored in this class and has to be properly
        implemented by the event handlers for the process's ProcessIO events.

        :param: cmd a list where the first item is the executable and the rest
            are arguments to the executable, each item may be a string or a
            list of strings and Substitutions to be resolved at runtime
        :param: cwd the directory in which to run the executable
        :param: name the label used to represent the process, as a string or a
            Substitution to be resolved at runtime, defaults to the basename of
            the executable
        :param: env dictionary of environment variables to be used, starting from
            a clean environment. If 'None', the current environment is used.
        :param: additional_env dictionary of environment variables to be added.
            If 'env' was None, they are added to the current environment.
            If not, 'env' is updated with additional_env.
        :param: shell if True, a shell is used to execute the cmd
        :param: sigterm_timeout time until shutdown should escalate to SIGTERM,
            as a string or a list of strings and Substitutions to be resolved
            at runtime, defaults to the LaunchConfiguration called
            'sigterm_timeout'
        :param: sigkill_timeout time until escalating to SIGKILL after SIGTERM,
            as a string or a list of strings and Substitutions to be resolved
            at runtime, defaults to the LaunchConfiguration called
            'sigkill_timeout'
        :param: emulate_tty emulate a tty (terminal), defaults to False, but can
            be overridden with the LaunchConfiguration called 'emulate_tty',
            the value of which is evaluated as true or false according to
            :py:func:`evaluate_condition_expression`.
            Throws :py:exception:`InvalidConditionExpressionError` if the
            'emulate_tty' configuration does not represent a boolean.
        :param: prefix a set of commands/arguments to preceed the cmd, used for
            things like gdb/valgrind and defaults to the LaunchConfiguration
            called 'launch-prefix'. Note that a non-default prefix provided in
            a launch file will override the prefix provided via the `launch-prefix`
            launch configuration regardless of whether the `launch-prefix-filter` launch
            configuration is provided.
        :param: output configuration for process output logging. Defaults to 'log'
            i.e. log both stdout and stderr to launch main log file and stderr to
            the screen.
            Overridden externally by the OVERRIDE_LAUNCH_PROCESS_OUTPUT envvar value.
            See `launch.logging.get_output_loggers()` documentation for further
            reference on all available options.
        :param: output_format for logging each output line, supporting `str.format()`
            substitutions with the following keys in scope: `line` to reference the raw
            output line and `this` to reference this action instance.
        :param: log_cmd if True, prints the final cmd before executing the
            process, which is useful for debugging when substitutions are
            involved.
        :param: cached_output if `True`, both stdout and stderr will be cached.
            Use get_stdout() and get_stderr() to read the buffered output.
        :param: on_exit list of actions to execute upon process exit.
        :param: respawn if 'True', relaunch the process that abnormally died.
            Defaults to 'False'.
        :param: respawn_delay a delay time to relaunch the died process if respawn is 'True'.
        """
        executable = Executable(cmd=cmd, prefix=prefix, name=name, cwd=cwd, env=env,
                                additional_env=additional_env)
        super().__init__(process_description=executable, **kwargs)

    @classmethod
    def _parse_cmdline(
        cls,
        cmd: Text,
        parser: Parser
    ) -> List[SomeSubstitutionsType]:
        """
        Parse text apt for command line execution.

        :param: cmd a space (' ') delimited command line arguments list.
           All found `TextSubstitution` items are split and added to the
           list again as a `TextSubstitution`.
        :returns: a list of command line arguments.
        """
        result_args = []
        arg = []

        def _append_arg():
            nonlocal arg
            result_args.append(arg)
            arg = []
        for sub in parser.parse_substitution(cmd):
            if isinstance(sub, TextSubstitution):
                tokens = shlex.split(sub.text)
                if not tokens:
                    # Sting with just spaces.
                    # Appending args allow splitting two substitutions
                    # separated by a space.
                    # e.g.: `$(subst1 asd) $(subst2 bsd)` will be two separate arguments.
                    _append_arg()
                    continue
                if sub.text[0].isspace():
                    # Needed for splitting from the previous argument
                    # e.g.: `$(find-exec bsd) asd`
                    # It splits `asd` from the path of `bsd` executable.
                    if len(arg) != 0:
                        _append_arg()
                arg.append(TextSubstitution(text=tokens[0]))
                if len(tokens) > 1:
                    # Needed to split the first argument when more than one token.
                    # e.g. `$(find-pkg-prefix csd)/asd bsd`
                    # will split `$(find-pkg-prefix csd)/asd` from `bsd`.
                    _append_arg()
                    arg.append(TextSubstitution(text=tokens[-1]))
                if len(tokens) > 2:
                    # If there are more than two tokens, just add all the middle tokens to
                    # `result_args`.
                    # e.g. `$(find-pkg-prefix csd)/asd bsd dsd xsd`
                    # 'bsd' 'dsd' will be added.
                    result_args.extend([TextSubstitution(text=x)] for x in tokens[1:-1])
                if sub.text[-1].isspace():
                    # Allows splitting from next argument.
                    # e.g. `exec $(find-some-file)`
                    # Will split `exec` argument from the result of `find-some-file` substitution.
                    _append_arg()
            else:
                arg.append(sub)
        if arg:
            result_args.append(arg)
        return result_args

    @classmethod
    def parse(
        cls,
        entity: Entity,
        parser: Parser,
        ignore: Optional[List[str]] = None
    ):
        """
        Return the `ExecuteProcess` action and kwargs for constructing it.

        :param: ignore A list of arguments that should be ignored while parsing.
            Intended for code reuse in derived classes (e.g.: launch_ros.actions.Node).
        """
        _, kwargs = super().parse(entity, parser)

        if ignore is None:
            ignore = []

        if 'cmd' not in ignore:
            kwargs['cmd'] = cls._parse_cmdline(entity.get_attr('cmd'), parser)

        if 'cwd' not in ignore:
            cwd = entity.get_attr('cwd', optional=True)
            if cwd is not None:
                kwargs['cwd'] = parser.parse_substitution(cwd)

        if 'name' not in ignore:
            name = entity.get_attr('name', optional=True)
            if name is not None:
                kwargs['name'] = parser.parse_substitution(name)

        if 'prefix' not in ignore:
            prefix = entity.get_attr('launch-prefix', optional=True)
            if prefix is not None:
                kwargs['prefix'] = parser.parse_substitution(prefix)

        if 'output' not in ignore:
            output = entity.get_attr('output', optional=True)
            if output is not None:
                kwargs['output'] = parser.parse_substitution(output)

        if 'respawn' not in ignore:
            respawn = entity.get_attr('respawn', data_type=bool, optional=True)
            if respawn is not None:
                kwargs['respawn'] = respawn

        if 'respawn_delay' not in ignore:
            respawn_delay = entity.get_attr('respawn_delay', data_type=float, optional=True)
            if respawn_delay is not None:
                if respawn_delay < 0.0:
                    raise ValueError(
                        'Attribute respawn_delay of Entity node expected to be '
                        'a non-negative value but got `{}`'.format(respawn_delay)
                    )
                kwargs['respawn_delay'] = respawn_delay

        if 'shell' not in ignore:
            shell = entity.get_attr('shell', data_type=bool, optional=True)
            if shell is not None:
                kwargs['shell'] = shell

        if 'additional_env' not in ignore:
            # Conditions won't be allowed in the `env` tag.
            # If that feature is needed, `set_enviroment_variable` and
            # `unset_enviroment_variable` actions should be used.
            env = entity.get_attr('env', data_type=List[Entity], optional=True)
            if env is not None:
                kwargs['additional_env'] = {
                    tuple(parser.parse_substitution(e.get_attr('name'))):
                    parser.parse_substitution(e.get_attr('value')) for e in env
                }
                for e in env:
                    e.assert_entity_completely_parsed()
        return cls, kwargs

    @property
    def name(self):
        """Getter for name."""
        if self.process_description.final_name is not None:
            return self.process_description.final_name
        return self.process_description.name

    @property
    def cmd(self):
        """Getter for cmd."""
        if self.process_description.final_cmd is not None:
            return self.process_description.final_cmd
        return self.process_description.cmd

    @property
    def cwd(self):
        """Getter for cwd."""
        if self.process_description.final_cwd is not None:
            return self.process_description.final_cwd
        return self.process_description.cwd

    @property
    def env(self):
        """Getter for env."""
        if self.process_description.final_env is not None:
            return self.process_description.final_env
        return self.process_description.env

    @property
    def additional_env(self):
        """Getter for additional_env."""
        return self.process_description.additional_env
