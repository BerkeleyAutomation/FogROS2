# Copyright 2020 Southwest Research Institute, All Rights Reserved.
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
#
# DISTRIBUTION A. Approved for public release; distribution unlimited.
# OPSEC #4584.

"""Module for a description of an Executable."""

import os
import re
import shlex
import threading
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Tuple

from ..action import Action
from ..launch_context import LaunchContext
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..substitutions import LaunchConfiguration
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions

_executable_process_counter_lock = threading.Lock()
_executable_process_counter = 0  # in Python3, this number is unbounded (no rollover)


class Executable:
    """Describes an executable (usually a single process) which may be run by the launch system."""

    def __init__(
        self, *,
        cmd: Iterable[SomeSubstitutionsType],
        prefix: Optional[SomeSubstitutionsType] = None,
        name: Optional[SomeSubstitutionsType] = None,
        cwd: Optional[SomeSubstitutionsType] = None,
        env: Optional[Dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        additional_env: Optional[Dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
    ) -> None:
        """
        Initialize an Executable description.

        :param cmd: A list where the first item is the executable and the rest are
            arguments to the executable, each item may be a string or a list of strings
            and Substitutions to be resolved at runtime
        :param prefix: a set of commands/arguments to preceed the cmd, used for
            things like gdb/valgrind and defaults to the LaunchConfiguration
            called 'launch-prefix'. Note that a non-default prefix provided in
            a launch file will override the prefix provided via the `launch-prefix`
            launch configuration regardless of whether the `launch-prefix-filter` launch
            configuration is provided.
        :param name: The label used to represent the process, as a string or a Substitution
            to be resolved at runtime, defaults to the basename of the executable
        :param cwd: The directory in which to run the executable
        :param env: Dictionary of environment variables to be used, starting from a clean
            environment. If None, the current environment is used.
        :param additional_env: Dictionary of environment variables to be added. If env was
            None, they are added to the current environment. If not, env is updated with
            additional_env.
        :param arguments: list of extra arguments for the executable
        """
        self.__cmd = [normalize_to_list_of_substitutions(x) for x in cmd]
        self.__cmd += ([] if arguments is None
                       else [normalize_to_list_of_substitutions(x) for x in arguments])
        self.__prefix = normalize_to_list_of_substitutions(
            LaunchConfiguration('launch-prefix', default='') if prefix is None else prefix
        )
        self.__prefix_filter = normalize_to_list_of_substitutions(
            LaunchConfiguration('launch-prefix-filter', default='')
        ) if prefix is None else None
        self.__name = name if name is None else normalize_to_list_of_substitutions(name)
        self.__cwd = cwd if cwd is None else normalize_to_list_of_substitutions(cwd)
        self.__env = None  # type: Optional[List[Tuple[List[Substitution], List[Substitution]]]]
        if env is not None:
            self.__env = []
            for key, value in env.items():
                self.__env.append((
                    normalize_to_list_of_substitutions(key),
                    normalize_to_list_of_substitutions(value)))
        self.__additional_env: Optional[List[Tuple[List[Substitution], List[Substitution]]]] = None
        if additional_env is not None:
            self.__additional_env = []
            for key, value in additional_env.items():
                self.__additional_env.append((
                    normalize_to_list_of_substitutions(key),
                    normalize_to_list_of_substitutions(value)))
        self.__arguments = arguments
        self.__final_cmd = None
        self.__final_cwd = None
        self.__final_env = None
        self.__final_name = None

    @property
    def name(self):
        """Getter for name."""
        return self.__name

    @property
    def prefix(self):
        """Getter for prefix."""
        return self.__prefix

    @property
    def cmd(self):
        """Getter for cmd."""
        return self.__cmd

    @property
    def cwd(self):
        """Getter for cwd."""
        return self.__cwd

    @property
    def env(self):
        """Getter for env."""
        return self.__env

    @property
    def additional_env(self):
        """Getter for additional_env."""
        return self.__additional_env

    @property
    def arguments(self):
        """Getter for arguments."""
        return self.__arguments

    @property
    def final_name(self):
        """Getter for final_name."""
        return self.__final_name

    @property
    def final_cmd(self):
        """Getter for final_cmd."""
        return self.__final_cmd

    @property
    def final_cwd(self):
        """Getter for cwd."""
        return self.__final_cwd

    @property
    def final_env(self):
        """Getter for final_env."""
        return self.__final_env

    def prepare(self, context: LaunchContext, action: Action):
        """
        Prepare an executable description for execution in a given environment.

        This does the following:
        - performs substitutions on various properties

        Note that 'action' is not used at this level; it is provided for use
        by subclasses which may override this method.
        """
        # expand substitutions in arguments to async_execute_process()
        cmd = [perform_substitutions(context, x) for x in self.__cmd]
        # Perform filtering for prefix application
        should_apply_prefix = True  # by default
        if self.__prefix_filter is not None:  # no prefix given on construction
            prefix_filter = perform_substitutions(context, self.__prefix_filter)
            # Apply if filter regex matches (empty regex matches all strings)
            should_apply_prefix = re.match(prefix_filter, os.path.basename(cmd[0]))
        if should_apply_prefix:
            cmd = shlex.split(perform_substitutions(context, self.__prefix)) + cmd
        self.__final_cmd = cmd
        name = os.path.basename(cmd[0]) if self.__name is None \
            else perform_substitutions(context, self.__name)
        with _executable_process_counter_lock:
            global _executable_process_counter
            _executable_process_counter += 1
            self.__final_name = f'{name}-{_executable_process_counter}'
        cwd = None
        if self.__cwd is not None:
            cwd = ''.join([context.perform_substitution(x) for x in self.__cwd])
        self.__final_cwd = cwd
        env = None
        if self.__env is not None:
            env = {}
            for key, value in self.__env:
                env[''.join([context.perform_substitution(x) for x in key])] = \
                    ''.join([context.perform_substitution(x) for x in value])
        if self.__additional_env is not None:
            if env is None:
                env = dict(os.environ)
            for key, value in self.__additional_env:
                env[''.join([context.perform_substitution(x) for x in key])] = \
                    ''.join([context.perform_substitution(x) for x in value])
        self.__final_env = env
