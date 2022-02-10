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

from .assert_exit_codes import assertExitCodes
from .assert_exit_codes import EXIT_OK
from .assert_exit_codes import EXIT_SIGINT
from .assert_exit_codes import EXIT_SIGKILL
from .assert_exit_codes import EXIT_SIGQUIT
from .assert_exit_codes import EXIT_SIGSEGV
from .assert_output import assertDefaultStream
from .assert_output import assertInStderr
from .assert_output import assertInStdout
from .assert_output import assertInStream
from .assert_sequential_output import assertSequentialStdout
from .assert_sequential_output import SequentialTextChecker

from ..util.proc_lookup import NO_CMD_ARGS

__all__ = [
    'assertDefaultStream',
    'assertExitCodes',
    'assertInStderr',
    'assertInStdout',
    'assertInStream',
    'assertSequentialStdout',

    'SequentialTextChecker',

    'NO_CMD_ARGS',

    'EXIT_OK',
    'EXIT_SIGINT',
    'EXIT_SIGKILL',
    'EXIT_SIGQUIT',
    'EXIT_SIGSEGV',
]
