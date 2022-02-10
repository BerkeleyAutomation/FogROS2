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

"""
A module providing exit code assertions.

To prevent pytest from rewriting this module assertions, please PYTEST_DONT_REWRITE.
See https://docs.pytest.org/en/latest/assert.html#disabling-assert-rewriting for
further reference.
"""


import os

from ..util import resolveProcesses

EXIT_OK = 0
EXIT_SIGINT = 130
EXIT_SIGQUIT = 131
EXIT_SIGKILL = 137
EXIT_SIGSEGV = 139

# TODO(hidmic): Drop when SIGINT is fixed on Windows
EXIT_FORCED = 1


def assertExitCodes(proc_info,
                    allowable_exit_codes=[EXIT_OK] if os.name != 'nt' else [EXIT_OK, EXIT_FORCED],
                    process=None,  # By default, checks all processes
                    cmd_args=None,
                    *,
                    strict_proc_matching=True):
    """
    Check the exit codes of the processes under test.

    :param iterable proc_info: A list of proc_info objects provided by the test framework to be
    checked
    """
    # Sanity check that the user didn't pass in something crazy for allowable exit codes
    for code in allowable_exit_codes:
        assert isinstance(code, int), 'Provided exit code {} is not an int'.format(code)

    to_check = resolveProcesses(
        info_obj=proc_info,
        process=process,
        cmd_args=cmd_args,
        strict_proc_matching=strict_proc_matching
    )

    for info in [proc_info[item] for item in to_check]:
        assert info.returncode in allowable_exit_codes, 'Proc {} exited with code {}'.format(
            info.process_name,
            info.returncode
        )
