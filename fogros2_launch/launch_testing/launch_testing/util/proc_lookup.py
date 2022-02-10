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

import launch.actions

NO_CMD_ARGS = object()


class NoMatchingProcessException(Exception):
    pass


class _FakeContextException(Exception):
    pass


class _fake_context:

    def __getattr__(self, attr):
        # If anything actually tries to use our fake context, we will raise an exception
        # that we can recognize.
        raise _FakeContextException()


def _proc_to_name_and_args(proc):
    # proc is a launch.actions.ExecuteProcess

    if proc.process_details:
        # process_details are added by the launch system when the process is started.
        # Before the process starts, this will be None
        return '{} {}'.format(
            proc.process_details['name'],
            ' '.join(proc.process_details['cmd'][1:])
        )
    else:
        # Process has not been launched yet, so we don't yet know the process name.
        # In the special case where the process's name is all TextSubstitutions, we
        # can resolve the name.  If there are any other substitutions, we can't give
        # additional info until after the process is launched
        def perform_subs(subs):
            return ''.join([sub.perform(_fake_context()) for sub in subs])
        try:
            cmd = [perform_subs(sub) for sub in proc.cmd]
            return ' '.join(cmd)
        except _FakeContextException:
            return 'Unknown - Process not launched yet'


def _str_name_to_process(info_obj, proc_name, cmd_args):

    def name_match_fn(proc):
        return proc_name in proc.process_details['name']

    def cmd_match_fn(proc):
        if cmd_args is None:
            return True
        elif cmd_args is NO_CMD_ARGS:
            return len(proc.process_details['cmd']) == 1
        else:
            return all(arg in proc.process_details['cmd'][1:] for arg in cmd_args)

    matches = [proc for proc in info_obj.processes()
               if name_match_fn(proc) and cmd_match_fn(proc)]

    return matches


def resolveProcesses(info_obj, *, process=None, cmd_args=None, strict_proc_matching=True):
    """
    Resolve a process name and cmd arguments to one or more launch.actions.ExecuteProcess.

    :param info_obj: a ProcInfoHandler or an IoHandler that contains processes that could match

    :param process:  One or more processes to match.  Pass None to match all processes
    :type process: A launch.actions.ExecuteProcess object to match a specific process, or a string
    to search by process name

    :param cmd_args: Optional.  If the process param is a string, the cmd_args will be used to
    disambiguate processes with the same name.  cmd_args=None will match all command arguments.
    cmd_args=launch_testing.asserts.NO_CMD_ARGS will match a process without command-line
    arguments

    :param strict_proc_matching:  Optional.  If the process param is a string and matches multiple
    processes, strict_proc_matching=True will raise an error

    :returns: A list of one or more matching processes taken from the info_obj.  If no processes
    in info_obj match, a NoMatchingProcessException will be raised.
    """
    if process is None:
        # We want to search all processes
        all_procs = info_obj.processes()
        if len(all_procs) == 0:
            raise NoMatchingProcessException('No data recorded for any process')
        return all_procs

    if isinstance(process, launch.actions.ExecuteProcess):
        # We want to search a specific process
        if process in info_obj.processes():
            return [process]
        else:
            raise NoMatchingProcessException(
                'No data recorded for proc {}'.format(_proc_to_name_and_args(process))
            )

    elif isinstance(process, str):
        # We want to search one (or more) processes that match a particular string.  The 'or more'
        # part is controlled by the strict_proc_matching argument

        # Old versions of this let you pass a string for cmd_args, but there was no way to search
        # for multiple arguments when you did that.  For backwards compatibility, we'll wrap string
        # cmd_args in a list
        if isinstance(cmd_args, str):
            cmd_args = [cmd_args]

        matches = _str_name_to_process(info_obj, process, cmd_args)
        if len(matches) == 0:
            names = ', '.join(sorted(_proc_to_name_and_args(p) for p in info_obj.processes()))

            raise NoMatchingProcessException(
                "Did not find any processes matching name '{}' and args '{}'. Procs: {}".format(
                    process,
                    cmd_args,
                    names
                )
            )

        if strict_proc_matching and len(matches) > 1:
            names = ', '.join(sorted(_proc_to_name_and_args(p) for p in info_obj.processes()))
            raise Exception(
                "Found multiple processes matching name '{}' and cmd_args '{}'. Procs: {}".format(
                    process,
                    cmd_args,
                    names
                )
            )
        return list(matches)

    else:
        # Invalid argument passed for 'process'
        raise TypeError(
            "proc argument must be 'ExecuteProcess' or 'str' not {}".format(type(process))
        )
