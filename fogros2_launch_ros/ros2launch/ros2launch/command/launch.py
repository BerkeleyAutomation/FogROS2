# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import os
import sys

from ament_index_python.packages import PackageNotFoundError
try:
    from argcomplete.completers import FilesCompleter
except ImportError:
    # argcomplete is optional
    pass
try:
    from argcomplete.completers import SuppressCompleter
except ImportError:
    # argcomplete < 1.9.0
    SuppressCompleter = object
from ros2cli.command import CommandExtension
from ros2launch.api import get_share_file_path_from_package
from ros2launch.api import is_launch_file
from ros2launch.api import launch_a_launch_file
from ros2launch.api import LaunchFileNameCompleter
from ros2launch.api import MultipleLaunchFilesError
from ros2launch.api import print_a_launch_file
from ros2launch.api import print_arguments_of_launch_file
from ros2launch.option import get_option_extensions
from ros2pkg.api import package_name_completer


class SuppressCompleterWorkaround(SuppressCompleter):
    """Workaround https://github.com/kislyuk/argcomplete/pull/289 ."""

    def __call__(self, *args, **kwargs):
        """Make SupressCompleter callable by returning no completions."""
        return ()


def package_name_or_launch_file_completer(prefix, parsed_args, **kwargs):
    """Complete package names or paths to launch files."""
    pass_through_kwargs = dict(kwargs)
    pass_through_kwargs['prefix'] = prefix
    pass_through_kwargs['parsed_args'] = parsed_args

    # Complete package names
    completions = list(package_name_completer(**pass_through_kwargs))

    def is_launch_file_or_dir(path):
        return is_launch_file(path) or os.path.isdir(path)

    # Complete paths to launch files
    try:
        completions.extend(filter(is_launch_file_or_dir, FilesCompleter()(**pass_through_kwargs)))
    except NameError:
        # argcomplete is optional
        pass

    return completions


class LaunchCommand(CommandExtension):
    """Run a launch file."""

    def add_arguments(self, parser, cli_name):
        """Add arguments to argparse."""
        parser.add_argument(
            '-n', '--noninteractive', default=not sys.stdin.isatty(), action='store_true',
            help='Run the launch system non-interactively, with no terminal associated')
        parser.add_argument(
            '-d', '--debug', default=False, action='store_true',
            help='Put the launch system in debug mode, provides more verbose output.')
        command_group = parser.add_mutually_exclusive_group()
        command_group.add_argument(
            '-p', '--print', '--print-description', default=False, action='store_true',
            help='Print the launch description to the console without launching it.')
        command_group.add_argument(
            '-s', '--show-args', '--show-arguments', default=False, action='store_true',
            help='Show arguments that may be given to the launch file.')
        parser.add_argument(
            '-a', '--show-all-subprocesses-output', default=False, action='store_true',
            help=("Show all launched subprocesses' output by overriding their output"
                  ' configuration using the OVERRIDE_LAUNCH_PROCESS_OUTPUT envvar.')
        )
        parser.add_argument(
            '--launch-prefix',
            help='Prefix command, which should go before all executables. '
                 'Command must be wrapped in quotes if it contains spaces '
                 "(e.g. --launch-prefix 'xterm -e gdb -ex run --args')."
        )
        parser.add_argument(
            '--launch-prefix-filter',
            help=('Regex pattern for filtering which executables the --launch-prefix is applied '
                  'to by matching the executable name.')
        )
        arg = parser.add_argument(
            'package_name',
            help='Name of the ROS package which contains the launch file')
        arg.completer = package_name_or_launch_file_completer
        arg = parser.add_argument(
            'launch_file_name',
            # TODO(wjwwood) make this not optional when full launch path is supported.
            nargs='?',
            help='Name of the launch file')
        arg.completer = LaunchFileNameCompleter()
        arg = parser.add_argument(
            'launch_arguments',
            nargs='*',
            help="Arguments to the launch file; '<name>:=<value>' (for duplicates, last one wins)")
        arg.completer = SuppressCompleterWorkaround()

        self._option_extensions = get_option_extensions()
        for name in sorted(self._option_extensions.keys()):
            self._option_extensions[name].add_arguments(parser, cli_name)

    def main(self, *, parser, args):
        """Entry point for CLI program."""
        mode = 'single file'
        # Test if first argument is a file, and if not change to pkg
        # file mode.
        if not os.path.isfile(args.package_name):
            mode = 'pkg file'

        path = None
        launch_arguments = []
        if mode == 'single file':
            # TODO(wjwwood): figure out how to have argparse and argcomplete
            # handle this, for now, hidden feature.
            if os.path.exists(args.package_name):
                path = args.package_name
            else:
                return 'No launch file supplied'

            if args.launch_file_name is not None:
                # Since in single file mode, the "launch file" argument is
                # actually part of the launch arguments, if set.
                launch_arguments.append(args.launch_file_name)
        elif mode == 'pkg file':
            try:
                path = get_share_file_path_from_package(
                    package_name=args.package_name,
                    file_name=args.launch_file_name)
            except PackageNotFoundError as exc:
                raise RuntimeError(
                    "Package '{}' not found: {}".format(args.package_name, exc))
            except (FileNotFoundError, MultipleLaunchFilesError) as exc:
                raise RuntimeError(str(exc))
        else:
            raise RuntimeError('unexpected mode')
        launch_arguments.extend(args.launch_arguments)

        if args.launch_prefix is None and args.launch_prefix_filter is not None:
            raise RuntimeError(
                '--launch-prefix must be specified if --launch-prefix-filter is provided')

        if args.show_all_subprocesses_output:
            os.environ['OVERRIDE_LAUNCH_PROCESS_OUTPUT'] = 'both'
        if args.print:
            return print_a_launch_file(launch_file_path=path)
        elif args.show_args:
            return print_arguments_of_launch_file(launch_file_path=path)
        else:
            return launch_a_launch_file(
                launch_file_path=path,
                launch_file_arguments=launch_arguments,
                noninteractive=args.noninteractive,
                args=args,
                option_extensions=self._option_extensions,
                debug=args.debug
            )
