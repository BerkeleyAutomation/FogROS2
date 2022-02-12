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

"""Python package for the ros2 launch api implementation."""

from collections import OrderedDict
import os
from typing import List
from typing import Text
from typing import Tuple

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
import launch
from launch.frontend import Parser
from launch.launch_description_sources import get_launch_description_from_any_launch_file


class MultipleLaunchFilesError(Exception):
    """Exception raised when multiple candidate launch files are found in a package."""

    def __init__(self, msg, paths):
        """Create a MultipleLaunchFilesError."""
        super().__init__(msg)
        self.paths = paths


def get_share_file_path_from_package(*, package_name, file_name):
    """
    Return the full path to a file in the share directory of a package.

    :raises: PackageNotFoundError if package is not found
    :raises: FileNotFoundError if the file is not found in the package
    :raises: MultipleLaunchFilesError if the file is found in multiple places
    """
    package_share_directory = get_package_share_directory(package_name)
    matching_file_paths = []
    for root, dirs, files in os.walk(package_share_directory):
        for name in files:
            if name == file_name:
                matching_file_paths.append(os.path.join(root, name))
    if len(matching_file_paths) == 0:
        raise FileNotFoundError(
            "file '{}' was not found in the share directory of package '{}' which is at '{}'"
            .format(file_name, package_name, package_share_directory))
    if len(matching_file_paths) > 1:
        raise MultipleLaunchFilesError(
            "file '{}' was found more than once in the share directory of package '{}': [{}]"
            .format(
                file_name,
                package_name,
                ', '.join(["'{}'".format(x) for x in matching_file_paths])),
            matching_file_paths)
    return matching_file_paths[0]


def get_launch_file_paths(*, path):
    """Return a list of paths to launch files within a given path."""
    launch_file_paths = []
    for root, dirs, files in os.walk(path):
        for file_name in files:
            file_path = os.path.join(root, file_name)
            if is_launch_file(path=file_path):
                launch_file_paths.append(file_path)
    return launch_file_paths


def is_launch_file(path):
    """Return True if the path is a launch file."""
    return path.endswith(is_launch_file.extensions) and os.path.isfile(path)


is_launch_file.extensions = Parser.get_file_extensions_from_parsers()
is_launch_file.extensions.add('launch.py')
is_launch_file.extensions = tuple(is_launch_file.extensions)


def print_a_launch_file(*, launch_file_path):
    """Print the description of a launch file to the console."""
    launch_description = get_launch_description_from_any_launch_file(launch_file_path)
    print(launch.LaunchIntrospector().format_launch_description(launch_description))


def print_arguments_of_launch_description(*, launch_description):
    """Print the arguments of a LaunchDescription to the console."""
    print("Arguments (pass arguments as '<name>:=<value>'):")
    launch_arguments = launch_description.get_launch_arguments()
    any_conditional_arguments = False
    for argument_action in launch_arguments:
        msg = "\n    '"
        msg += argument_action.name
        msg += "':"
        if argument_action._conditionally_included:
            any_conditional_arguments = True
            msg += '*'
        msg += '\n        '
        msg += argument_action.description
        if argument_action.default_value is not None:
            default_str = ' + '.join([token.describe() for token in argument_action.default_value])
            msg += '\n        (default: {})'.format(default_str)
        print(msg)

    if len(launch_arguments) > 0:
        if any_conditional_arguments:
            print('\n* argument(s) which are only used if specific conditions occur')
    else:
        print('\n  No arguments.')


def print_arguments_of_launch_file(*, launch_file_path):
    """Print the arguments of a launch file to the console."""
    launch_description = get_launch_description_from_any_launch_file(launch_file_path)
    print_arguments_of_launch_description(launch_description=launch_description)


def parse_launch_arguments(launch_arguments: List[Text]) -> List[Tuple[Text, Text]]:
    """Parse the given launch arguments from the command line, into list of tuples for launch."""
    parsed_launch_arguments = OrderedDict()  # type: ignore
    for argument in launch_arguments:
        count = argument.count(':=')
        if count == 0 or argument.startswith(':=') or (count == 1 and argument.endswith(':=')):
            raise RuntimeError(
                "malformed launch argument '{}', expected format '<name>:=<value>'"
                .format(argument))
        name, value = argument.split(':=', maxsplit=1)
        parsed_launch_arguments[name] = value  # last one wins is intentional
    return parsed_launch_arguments.items()


def launch_a_launch_file(
    *,
    launch_file_path,
    launch_file_arguments,
    noninteractive=False,
    args=None,
    option_extensions={},
    debug=False
):
    """Launch a given launch file (by path) and pass it the given launch file arguments."""
    for name in sorted(option_extensions.keys()):
        option_extensions[name].prestart(args)

    # If 'launch-prefix' launch file argument is also provided in the user input,
    # the 'launch-prefix' option is applied since the last duplicate argument is used
    if args and args.launch_prefix:
        launch_file_arguments.append(f'launch-prefix:={args.launch_prefix}')

    if args and args.launch_prefix_filter:
        launch_file_arguments.append(f'launch-prefix-filter:={args.launch_prefix_filter}')

    launch_service = launch.LaunchService(
        argv=launch_file_arguments,
        noninteractive=noninteractive,
        debug=debug)

    parsed_launch_arguments = parse_launch_arguments(launch_file_arguments)
    # Include the user provided launch file using IncludeLaunchDescription so that the
    # location of the current launch file is set.
    launch_description = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.AnyLaunchDescriptionSource(
                launch_file_path
            ),
            launch_arguments=parsed_launch_arguments,
        ),
    ])
    for name in sorted(option_extensions.keys()):
        result = option_extensions[name].prelaunch(
            launch_description,
            args
        )
        launch_description = result[0]
    launch_service.include_launch_description(launch_description)
    ret = launch_service.run()
    for name in sorted(option_extensions.keys()):
        option_extensions[name].postlaunch(ret, args)
    return ret


class LaunchFileNameCompleter:
    """Callable returning a list of launch file names within a package's share directory."""

    def __init__(self, *, package_name_key=None):
        """Create LaunchFileNameCompleter."""
        self.package_name_key = 'package_name' if package_name_key is None else package_name_key

    def __call__(self, prefix, parsed_args, **kwargs):
        """Return a list of file names for launch files found within the package."""
        package_name = getattr(parsed_args, self.package_name_key)
        try:
            package_share_directory = get_package_share_directory(package_name)
            paths = get_launch_file_paths(path=package_share_directory)
        except PackageNotFoundError:
            return []
        return [os.path.basename(p) for p in paths]
