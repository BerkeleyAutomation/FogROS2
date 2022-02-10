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
