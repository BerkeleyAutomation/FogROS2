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

from ros2cli.plugin_system import instantiate_extensions
from ros2cli.plugin_system import satisfies_version


class OptionExtension:
    """
    The extension point for ros2 launch 'option' extensions.

    The following properties must be defined:
    * `NAME` (will be set to the entry point name)

    The following methods are optional:
    * `add_arguments`
    * `prestart`
    * `prelaunch`
    * `postlaunch`

    """

    NAME = None
    EXTENSION_POINT_VERSION = '0.1'

    def __init__(self):
        super(OptionExtension, self).__init__()
        satisfies_version(self.EXTENSION_POINT_VERSION, '^0.1')

    def add_arguments(self, parser, cli_name, *, argv=None):
        """Add arguments to the argparse parser."""
        return

    def prestart(self, args):
        """
        Perform actions the prior to the LaunchService being started.

        This method does not need to return anything.
        """
        return

    def prelaunch(self, launch_description, args):
        """
        Perform actions prior to the launch process running.

        This executes after creation and setup of the launch process, but
        before it is executed.

        This method must return a tuple where the first element is a
        LaunchDescription, which will replace the LaunchDescription that was
        passed in and will be launched by the launch service.
        """
        return (launch_description,)

    def postlaunch(self, launch_return_code, args):
        """
        Perform cleanup actions after the launch process finishes.

        This method does not need to return anything.
        """
        return


def get_option_extensions(exclude_names=None):
    extensions = instantiate_extensions(
        'ros2launch.option',
        exclude_names=exclude_names)
    for name, extension in extensions.items():
        extension.NAME = name
    return extensions
