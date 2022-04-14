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

import logging


class BashBuilder:
    def __init__(self, cmd_save_path="/opt/ros_ws/cmd.sh"):
        self.cmd_save_path = cmd_save_path
        self.command = ""
        self.logger = logging.getLogger(__name__)

    def save(self):
        with open(self.cmd_save_path, "w+") as f:
            f.write(self.command)
        self.logger.info(self.command)

    def get(self):
        return self.command

    def append(self, cmd):
        if self.command:
            self.command += " && " + cmd
        else:
            self.command = cmd
