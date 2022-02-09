

import logging

class BashBuilder:
    def __init__(self, cmd_save_path = "/opt/ros_ws/cmd.sh"):
        self.cmd_save_path = cmd_save_path
        self.command = ""
        self.logger = logging.getLogger(__name__)

    def save(self):
        with open(cmd_save_path, "w+") as f:
            f.write(self.command)
        self.logger.info(self.command)

    def get(self):
        return self.command

    def append(self, cmd):
        if self.command:
            self.command += " && " + cmd
        else:
            self.command = cmd
