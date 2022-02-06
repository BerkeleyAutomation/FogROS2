

import logging
class CommandBuilder:
    def __init__(self, cmd_save_path = "/opt/ros_ws/cmd.sh"):
        self.cmd_save_path = cmd_save_path
        self.command = ""
        self.logger = logging.getLogger(__name__)

    def save(self):
        with open(cmd_save_path, "w+") as f:
            f.write(self.command)
        self.logger.info(self.command)
