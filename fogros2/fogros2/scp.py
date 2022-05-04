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

import select
import sys
from time import sleep

import paramiko
from rclpy import logging
from scp import SCPClient

# ec2 console coloring
CRED = "\033[91m"
CEND = "\033[0m"


class SCP_Client:
    def __init__(self, ip, ssh_key_path):
        self.ip = ip
        self.ssh_key = paramiko.RSAKey.from_private_key_file(ssh_key_path)
        self.ssh_client = paramiko.SSHClient()
        self.logger = logging.get_logger(__name__)

    def connect(self):
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        connected = False
        while not connected:
            try:
                self.ssh_client.connect(
                    hostname=self.ip,
                    username="ubuntu",
                    pkey=self.ssh_key,
                    look_for_keys=False,
                )
                connected = True
            # TODO: Handle specific exceptions differently?
            # See https://docs.paramiko.org/en/stable/api/client.html
            except Exception as e:
                self.logger.warn(f"{e}, retrying...")
                sleep(1)
        self.logger.info("SCP connected!")

    def send_file(self, src_path, dst_path):
        with SCPClient(self.ssh_client.get_transport()) as scp:
            scp.put(src_path, dst_path)

    def execute_cmd(self, cmd):
        timeout = 300
        stdin, stdout, stderr = self.ssh_client.exec_command(cmd, get_pty=False)
        # for line in iter(stdout.readline, ""):
        #     print("ec2 (out): " + CRED + line + CEND, end="")
        # See https://stackoverflow.com/a/32758464
        ch = stdout.channel  # channel shared by stdin, stdout, stderr
        stdin.close()  # we don't need stdin
        ch.shutdown_write()  # not going to write
        while not ch.closed:
            readq, _, _ = select.select([ch], [], [], timeout)
            for c in readq:
                if c.recv_ready():
                    sys.stdout.buffer.write(c.recv(len(c.in_buffer)))
                    sys.stdout.buffer.flush()
                if c.recv_stderr_ready():
                    sys.stderr.buffer.write(c.recv_stderr(len(c.in_stderr_buffer)))
                    sys.stderr.buffer.flush()
        stdout.close()
        stderr.close()
        ch.recv_exit_status()
