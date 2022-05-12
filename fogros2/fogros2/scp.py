# Copyright 2022 The Regents of the University of California (Regents)
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
#
# Copyright ©2022. The Regents of the University of California (Regents).
# All Rights Reserved. Permission to use, copy, modify, and distribute this
# software and its documentation for educational, research, and not-for-profit
# purposes, without fee and without a signed licensing agreement, is hereby
# granted, provided that the above copyright notice, this paragraph and the
# following two paragraphs appear in all copies, modifications, and
# distributions. Contact The Office of Technology Licensing, UC Berkeley, 2150
# Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-7201,
# otl@berkeley.edu, http://ipira.berkeley.edu/industry-info for commercial
# licensing opportunities. IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE. REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY,
# PROVIDED HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
# MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

import select
import sys
from time import sleep

import paramiko
from rclpy import logging
from scp import SCPClient as SCPClientBase

# ec2 console coloring
CRED = "\033[91m"
CEND = "\033[0m"


class SCPClient:
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
        with SCPClientBase(self.ssh_client.get_transport()) as scp:
            scp.put(src_path, dst_path)

    def execute_cmd(self, cmd):
        timeout = 300
        stdin, stdout, stderr = self.ssh_client.exec_command(
            cmd, get_pty=False
        )

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
                    sys.stderr.buffer.write(
                        c.recv_stderr(len(c.in_stderr_buffer))
                    )
                    sys.stderr.buffer.flush()
        stdout.close()
        stderr.close()
        ch.recv_exit_status()
