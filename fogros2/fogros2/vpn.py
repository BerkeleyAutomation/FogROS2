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

import os

import wgconfig
import wgconfig.wgexec as wgexec


class VPN:
    def __init__(
        self,
        cloud_key_path="/tmp/fogros-cloud.conf",
        robot_key_path="/tmp/fogros-local.conf",
    ):
        self.cloud_key_path = cloud_key_path
        self.robot_key_path = robot_key_path

        self.cloud_name_to_pub_key_path = dict()
        self.cloud_name_to_priv_key_path = dict()

        self.robot_private_key = wgexec.generate_privatekey()
        self.robot_public_key = wgexec.get_publickey(self.robot_private_key)

    def generate_key_pairs(self, machines):
        """
        Create key pair for each machine.

        @param machines: List<machine>
        """
        for machine in machines:
            name = machine.name
            cloud_private_key = wgexec.generate_privatekey()
            self.cloud_name_to_priv_key_path[name] = cloud_private_key
            cloud_public_key = wgexec.get_publickey(cloud_private_key)
            self.cloud_name_to_pub_key_path[name] = cloud_public_key

    def generate_wg_config_files(self, machines):
        self.generate_key_pairs(machines)

        # generate cloud configs
        counter = 2  # start the static ip addr counter from 2
        for machine in machines:
            name = machine.name
            machine_config_pwd = self.cloud_key_path + name
            machine_priv_key = self.cloud_name_to_priv_key_path[name]
            aws_config = wgconfig.WGConfig(machine_config_pwd)
            aws_config.add_attr(None, "PrivateKey", machine_priv_key)
            aws_config.add_attr(None, "ListenPort", 51820)
            aws_config.add_attr(None, "Address", f"10.0.0.{counter:d}/24")
            aws_config.add_peer(self.robot_public_key, "# fogROS Robot")
            aws_config.add_attr(
                self.robot_public_key, "AllowedIPs", "10.0.0.1/32"
            )
            aws_config.write_file()
            counter += 1

        # generate robot configs
        robot_config = wgconfig.WGConfig(self.robot_key_path)
        robot_config.add_attr(None, "PrivateKey", self.robot_private_key)
        robot_config.add_attr(None, "ListenPort", 51820)
        robot_config.add_attr(None, "Address", "10.0.0.1/24")
        for machine in machines:
            name = machine.name
            ip = machine.ip
            cloud_pub_key = self.cloud_name_to_pub_key_path[name]
            robot_config.add_peer(cloud_pub_key, f"# AWS{name}")
            robot_config.add_attr(cloud_pub_key, "AllowedIPs", "10.0.0.2/32")
            robot_config.add_attr(cloud_pub_key, "Endpoint", f"{ip}:51820")
            robot_config.add_attr(cloud_pub_key, "PersistentKeepalive", 3)
        robot_config.write_file()

    def start_robot_vpn(self):
        # Copy /tmp/fogros-local.conf to /etc/wireguard/wg0.conf locally.
        # TODO: This needs root. Move this to a separate script with setuid.
        os.system("sudo cp /tmp/fogros-local.conf /etc/wireguard/wg0.conf")
        os.system("sudo chmod 600 /etc/wireguard/wg0.conf")
        os.system("sudo wg-quick down wg0")
        os.system("sudo wg-quick up wg0")
