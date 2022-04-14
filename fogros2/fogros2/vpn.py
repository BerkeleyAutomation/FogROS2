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
        @param machines: List<machine>
        """
        for machine in machines:
            name = machine.get_name()
            cloud_private_key = wgexec.generate_privatekey()
            self.cloud_name_to_priv_key_path[name] = cloud_private_key
            cloud_public_key = wgexec.get_publickey(cloud_private_key)
            self.cloud_name_to_pub_key_path[name] = cloud_public_key

    def generate_wg_config_files(self, machines):
        self.generate_key_pairs(machines)

        # generate cloud configs
        counter = 2  # start the static ip addr counter from 2
        for machine in machines:
            name = machine.get_name()
            machine_config_pwd = self.cloud_key_path + name
            machine_priv_key = self.cloud_name_to_priv_key_path[name]
            aws_config = wgconfig.WGConfig(machine_config_pwd)
            aws_config.add_attr(None, "PrivateKey", machine_priv_key)
            aws_config.add_attr(None, "ListenPort", 51820)
            aws_config.add_attr(None, "Address", "10.0.0." + str(counter) + "/24")
            aws_config.add_peer(self.robot_public_key, "# fogROS Robot")
            aws_config.add_attr(self.robot_public_key, "AllowedIPs", "10.0.0.1/32")
            aws_config.write_file()
            counter += 1

        # generate robot configs
        robot_config = wgconfig.WGConfig(self.robot_key_path)
        robot_config.add_attr(None, "PrivateKey", self.robot_private_key)
        robot_config.add_attr(None, "ListenPort", 51820)
        robot_config.add_attr(None, "Address", "10.0.0.1/24")
        for machine in machines:
            name = machine.get_name()
            ip = machine.get_ip()
            robot_config.add_peer(self.cloud_name_to_pub_key_path[name], "# AWS" + name)
            robot_config.add_attr(self.cloud_name_to_pub_key_path[name], "AllowedIPs", "10.0.0.2/32")
            robot_config.add_attr(self.cloud_name_to_pub_key_path[name], "Endpoint", f"{ip}:51820")
            robot_config.add_attr(self.cloud_name_to_pub_key_path[name], "PersistentKeepalive", 3)
        robot_config.write_file()

    def start_robot_vpn(self):
        # Copy /tmp/fogros-local.conf to /etc/wireguard/wg0.conf locally.
        # TODO: This needs root. Move this to a separate script with setuid.
        os.system("sudo wg-quick down wg0")
        os.system("sudo cp /tmp/fogros-local.conf /etc/wireguard/wg0.conf")
        os.system("sudo chmod 600 /etc/wireguard/wg0.conf")
        os.system("sudo wg-quick up wg0")
