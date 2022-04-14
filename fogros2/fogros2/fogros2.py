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

from .aws import AWS
from .scp import SCP_Client
from .vpn import VPN


def start():
    launch_new_instance = True
    if launch_new_instance:
        aws_instance = AWS(region="us-west-1", store_key_path="/home/root/fog_ws/", ec2_instance_type="t2.medium")
        aws_instance.create()
        ip = aws_instance.get_ip()
        key_path = aws_instance.get_ssh_key_path()
        print(ip, key_path)
        vpn = VPN(ip)
        vpn.make_wireguard_keypair()
    else:
        ip = "13.52.249.171"
        key_path = os.path.join(os.getenv("COLCON_PREFIX_PATH"), "..", "FogROSKEY905.pem")
        # Note that we don't need to make new keypair if we keep the old ones
        vpn = VPN(ip)

    scp = SCP_Client(ip, key_path)
    scp.connect()

    vpn.start()


def main():
    import socket

    HOST = "localhost"
    PORT = 65432
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print("Connected by", addr)
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                conn.sendall(b"ACK")
                start()


if __name__ == "__main__":
    main()
