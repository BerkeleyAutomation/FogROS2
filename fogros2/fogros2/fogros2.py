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

from .aws import AWS
from .scp import SCP_Client
from .vpn import VPN


def start():
    launch_new_instance = True
    if launch_new_instance:
        aws_instance = AWS(
            region="us-west-1",
            store_key_path="/home/root/fog_ws/",
            ec2_instance_type="t2.medium",
        )
        aws_instance.create()
        ip = aws_instance.get_ip()
        key_path = aws_instance.get_ssh_key_path()
        print(ip, key_path)
        vpn = VPN(ip)
        vpn.make_wireguard_keypair()
    else:
        ip = "13.52.249.171"
        key_path = os.path.join(
            os.getenv("COLCON_PREFIX_PATH"), "..", "FogROSKEY905.pem"
        )
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
