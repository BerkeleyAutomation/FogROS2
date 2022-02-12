
import wgconfig
import wgconfig.wgexec as wgexec
import os

class VPN:
    def __init__(self, robot_ip, aws_key_path = '/tmp/fogros-aws.conf',
                    robot_key_path = '/tmp/fogros-local.conf'):
        self.robot_ip = robot_ip
        self.aws_key_path = aws_key_path
        self.robot_key_path = robot_key_path


    def make_wireguard_keypair(self):
        ###
        ### Begin WireGuard Setup
        ###
        aws_private_key = wgexec.generate_privatekey()
        aws_public_key = wgexec.get_publickey(aws_private_key)
        robot_private_key = wgexec.generate_privatekey()
        robot_public_key = wgexec.get_publickey(robot_private_key)

        aws_config = wgconfig.WGConfig(self.aws_key_path)
        aws_config.add_attr(None, 'PrivateKey', aws_private_key)
        aws_config.add_attr(None, 'ListenPort', 51820)
        aws_config.add_attr(None, 'Address', '10.0.0.1/24')
        aws_config.add_peer(robot_public_key, '# fogROS Robot')
        aws_config.add_attr(robot_public_key, 'AllowedIPs', '10.0.0.2/32')
        aws_config.write_file()

        robot_config = wgconfig.WGConfig(self.robot_key_path)
        robot_config.add_attr(None, 'PrivateKey', robot_private_key)
        robot_config.add_attr(None, 'ListenPort', 51820)
        robot_config.add_attr(None, 'Address', '10.0.0.2/24')
        robot_config.add_peer(aws_public_key, '# AWS')
        robot_config.add_attr(aws_public_key, 'AllowedIPs', '10.0.0.1/32')
        robot_config.add_attr(aws_public_key, 'Endpoint', f"{self.robot_ip}:51820")
        robot_config.add_attr(aws_public_key, 'PersistentKeepalive', 3)
        robot_config.write_file()

    def start(self):
        # Copy /tmp/fogros-local.conf to /etc/wireguard/wg0.conf locally.
        # TODO: This needs root. Move this to a separate script with setuid.
        os.system("sudo wg-quick down wg0")
        os.system("sudo cp /tmp/fogros-local.conf /etc/wireguard/wg0.conf")
        os.system("sudo chmod 600 /etc/wireguard/wg0.conf")
        os.system("sudo wg-quick up wg0")