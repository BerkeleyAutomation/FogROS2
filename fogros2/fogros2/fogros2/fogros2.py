
from launch_ros.actions import Node
import pickle
from .aws import AWS
from .scp import SCP_Client
from .vpn import VPN
import logging
import os

import shutil
def make_zip_file(dir_name, target_path):
    root_dir, workspace_name = os.path.split(dir_name)
    print(root_dir, workspace_name)
    return shutil.make_archive(base_dir = workspace_name,
                               root_dir = root_dir,
                               format = "zip",
                               base_name = target_path)

def main():
    launch_new_instance = True
    if launch_new_instance:
        aws_instance = AWS()
        aws_instance.create()
        ip = aws_instance.get_ip()
        key_path = aws_instance.get_ssh_key_path()
        print(ip, key_path)
    else:
        ip = "54.219.177.93"
        key_path = "/opt/ros2_ws/FogROSKEY343.pem"

    scp = SCP_Client(ip, key_path)

    vpn = VPN(ip)
    vpn.make_wireguard_keypair()
    vpn.start()

    scp.execute_cmd("sudo apt install wireguard unzip")
    scp.send_file("/tmp/fogros-aws.conf", "/tmp/fogros-aws.conf")
    scp.execute_cmd("sudo cp /tmp/fogros-aws.conf /etc/wireguard/wg0.conf && sudo chmod 600 /etc/wireguard/wg0.conf && sudo wg-quick up wg0")

    workspace_path = "/opt/ros2_ws"
    zip_dst = "/tmp/ros_workspace"
    make_zip_file(workspace_path, zip_dst)
    scp.execute_cmd("echo removing old workspace")
    scp.execute_cmd("rm -rf ros_workspace.zip ros2_ws")
    scp.send_file(zip_dst+".zip", "/home/ubuntu/")
    scp.execute_cmd("unzip /home/ubuntu/ros_workspace.zip")

    scp.execute_cmd('''source /home/ubuntu/ros2_eloquent/install/setup.bash && cd ~/ros2_ws && colcon build && . /home/ubuntu/ros2_ws/install/setup.bash && ros2 launch fogros2 cloud.launch.py''')


if __name__ == '__main__':
    main()
