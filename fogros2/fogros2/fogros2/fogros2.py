
from launch_ros.actions import Node
import pickle
from .aws import AWS
from .scp import SCP_Client
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

    # aws_instance = AWS()
    # aws_instance.create()
    # ip = aws_instance.get_ip()
    # key_path = aws_instance.get_ssh_key_path()

    workspace_path = "/opt/ros2_ws"
    zip_dst = "/tmp/ros_workspace"
    make_zip_file(workspace_path, zip_dst)


    ip = "54.193.186.200"
    key_path = "/opt/ros2_ws/FogROSKEY602.pem"
    scp = SCP_Client(ip, key_path)
    #scp.execute_cmd("echo hello")

    scp.execute_cmd("echo removing old workspace")
    scp.execute_cmd("rm -rf ros_workspace.zip ros2_ws")

    scp.send_file(zip_dst+".zip", "/home/ubuntu/")
    scp.execute_cmd("unzip /home/ubuntu/ros_workspace.zip")
    scp.execute_cmd("source /home/ubuntu/ros2_eloquent/install/setup.bash && cd ~/ros2_ws && colcon build && . /home/ubuntu/ros2_ws/install/setup.bash && ros2 launch fogros2 cloud.launch.py")


if __name__ == '__main__':
    main()
