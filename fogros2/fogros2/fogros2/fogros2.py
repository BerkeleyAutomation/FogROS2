
from launch_ros.actions import Node
import pickle
from .aws import AWS
from .scp import SCP_Client
import logging

def main():

    # aws_instance = AWS()
    # aws_instance.create()
    ip = "13.57.18.68"
    key_path = "/opt/ros2_ws/FogROSKEY677.pem"
    scp = SCP_Client(ip, key_path)
    scp.execute_cmd("echo hello")




if __name__ == '__main__':
    main()
