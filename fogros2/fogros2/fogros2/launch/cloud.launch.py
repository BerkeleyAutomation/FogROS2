

from launch import LaunchDescription
from launch_ros.actions import Node
import os
import pickle

def generate_launch_description():
    ld = LaunchDescription()
    node_dir = "/opt/ros2_ws/src/fogros2/fogros2"
    node_files = [os.path.join(node_dir, file) for file in os.listdir(node_dir) if file.startswith("to_cloud")]
    print(node_files)
    for node in node_files:
        with open(node, "rb") as f:
            node_in_str = f.read()
            node = pickle.loads(node_in_str)
        ld.add_action(node)
    return ld

