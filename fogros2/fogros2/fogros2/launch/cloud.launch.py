

from launch import LaunchDescription
from launch_ros.actions import Node
import os
import pickle

def generate_launch_description():
    ld = LaunchDescription()
    node_path = "/home/ubuntu/fog_ws/src/fogros2/fogros2/fogros2/to_cloud_nodes"
    #[os.path.join(node_dir, file) for file in os.listdir(node_dir) if file.startswith("to_cloud")]
    with open(node_path, "rb") as f:
        nodes_in_str = f.read()
    nodes = pickle.loads(nodes_in_str)
    for node in nodes:
        ld.add_action(node)
        print("action added")
    return ld

