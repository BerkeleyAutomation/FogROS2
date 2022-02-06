

from launch import FogROSLaunchDescription
from launch_ros.actions import Node
import os
import pickle

# with open("/opt/ros2_ws/src/fogros2/fogros2/test_node", "rb") as f:
#     node_as_str = f.read()
# node = pickle.loads(node_as_str)
# print('Hi from fogros2.')

def generate_launch_description():
    ld = FogROSLaunchDescription()
    node_dir = "/opt/ros2_ws/src/fogros2/fogros2"
    node_files = [os.path.join(node_dir, file) for file in os.listdir(node_dir) if file.startswith("to_cloud")]
    print(node_files)
    for node in node_files:
        with open(node, "rb") as f:
            node_in_str = f.read()
            node = pickle.loads(node_in_str)
        ld.add_action(node)
    return ld

