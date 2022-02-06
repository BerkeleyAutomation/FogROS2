
from launch_ros.actions import Node
import pickle
from .aws import AWS
import logging

def main():
    with open("/opt/ros2_ws/src/fogros2/fogros2/test_node", "rb") as f:
        node_as_str = f.read()
    node = pickle.loads(node_as_str)
    print('Hi from fogros2.')

    aws_instance = AWS()
    aws_instance.create()


if __name__ == '__main__':
    main()
