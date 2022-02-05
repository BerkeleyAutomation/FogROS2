from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    talker_node = Node(
        package="fogros2_examples",
        node_executable="talker",
        output='screen'
    )
    ld.add_action(talker_node)
    return ld
