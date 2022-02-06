from launch import FogROSLaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = FogROSLaunchDescription()
    talker_node = Node(
        package="fogros2_examples",
        executable="talker",
        output='screen',
        to_cloud = True
    )
    listener_node = Node(
        package="fogros2_examples",
        executable="listener",
        output='screen',
        to_cloud = False
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld
