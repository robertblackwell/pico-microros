from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    talker_node = Node(
        package="py_pubsub",
        executable="talker",
    )
    listener_node = Node(
        package="py_pubsub",
        executable="listener",
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)

    return ld