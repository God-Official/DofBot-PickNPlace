from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="dofbot_pick_place",
            executable="pick_place_node",
            name="pick_place_node",
            output="screen"
        )
    ])
