from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="st7789v2_display",
            executable="display_node",
            name="st7789v2_display_node",
            output="screen",
            emulate_tty=True,
        )
    ])
