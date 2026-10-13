"""g1_loco_bridge: walk the G1 with Unitree's controller from /g1/cmd_vel. On the robot."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="unitree_g1_driver",
                executable="g1_loco_bridge.py",
                name="g1_loco_bridge",
                parameters=[PathJoinSubstitution([FindPackageShare("unitree_g1_driver"), "config", "loco_bridge.yaml"])],
                output="both",
            )
        ]
    )
