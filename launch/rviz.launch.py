"""RViz with the G1 model and the arm interactive markers, for the PC while the control runs on the robot."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    PathJoinSubstitution([FindPackageShare("unitree_g1_driver"), "rviz", "g1.rviz"]),
                ],
                output="log",
            )
        ]
    )
