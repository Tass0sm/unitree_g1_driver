"""Controller manager, robot_state_publisher and Cartesian arm controllers for the G1.

use_mock:=true  runs everything against mock_components (no robot); controllers start active.
use_mock:=false runs the arm SDK hardware interface. The hardware and the arm controllers start
                inactive; activate them explicitly (see the package README).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

HARDWARE_NAME = "G1ArmSdk"


def launch_setup(context):
    use_mock = LaunchConfiguration("use_mock").perform(context).lower() == "true"
    arms = LaunchConfiguration("arms").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    hand = LaunchConfiguration("hand").perform(context)
    if hand not in ("inspire", "rubber"):
        raise RuntimeError(f"hand must be inspire or rubber, got '{hand}'")

    sides = ["left", "right"] if arms == "both" else [arms]
    if not set(sides) <= {"left", "right"}:
        raise RuntimeError(f"arms must be left, right or both, got '{arms}'")

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare("unitree_g1_driver"), "urdf", "g1.urdf.xacro"]
                    ),
                    " use_mock:=",
                    str(use_mock).lower(),
                    " hand:=",
                    hand,
                ]
            ),
            value_type=str,
        )
    }
    controllers = PathJoinSubstitution(
        [FindPackageShare("unitree_g1_driver"), "config", "controllers.yaml"]
    )

    cm_parameters = [controllers]
    if not use_mock:
        # On the real robot, nothing is sent on arm_sdk until the hardware is activated by hand.
        cm_parameters.append({"hardware_components_initial_state.inactive": [HARDWARE_NAME]})

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=cm_parameters,
        remappings=[("~/robot_description", "/robot_description")],
        output="both",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    def spawner(names, active, extra_args=()):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=names + ([] if active else ["--inactive"]) + list(extra_args),
            output="screen",
        )

    # Each handle and its motion controller meet on /<side>_target_frame.
    arm_controllers, remaps = [], []
    for side in sides:
        arm_controllers += [f"{side}_arm_motion_controller", f"{side}_arm_handle"]
        remaps += [
            f"-r {side}_arm_handle/target_frame:=/{side}_target_frame",
            f"-r {side}_arm_motion_controller/target_frame:=/{side}_target_frame",
        ]

    nodes = [
        control_node,
        robot_state_publisher,
        spawner(["joint_state_broadcaster"], active=True),
        spawner(
            arm_controllers,
            active=use_mock,
            extra_args=["--controller-ros-args", " ".join(remaps)],
        ),
    ]
    if hand == "inspire":
        # The URDF's finger joints, from the Inspire hand drivers (open when they aren't running)
        nodes.append(Node(package="unitree_g1_driver", executable="inspire_hand_joint_states.py", output="both"))
    if rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [FindPackageShare("unitree_g1_driver"), "rviz", "g1.rviz"]
                    ),
                ],
                output="log",
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mock",
                default_value="true",
                description="true: mock hardware (no robot); false: Unitree arm SDK",
            ),
            DeclareLaunchArgument(
                "arms", default_value="left", description="Arm controllers to load: left, right or both"
            ),
            DeclareLaunchArgument(
                "hand", default_value="inspire", description="Hands in the URDF: inspire or rubber"
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz here (on the robot, run rviz.launch.py on the PC instead)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
