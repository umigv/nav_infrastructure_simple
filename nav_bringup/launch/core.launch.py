from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("nav_bringup")
    twist_mux_params = PathJoinSubstitution([bringup_share, "config", "twist_mux.yaml"])

    return LaunchDescription(
        [
            Node(
                package="state_machine",
                executable="state_machine",
                name="state_machine",
                output="screen",
                remappings=[
                    ("state", "state"),
                    ("state/set_ramp", "state/set_ramp"),
                    ("state/set_recovery", "state/set_recovery"),
                ],
            ),
            Node(
                package="twist_mux",
                executable="twist_mux",
                name="twist_mux",
                output="screen",
                parameters=[
                    twist_mux_params,
                ],
                remappings=[
                    ("cmd_vel_out", "cmd_vel"),
                ],
            ),
        ]
    )
