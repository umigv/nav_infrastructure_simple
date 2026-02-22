import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav_bringup.global_config import FRAMES

CONTROLLERS = ("ps4", "xbox")


def launch_setup(context, *args, **kwargs) -> list[LaunchDescriptionEntity]:
    pkg_share = get_package_share_directory("nav_bringup")
    controller = LaunchConfiguration("controller").perform(context).strip().lower()
    teleop_config_file = os.path.join(pkg_share, "config", f"teleop_{controller}.yaml")
    twist_mux_config_file = os.path.join(pkg_share, "config", "twist_mux.yaml")
    joystick_dev = LaunchConfiguration("joystick_dev").perform(context).strip()

    return [
        Node(
            package="joy",
            executable="joy_node",
            name="joystick_node",
            output="screen",
            parameters=[
                teleop_config_file,
                {"dev": joystick_dev},
            ],
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            output="screen",
            parameters=[teleop_config_file, {"frame": FRAMES["base_frame"]}],
            remappings=[("cmd_vel", "teleop_cmd_vel")],
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux_node",
            output="screen",
            parameters=[twist_mux_config_file],
            remappings=[
                ("cmd_vel_out", "cmd_vel"),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller",
                choices=list(CONTROLLERS),
                description="Controller profile",
            ),
            DeclareLaunchArgument(
                "joystick_dev",
                default_value="/dev/input/js0",
                description="Device of joystick",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
