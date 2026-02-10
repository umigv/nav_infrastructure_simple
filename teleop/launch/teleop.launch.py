from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

pkg_share = get_package_share_directory("teleop")

controller_config = {
    "ps4": os.path.join(pkg_share, "config", "teleop_ps4.yaml"),
    "xbox": os.path.join(pkg_share, "config", "teleop_xbox.yaml"),
}


def _launch_setup(context, *args, **kwargs):
    controller = LaunchConfiguration("controller").perform(context).strip().lower()
    if controller not in controller_config:
        raise RuntimeError(f"Unknown controller '{controller}'. Expected one of: {', '.join(controller_config.keys())}")

    teleop_config_file = controller_config[controller]
    twist_mux_topics = os.path.join(pkg_share, "config", "twist_mux_topics.yaml")
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
            parameters=[teleop_config_file],
            remappings=[("cmd_vel", "teleop_cmd_vel")],
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux_node",
            output="screen",
            parameters=[{"topics": twist_mux_topics}],
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
                description=f"Controller profile: {', '.join(controller_config.keys())}",
            ),
            DeclareLaunchArgument(
                "joystick_dev",
                default_value="/dev/input/js0",
                description="Device of joystick",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
